#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <linux/spi/spidev.h>
#include <pigpiod_if2.h>

#include <atomic>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <vector>
#include <csignal>

constexpr uint8_t  SPI_MODE  = 1;          // CPOL=0, CPHA=1
constexpr uint32_t SPI_SPEED = 1000000;    // 1 MHz
constexpr const char* SPI_DEVICE = "/dev/spidev0.0";

#define GPIO_CS1 23
#define GPIO_CS2 24
#define GPIO_CS3 25
#define GPIO_CS4 27
#define GPIO_CS5 22

#pragma pack(push, 1)
struct SensorSnapshot {
  uint32_t ts_sg1; uint16_t sg1[3];
  uint32_t ts_sg2; uint16_t sg2[3];
  uint32_t ts_sg3; uint16_t sg3[3];

  uint32_t ts_power;
  float current;
  float voltage;

  uint32_t ts_motor;
  float throttle;
  float velocity;
};
#pragma pack(pop)

struct SharedBlock {
  std::atomic<uint64_t> seq;   // seqlock
  SensorSnapshot data;
};

static constexpr const char* SHM_NAME = "/sensor_shm";

// Ctrl+C flag
static volatile sig_atomic_t g_stop = 0;
static void handle_sigint(int) { g_stop = 1; }

class SPIMasterShm {
public:
  SPIMasterShm() {
    pi_ = pigpio_start(nullptr, nullptr);
    if (pi_ < 0) {
      std::fprintf(stderr, "Failed to connect to pigpiod\n");
      return;
    }
    std::fprintf(stderr, "pigpio initialized\n");

    set_mode(pi_, GPIO_CS1, PI_OUTPUT);
    set_mode(pi_, GPIO_CS2, PI_OUTPUT);
    set_mode(pi_, GPIO_CS3, PI_OUTPUT);
    set_mode(pi_, GPIO_CS4, PI_OUTPUT);
    set_mode(pi_, GPIO_CS5, PI_OUTPUT);
    deselect_all_cs();

    spi_fd_ = open(SPI_DEVICE, O_RDWR);
    if (spi_fd_ < 0) {
      std::perror("open spidev");
      return;
    }

    uint8_t mode = SPI_MODE;
    if (ioctl(spi_fd_, SPI_IOC_WR_MODE, &mode) < 0) {
      std::perror("SPI_IOC_WR_MODE");
      return;
    }

    uint32_t speed = SPI_SPEED;
    if (ioctl(spi_fd_, SPI_IOC_WR_MAX_SPEED_HZ, &speed) < 0) {
      std::perror("SPI_IOC_WR_MAX_SPEED_HZ");
      return;
    }

    std::fprintf(stderr, "SPI device initialized\n");

    if (!init_shm()) {
      std::fprintf(stderr, "Shared memory init failed; continuing without SHM\n");
      // still allow SPI to run if desired
    }

    ok_ = true;
  }

  ~SPIMasterShm() {
    shutdown_shm();
    if (spi_fd_ >= 0) close(spi_fd_);
    if (pi_ >= 0) pigpio_stop(pi_);
  }

  bool ok() const { return ok_; }

  void spin() {
    while (!g_stop) {
      timer_callback();
      usleep(5000); // 5 ms => 200 Hz-ish
    }
    std::fprintf(stderr, "SIGINT received, exiting...\n");
  }

private:
  int pi_{-1};
  int spi_fd_{-1};
  bool ok_{false};

  // SHM
  int shm_fd_{-1};
  SharedBlock* shm_{nullptr};

  // same buffers as original version
  std::vector<uint8_t> p1 = std::vector<uint8_t>(10, 0x00);
  std::vector<uint8_t> p2 = std::vector<uint8_t>(10, 0x00);
  std::vector<uint8_t> p3 = std::vector<uint8_t>(10, 0x00);
  std::vector<uint8_t> p4 = std::vector<uint8_t>(12, 0x00);
  std::vector<uint8_t> p5 = std::vector<uint8_t>(12, 0x00);

  bool init_shm() {
    shm_fd_ = shm_open(SHM_NAME, O_CREAT | O_RDWR, 0666);
    if (shm_fd_ < 0) { std::perror("shm_open"); return false; }

    const size_t size = sizeof(SharedBlock);
    if (ftruncate(shm_fd_, size) != 0) {
      std::perror("ftruncate");
      close(shm_fd_);
      shm_fd_ = -1;
      return false;
    }

    void* p = mmap(nullptr, size, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd_, 0);
    if (p == MAP_FAILED) {
      std::perror("mmap");
      close(shm_fd_);
      shm_fd_ = -1;
      return false;
    }

    shm_ = static_cast<SharedBlock*>(p);
    shm_->seq.store(0, std::memory_order_relaxed);
    std::memset(&shm_->data, 0, sizeof(shm_->data));
    std::fprintf(stderr, "Shared memory ready: %s (%zu bytes)\n", SHM_NAME, size);
    return true;
  }

  void shutdown_shm() {
    if (shm_) {
      munmap(shm_, sizeof(SharedBlock));
      shm_ = nullptr;
    }
    if (shm_fd_ >= 0) {
      close(shm_fd_);
      shm_fd_ = -1;
    }
    // If you want SHM to persist across writer restarts, comment this out.
    shm_unlink(SHM_NAME);
  }

  inline void write_snapshot(const SensorSnapshot& snap) {
    if (!shm_) return;
    uint64_t s = shm_->seq.load(std::memory_order_relaxed);
    shm_->seq.store(s + 1, std::memory_order_release); // odd => write in progress
    shm_->data = snap;                                 // plain memcpy-able struct
    shm_->seq.store(s + 2, std::memory_order_release); // even => stable
  }

  static float unpack_float(uint8_t b0, uint8_t b1, uint8_t b2, uint8_t b3) {
    uint32_t bits = (static_cast<uint32_t>(b3) << 24) |
                    (static_cast<uint32_t>(b2) << 16) |
                    (static_cast<uint32_t>(b1) <<  8) |
                    (static_cast<uint32_t>(b0));
    float result;
    std::memcpy(&result, &bits, sizeof(result));
    return result;
  }

  void select_cs(int chipSelect) {
    gpio_write(pi_, GPIO_CS1, chipSelect == 1 ? 0 : 1);
    gpio_write(pi_, GPIO_CS2, chipSelect == 2 ? 0 : 1);
    gpio_write(pi_, GPIO_CS3, chipSelect == 3 ? 0 : 1);
    gpio_write(pi_, GPIO_CS4, chipSelect == 4 ? 0 : 1);
    gpio_write(pi_, GPIO_CS5, chipSelect == 5 ? 0 : 1);
  }

  void deselect_all_cs() {
    gpio_write(pi_, GPIO_CS1, 1);
    gpio_write(pi_, GPIO_CS2, 1);
    gpio_write(pi_, GPIO_CS3, 1);
    gpio_write(pi_, GPIO_CS4, 1);
    gpio_write(pi_, GPIO_CS5, 1);
  }

  std::vector<uint8_t> readData(int chipSelect, int len) {
    select_cs(chipSelect);

    std::vector<uint8_t> tx(len, 0x00);
    std::vector<uint8_t> rx(len, 0x00);

    spi_ioc_transfer t{};
    t.tx_buf = reinterpret_cast<unsigned long>(tx.data());
    t.rx_buf = reinterpret_cast<unsigned long>(rx.data());
    t.len = len;
    t.speed_hz = SPI_SPEED;
    t.bits_per_word = 8;

    if (ioctl(spi_fd_, SPI_IOC_MESSAGE(1), &t) < 0) {
      std::perror("SPI transfer failed");
    }

    deselect_all_cs();
    return rx;
  }

  void timer_callback() {
    p1 = readData(1, 10);
    p2 = readData(2, 10);
    p3 = readData(3, 10);
    p4 = readData(4, 12);
    p5 = readData(5, 12);

    SensorSnapshot snap{};
    uint32_t timestamp = 0;

    timestamp = (p1[3]) | (p1[2] << 8) | (p1[1] << 16) | (p1[0] << 24);
    snap.ts_sg1 = timestamp;
    snap.sg1[0] = (p1[5]) | (p1[4] << 8);
    snap.sg1[1] = (p1[7]) | (p1[6] << 8);
    snap.sg1[2] = (p1[9]) | (p1[8] << 8);

    timestamp = (p2[3]) | (p2[2] << 8) | (p2[1] << 16) | (p2[0] << 24);
    snap.ts_sg2 = timestamp;
    snap.sg2[0] = (p2[5]) | (p2[4] << 8);
    snap.sg2[1] = (p2[7]) | (p2[6] << 8);
    snap.sg2[2] = (p2[9]) | (p2[8] << 8);

    timestamp = (p3[3]) | (p3[2] << 8) | (p3[1] << 16) | (p3[0] << 24);
    snap.ts_sg3 = timestamp;
    snap.sg3[0] = (p3[5]) | (p3[4] << 8);
    snap.sg3[1] = (p3[7]) | (p3[6] << 8);
    snap.sg3[2] = (p3[9]) | (p3[8] << 8);

    timestamp = (p4[3]) | (p4[2] << 8) | (p4[1] << 16) | (p4[0] << 24);
    snap.ts_power = timestamp;
    snap.current  = unpack_float(p4[4], p4[5], p4[6], p4[7]);
    snap.voltage  = unpack_float(p4[8], p4[9], p4[10], p4[11]);

    timestamp = (p5[3]) | (p5[2] << 8) | (p5[1] << 16) | (p5[0] << 24);
    snap.ts_motor = timestamp;
    snap.throttle = unpack_float(p5[4], p5[5], p5[6], p5[7]);
    snap.velocity = unpack_float(p5[8], p5[9], p5[10], p5[11]);

    write_snapshot(snap);
  }
};

int main() {
  std::signal(SIGINT, handle_sigint);

  SPIMasterShm node;
  if (!node.ok()) return 1;
  node.spin(); // returns on Ctrl+C
  return 0;
}
