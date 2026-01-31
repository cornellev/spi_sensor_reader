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
#include <cstddef>


constexpr uint8_t  SPI_MODE  = 1;          // CPOL=0, CPHA=1
constexpr uint32_t SPI_SPEED = 1000000;    // 1 MHz
constexpr const char* SPI_DEVICE = "/dev/spidev0.0";

static constexpr int CS_PINS[4] = {22, 23, 24, 25};

#pragma pack(push, 1)
struct Power {
  uint32_t ts;
  float current;
  float voltage;
};
#pragma pack(pop)

#pragma pack(push, 1)
struct Motor {
  uint32_t ts;
  float throttle;
  float velocity;
};
#pragma pack(pop)

#pragma pack(push, 1)
struct RPM {
  uint32_t ts;
  float rpm_left;
  float rpm_right;
};
#pragma pack(pop)

#pragma pack(push, 1)
struct GPS {
  uint32_t ts;
  float gps_lat;
  float gps_long;
};
#pragma pack(pop)

#pragma pack(push, 1)
struct SensorSnapshot { // 12 * 5 = 60 bytes
  Power power_snap;
  Motor motor_snap;
  RPM rpm_snap_front;
  RPM rpm_snap_back;
  GPS gps_snap; // TODO
};
#pragma pack(pop)

#pragma pack(push, 1)
struct SharedBlock { // 64 bytes
  std::atomic<uint32_t> seq;
  SensorSnapshot data;
};
#pragma pack(pop)

static_assert(sizeof(std::atomic<uint32_t>) == 4, "atomic<uint32_t> must be 4 bytes");
static_assert(offsetof(SharedBlock, seq) == 0, "seq must be at offset 0");
static_assert(offsetof(SharedBlock, data) == 4, "data must start immediately after seq");
static_assert(sizeof(Power) == 12);
static_assert(sizeof(Motor) == 12);
static_assert(sizeof(RPM) == 12);
static_assert(sizeof(GPS) == 12);
static_assert(sizeof(SensorSnapshot) == 60);
static_assert(sizeof(SharedBlock) == 64);

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

    for (int i = 0; i < 4; i++) {
      set_mode(pi_, CS_PINS[i], PI_OUTPUT);
    }
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
      usleep(1000); // 1 ms => 1 kHz
    }
    std::fprintf(stderr, "SIGINT received, exiting...\n");
  }

private:
  int pi_{-1};
  int spi_fd_{-1};
  bool ok_{false};

  int shm_fd_{-1};
  SharedBlock* shm_{nullptr};

  std::vector<uint8_t> power_buffer = std::vector<uint8_t>(sizeof(Power), 0x00);
  std::vector<uint8_t> motor_buffer = std::vector<uint8_t>(sizeof(Motor), 0x00);
  std::vector<uint8_t> rpm_buffer_front = std::vector<uint8_t>(sizeof(RPM), 0x00);
  std::vector<uint8_t> rpm_buffer_back = std::vector<uint8_t>(sizeof(RPM), 0x00);
  // std::vector<uint8_t> gps_buffer = std::vector<uint8_t>(sizeof(GPS), 0x00);
  
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
    uint32_t s = shm_->seq.load(std::memory_order_relaxed);
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
    for (int i = 1; i < 5; i++) { 
      gpio_write(pi_, CS_PINS[i-1], chipSelect == i ? 0 : 1);
    }
  }

  void deselect_all_cs() {
    for (int i = 1; i < 5; i++) { 
      gpio_write(pi_, CS_PINS[i-1], 1);
    }
  }

  std::vector<uint8_t> readData(int chipSelect, int len) {
    select_cs(chipSelect);

    std::vector<uint8_t> tx(len, 0x00);
    std::vector<uint8_t> rx(len, 0x00);

    spi_ioc_transfer t{};
    t.tx_buf = reinterpret_cast<unsigned long>(tx.data());
    t.rx_buf = reinterpret_cast<unsigned long>(rx.data());
    t.len = static_cast<uint32_t>(len);
    t.speed_hz = SPI_SPEED;
    t.bits_per_word = 8;

    if (ioctl(spi_fd_, SPI_IOC_MESSAGE(1), &t) < 0) {
      std::perror("SPI transfer failed");
    }

    deselect_all_cs();
    return rx;
  }

  static inline uint32_t u32_le(const std::vector<uint8_t>& b, size_t off) {
    return (static_cast<uint32_t>(b[off + 0])      ) |
          (static_cast<uint32_t>(b[off + 1]) <<  8) |
          (static_cast<uint32_t>(b[off + 2]) << 16) |
          (static_cast<uint32_t>(b[off + 3]) << 24);
  }

  static inline float f32_le(const std::vector<uint8_t>& b, size_t off) {
    // little-endian: b[off] is LSB
    return unpack_float(b[off + 0], b[off + 1], b[off + 2], b[off + 3]);
  }

  void timer_callback() {
    power_buffer = readData(1, sizeof(Power));
    motor_buffer = readData(2, sizeof(Motor));
    rpm_buffer_front = readData(3, sizeof(RPM));
    rpm_buffer_back = readData(4, sizeof(RPM));

    SensorSnapshot snap{};

    snap.power_snap.ts      = u32_le(power_buffer, 0);
    snap.power_snap.current = f32_le(power_buffer, 4);
    snap.power_snap.voltage = f32_le(power_buffer, 8);

    snap.motor_snap.ts       = u32_le(motor_buffer, 0);
    snap.motor_snap.throttle = f32_le(motor_buffer, 4);
    snap.motor_snap.velocity = f32_le(motor_buffer, 8);

    snap.rpm_snap_front.ts        = u32_le(rpm_buffer_front, 0);
    snap.rpm_snap_front.rpm_left  = f32_le(rpm_buffer_front, 4);
    snap.rpm_snap_front.rpm_right = f32_le(rpm_buffer_front, 8);

    snap.rpm_snap_back.ts        = u32_le(rpm_buffer_back, 0);
    snap.rpm_snap_back.rpm_left  = f32_le(rpm_buffer_back, 4);
    snap.rpm_snap_back.rpm_right = f32_le(rpm_buffer_back, 8);

    // TODO: GPS reading
    snap.gps_snap.ts = 0;
    snap.gps_snap.gps_lat = 0.0f;
    snap.gps_snap.gps_long = 0.0f;

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
