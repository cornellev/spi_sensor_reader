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
struct SensorSnapshot {
  Power power_snap;
  Motor motor_snap;
  RPM rpm_snap_front;
  RPM rpm_snap_back;
  // GPS gps_snap; TODO
};
#pragma pack(pop)

#pragma pack(push, 1)
struct SharedBlock {
  std::atomic<uint32_t> seq;
  SensorSnapshot data;
};
#pragma pack(pop)

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
      usleep(5000); // 5 ms => 200 Hz-ish
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

  void timer_callback() {
    power_buffer = readData(1, sizeof(Power));
    motor_buffer = readData(2, sizeof(Motor));
    rpm_buffer_front = readData(3, sizeof(RPM));
    rpm_buffer_back = readData(4, sizeof(RPM));

    SensorSnapshot snap{};

    // TODO: factor bitwise ops into helpers
    snap.power_snap.ts = (power_buffer[3]) | (power_buffer[2] << 8) | (power_buffer[1] << 16) | (power_buffer[0] << 24);
    snap.power_snap.current = unpack_float(power_buffer[4], power_buffer[5], power_buffer[6], power_buffer[7]);
    snap.power_snap.voltage = unpack_float(power_buffer[8], power_buffer[9], power_buffer[10], power_buffer[11]);

    snap.motor_snap.ts = (motor_buffer[3]) | (motor_buffer[2] << 8) | (motor_buffer[1] << 16) | (motor_buffer[0] << 24);
    snap.motor_snap.throttle = unpack_float(motor_buffer[4], motor_buffer[5], motor_buffer[6], motor_buffer[7]);
    snap.motor_snap.velocity = unpack_float(motor_buffer[8], motor_buffer[9], motor_buffer[10], motor_buffer[11]);

    snap.rpm_snap_front.ts = (rpm_buffer_front[3]) | (rpm_buffer_front[2] << 8) | (rpm_buffer_front[1] << 16) | (rpm_buffer_front[0] << 24);
    snap.rpm_snap_front.rpm_left = unpack_float(rpm_buffer_front[4], rpm_buffer_front[5], rpm_buffer_front[6], rpm_buffer_front[7]);
    snap.rpm_snap_front.rpm_right = unpack_float(rpm_buffer_front[8], rpm_buffer_front[9], rpm_buffer_front[10], rpm_buffer_front[11]);

    snap.rpm_snap_back.ts = (rpm_buffer_back[3]) | (rpm_buffer_back[2] << 8) | (rpm_buffer_back[1] << 16) | (rpm_buffer_back[0] << 24);
    snap.rpm_snap_back.rpm_left = unpack_float(rpm_buffer_back[4], rpm_buffer_back[5], rpm_buffer_back[6], rpm_buffer_back[7]);
    snap.rpm_snap_back.rpm_right = unpack_float(rpm_buffer_back[8], rpm_buffer_back[9], rpm_buffer_back[10], rpm_buffer_back[11]);

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
