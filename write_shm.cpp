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
#include <ctime>
#include <string>
#include <thread>

#include "lib/sim7x00.h"
#include "lib/arduPi.h"

constexpr uint8_t  SPI_MODE  = 1;          // CPOL=0, CPHA=1
constexpr uint32_t SPI_SPEED = 1000000;    // 1 MHz
constexpr const char* SPI_DEVICE = "/dev/spidev0.0";
constexpr uint8_t GPS_POWERKEY = 8;

static constexpr int CS_PINS[4] = {22, 23, 24, 25};
static constexpr const char* SHM_NAME = "/sensor_shm";

// Ctrl+C flag
static volatile sig_atomic_t g_stop = 0;
static void handle_sigint(int) { g_stop = 1; }

static inline uint64_t now_us() {
  timespec ts{};
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return uint64_t(ts.tv_sec) * 1000000ULL + uint64_t(ts.tv_nsec) / 1000ULL;
}

// Sensor Struct Layout

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
  GPS gps_snap;
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

// SHM Writer Class

class SHM_Writer {
public:
  SHM_Writer() {
    if (!init_pigpio()) return;
    if (!init_spi()) return;
    init_gps(); // non-fatal if it fails
    if (!init_shm()) {
      std::fprintf(stderr, "Shared memory init failed; continuing without SHM\n");
    }
    ok_ = true;
  }

  ~SHM_Writer() {
    stop_.store(true, std::memory_order_relaxed);
    if (gps_thread_.joinable()) gps_thread_.join();

    if (gps_started_) {
      sim7600.sendATcommand("AT+CGPS=0", "OK", 2000);
    }

    shutdown_shm();
    if (spi_fd_ >= 0) close(spi_fd_);
    if (pi_ >= 0) pigpio_stop(pi_);
  }

  bool ok() const { return ok_; }

  void spin() {
    while (!g_stop) {
      timer_callback();
      usleep(5000); // 200 Hz
    }
    std::fprintf(stderr, "SIGINT received, exiting...\n");
  }

private:
  int pi_{-1};
  int spi_fd_{-1};
  bool ok_{false};

  int shm_fd_{-1};
  SharedBlock* shm_{nullptr};

  // SPI buffers
  // (all these are structs, not classes, so read spi directly into vectors and then pack into the structs)
  // goofy, I know.
  std::vector<uint8_t> power_buffer_{sizeof(Power), 0x00};
  std::vector<uint8_t> motor_buffer_{sizeof(Motor), 0x00};
  std::vector<uint8_t> rpm_buffer_front_{sizeof(RPM), 0x00};
  std::vector<uint8_t> rpm_buffer_back_{sizeof(RPM), 0x00};

  // GPS thread 
  std::atomic<uint32_t> gps_seq_{0};
  GPS gps_cache_{};
  std::thread gps_thread_;
  std::atomic<bool> stop_{false};
  bool gps_started_{false};

  // initialization methods
  // idgaf abt this u can skip reading it

  bool init_pigpio() {
    pi_ = pigpio_start(nullptr, nullptr);
    if (pi_ < 0) {
      std::fprintf(stderr, "Failed to connect to pigpiod\n");
      return false;
    }
    std::fprintf(stderr, "pigpio initialized\n");

    for (int i = 0; i < 4; i++) {
      set_mode(pi_, CS_PINS[i], PI_OUTPUT);
    }
    deselect_all_cs();
    return true;
  }

  bool init_spi() {
    spi_fd_ = open(SPI_DEVICE, O_RDWR);
    if (spi_fd_ < 0) {
      std::perror("open spidev");
      return false;
    }

    uint8_t mode = SPI_MODE;
    if (ioctl(spi_fd_, SPI_IOC_WR_MODE, &mode) < 0) {
      std::perror("SPI_IOC_WR_MODE");
      return false;
    }

    uint32_t speed = SPI_SPEED;
    if (ioctl(spi_fd_, SPI_IOC_WR_MAX_SPEED_HZ, &speed) < 0) {
      std::perror("SPI_IOC_WR_MAX_SPEED_HZ");
      return false;
    }

    std::fprintf(stderr, "SPI device initialized\n");
    return true;
  }

  void init_gps() {
    sim7600.PowerOn(GPS_POWERKEY);

    if (sim7600.sendATcommand("AT+CGPS=1,1", "OK", 2000) == 1) {
      gps_started_ = true;
      std::fprintf(stderr, "GPS started\n");
    } else {
      std::fprintf(stderr, "GPS failed to start\n");
    }

    gps_thread_ = std::thread([this]() { this->gps_loop(); });
  }

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
    shm_unlink(SHM_NAME);
  }

  // SHM writer

  inline void write_snapshot(const SensorSnapshot& snap) {
    if (!shm_) return;
    uint32_t s = shm_->seq.load(std::memory_order_relaxed);
    shm_->seq.store(s + 1, std::memory_order_release); // odd => write in progress
    shm_->data = snap;
    shm_->seq.store(s + 2, std::memory_order_release); // even => stable
  }

  // SPI helpers

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

  std::vector<uint8_t> readSPIData(int chipSelect, int len) {
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

  // L Erica BOOO SHAMEE
  static inline uint32_t u32_be(const std::vector<uint8_t>& b, size_t off) {
    return (static_cast<uint32_t>(b[off + 0]) << 24) |
           (static_cast<uint32_t>(b[off + 1]) << 16) |
           (static_cast<uint32_t>(b[off + 2]) <<  8) |
           (static_cast<uint32_t>(b[off + 3])      );
  }

  static inline float f32_le(const std::vector<uint8_t>& b, size_t off) {
    uint32_t bits = (static_cast<uint32_t>(b[off + 3]) << 24) |
                    (static_cast<uint32_t>(b[off + 2]) << 16) |
                    (static_cast<uint32_t>(b[off + 1]) <<  8) |
                    (static_cast<uint32_t>(b[off + 0])      );
    float out;
    std::memcpy(&out, &bits, sizeof(out));
    return out;
  }

  // GPS cache helpers

  inline void publish_gps(const GPS& g) {
    uint32_t s = gps_seq_.load(std::memory_order_relaxed);
    gps_seq_.store(s + 1, std::memory_order_release); // odd => write in progress
    gps_cache_ = g;
    gps_seq_.store(s + 2, std::memory_order_release); // even => stable
  }

  inline GPS read_gps_cached() {
    GPS g{};
    while (true) {
      uint32_t s1 = gps_seq_.load(std::memory_order_acquire);
      if (s1 & 1) continue; // writer in progress
      g = gps_cache_;
      uint32_t s2 = gps_seq_.load(std::memory_order_acquire);
      if (s1 == s2) return g;
    }
  }

  // GPS poll over UART on SIM7600 (thread-safe by design: only GPS thread calls this)

  bool poll_gps_once(GPS& out) {
    if (!gps_started_) return false;

    if (sim7600.sendATcommand("AT+CGPSINFO", "+CGPSINFO:", 500) != 1) {
      return false;
    }

    // Read remainder for a bounded time window
    std::string buf;
    buf.reserve(256);
    uint64_t t0 = now_us();

    while (now_us() - t0 < 500000ULL) { // 500 ms max
      while (Serial.available() > 0) {
        buf.push_back(char(Serial.read()));
      }
      if (buf.find("\r\nOK") != std::string::npos || buf.find("\nOK") != std::string::npos) break;
      usleep(2000);
    }

    auto p = buf.find("+CGPSINFO:");
    if (p == std::string::npos) return false;

    std::string line = buf.substr(p);
    if (line.find(",,,,") != std::string::npos) return false; // no fix / empty

    char lat_s[16]{}, lon_s[16]{};
    char ns = 0, ew = 0;

    if (sscanf(line.c_str(), "+CGPSINFO: %15[^,],%c,%15[^,],%c", lat_s, &ns, lon_s, &ew) != 4) {
      return false;
    }

    // ddmm.mmmm / dddmm.mmmm -> decimal degrees
    double lat_ddmm = atof(lat_s);
    double lon_dddmm = atof(lon_s);

    int lat_deg = int(lat_ddmm / 100.0);
    double lat_min = lat_ddmm - (lat_deg * 100.0);
    double lat = double(lat_deg) + lat_min / 60.0;

    int lon_deg = int(lon_dddmm / 100.0);
    double lon_min = lon_dddmm - (lon_deg * 100.0);
    double lon = double(lon_deg) + lon_min / 60.0;

    if (ns == 'S') lat = -lat;
    else if (ns != 'N') return false;

    if (ew == 'W') lon = -lon;
    else if (ew != 'E') return false;

    out.ts = static_cast<uint32_t>(now_us());
    out.gps_lat  = static_cast<float>(lat);
    out.gps_long = static_cast<float>(lon);
    return true;
  }

  void gps_loop() {
    uint64_t next_poll = now_us();

    while (!stop_.load(std::memory_order_relaxed)) {
      if (!gps_started_) {
        usleep(100000); // 100 ms backoff if GPS never started
        continue;
      }

      uint64_t t = now_us();
      if (t >= next_poll) {
        next_poll += 1'000'000ULL;

        GPS g{};
        if (poll_gps_once(g)) {
          publish_gps(g);
        }
      }

      usleep(2000);
    }
  }

  // main loop callback

  void timer_callback() {
    power_buffer_ = readSPIData(1, sizeof(Power));
    motor_buffer_ = readSPIData(2, sizeof(Motor));
    rpm_buffer_front_ = readSPIData(3, sizeof(RPM));
    rpm_buffer_back_ = readSPIData(4, sizeof(RPM));

    SensorSnapshot snap{};

    // SPI slaves
    snap.power_snap.ts      = u32_be(power_buffer_, 0);
    snap.power_snap.current = f32_le(power_buffer_, 4);
    snap.power_snap.voltage = f32_le(power_buffer_, 8);

    snap.motor_snap.ts       = u32_be(motor_buffer_, 0);
    snap.motor_snap.throttle = f32_le(motor_buffer_, 4);
    snap.motor_snap.velocity = f32_le(motor_buffer_, 8);

    snap.rpm_snap_front.ts        = u32_be(rpm_buffer_front_, 0);
    snap.rpm_snap_front.rpm_left  = f32_le(rpm_buffer_front_, 4);
    snap.rpm_snap_front.rpm_right = f32_le(rpm_buffer_front_, 8);

    snap.rpm_snap_back.ts        = u32_be(rpm_buffer_back_, 0);
    snap.rpm_snap_back.rpm_left  = f32_le(rpm_buffer_back_, 4);
    snap.rpm_snap_back.rpm_right = f32_le(rpm_buffer_back_, 8);

    // GPS
    snap.gps_snap = read_gps_cached();

    write_snapshot(snap);
  }
};

int main() {
  std::signal(SIGINT, handle_sigint);

  SHM_Writer node;
  if (!node.ok()) return 1;
  node.spin();
  return 0;
}
