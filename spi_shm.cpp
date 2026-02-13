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

static constexpr uint8_t  FLAG_BYTE = 0x7E;
static constexpr int SPI_READ_MAX = 64;    // bytes clocked per transaction (>= worst-case frame)

constexpr uint8_t  SPI_MODE  = 1;          // CPOL=0, CPHA=1
constexpr uint32_t SPI_SPEED = 1000000;    // 1 MHz
constexpr const char* SPI_DEVICE = "/dev/spidev0.0";

static constexpr int CS_PINS[4] = {22, 23, 24, 25};

static uint32_t crc32_ieee(const uint8_t* data, size_t len) {
  uint32_t crc = 0xFFFFFFFFu;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (int j = 0; j < 8; j++) {
      crc = (crc & 1u) ? ((crc >> 1) ^ 0xEDB88320u) : (crc >> 1);
    }
  }
  return ~crc;
}

static inline uint32_t u32_le_bytes(const uint8_t* p) {
  return (uint32_t)p[0] |
         ((uint32_t)p[1] << 8) |
         ((uint32_t)p[2] << 16) |
         ((uint32_t)p[3] << 24);
}

static inline float f32_le_bytes(const uint8_t* p) {
  uint32_t bits = u32_le_bytes(p);
  float out;
  std::memcpy(&out, &bits, sizeof(out));
  return out;
}

// Returns true on success and fills payload_out with the decoded payload
static bool decode_hdlc_frame(const std::vector<uint8_t>& rx,
                              size_t payload_len,
                              std::vector<uint8_t>& payload_out)
{
  payload_out.clear();
  if (payload_len == 0) {
    return false;
  }
  

  int first_flag_end_bit = -1;  // bit index where first flag ends (inclusive)
  int second_flag_start_bit = -1; // bit index where second flag starts (inclusive, i.e., last bit of flag)
  uint8_t sh = 0;
  int bit_index = 0;

  auto get_bit_msb = [&](int bi) -> uint8_t {
    int byte_i = bi >> 3;
    int bit_i  = bi & 7;
    if ((size_t)byte_i >= rx.size()) return 0;
    return (rx[byte_i] >> (7 - bit_i)) & 1u;
  };

  for (bit_index = 0; bit_index < (int)rx.size() * 8; bit_index++) {
    sh = (uint8_t)((sh << 1) | get_bit_msb(bit_index));
    if (sh == FLAG_BYTE) {
      if (first_flag_end_bit < 0) {
        first_flag_end_bit = bit_index;
        sh = 0;
      } else {
        second_flag_start_bit = bit_index;
        break;
      }
    }
  }

  if (first_flag_end_bit < 0 || second_flag_start_bit < 0) return false;

  int data_start_bit = first_flag_end_bit + 1;
  int data_end_bit_exclusive = second_flag_start_bit - 7;

  if (data_end_bit_exclusive <= data_start_bit) return false;

  const size_t want_bytes = payload_len + 4;
  std::vector<uint8_t> out(want_bytes, 0);

  size_t out_bitpos = 0;
  int ones = 0;

  for (int bi = data_start_bit; bi < data_end_bit_exclusive && out_bitpos < want_bytes * 8; bi++) {
    uint8_t bit = get_bit_msb(bi);

    if (ones == 5) {
      if (bit != 0) {
        return false;
      }
      ones = 0;
      continue;
    }

    size_t byte_i = out_bitpos >> 3;
    size_t bit_i  = out_bitpos & 7;
    if (bit) out[byte_i] |= (uint8_t)(1u << (7 - bit_i));
    out_bitpos++;

    if (bit) ones++;
    else ones = 0;
  }

  if (out_bitpos < want_bytes * 8) return false;

  uint32_t crc_rx = u32_le_bytes(out.data() + payload_len);
  uint32_t crc_ok = crc32_ieee(out.data(), payload_len);
  if (crc_rx != crc_ok) return false;

  payload_out.assign(out.begin(), out.begin() + (ptrdiff_t)payload_len);
  return true;
}

#pragma pack(push, 1)
struct Power {
  uint32_t ts;
  float current;
  float voltage;
};
#pragma pack(pop)

#pragma pack(push, 1)
struct Driver {
  uint32_t ts;
  float throttle;
  float brake;
  float turn_angle;
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
struct SensorSnapshot { // 12 * 4 + 16 = 64 bytes
  Power power_snap;
  Driver driver_snap;
  RPM rpm_snap_front;
  RPM rpm_snap_back;
  GPS gps_snap; // TODO
};
#pragma pack(pop)

#pragma pack(push, 1)
struct SharedBlock { // 68 bytes
  std::atomic<uint32_t> seq;
  SensorSnapshot data;
};
#pragma pack(pop)

static_assert(sizeof(std::atomic<uint32_t>) == 4, "atomic<uint32_t> must be 4 bytes");
static_assert(offsetof(SharedBlock, seq) == 0, "seq must be at offset 0");
static_assert(offsetof(SharedBlock, data) == 4, "data must start immediately after seq");
static_assert(sizeof(Power) == 12);
static_assert(sizeof(Driver) == 16);
static_assert(sizeof(RPM) == 12);
static_assert(sizeof(GPS) == 12);
static_assert(sizeof(SensorSnapshot) == 64);
static_assert(sizeof(SharedBlock) == 68);

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
      usleep(5000); // 200 Hz for now
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
  std::vector<uint8_t> driver_buffer = std::vector<uint8_t>(sizeof(Driver), 0x00);
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

  std::vector<uint8_t> readFramePayload(int chipSelect, size_t payload_len) {
    select_cs(chipSelect);

    std::vector<uint8_t> tx(SPI_READ_MAX, 0x00);
    std::vector<uint8_t> rx(SPI_READ_MAX, 0x00);

    spi_ioc_transfer t{};
    t.tx_buf = reinterpret_cast<unsigned long>(tx.data());
    t.rx_buf = reinterpret_cast<unsigned long>(rx.data());
    t.len = static_cast<uint32_t>(rx.size());
    t.speed_hz = SPI_SPEED;
    t.bits_per_word = 8;

    if (ioctl(spi_fd_, SPI_IOC_MESSAGE(1), &t) < 0) {
      std::perror("SPI transfer failed");
    }

    deselect_all_cs();

    std::vector<uint8_t> payload;
    if (!decode_hdlc_frame(rx, payload_len, payload)) {
      if (chipSelect == 1) { // For debugging, we only use Power
        std::fprintf(stderr, "Failed to decode HDLC frame from CS%d\n", chipSelect);
      }
      payload.clear();
    }
    return payload;
  }

  void timer_callback() {
    auto power_p = readFramePayload(1, sizeof(Power));   // 12
    auto driver_p = readFramePayload(2, sizeof(Driver)); // 16
    auto rpm_f_p  = readFramePayload(3, sizeof(RPM));    // 12
    auto rpm_b_p  = readFramePayload(4, sizeof(RPM));    // 12

    SensorSnapshot snap{};

    if (power_p.size() == sizeof(Power)) {
      const uint8_t* p = power_p.data();
      snap.power_snap.ts      = u32_le_bytes(p + 0);
      snap.power_snap.current = f32_le_bytes(p + 4);
      snap.power_snap.voltage = f32_le_bytes(p + 8);
    }

    if (driver_p.size() == sizeof(Driver)) {
      const uint8_t* p = driver_p.data();
      snap.driver_snap.ts         = u32_le_bytes(p + 0);
      snap.driver_snap.throttle   = f32_le_bytes(p + 4);
      snap.driver_snap.brake      = f32_le_bytes(p + 8);
      snap.driver_snap.turn_angle = f32_le_bytes(p + 12);
    }

    if (rpm_f_p.size() == sizeof(RPM)) {
      const uint8_t* p = rpm_f_p.data();
      snap.rpm_snap_front.ts        = u32_le_bytes(p + 0);
      snap.rpm_snap_front.rpm_left  = f32_le_bytes(p + 4);
      snap.rpm_snap_front.rpm_right = f32_le_bytes(p + 8);
    }

    if (rpm_b_p.size() == sizeof(RPM)) {
      const uint8_t* p = rpm_b_p.data();
      snap.rpm_snap_back.ts        = u32_le_bytes(p + 0);
      snap.rpm_snap_back.rpm_left  = f32_le_bytes(p + 4);
      snap.rpm_snap_back.rpm_right = f32_le_bytes(p + 8);
    }

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
