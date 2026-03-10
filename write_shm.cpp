#include <atomic>
#include <cmath>
#include <csignal>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <linux/spi/spidev.h>
#include <pigpiod_if2.h>
#include <string>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <thread>
#include <unistd.h>
#include <vector>

// #include "lib/arduPi.h"
// #include "lib/sim7x00.h"

constexpr uint8_t GPS_POWERKEY = 8;

static constexpr uint8_t FLAG_BYTE = 0x7E;
static constexpr int SPI_READ_MAX = 64; // >= worst-case frame

// constexpr uint8_t SPI_MODE = 1;         // CPOL=0, CPHA=1
constexpr uint8_t SPI_MODE = SPI_MODE_1 | SPI_CS_HIGH;  // CPOL=0, CPHA=1, CS active high
constexpr uint32_t SPI_SPEED = 1000000; // 1 MHz
constexpr const char *SPI_DEVICE = "/dev/spidev0.0";

static constexpr int CS_PINS[5] = {22, 23, 24, 25, 26};

// Boards:
// RPM (fl, fr), RPM (bl, br), Joulemeter (current, voltage), Steering (brake pressure, steer angle), Motor (rpm, throttle)
// 22 - Power
// 23 - Steering
// 24 - RPM front
// 25 - RPM back
// 26 - Motor

constexpr float NANF = (std::nanf("1"));

static inline uint64_t now_us() {
    timespec ts{};
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return uint64_t(ts.tv_sec) * 1000000ULL + uint64_t(ts.tv_nsec) / 1000ULL;
}

static uint32_t crc32_ieee(const uint8_t *data, size_t len) {
    uint32_t crc = 0xFFFFFFFFu;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            crc = (crc & 1u) ? ((crc >> 1) ^ 0xEDB88320u) : (crc >> 1);
        }
    }
    return ~crc;
}

static inline uint32_t u32_le_bytes(const uint8_t *p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static inline float f32_le_bytes(const uint8_t *p) {
    uint32_t bits = u32_le_bytes(p);
    float out;
    std::memcpy(&out, &bits, sizeof(out));
    return out;
}

static inline uint8_t get_bit_msb(const std::vector<uint8_t>& data, int bit_index) {
    const int byte_i = bit_index >> 3;
    const int bit_i  = bit_index & 7;
    if (byte_i < 0 || (size_t)byte_i >= data.size()) return 0;
    return (data[(size_t)byte_i] >> (7 - bit_i)) & 1u;
}

static bool find_two_flags(const std::vector<uint8_t> &rx,
                           int *first_flag_end_bit,
                           int *second_flag_end_bit) {
    *first_flag_end_bit = -1;
    *second_flag_end_bit = -1;

    uint8_t sh = 0;
    const int nbits = (int)rx.size() * 8;

    for (int bi = 0; bi < nbits; bi++) {
        sh = (uint8_t)((sh << 1) | get_bit_msb(rx, bi));
        if (sh == FLAG_BYTE) {
            if (*first_flag_end_bit < 0) {
                *first_flag_end_bit = bi; // last bit of first flag
                sh = 0;
            } else {
                *second_flag_end_bit = bi; // last bit of second flag
                return true;
            }
        }
    }
    return false;
}


static bool bit_unstuff_range(const std::vector<uint8_t>& in,
                              int start_bit,
                              int end_bit_exclusive,
                              uint8_t* out,
                              size_t out_cap_bytes,
                              size_t* out_bitpos) {
    int ones = 0;

    for (int bi = start_bit; bi < end_bit_exclusive && *out_bitpos < out_cap_bytes * 8; bi++) {
        const uint8_t bit = get_bit_msb(in, bi);

        // If we've already seen five 1s, this bit MUST be a stuffed 0 and must be skipped.
        if (ones == 5) {
            if (bit != 0) return false; // invalid stuffing
            ones = 0;
            continue;
        }

        const size_t byte_i = (*out_bitpos) >> 3;
        const size_t bit_i  = (*out_bitpos) & 7;
        if (byte_i >= out_cap_bytes) return false;

        if (bit) out[byte_i] |= (uint8_t)(1u << (7 - bit_i));
        (*out_bitpos)++;

        if (bit) ones++;
        else     ones = 0;
    }

    return true;
}

static bool decode_frame(const std::vector<uint8_t>& rx,
                         size_t payload_len,
                         std::vector<uint8_t>& payload_out) {
    payload_out.clear();
    if (payload_len == 0) return false;

    // Find flags and compute where the data bits are.
    int first_flag_end_bit;
    int second_flag_end_bit;
    if (!find_two_flags(rx, &first_flag_end_bit, &second_flag_end_bit)) return false;

    const int data_start_bit = first_flag_end_bit + 1;
    const int data_end_bit_exclusive = second_flag_end_bit - 7;

    if (data_end_bit_exclusive <= data_start_bit) return false;

    const size_t want_bytes = payload_len + 4; // payload + crc32

    // Unstuff bits in the data range into out[], which should now contain payload || crc_le
    std::vector<uint8_t> out(want_bytes, 0);
    size_t out_bitpos = 0;
    if (!bit_unstuff_range(rx, data_start_bit, data_end_bit_exclusive,
                           out.data(), out.size(), &out_bitpos)) {
        return false;
    }

    if (out_bitpos < want_bytes * 8) return false;

    // Check CRC
    const uint32_t crc_rx = u32_le_bytes(out.data() + payload_len);
    const uint32_t crc_ok = crc32_ieee(out.data(), payload_len);
    if (crc_rx != crc_ok) return false;

    payload_out.assign(out.begin(), out.begin() + (ptrdiff_t)payload_len);
    return true;
}

#pragma pack(push, 1)
struct Power { // All from Power Pico
    uint32_t ts;
    float current;
    float voltage;
};
#pragma pack(pop)

#pragma pack(push, 1)
struct Steering { // All from Steering Pico
    uint32_t ts;
    float brake_pressure;
    float turn_angle;
};
#pragma pack(pop)

#pragma pack(push, 1)
struct RPM { // All from RPM Picos
    uint32_t ts;
    float rpm_left;
    float rpm_right;
};
#pragma pack(pop)

#pragma pack(push, 1)
struct GPS { // All from GPS thread reading SIM7600
    uint32_t ts;
    float gps_lat;
    float gps_long;
	float heading;
	float speed;
};
#pragma pack(pop)

#pragma pack(push, 1)
struct Motor { // All from Motor Pico
    uint32_t ts;
    float rpm;
    float throttle;
};
#pragma pack(pop)

#pragma pack(push, 1)
struct SensorSnapshot { // 8 + 5 * 12 + 20 = 88 bytes
    uint64_t global_ts;
    Power power_snap;
    Steering steering_snap;
    RPM rpm_snap_front;
    RPM rpm_snap_back;
    GPS gps_snap;
    Motor motor_snap;
};
#pragma pack(pop)

#pragma pack(push, 1)
struct SharedBlock { // 92 bytes
    std::atomic<uint32_t> seq;
    SensorSnapshot data;
};
#pragma pack(pop)

static_assert(sizeof(std::atomic<uint32_t>) == 4, "atomic<uint32_t> must be 4 bytes");
static_assert(offsetof(SharedBlock, seq) == 0, "seq must be at offset 0");
static_assert(offsetof(SharedBlock, data) == 4, "data must start immediately after seq");
static_assert(sizeof(Power) == 12);
static_assert(sizeof(Steering) == 12);
static_assert(sizeof(RPM) == 12);
static_assert(sizeof(GPS) == 20);
static_assert(sizeof(Motor) == 12);
static_assert(sizeof(SensorSnapshot) == 88);
static_assert(sizeof(SharedBlock) == 92);

static constexpr const char *SHM_NAME = "/sensor_shm";

// Ctrl+C flag
static volatile sig_atomic_t g_stop = 0;
static void handle_sigint(int) {
    g_stop = 1;
}

class MasterShm {
  public:
    MasterShm() {
        pi_ = pigpio_start(nullptr, nullptr);
        if (pi_ < 0) {
            std::fprintf(stderr, "Failed to connect to pigpiod\n");
            return;
        }
        std::fprintf(stderr, "pigpio initialized\n");

        for (int i = 0; i < 5; i++) {
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

        init_gps();
    }

    ~MasterShm() {
        stop_.store(true, std::memory_order_relaxed);
        if (gps_thread_.joinable())
            gps_thread_.join();

        shutdown_shm();
        if (spi_fd_ >= 0)
            close(spi_fd_);
        if (pi_ >= 0)
            pigpio_stop(pi_);
    }

    bool ok() const {
        return ok_;
    }

    // Spin calls timer_callback at every interval microseconds until SIGNINT.
    void spin(int interval) {
        while (!g_stop) {
            timer_callback();
            usleep(interval);
        }
        std::fprintf(stderr, "SIGINT received, exiting...\n");
    }

  private:
    int pi_{-1};
    int spi_fd_{-1};
    bool ok_{false};

    int shm_fd_{-1};
    SharedBlock *shm_{nullptr};

    int errcount = 0;

    // GPS thread
    std::atomic<uint32_t> gps_seq_{0};
    GPS gps_cache_{};
    std::thread gps_thread_;
    std::atomic<bool> stop_{false};
	FILE *gps_file_{nullptr};
    bool gps_started_{false};
    int gps_serial_{-1};

	void init_gps() {
	    gps_serial_ = open("/dev/serial0", O_RDONLY | O_NOCTTY);
	    if (gps_serial_ < 0) {
	        std::perror("Failed to open GPS serial");
	        return;
	    }
	
	    gps_file_ = fdopen(gps_serial_, "r");
	    if (!gps_file_) {
	        std::perror("fdopen failed");
	        close(gps_serial_);
	        gps_serial_ = -1;
	        return;
	    }
	
	    std::fprintf(stdout, "GPS fd: %d\n", gps_serial_);
	
	    GPS g{};
	    g.ts = 0;
	    g.gps_lat = NANF;
	    g.gps_long = NANF;
	    g.heading = NANF;
	    g.speed = NANF;
	    publish_gps(g);
	
	    gps_started_ = true;
	    gps_thread_ = std::thread([this]() { this->gps_loop(); });
	}

    bool init_shm() {
        shm_fd_ = shm_open(SHM_NAME, O_CREAT | O_RDWR, 0666);
        if (shm_fd_ < 0) {
            std::perror("shm_open");
            return false;
        }

        const size_t size = sizeof(SharedBlock);
        if (ftruncate(shm_fd_, size) != 0) {
            std::perror("ftruncate");
            close(shm_fd_);
            shm_fd_ = -1;
            return false;
        }

        void *p = mmap(nullptr, size, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd_, 0);
        if (p == MAP_FAILED) {
            std::perror("mmap");
            close(shm_fd_);
            shm_fd_ = -1;
            return false;
        }

        shm_ = static_cast<SharedBlock *>(p);
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
        // shm_unlink(SHM_NAME);
    }

    // Writes snap to shared memory with a simple sequence lock for synchronization.
    inline void write_snapshot(const SensorSnapshot &snap) {
        if (!shm_)
            return;
        uint32_t s = shm_->seq.load(std::memory_order_relaxed);
        shm_->seq.store(s + 1, std::memory_order_release); // odd => write in progress
        shm_->data = snap;                                 // plain memcpy-able struct
        shm_->seq.store(s + 2, std::memory_order_release); // even => stable
    }

    void select_cs(int chipSelect) {
        for (int i = 1; i < 6; i++) {
            gpio_write(pi_, CS_PINS[i - 1], chipSelect == i ? 1 : 0);
        }
    }

    void deselect_all_cs() {
        for (int i = 1; i < 6; i++) {
            gpio_write(pi_, CS_PINS[i - 1], 0);
        }
    }

    // Reads a frame from the given chip select and decodes the payload. Returns empty vector on
    // failure.
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
        if (!decode_frame(rx, payload_len, payload)) {
            payload.clear();
        }
        return payload;
    }

    // GPS helpers, lowk should be in a separate class but whatever
    inline void publish_gps(const GPS &g) {
        uint32_t s = gps_seq_.load(std::memory_order_relaxed);
        gps_seq_.store(s + 1, std::memory_order_release); // odd => write in progress
        gps_cache_ = g;
        gps_seq_.store(s + 2, std::memory_order_release); // even => stable
    }

    inline GPS read_gps_cached() {
        GPS g{}; // This is fine because if we never return empty GPS (failed read uses cache)
        while (true) {
            uint32_t s1 = gps_seq_.load(std::memory_order_acquire);
            if (s1 & 1)
                continue; // writer in progress
            g = gps_cache_;
            uint32_t s2 = gps_seq_.load(std::memory_order_acquire);
            if (s1 == s2)
                return g;
        }
    }

	bool poll_gps_once(GPS &out) {
	    if (!gps_started_ || !gps_file_) return false;
	
	    char line[512];
	    if (!fgets(line, sizeof(line), gps_file_)) return false;
	
	    if (std::strncmp(line, "$GNRMC,", 7) != 0 &&
	        std::strncmp(line, "$GPRMC,", 7) != 0) {
	        return false;
	    }
	
	    double lat_ddmm, lon_ddmm;
	    char status, ns, ew;
	
	    if (std::sscanf(line, "%*[^,],%*[^,],%c,%lf,%c,%lf,%c",
	                    &status, &lat_ddmm, &ns, &lon_ddmm, &ew) != 5) {
	        return false;
	    }
	
	    if (status != 'A') return false;
	
	    int lat_deg = int(lat_ddmm / 100.0);
	    double lat_min = lat_ddmm - lat_deg * 100.0;
	    double lat = lat_deg + lat_min / 60.0;
	
	    int lon_deg = int(lon_ddmm / 100.0);
	    double lon_min = lon_ddmm - lon_deg * 100.0;
	    double lon = lon_deg + lon_min / 60.0;
	
	    if (ns == 'S') lat = -lat;
	    if (ew == 'W') lon = -lon;
	
	    float speed = NANF;
	    float heading = NANF;
	
	    int commas = 0;
	    char *p = line;
	    while (*p && commas < 7) {
	        if (*p == ',') commas++;
	        p++;
	    }
	    if (commas < 7) return false;
	
	    if (*p && *p != ',' && *p != '*') {
	        double knots;
	        if (std::sscanf(p, "%lf", &knots) == 1) {
	            speed = static_cast<float>(0.514444 * knots);
	        }
	    }
	
	    while (*p && *p != ',' && *p != '*') p++;
	    if (*p == ',') p++;
	
	    if (*p && *p != ',' && *p != '*') {
	        double course;
	        if (std::sscanf(p, "%lf", &course) == 1) {
	            heading = static_cast<float>(course);
	        }
	    }
	
	    out.ts = static_cast<uint32_t>(now_us());
	    out.gps_lat = static_cast<float>(lat);
	    out.gps_long = static_cast<float>(lon);
	    out.heading = heading;
	    out.speed = speed;
	
	    return true;
	}

	void gps_loop() {
	    while (!stop_.load(std::memory_order_relaxed)) {
	        GPS g{};
	        if (poll_gps_once(g)) {
	            publish_gps(g);
	        }
	    }
	}

    void timer_callback() {
        auto power_p = readFramePayload(1, sizeof(Power));       // 12
        auto steering_p = readFramePayload(2, sizeof(Steering)); // 12
        auto rpm_f_p = readFramePayload(3, sizeof(RPM));         // 12
        auto rpm_b_p = readFramePayload(4, sizeof(RPM));         // 12
        auto motor_p = readFramePayload(5, sizeof(Motor));       // 12

        // Power: ts (4), current (4), voltage (4)
        // Steering: ts (4), brake_pressure (4), turn_angle (4)
        // RPM: ts (4), rpm_left (4), rpm_right (4)
        // Motor: ts (4), rpm (4), throttle (4)

        SensorSnapshot snap{};

        snap.global_ts = now_us();

        if (power_p.size() == sizeof(Power)) {
            const uint8_t *p = power_p.data();
            snap.power_snap.ts = u32_le_bytes(p + 0);
            snap.power_snap.current = f32_le_bytes(p + 4);
            snap.power_snap.voltage = f32_le_bytes(p + 8);
        } else {
            // For debugging, we only use Power for now
            errcount++;
            std::fprintf(stderr, "Failed to read Power frame, errcount %d\n", errcount);

            snap.power_snap.ts = 0;
            snap.power_snap.current = NANF;
            snap.power_snap.voltage = NANF;
        }

        if (steering_p.size() == sizeof(Steering)) {
            const uint8_t *p = steering_p.data();
            snap.steering_snap.ts = u32_le_bytes(p + 0);
            snap.steering_snap.brake_pressure = f32_le_bytes(p + 4);
            snap.steering_snap.turn_angle = f32_le_bytes(p + 8);
        } else {
			errcount++;
            std::fprintf(stderr, "Failed to read Steering frame, errcount %d\n", errcount);
			
            snap.steering_snap.ts = 0;
            snap.steering_snap.brake_pressure = NANF;
            snap.steering_snap.turn_angle = NANF;
        }

        if (rpm_f_p.size() == sizeof(RPM)) {
            const uint8_t *p = rpm_f_p.data();
            snap.rpm_snap_front.ts = u32_le_bytes(p + 0);
            snap.rpm_snap_front.rpm_left = f32_le_bytes(p + 4);
            snap.rpm_snap_front.rpm_right = f32_le_bytes(p + 8);
        } else {
			errcount++;
            std::fprintf(stderr, "Failed to read RPM frame, errcount %d\n", errcount);
			
            snap.rpm_snap_front.ts = 0;
            snap.rpm_snap_front.rpm_left = NANF;
            snap.rpm_snap_front.rpm_right = NANF;
        }

        if (rpm_b_p.size() == sizeof(RPM)) {
            const uint8_t *p = rpm_b_p.data();
            snap.rpm_snap_back.ts = u32_le_bytes(p + 0);
            snap.rpm_snap_back.rpm_left = f32_le_bytes(p + 4);
            snap.rpm_snap_back.rpm_right = f32_le_bytes(p + 8);
        } else {
            snap.rpm_snap_back.ts = 0;
            snap.rpm_snap_back.rpm_left = NANF;
            snap.rpm_snap_back.rpm_right = NANF;
        }

        if (motor_p.size() == sizeof(Motor)) {
            const uint8_t *p = motor_p.data();
            snap.motor_snap.ts = u32_le_bytes(p + 0);
            snap.motor_snap.rpm = f32_le_bytes(p + 4);
            snap.motor_snap.throttle = f32_le_bytes(p + 8);
        } else {
			errcount++;
            std::fprintf(stderr, "Failed to read Motor frame, errcount %d\n", errcount);
			
            snap.motor_snap.ts = 0;
            snap.motor_snap.rpm = NANF;
            snap.motor_snap.throttle = NANF;
        }

        snap.gps_snap = read_gps_cached(); // Latest GPS snapshot from GPS thread

        write_snapshot(snap);
    }
};

int main() {
    std::signal(SIGINT, handle_sigint);

    MasterShm node;
    if (!node.ok())
        return 1;
    node.spin(5000); // 200 Hz
    return 0;
}
