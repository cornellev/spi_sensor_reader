#include "unframe.h"

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
