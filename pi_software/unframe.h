#include <cstdint>
#include <cstddef>
#include <cstring>
#include <vector>
#include <ctime>

constexpr uint8_t FLAG_BYTE = 0x7E;

// Return current monotonic time in microseconds.
uint64_t now_us();

// Compute IEEE CRC32 of a byte buffer.
uint32_t crc32_ieee(const uint8_t *data, size_t len);

// Read little-endian 32-bit integer from byte buffer.
uint32_t u32_le_bytes(const uint8_t *p);

// Read little-endian float from byte buffer.
float f32_le_bytes(const uint8_t *p);

// Decode a framed SPI message.
// rx: raw received bytes
// payload_len: expected payload length (excluding CRC)
// payload_out: decoded payload if successful
// returns true if frame was valid and CRC passed.
bool decode_frame(const std::vector<uint8_t>& rx,
                  size_t payload_len,
                  std::vector<uint8_t>& payload_out);
