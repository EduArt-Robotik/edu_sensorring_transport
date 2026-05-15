/**
 * @file   ByteOperations.hpp
 * @brief  Little-endian serialization and deserialization helpers for raw byte buffers.
 */

#pragma once

#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <vector>

namespace eduart {
namespace sensorring {
namespace transport {

/**
 * @brief Provides static helpers to serialize (write) primitive types into a byte buffer
 *        and deserialize (read) them back, using little-endian byte order.
 *
 * All methods operate on a @c std::vector<uint8_t> at a caller-specified byte offset.
 * Out-of-range access (read or write) throws @c std::out_of_range.
 */
class ByteOperations {
public:
  ByteOperations() = delete;

  // ── Deserialization (read from buffer) ──────────────────────────────────

  /**
   * @brief Read a single unsigned byte from the buffer.
   * @param buffer  Source byte buffer.
   * @param offset  Byte offset to read from.
   * @return The uint8_t value at @p offset.
   * @throws std::out_of_range if the buffer is too small.
   */
  static uint8_t readUint8(const std::vector<uint8_t>& buffer, size_t offset) {
    throwIfOutOfRange(buffer, offset, sizeof(uint8_t));
    return buffer[offset];
  }

  /**
   * @brief Read a little-endian uint16_t from the buffer.
   * @param buffer  Source byte buffer.
   * @param offset  Byte offset to read from.
   * @return The uint16_t value decoded from @p offset (little-endian).
   * @throws std::out_of_range if the buffer is too small.
   */
  static uint16_t readUint16(const std::vector<uint8_t>& buffer, size_t offset) {
    throwIfOutOfRange(buffer, offset, sizeof(uint16_t));
    return static_cast<uint16_t>(buffer[offset]) | (static_cast<uint16_t>(buffer[offset + 1]) << 8);
  }

  /**
   * @brief Read a little-endian uint32_t from the buffer.
   * @param buffer  Source byte buffer.
   * @param offset  Byte offset to read from.
   * @return The uint32_t value decoded from @p offset (little-endian).
   * @throws std::out_of_range if the buffer is too small.
   */
  static uint32_t readUint32(const std::vector<uint8_t>& buffer, size_t offset) {
    throwIfOutOfRange(buffer, offset, sizeof(uint32_t));
    return static_cast<uint32_t>(buffer[offset]) | (static_cast<uint32_t>(buffer[offset + 1]) << 8) | (static_cast<uint32_t>(buffer[offset + 2]) << 16) | (static_cast<uint32_t>(buffer[offset + 3]) << 24);
  }

  /**
   * @brief Read a little-endian int16_t from the buffer.
   * @param buffer  Source byte buffer.
   * @param offset  Byte offset to read from.
   * @return The int16_t value decoded from @p offset (little-endian).
   * @throws std::out_of_range if the buffer is too small.
   */
  static int16_t readInt16(const std::vector<uint8_t>& buffer, size_t offset) { return static_cast<int16_t>(readUint16(buffer, offset)); }

  /**
   * @brief Read a little-endian int32_t from the buffer.
   * @param buffer  Source byte buffer.
   * @param offset  Byte offset to read from.
   * @return The int32_t value decoded from @p offset (little-endian).
   * @throws std::out_of_range if the buffer is too small.
   */
  static int32_t readInt32(const std::vector<uint8_t>& buffer, size_t offset) { return static_cast<int32_t>(readUint32(buffer, offset)); }

  /**
   * @brief Read a little-endian IEEE 754 float (32-bit) from the buffer.
   * @param buffer  Source byte buffer.
   * @param offset  Byte offset to read from.
   * @return The float value decoded from @p offset (little-endian).
   * @throws std::out_of_range if the buffer is too small.
   */
  static float readFloat(const std::vector<uint8_t>& buffer, size_t offset) {
    static_assert(sizeof(float) == 4, "float must be 4 bytes");
    const uint32_t bits = readUint32(buffer, offset);
    float value;
    std::memcpy(&value, &bits, sizeof(float));
    return value;
  }

  /**
   * @brief Read a little-endian IEEE 754 double (64-bit) from the buffer.
   * @param buffer  Source byte buffer.
   * @param offset  Byte offset to read from.
   * @return The double value decoded from @p offset (little-endian).
   * @throws std::out_of_range if the buffer is too small.
   */
  static double readDouble(const std::vector<uint8_t>& buffer, size_t offset) {
    static_assert(sizeof(double) == 8, "double must be 8 bytes");
    throwIfOutOfRange(buffer, offset, sizeof(double));
    uint64_t bits = 0;
    for (size_t i = 0; i < 8; ++i) {
      bits |= static_cast<uint64_t>(buffer[offset + i]) << (i * 8);
    }
    double value;
    std::memcpy(&value, &bits, sizeof(double));
    return value;
  }

  // ── Serialization (write into buffer) ───────────────────────────────────

  /**
   * @brief Write a single unsigned byte into the buffer.
   * @param buffer  Destination byte buffer.
   * @param offset  Byte offset to write to.
   * @param value   The uint8_t value to store.
   * @throws std::out_of_range if the buffer is too small.
   */
  static void writeUint8(std::vector<uint8_t>& buffer, size_t offset, uint8_t value) {
    throwIfOutOfRange(buffer, offset, sizeof(uint8_t));
    buffer[offset] = value;
  }

  /**
   * @brief Write a uint16_t into the buffer in little-endian byte order.
   * @param buffer  Destination byte buffer.
   * @param offset  Byte offset to write to.
   * @param value   The uint16_t value to store.
   * @throws std::out_of_range if the buffer is too small.
   */
  static void writeUint16(std::vector<uint8_t>& buffer, size_t offset, uint16_t value) {
    throwIfOutOfRange(buffer, offset, sizeof(uint16_t));
    buffer[offset]     = static_cast<uint8_t>(value);
    buffer[offset + 1] = static_cast<uint8_t>(value >> 8);
  }

  /**
   * @brief Write a uint32_t into the buffer in little-endian byte order.
   * @param buffer  Destination byte buffer.
   * @param offset  Byte offset to write to.
   * @param value   The uint32_t value to store.
   * @throws std::out_of_range if the buffer is too small.
   */
  static void writeUint32(std::vector<uint8_t>& buffer, size_t offset, uint32_t value) {
    throwIfOutOfRange(buffer, offset, sizeof(uint32_t));
    buffer[offset]     = static_cast<uint8_t>(value);
    buffer[offset + 1] = static_cast<uint8_t>(value >> 8);
    buffer[offset + 2] = static_cast<uint8_t>(value >> 16);
    buffer[offset + 3] = static_cast<uint8_t>(value >> 24);
  }

  /**
   * @brief Write an int16_t into the buffer in little-endian byte order.
   * @param buffer  Destination byte buffer.
   * @param offset  Byte offset to write to.
   * @param value   The int16_t value to store.
   * @throws std::out_of_range if the buffer is too small.
   */
  static void writeInt16(std::vector<uint8_t>& buffer, size_t offset, int16_t value) { writeUint16(buffer, offset, static_cast<uint16_t>(value)); }

  /**
   * @brief Write an int32_t into the buffer in little-endian byte order.
   * @param buffer  Destination byte buffer.
   * @param offset  Byte offset to write to.
   * @param value   The int32_t value to store.
   * @throws std::out_of_range if the buffer is too small.
   */
  static void writeInt32(std::vector<uint8_t>& buffer, size_t offset, int32_t value) { writeUint32(buffer, offset, static_cast<uint32_t>(value)); }

  /**
   * @brief Write an IEEE 754 float (32-bit) into the buffer in little-endian byte order.
   * @param buffer  Destination byte buffer.
   * @param offset  Byte offset to write to.
   * @param value   The float value to store.
   * @throws std::out_of_range if the buffer is too small.
   */
  static void writeFloat(std::vector<uint8_t>& buffer, size_t offset, float value) {
    static_assert(sizeof(float) == 4, "float must be 4 bytes");
    uint32_t bits;
    std::memcpy(&bits, &value, sizeof(float));
    writeUint32(buffer, offset, bits);
  }

  /**
   * @brief Write an IEEE 754 double (64-bit) into the buffer in little-endian byte order.
   * @param buffer  Destination byte buffer.
   * @param offset  Byte offset to write to.
   * @param value   The double value to store.
   * @throws std::out_of_range if the buffer is too small.
   */
  static void writeDouble(std::vector<uint8_t>& buffer, size_t offset, double value) {
    static_assert(sizeof(double) == 8, "double must be 8 bytes");
    throwIfOutOfRange(buffer, offset, sizeof(double));
    uint64_t bits;
    std::memcpy(&bits, &value, sizeof(double));
    for (size_t i = 0; i < 8; ++i) {
      buffer[offset + i] = static_cast<uint8_t>(bits >> (i * 8));
    }
  }

private:
  /**
   * @brief Validate that @p offset + @p size fits within the buffer.
   * @param buffer  The byte buffer to check.
   * @param offset  Start offset of the access.
   * @param size    Number of bytes to access.
   * @throws std::out_of_range if the access would exceed the buffer bounds.
   */
  static void throwIfOutOfRange(const std::vector<uint8_t>& buffer, size_t offset, size_t size) {
    if (offset + size > buffer.size()) {
      throw std::out_of_range("BufferOps: access past end of buffer");
    }
  }
};

} // namespace transport
} // namespace sensorring
} // namespace eduart
