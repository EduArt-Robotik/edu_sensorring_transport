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
 * All methods are available in two overloads: a raw-buffer form taking a @c uint8_t*
 * and an explicit @p length, and a convenience form taking a @c std::vector<uint8_t>.
 * The vector overloads delegate to the raw-buffer implementations.
 * Out-of-range access (read or write) throws @c std::out_of_range.
 */
class ByteOperations {
public:
  ByteOperations() = delete;

  // ── Deserialization (read from buffer) ──────────────────────────────────

  /**
   * @brief Read a single unsigned byte from the buffer.
   * @param buffer  Source byte buffer.
   * @param length  Total length of @p buffer in bytes.
   * @param offset  Byte offset to read from.
   * @return The uint8_t value at @p offset.
   * @throws std::out_of_range if the buffer is too small.
   */
  static uint8_t readUint8(const uint8_t* buffer, size_t length, size_t offset);
  static uint8_t readUint8(const std::vector<uint8_t>& buffer, size_t offset);

  /**
   * @brief Read a little-endian uint16_t from the buffer.
   * @param buffer  Source byte buffer.
   * @param length  Total length of @p buffer in bytes.
   * @param offset  Byte offset to read from.
   * @return The uint16_t value decoded from @p offset (little-endian).
   * @throws std::out_of_range if the buffer is too small.
   */
  static uint16_t readUint16(const uint8_t* buffer, size_t length, size_t offset);
  static uint16_t readUint16(const std::vector<uint8_t>& buffer, size_t offset);

  /**
   * @brief Read a little-endian uint32_t from the buffer.
   * @param buffer  Source byte buffer.
   * @param length  Total length of @p buffer in bytes.
   * @param offset  Byte offset to read from.
   * @return The uint32_t value decoded from @p offset (little-endian).
   * @throws std::out_of_range if the buffer is too small.
   */
  static uint32_t readUint32(const uint8_t* buffer, size_t length, size_t offset);
  static uint32_t readUint32(const std::vector<uint8_t>& buffer, size_t offset);

  /**
   * @brief Read a little-endian int16_t from the buffer.
   * @param buffer  Source byte buffer.
   * @param length  Total length of @p buffer in bytes.
   * @param offset  Byte offset to read from.
   * @return The int16_t value decoded from @p offset (little-endian).
   * @throws std::out_of_range if the buffer is too small.
   */
  static int16_t readInt16(const uint8_t* buffer, size_t length, size_t offset);
  static int16_t readInt16(const std::vector<uint8_t>& buffer, size_t offset);

  /**
   * @brief Read a little-endian int32_t from the buffer.
   * @param buffer  Source byte buffer.
   * @param length  Total length of @p buffer in bytes.
   * @param offset  Byte offset to read from.
   * @return The int32_t value decoded from @p offset (little-endian).
   * @throws std::out_of_range if the buffer is too small.
   */
  static int32_t readInt32(const uint8_t* buffer, size_t length, size_t offset);
  static int32_t readInt32(const std::vector<uint8_t>& buffer, size_t offset);

  /**
   * @brief Read a little-endian IEEE 754 float (32-bit) from the buffer.
   * @param buffer  Source byte buffer.
   * @param length  Total length of @p buffer in bytes.
   * @param offset  Byte offset to read from.
   * @return The float value decoded from @p offset (little-endian).
   * @throws std::out_of_range if the buffer is too small.
   */
  static float readFloat(const uint8_t* buffer, size_t length, size_t offset);
  static float readFloat(const std::vector<uint8_t>& buffer, size_t offset);

  /**
   * @brief Read a little-endian IEEE 754 double (64-bit) from the buffer.
   * @param buffer  Source byte buffer.
   * @param length  Total length of @p buffer in bytes.
   * @param offset  Byte offset to read from.
   * @return The double value decoded from @p offset (little-endian).
   * @throws std::out_of_range if the buffer is too small.
   */
  static double readDouble(const uint8_t* buffer, size_t length, size_t offset);
  static double readDouble(const std::vector<uint8_t>& buffer, size_t offset);

  // ── Serialization (write into buffer) ───────────────────────────────────

  /**
   * @brief Write a single unsigned byte into the buffer.
   * @param buffer  Destination byte buffer.
   * @param length  Total length of @p buffer in bytes.
   * @param offset  Byte offset to write to.
   * @param value   The uint8_t value to store.
   * @throws std::out_of_range if the buffer is too small.
   */
  static void writeUint8(uint8_t* buffer, size_t length, size_t offset, uint8_t value);
  static void writeUint8(std::vector<uint8_t>& buffer, size_t offset, uint8_t value);

  /**
   * @brief Write a uint16_t into the buffer in little-endian byte order.
   * @param buffer  Destination byte buffer.
   * @param length  Total length of @p buffer in bytes.
   * @param offset  Byte offset to write to.
   * @param value   The uint16_t value to store.
   * @throws std::out_of_range if the buffer is too small.
   */
  static void writeUint16(uint8_t* buffer, size_t length, size_t offset, uint16_t value);
  static void writeUint16(std::vector<uint8_t>& buffer, size_t offset, uint16_t value);

  /**
   * @brief Write a uint32_t into the buffer in little-endian byte order.
   * @param buffer  Destination byte buffer.
   * @param length  Total length of @p buffer in bytes.
   * @param offset  Byte offset to write to.
   * @param value   The uint32_t value to store.
   * @throws std::out_of_range if the buffer is too small.
   */
  static void writeUint32(uint8_t* buffer, size_t length, size_t offset, uint32_t value);
  static void writeUint32(std::vector<uint8_t>& buffer, size_t offset, uint32_t value);

  /**
   * @brief Write an int16_t into the buffer in little-endian byte order.
   * @param buffer  Destination byte buffer.
   * @param length  Total length of @p buffer in bytes.
   * @param offset  Byte offset to write to.
   * @param value   The int16_t value to store.
   * @throws std::out_of_range if the buffer is too small.
   */
  static void writeInt16(uint8_t* buffer, size_t length, size_t offset, int16_t value);
  static void writeInt16(std::vector<uint8_t>& buffer, size_t offset, int16_t value);

  /**
   * @brief Write an int32_t into the buffer in little-endian byte order.
   * @param buffer  Destination byte buffer.
   * @param length  Total length of @p buffer in bytes.
   * @param offset  Byte offset to write to.
   * @param value   The int32_t value to store.
   * @throws std::out_of_range if the buffer is too small.
   */
  static void writeInt32(uint8_t* buffer, size_t length, size_t offset, int32_t value);
  static void writeInt32(std::vector<uint8_t>& buffer, size_t offset, int32_t value);

  /**
   * @brief Write an IEEE 754 float (32-bit) into the buffer in little-endian byte order.
   * @param buffer  Destination byte buffer.
   * @param length  Total length of @p buffer in bytes.
   * @param offset  Byte offset to write to.
   * @param value   The float value to store.
   * @throws std::out_of_range if the buffer is too small.
   */
  static void writeFloat(uint8_t* buffer, size_t length, size_t offset, float value);
  static void writeFloat(std::vector<uint8_t>& buffer, size_t offset, float value);

  /**
   * @brief Write an IEEE 754 double (64-bit) into the buffer in little-endian byte order.
   * @param buffer  Destination byte buffer.
   * @param length  Total length of @p buffer in bytes.
   * @param offset  Byte offset to write to.
   * @param value   The double value to store.
   * @throws std::out_of_range if the buffer is too small.
   */
  static void writeDouble(uint8_t* buffer, size_t length, size_t offset, double value);
  static void writeDouble(std::vector<uint8_t>& buffer, size_t offset, double value);

  /**
   * @brief Serialize a uint16_t into a new byte vector in little-endian byte order.
   * @param value  The uint16_t value to serialize.
   * @return A byte vector containing the serialized value.
   */
  static std::vector<uint8_t> toBytes(uint16_t value);

  /**
   * @brief Serialize a uint32_t into a new byte vector in little-endian byte order.
   * @param value  The uint32_t value to serialize.
   * @return A byte vector containing the serialized value.
   */
  static std::vector<uint8_t> toBytes(uint32_t value);

  /**
   * @brief Serialize an int16_t into a new byte vector in little-endian byte order.
   * @param value  The int16_t value to serialize.
   * @return A byte vector containing the serialized value.
   */
  static std::vector<uint8_t> toBytes(int16_t value);

  /**
   * @brief Serialize an int32_t into a new byte vector in little-endian byte order.
   * @param value  The int32_t value to serialize.
   * @return A byte vector containing the serialized value.
   */
  static std::vector<uint8_t> toBytes(int32_t value);

  /**
   * @brief Serialize an IEEE 754 float (32-bit) into a new byte vector in little-endian byte order.
   * @param value  The float value to serialize.
   * @return A byte vector containing the serialized value.
   */
  static std::vector<uint8_t> toBytes(float value);

  /**
   * @brief Serialize an IEEE 754 double (64-bit) into a new byte vector in little-endian byte order.
   * @param value  The double value to serialize.
   * @return A byte vector containing the serialized value.
   */
  static std::vector<uint8_t> toBytes(double value);

private:
  /**
   * @brief Validate that @p offset + @p size fits within a buffer of @p length bytes.
   * @param length  Total buffer length.
   * @param offset  Start offset of the access.
   * @param size    Number of bytes to access.
   * @throws std::out_of_range if the access would exceed the buffer bounds.
   */
  static void throwIfOutOfRange(size_t length, size_t offset, size_t size);
};

} // namespace transport
} // namespace sensorring
} // namespace eduart
