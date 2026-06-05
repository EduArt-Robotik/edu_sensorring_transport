/**
 * @file   ByteOperations.cpp
 * @brief  Little-endian serialization and deserialization helpers for raw byte buffers.
 */

#include "sensorring_transport/ByteOperations.hpp"

#if defined(__cpp_exceptions)
#include <stdexcept>
#else
#include <cassert>
#endif

namespace eduart {
namespace sensorring {
namespace transport {

// ── Private helper ───────────────────────────────────────────────────────────

void ByteOperations::throwIfOutOfRange(size_t length, size_t offset, size_t size) {
  if (offset + size > length) {
#if defined(__cpp_exceptions)
    throw std::out_of_range("BufferOps: access past end of buffer");
#else
    assert(false && "BufferOps: access past end of buffer");
#endif
  }
}

// ── Deserialization (read from buffer) ───────────────────────────────────────

uint8_t ByteOperations::readUint8(const uint8_t* buffer, size_t length, size_t offset) {
  throwIfOutOfRange(length, offset, sizeof(uint8_t));
  return buffer[offset];
}

uint8_t ByteOperations::readUint8(const std::vector<uint8_t>& buffer, size_t offset) {
  return readUint8(buffer.data(), buffer.size(), offset);
}

uint16_t ByteOperations::readUint16(const uint8_t* buffer, size_t length, size_t offset) {
  throwIfOutOfRange(length, offset, sizeof(uint16_t));
  return static_cast<uint16_t>(buffer[offset]) | (static_cast<uint16_t>(buffer[offset + 1]) << 8);
}

uint16_t ByteOperations::readUint16(const std::vector<uint8_t>& buffer, size_t offset) {
  return readUint16(buffer.data(), buffer.size(), offset);
}

uint32_t ByteOperations::readUint32(const uint8_t* buffer, size_t length, size_t offset) {
  throwIfOutOfRange(length, offset, sizeof(uint32_t));
  return static_cast<uint32_t>(buffer[offset]) | (static_cast<uint32_t>(buffer[offset + 1]) << 8) | (static_cast<uint32_t>(buffer[offset + 2]) << 16) | (static_cast<uint32_t>(buffer[offset + 3]) << 24);
}

uint32_t ByteOperations::readUint32(const std::vector<uint8_t>& buffer, size_t offset) {
  return readUint32(buffer.data(), buffer.size(), offset);
}

int16_t ByteOperations::readInt16(const uint8_t* buffer, size_t length, size_t offset) {
  return static_cast<int16_t>(readUint16(buffer, length, offset));
}

int16_t ByteOperations::readInt16(const std::vector<uint8_t>& buffer, size_t offset) {
  return readInt16(buffer.data(), buffer.size(), offset);
}

int32_t ByteOperations::readInt32(const uint8_t* buffer, size_t length, size_t offset) {
  return static_cast<int32_t>(readUint32(buffer, length, offset));
}

int32_t ByteOperations::readInt32(const std::vector<uint8_t>& buffer, size_t offset) {
  return readInt32(buffer.data(), buffer.size(), offset);
}

float ByteOperations::readFloat(const uint8_t* buffer, size_t length, size_t offset) {
  static_assert(sizeof(float) == 4, "float must be 4 bytes");
  const uint32_t bits = readUint32(buffer, length, offset);
  float value;
  std::memcpy(&value, &bits, sizeof(float));
  return value;
}

float ByteOperations::readFloat(const std::vector<uint8_t>& buffer, size_t offset) {
  return readFloat(buffer.data(), buffer.size(), offset);
}

double ByteOperations::readDouble(const uint8_t* buffer, size_t length, size_t offset) {
  static_assert(sizeof(double) == 8, "double must be 8 bytes");
  throwIfOutOfRange(length, offset, sizeof(double));
  uint64_t bits = 0;
  for (size_t i = 0; i < 8; ++i) {
    bits |= static_cast<uint64_t>(buffer[offset + i]) << (i * 8);
  }
  double value;
  std::memcpy(&value, &bits, sizeof(double));
  return value;
}

double ByteOperations::readDouble(const std::vector<uint8_t>& buffer, size_t offset) {
  return readDouble(buffer.data(), buffer.size(), offset);
}

// ── Serialization (write into buffer) ────────────────────────────────────────

void ByteOperations::writeUint8(uint8_t* buffer, size_t length, size_t offset, uint8_t value) {
  throwIfOutOfRange(length, offset, sizeof(uint8_t));
  buffer[offset] = value;
}

void ByteOperations::writeUint8(std::vector<uint8_t>& buffer, size_t offset, uint8_t value) {
  writeUint8(buffer.data(), buffer.size(), offset, value);
}

void ByteOperations::writeUint16(uint8_t* buffer, size_t length, size_t offset, uint16_t value) {
  throwIfOutOfRange(length, offset, sizeof(uint16_t));
  buffer[offset]     = static_cast<uint8_t>(value);
  buffer[offset + 1] = static_cast<uint8_t>(value >> 8);
}

void ByteOperations::writeUint16(std::vector<uint8_t>& buffer, size_t offset, uint16_t value) {
  writeUint16(buffer.data(), buffer.size(), offset, value);
}

void ByteOperations::writeUint32(uint8_t* buffer, size_t length, size_t offset, uint32_t value) {
  throwIfOutOfRange(length, offset, sizeof(uint32_t));
  buffer[offset]     = static_cast<uint8_t>(value);
  buffer[offset + 1] = static_cast<uint8_t>(value >> 8);
  buffer[offset + 2] = static_cast<uint8_t>(value >> 16);
  buffer[offset + 3] = static_cast<uint8_t>(value >> 24);
}

void ByteOperations::writeUint32(std::vector<uint8_t>& buffer, size_t offset, uint32_t value) {
  writeUint32(buffer.data(), buffer.size(), offset, value);
}

void ByteOperations::writeInt16(uint8_t* buffer, size_t length, size_t offset, int16_t value) {
  writeUint16(buffer, length, offset, static_cast<uint16_t>(value));
}

void ByteOperations::writeInt16(std::vector<uint8_t>& buffer, size_t offset, int16_t value) {
  writeInt16(buffer.data(), buffer.size(), offset, value);
}

void ByteOperations::writeInt32(uint8_t* buffer, size_t length, size_t offset, int32_t value) {
  writeUint32(buffer, length, offset, static_cast<uint32_t>(value));
}

void ByteOperations::writeInt32(std::vector<uint8_t>& buffer, size_t offset, int32_t value) {
  writeInt32(buffer.data(), buffer.size(), offset, value);
}

void ByteOperations::writeFloat(uint8_t* buffer, size_t length, size_t offset, float value) {
  static_assert(sizeof(float) == 4, "float must be 4 bytes");
  uint32_t bits;
  std::memcpy(&bits, &value, sizeof(float));
  writeUint32(buffer, length, offset, bits);
}

void ByteOperations::writeFloat(std::vector<uint8_t>& buffer, size_t offset, float value) {
  writeFloat(buffer.data(), buffer.size(), offset, value);
}

void ByteOperations::writeDouble(uint8_t* buffer, size_t length, size_t offset, double value) {
  static_assert(sizeof(double) == 8, "double must be 8 bytes");
  throwIfOutOfRange(length, offset, sizeof(double));
  uint64_t bits;
  std::memcpy(&bits, &value, sizeof(double));
  for (size_t i = 0; i < 8; ++i) {
    buffer[offset + i] = static_cast<uint8_t>(bits >> (i * 8));
  }
}

void ByteOperations::writeDouble(std::vector<uint8_t>& buffer, size_t offset, double value) {
  writeDouble(buffer.data(), buffer.size(), offset, value);
}

} // namespace transport
} // namespace sensorring
} // namespace eduart
