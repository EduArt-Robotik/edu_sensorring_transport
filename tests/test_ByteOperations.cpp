#include <catch2/catch_all.hpp>
#include <cstdint>
#include <cstring>
#include <limits>
#include <vector>

#include "sensorring_transport/ByteOperations.hpp"

using ByteOps = eduart::sensorring::transport::ByteOperations;

// All tests use the std::vector<uint8_t> overloads. Those overloads delegate
// directly to the raw-buffer (uint8_t*, size_t length) implementations, so the
// tests below indirectly cover the raw-buffer code paths as well.

// ── uint8 ────────────────────────────────────────────────────────────────────

TEST_CASE("BufferOps: write/read uint8", "[BufferOps]") {
  std::vector<uint8_t> buf(1);
  ByteOps::writeUint8(buf, 0, 0xAB);
  REQUIRE(ByteOps::readUint8(buf, 0) == 0xAB);
}

// ── uint16 ───────────────────────────────────────────────────────────────────

TEST_CASE("BufferOps: write/read uint16 little-endian", "[BufferOps]") {
  std::vector<uint8_t> buf(2);
  ByteOps::writeUint16(buf, 0, 0x1234);
  // Little-endian: low byte first
  REQUIRE(buf[0] == 0x34);
  REQUIRE(buf[1] == 0x12);
  REQUIRE(ByteOps::readUint16(buf, 0) == 0x1234);
}

TEST_CASE("BufferOps: uint16 boundary values", "[BufferOps]") {
  std::vector<uint8_t> buf(4);
  ByteOps::writeUint16(buf, 0, 0x0000);
  ByteOps::writeUint16(buf, 2, 0xFFFF);
  REQUIRE(ByteOps::readUint16(buf, 0) == 0x0000);
  REQUIRE(ByteOps::readUint16(buf, 2) == 0xFFFF);
}

// ── uint32 ───────────────────────────────────────────────────────────────────

TEST_CASE("BufferOps: write/read uint32 little-endian", "[BufferOps]") {
  std::vector<uint8_t> buf(4);
  ByteOps::writeUint32(buf, 0, 0xDEADBEEF);
  REQUIRE(buf[0] == 0xEF);
  REQUIRE(buf[1] == 0xBE);
  REQUIRE(buf[2] == 0xAD);
  REQUIRE(buf[3] == 0xDE);
  REQUIRE(ByteOps::readUint32(buf, 0) == 0xDEADBEEF);
}

// ── int16 ────────────────────────────────────────────────────────────────────

TEST_CASE("BufferOps: write/read negative int16", "[BufferOps]") {
  std::vector<uint8_t> buf(2);
  ByteOps::writeInt16(buf, 0, -1);
  REQUIRE(ByteOps::readInt16(buf, 0) == -1);
}

TEST_CASE("BufferOps: write/read positive int16", "[BufferOps]") {
  std::vector<uint8_t> buf(2);
  ByteOps::writeInt16(buf, 0, 1000);
  REQUIRE(ByteOps::readInt16(buf, 0) == 1000);
}

// ── int32 ────────────────────────────────────────────────────────────────────

TEST_CASE("BufferOps: write/read negative int32", "[BufferOps]") {
  std::vector<uint8_t> buf(4);
  ByteOps::writeInt32(buf, 0, -123456);
  REQUIRE(ByteOps::readInt32(buf, 0) == -123456);
}

// ── float ────────────────────────────────────────────────────────────────────

TEST_CASE("BufferOps: write/read float", "[BufferOps]") {
  std::vector<uint8_t> buf(4);
  ByteOps::writeFloat(buf, 0, 3.14f);
  REQUIRE(ByteOps::readFloat(buf, 0) == Catch::Approx(3.14f));
}

TEST_CASE("BufferOps: float zero and negative", "[BufferOps]") {
  std::vector<uint8_t> buf(8);
  ByteOps::writeFloat(buf, 0, 0.0f);
  ByteOps::writeFloat(buf, 4, -1.5f);
  REQUIRE(ByteOps::readFloat(buf, 0) == 0.0f);
  REQUIRE(ByteOps::readFloat(buf, 4) == Catch::Approx(-1.5f));
}

// ── double ───────────────────────────────────────────────────────────────────

TEST_CASE("BufferOps: write/read double", "[BufferOps]") {
  std::vector<uint8_t> buf(8);
  ByteOps::writeDouble(buf, 0, 3.141592653589793);
  REQUIRE(ByteOps::readDouble(buf, 0) == Catch::Approx(3.141592653589793));
}

// ── toBytes convenience helpers ─────────────────────────────────────────────

TEST_CASE("BufferOps: toBytes uint16 returns little-endian bytes", "[BufferOps]") {
  const std::vector<uint8_t> bytes = ByteOps::toBytes(static_cast<uint16_t>(0x1234));

  REQUIRE(bytes == std::vector<uint8_t>{ 0x34, 0x12 });
  REQUIRE(ByteOps::readUint16(bytes, 0) == 0x1234);
}

TEST_CASE("BufferOps: toBytes uint32 returns little-endian bytes", "[BufferOps]") {
  const std::vector<uint8_t> bytes = ByteOps::toBytes(static_cast<uint32_t>(0xDEADBEEF));

  REQUIRE(bytes == std::vector<uint8_t>{ 0xEF, 0xBE, 0xAD, 0xDE });
  REQUIRE(ByteOps::readUint32(bytes, 0) == 0xDEADBEEF);
}

TEST_CASE("BufferOps: toBytes int16 preserves two's-complement representation", "[BufferOps]") {
  const std::vector<uint8_t> bytes = ByteOps::toBytes(static_cast<int16_t>(-2));

  REQUIRE(bytes == std::vector<uint8_t>{ 0xFE, 0xFF });
  REQUIRE(ByteOps::readInt16(bytes, 0) == -2);
}

TEST_CASE("BufferOps: toBytes int32 preserves two's-complement representation", "[BufferOps]") {
  const std::vector<uint8_t> bytes = ByteOps::toBytes(static_cast<int32_t>(-123456));

  REQUIRE(bytes == std::vector<uint8_t>{ 0xC0, 0x1D, 0xFE, 0xFF });
  REQUIRE(ByteOps::readInt32(bytes, 0) == -123456);
}

TEST_CASE("BufferOps: toBytes float matches writeFloat bit pattern", "[BufferOps]") {
  const std::vector<uint8_t> bytes = ByteOps::toBytes(3.14f);

  REQUIRE(bytes == std::vector<uint8_t>{ 0xC3, 0xF5, 0x48, 0x40 });
  REQUIRE(ByteOps::readFloat(bytes, 0) == Catch::Approx(3.14f));
}

TEST_CASE("BufferOps: toBytes double matches writeDouble bit pattern", "[BufferOps]") {
  const std::vector<uint8_t> bytes = ByteOps::toBytes(1.0);

  REQUIRE(bytes == std::vector<uint8_t>{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF0, 0x3F });
  REQUIRE(ByteOps::readDouble(bytes, 0) == 1.0);
}

TEST_CASE("BufferOps: double boundary values", "[BufferOps]") {
  std::vector<uint8_t> buf(32);
  ByteOps::writeDouble(buf, 0, 0.0);
  ByteOps::writeDouble(buf, 8, -0.0);
  ByteOps::writeDouble(buf, 16, std::numeric_limits<double>::max());
  ByteOps::writeDouble(buf, 24, std::numeric_limits<double>::min());

  REQUIRE(ByteOps::readDouble(buf, 0) == 0.0);
  REQUIRE(ByteOps::readDouble(buf, 8) == -0.0);
  REQUIRE(ByteOps::readDouble(buf, 16) == std::numeric_limits<double>::max());
  REQUIRE(ByteOps::readDouble(buf, 24) == std::numeric_limits<double>::min());
}

// ── write at offset ──────────────────────────────────────────────────────────

TEST_CASE("BufferOps: write at specific offset", "[BufferOps]") {
  std::vector<uint8_t> buf(10, 0x00);
  ByteOps::writeUint16(buf, 4, 0xCAFE);
  REQUIRE(ByteOps::readUint16(buf, 4) == 0xCAFE);
  // Surrounding bytes unaffected
  REQUIRE(buf[3] == 0x00);
  REQUIRE(buf[6] == 0x00);
}

// ── multiple values in sequence ──────────────────────────────────────────────

TEST_CASE("BufferOps: mixed write and read back", "[BufferOps]") {
  std::vector<uint8_t> buf(15);
  ByteOps::writeUint8(buf, 0, 0x01);
  ByteOps::writeUint16(buf, 1, 0x0203);
  ByteOps::writeUint32(buf, 3, 0x04050607);
  ByteOps::writeDouble(buf, 7, 1.0);

  REQUIRE(ByteOps::readUint8(buf, 0) == 0x01);
  REQUIRE(ByteOps::readUint16(buf, 1) == 0x0203);
  REQUIRE(ByteOps::readUint32(buf, 3) == 0x04050607);
  REQUIRE(ByteOps::readDouble(buf, 7) == 1.0);
}

// ── out-of-range ─────────────────────────────────────────────────────────────

TEST_CASE("BufferOps: read out of range throws", "[BufferOps]") {
  std::vector<uint8_t> buf = { 0x01, 0x02 };
  REQUIRE_THROWS_AS(ByteOps::readUint32(buf, 0), std::out_of_range);
  REQUIRE_THROWS_AS(ByteOps::readDouble(buf, 0), std::out_of_range);
  REQUIRE_THROWS_AS(ByteOps::readUint16(buf, 1), std::out_of_range);
}

TEST_CASE("BufferOps: write out of range throws", "[BufferOps]") {
  std::vector<uint8_t> buf(2);
  REQUIRE_THROWS_AS(ByteOps::writeUint32(buf, 0, 0), std::out_of_range);
  REQUIRE_THROWS_AS(ByteOps::writeDouble(buf, 0, 0.0), std::out_of_range);
  REQUIRE_THROWS_AS(ByteOps::writeUint16(buf, 1, 0), std::out_of_range);
}
