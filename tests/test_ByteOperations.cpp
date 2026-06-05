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

// ── variable-width uint32 (byteCount overloads) ──────────────────────────────

TEST_CASE("BufferOps: write/read 3-byte uint32", "[BufferOps]") {
  std::vector<uint8_t> buf(3, 0x00);
  ByteOps::writeUint32(buf, 0, 0x123456, 3);
  REQUIRE(buf[0] == 0x56);
  REQUIRE(buf[1] == 0x34);
  REQUIRE(buf[2] == 0x12);
  REQUIRE(ByteOps::readUint32(buf, 0, 3) == 0x123456);
}

TEST_CASE("BufferOps: 3-byte uint32 max value", "[BufferOps]") {
  std::vector<uint8_t> buf(3);
  ByteOps::writeUint32(buf, 0, 0xFFFFFF, 3);
  REQUIRE(ByteOps::readUint32(buf, 0, 3) == 0xFFFFFF);
}

TEST_CASE("BufferOps: 1-byte uint32 overload", "[BufferOps]") {
  std::vector<uint8_t> buf(1);
  ByteOps::writeUint32(buf, 0, 0xAB, 1);
  REQUIRE(ByteOps::readUint32(buf, 0, 1) == 0xAB);
}

TEST_CASE("BufferOps: 2-byte uint32 overload", "[BufferOps]") {
  std::vector<uint8_t> buf(2);
  ByteOps::writeUint32(buf, 0, 0xBEEF, 2);
  REQUIRE(ByteOps::readUint32(buf, 0, 2) == 0xBEEF);
}

TEST_CASE("BufferOps: 4-byte uint32 overload matches default", "[BufferOps]") {
  std::vector<uint8_t> buf(4);
  ByteOps::writeUint32(buf, 0, 0xDEADBEEF, 4);
  REQUIRE(ByteOps::readUint32(buf, 0, 4) == 0xDEADBEEF);
}

TEST_CASE("BufferOps: variable-width uint32 invalid byteCount throws", "[BufferOps]") {
  std::vector<uint8_t> buf(8);
  REQUIRE_THROWS_AS(ByteOps::readUint32(buf, 0, 0), std::invalid_argument);
  REQUIRE_THROWS_AS(ByteOps::readUint32(buf, 0, 5), std::invalid_argument);
  REQUIRE_THROWS_AS(ByteOps::writeUint32(buf, 0, 0, 0), std::invalid_argument);
  REQUIRE_THROWS_AS(ByteOps::writeUint32(buf, 0, 0, 5), std::invalid_argument);
}

TEST_CASE("BufferOps: 3-byte uint32 read out of range throws", "[BufferOps]") {
  std::vector<uint8_t> buf(2);
  REQUIRE_THROWS_AS(ByteOps::readUint32(buf, 0, 3), std::out_of_range);
}
