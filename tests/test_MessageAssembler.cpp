#include <catch2/catch_all.hpp>
#include <cstdint>
#include <numeric>
#include <vector>

#include "sensorring_transport/MessageAssembler.hpp"
#include "sensorring_transport/Protocol.hpp"
using namespace eduart::sensorring::transport::protocol;

using eduart::sensorring::transport::Direction;
using eduart::sensorring::transport::MessageAssembler;
using eduart::sensorring::transport::TransportFrame;

namespace {

std::vector<std::uint8_t> makeData(std::size_t n, std::uint8_t start = 0) {
  std::vector<std::uint8_t> d(n);
  for (std::size_t i = 0; i < n; ++i)
    d[i] = static_cast<std::uint8_t>((start + i) & 0xFF);
  return d;
}

} // namespace

TEST_CASE("Assembly: empty payload produces single frame", "[Assembly]") {
  MessageAssembler asm_(61);
  auto frames = asm_.assemble(Direction::Output, 0x01, devbyte::BOARD, 0x03, {});

  REQUIRE(frames.size() == 1);
  REQUIRE(frames[0].fragmentType == fragment::SINGLE);
  REQUIRE(frames[0].fragmentCount == 0);
  REQUIRE(frames[0].data.empty());
  REQUIRE(frames[0].direction == Direction::Output);
  REQUIRE(frames[0].boardAddress == 0x01);
  REQUIRE(frames[0].deviceId == devbyte::BOARD);
  REQUIRE(frames[0].command == 0x03);
}

TEST_CASE("Assembly: small payload fits single frame", "[Assembly]") {
  MessageAssembler asm_(61);
  auto data   = makeData(30);
  auto frames = asm_.assemble(Direction::Input, 0x05, devbyte::VL53L8CX, 0x01, data);

  REQUIRE(frames.size() == 1);
  REQUIRE(frames[0].fragmentType == fragment::SINGLE);
  REQUIRE(frames[0].fragmentCount == 30);
  REQUIRE(frames[0].data == data);
}

TEST_CASE("Assembly: exactly max payload fits single frame", "[Assembly]") {
  MessageAssembler asm_(61);
  auto data   = makeData(61);
  auto frames = asm_.assemble(Direction::Output, 0x00, devbyte::HTPA32, 0x01, data);

  REQUIRE(frames.size() == 1);
  REQUIRE(frames[0].fragmentType == fragment::SINGLE);
  REQUIRE(frames[0].fragmentCount == 61);
  REQUIRE(frames[0].data == data);
}

#ifndef SENSORRING_TRANSPORT_NO_MULTIFRAME

TEST_CASE("Assembly: max+1 produces two fragments", "[Assembly]") {
  MessageAssembler asm_(61);
  auto data   = makeData(62);
  auto frames = asm_.assemble(Direction::Output, 0x01, devbyte::VL53L8CX, 0x04, data);

  REQUIRE(frames.size() == 2);

  // First frame: countWidth=1, data = [0x02 (totalFrames)] ++ first 60 bytes of payload
  REQUIRE(frames[0].fragmentType == fragment::FIRST);
  REQUIRE(frames[0].fragmentCount == 1); // count-width in bytes
  REQUIRE(frames[0].data.size() == 61);  // 1 count byte + 60 payload bytes
  REQUIRE(frames[0].data[0] == 2);       // total fragment count
  REQUIRE(std::vector<std::uint8_t>(frames[0].data.begin() + 1, frames[0].data.end()) == makeData(60, 0));

  // Last frame: remaining 2 bytes of payload
  REQUIRE(frames[1].fragmentType == fragment::LAST);
  REQUIRE(frames[1].fragmentCount == 2); // valid bytes in last
  REQUIRE(frames[1].data.size() == 2);
  REQUIRE(frames[1].data == makeData(2, 60));
}

TEST_CASE("Assembly: three fragments with middle", "[Assembly]") {
  MessageAssembler asm_(61);
  auto data   = makeData(142); // needs 3 frames: first has 60 payload, middle 61, last 21
  auto frames = asm_.assemble(Direction::Input, 0x02, devbyte::VL53L8CX, 0x00, data);

  REQUIRE(frames.size() == 3);

  REQUIRE(frames[0].fragmentType == fragment::FIRST);
  REQUIRE(frames[0].fragmentCount == 1); // count-width
  REQUIRE(frames[0].data.size() == 61);  // 1 count byte + 60 payload
  REQUIRE(frames[0].data[0] == 3);       // total fragments

  REQUIRE(frames[1].fragmentType == fragment::MIDDLE);
  REQUIRE(frames[1].fragmentCount == 0);
  REQUIRE(frames[1].data.size() == 61);

  REQUIRE(frames[2].fragmentType == fragment::LAST);
  REQUIRE(frames[2].fragmentCount == 21);
  REQUIRE(frames[2].data.size() == 21);

  // Verify data integrity: skip count prefix byte, then concatenate all data
  std::vector<std::uint8_t> reassembled;
  reassembled.insert(reassembled.end(), frames[0].data.begin() + 1, frames[0].data.end());
  for (std::size_t i = 1; i < frames.size(); ++i) {
    reassembled.insert(reassembled.end(), frames[i].data.begin(), frames[i].data.end());
  }
  REQUIRE(reassembled == data);
}

TEST_CASE("Assembly: all frames carry correct routing info", "[Assembly]") {
  MessageAssembler asm_(61);
  auto data   = makeData(130);
  auto frames = asm_.assemble(Direction::Output, 0x0A, devbyte::WS2812B, 0x02, data);

  for (const auto& f : frames) {
    REQUIRE(f.direction == Direction::Output);
    REQUIRE(f.boardAddress == 0x0A);
    REQUIRE(f.deviceId == devbyte::WS2812B);
    REQUIRE(f.command == 0x02);
  }
}

TEST_CASE("Assembly: different maxPayloadPerFrame", "[Assembly]") {
  MessageAssembler asm_(20);
  auto data   = makeData(50);
  auto frames = asm_.assemble(Direction::Input, 0x01, devbyte::VL53L8CX, 0x01, data);

  // 50 bytes, first frame has 19 payload (20 - 1 count byte), then 20 + 11
  REQUIRE(frames.size() == 3);

  REQUIRE(frames[0].fragmentType == fragment::FIRST);
  REQUIRE(frames[0].fragmentCount == 1); // count-width
  REQUIRE(frames[0].data.size() == 20);  // 1 count byte + 19 payload
  REQUIRE(frames[0].data[0] == 3);       // total fragments

  REQUIRE(frames[1].fragmentType == fragment::MIDDLE);
  REQUIRE(frames[1].data.size() == 20);

  REQUIRE(frames[2].fragmentType == fragment::LAST);
  REQUIRE(frames[2].fragmentCount == 11);
  REQUIRE(frames[2].data.size() == 11);
}

TEST_CASE("Assembly: large transfer (EEPROM-sized, 7289 bytes)", "[Assembly]") {
  MessageAssembler asm_(61);
  auto data   = makeData(7289);
  auto frames = asm_.assemble(Direction::Input, 0x01, devbyte::HTPA32, 0x06, data);

  // FIRST carries 60 B payload (61 - 1 count byte), 120 fits in 1 byte
  REQUIRE(frames.size() == 120);

  REQUIRE(frames[0].fragmentType == fragment::FIRST);
  REQUIRE(frames[0].fragmentCount == 1);
  REQUIRE(frames[0].data[0] == 120);
  REQUIRE(frames[0].data.size() == 61);

  for (std::size_t i = 1; i < frames.size() - 1; ++i) {
    REQUIRE(frames[i].fragmentType == fragment::MIDDLE);
    REQUIRE(frames[i].data.size() == 61);
  }

  REQUIRE(frames.back().fragmentType == fragment::LAST);

  // Data integrity: skip count prefix, then concatenate
  std::vector<std::uint8_t> reassembled;
  reassembled.insert(reassembled.end(), frames[0].data.begin() + 1, frames[0].data.end());
  for (std::size_t i = 1; i < frames.size(); ++i) {
    auto validLen = (i == frames.size() - 1) ? frames[i].fragmentCount : frames[i].data.size();
    reassembled.insert(reassembled.end(), frames[i].data.begin(), frames[i].data.begin() + validLen);
  }
  REQUIRE(reassembled.size() == 7289);
  REQUIRE(reassembled == data);
}

TEST_CASE("Assembly: >255 fragments uses 2-byte count width", "[Assembly]") {
  MessageAssembler asm_(61);
  // Need >255 frames. With 2-byte width, FIRST has 59 payload bytes.
  // 59 + 255*61 = 59 + 15555 = 15614 bytes fits in 256 frames (1-byte count).
  // So 15615 bytes needs 257 frames => 2-byte width.
  auto data   = std::vector<std::uint8_t>(15615, 0xAB);
  auto frames = asm_.assemble(Direction::Input, 0x01, devbyte::HTPA32, 0x06, data);

  REQUIRE(frames[0].fragmentType == fragment::FIRST);
  REQUIRE(frames[0].fragmentCount == 2);
  std::uint16_t totalFromData = (static_cast<std::uint16_t>(frames[0].data[0]) << 8) | frames[0].data[1];
  REQUIRE(totalFromData == frames.size());
  REQUIRE(frames[0].data.size() == 61);
}

#else // SENSORRING_TRANSPORT_NO_MULTIFRAME

TEST_CASE("Assembly: oversized payload returns no frames in single-frame-only build", "[Assembly]") {
  MessageAssembler asm_(61);
  auto data   = makeData(62);
  auto frames = asm_.assemble(Direction::Output, 0x01, devbyte::VL53L8CX, 0x04, data);

  REQUIRE(frames.empty());
}

#endif // SENSORRING_TRANSPORT_NO_MULTIFRAME
