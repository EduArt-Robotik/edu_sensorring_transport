#include <catch2/catch_all.hpp>
#include <cstdint>
#include <vector>

#include "sensorring_transport/MessageReassembler.hpp"
#include "sensorring_transport/Protocol.hpp"

using namespace eduart::sensorring::transport::protocol;

using eduart::sensorring::transport::Direction;
using eduart::sensorring::transport::MessageReassembler;
using eduart::sensorring::transport::TransportFrame;

namespace {

struct DeliveredMessage {
  Direction direction;
  std::uint8_t boardAddress;
  std::uint8_t deviceId;
  std::uint8_t command;
  std::vector<std::uint8_t> data;
};

class TestFixture {
public:
  TestFixture()
      : reassembler([this](const TransportFrame& frame) {
        delivered.push_back({ frame.direction, frame.boardAddress, frame.deviceId, frame.command, frame.data });
      }) {}

  void process(Direction dir, std::uint8_t board, std::uint8_t device, std::uint8_t cmd, std::uint8_t fragType, std::uint8_t fragCount, const std::vector<std::uint8_t>& data) {
    TransportFrame frame;
    frame.direction     = dir;
    frame.boardAddress  = board;
    frame.deviceId      = device;
    frame.command       = cmd;
    frame.fragmentType  = fragType;
    frame.fragmentCount = fragCount;
    frame.data          = data;
    reassembler.processFrame(frame);
  }

  MessageReassembler reassembler;
  std::vector<DeliveredMessage> delivered;
};

std::vector<std::uint8_t> makeData(std::size_t n, std::uint8_t start = 0) {
  std::vector<std::uint8_t> d(n);
  for (std::size_t i = 0; i < n; ++i)
    d[i] = static_cast<std::uint8_t>((start + i) & 0xFF);
  return d;
}

} // namespace

TEST_CASE("Reassembly: single frame delivers immediately", "[Reassembly]") {
  TestFixture f;
  auto data = makeData(5);
  f.process(Direction::Input, 1, devbyte::BOARD, 0x01, fragment::SINGLE, 5, data);

  REQUIRE(f.delivered.size() == 1);
  REQUIRE(f.delivered[0].boardAddress == 1);
  REQUIRE(f.delivered[0].deviceId == devbyte::BOARD);
  REQUIRE(f.delivered[0].command == 0x01);
  REQUIRE(f.delivered[0].data == data);
}

TEST_CASE("Reassembly: single frame strips padding", "[Reassembly]") {
  TestFixture f;
  auto data = makeData(5);
  data.resize(9, 0x00); // CAN FD padding
  f.process(Direction::Input, 1, devbyte::VL53L8CX, 0x00, fragment::SINGLE, 5, data);

  REQUIRE(f.delivered.size() == 1);
  REQUIRE(f.delivered[0].data.size() == 5);
  REQUIRE(f.delivered[0].data == makeData(5));
}

TEST_CASE("Reassembly: multi-frame 3 fragments", "[Reassembly]") {
  TestFixture f;
  // FIRST: countWidth=1, data = [0x03 (totalFragments)] ++ 61 payload bytes
  auto first_data = std::vector<std::uint8_t>{ 3 }; // total fragment count
  auto payload1   = makeData(60, 0);                // 60 bytes of actual data (61 - 1 count byte)
  first_data.insert(first_data.end(), payload1.begin(), payload1.end());

  auto data2 = makeData(61, 60);
  auto data3 = makeData(21, 121);

  f.process(Direction::Input, 2, devbyte::VL53L8CX, 0x00, fragment::FIRST, 1, first_data);
  REQUIRE(f.delivered.empty());

  f.process(Direction::Input, 2, devbyte::VL53L8CX, 0x00, fragment::MIDDLE, 0, data2);
  REQUIRE(f.delivered.empty());

  f.process(Direction::Input, 2, devbyte::VL53L8CX, 0x00, fragment::LAST, 21, data3);
  REQUIRE(f.delivered.size() == 1);
  REQUIRE(f.delivered[0].command == 0x00);
  REQUIRE(f.delivered[0].data.size() == 60 + 61 + 21);
  REQUIRE(f.delivered[0].data == makeData(60 + 61 + 21, 0));
}

TEST_CASE("Reassembly: last frame strips padding", "[Reassembly]") {
  TestFixture f;
  // FIRST: countWidth=1, data = [0x02] ++ 60 payload bytes
  auto first_data = std::vector<std::uint8_t>{ 2 };
  auto payload1   = makeData(60, 0);
  first_data.insert(first_data.end(), payload1.begin(), payload1.end());

  auto data2 = makeData(10, 60);
  data2.resize(13, 0x00); // CAN FD padding

  f.process(Direction::Input, 3, devbyte::HTPA32, 0x05, fragment::FIRST, 1, first_data);
  f.process(Direction::Input, 3, devbyte::HTPA32, 0x05, fragment::LAST, 10, data2);

  REQUIRE(f.delivered.size() == 1);
  REQUIRE(f.delivered[0].data.size() == 60 + 10);
}

TEST_CASE("Reassembly: command mismatch discards transfer", "[Reassembly]") {
  TestFixture f;
  auto first_data = std::vector<std::uint8_t>{ 2 };
  auto p          = makeData(60);
  first_data.insert(first_data.end(), p.begin(), p.end());

  f.process(Direction::Input, 4, devbyte::VL53L8CX, 0x00, fragment::FIRST, 1, first_data);
  f.process(
      Direction::Input, 4, devbyte::VL53L8CX, 0x01, // wrong!
      fragment::LAST, 10, makeData(10));

  REQUIRE(f.delivered.empty());
}

TEST_CASE("Reassembly: restart on new first-fragment", "[Reassembly]") {
  TestFixture f;
  // First incomplete transfer
  auto first1 = std::vector<std::uint8_t>{ 3 };
  auto p1     = makeData(60);
  first1.insert(first1.end(), p1.begin(), p1.end());
  f.process(Direction::Input, 5, devbyte::VL53L8CX, 0x00, fragment::FIRST, 1, first1);

  // Second transfer replaces the first
  auto first2 = std::vector<std::uint8_t>{ 2 };
  auto p2     = makeData(60, 100);
  first2.insert(first2.end(), p2.begin(), p2.end());
  f.process(Direction::Input, 5, devbyte::VL53L8CX, 0x01, fragment::FIRST, 1, first2);
  f.process(Direction::Input, 5, devbyte::VL53L8CX, 0x01, fragment::LAST, 10, makeData(10, 160));

  REQUIRE(f.delivered.size() == 1);
  REQUIRE(f.delivered[0].command == 0x01);
  REQUIRE(f.delivered[0].data.size() == 60 + 10);
}

TEST_CASE("Reassembly: overrun discards transfer", "[Reassembly]") {
  TestFixture f;
  auto first = std::vector<std::uint8_t>{ 2 };
  auto p     = makeData(60);
  first.insert(first.end(), p.begin(), p.end());

  f.process(Direction::Input, 6, devbyte::VL53L8CX, 0x00, fragment::FIRST, 1, first);
  f.process(Direction::Input, 6, devbyte::VL53L8CX, 0x00, fragment::MIDDLE, 0, makeData(61));
  f.process(Direction::Input, 6, devbyte::VL53L8CX, 0x00, fragment::MIDDLE, 0, makeData(61)); // overrun

  REQUIRE(f.delivered.empty());
}

TEST_CASE("Reassembly: stray middle/last fragments ignored", "[Reassembly]") {
  TestFixture f;
  f.process(Direction::Input, 7, devbyte::VL53L8CX, 0x00, fragment::MIDDLE, 0, makeData(61));
  f.process(Direction::Input, 7, devbyte::VL53L8CX, 0x00, fragment::LAST, 10, makeData(10));

  REQUIRE(f.delivered.empty());
}

TEST_CASE("Reassembly: buffer isolation between board/device pairs", "[Reassembly]") {
  TestFixture f;

  // Helper to build FIRST frame data: [count] ++ payload
  auto mkFirst = [](std::uint8_t totalFragments, std::size_t payloadLen, std::uint8_t start) {
    std::vector<std::uint8_t> d = { totalFragments };
    auto p                      = makeData(payloadLen, start);
    d.insert(d.end(), p.begin(), p.end());
    return d;
  };

  // Start three concurrent transfers (each 2 fragments, 60 payload in FIRST)
  f.process(Direction::Input, 1, devbyte::VL53L8CX, 0x00, fragment::FIRST, 1, mkFirst(2, 60, 0));
  f.process(Direction::Input, 2, devbyte::VL53L8CX, 0x01, fragment::FIRST, 1, mkFirst(2, 60, 10));
  f.process(Direction::Input, 1, devbyte::HTPA32, 0x02, fragment::FIRST, 1, mkFirst(2, 60, 20));

  // Complete them in different order
  f.process(Direction::Input, 2, devbyte::VL53L8CX, 0x01, fragment::LAST, 5, makeData(5, 70));
  f.process(Direction::Input, 1, devbyte::HTPA32, 0x02, fragment::LAST, 8, makeData(8, 80));
  f.process(Direction::Input, 1, devbyte::VL53L8CX, 0x00, fragment::LAST, 3, makeData(3, 60));

  REQUIRE(f.delivered.size() == 3);

  REQUIRE(f.delivered[0].boardAddress == 2);
  REQUIRE(f.delivered[0].command == 0x01);
  REQUIRE(f.delivered[0].data.size() == 65); // 60 + 5

  REQUIRE(f.delivered[1].boardAddress == 1);
  REQUIRE(f.delivered[1].deviceId == devbyte::HTPA32);
  REQUIRE(f.delivered[1].data.size() == 68); // 60 + 8

  REQUIRE(f.delivered[2].boardAddress == 1);
  REQUIRE(f.delivered[2].deviceId == devbyte::VL53L8CX);
  REQUIRE(f.delivered[2].data.size() == 63); // 60 + 3
}

TEST_CASE("Reassembly: fragment count mismatch on last", "[Reassembly]") {
  TestFixture f;
  auto first = std::vector<std::uint8_t>{ 3 }; // expects 3 total fragments
  auto p     = makeData(60);
  first.insert(first.end(), p.begin(), p.end());

  f.process(Direction::Input, 8, devbyte::VL53L8CX, 0x00, fragment::FIRST, 1, first);
  f.process(Direction::Input, 8, devbyte::VL53L8CX, 0x00, fragment::LAST, 10, makeData(10)); // only fragment 2, expected 3

  REQUIRE(f.delivered.empty());
}

TEST_CASE("Reassembly: invalid FIRST with countWidth=0 is ignored", "[Reassembly]") {
  TestFixture f;
  f.process(Direction::Input, 1, devbyte::BOARD, 0x01, fragment::FIRST, 0, makeData(61));
  REQUIRE(f.delivered.empty());
}
