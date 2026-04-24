#include <catch2/catch_all.hpp>
#include <cstdint>

#include "sensorring_transport/can/CanFdDlc.hpp"

using eduart::sensorring::transport::can::bytesToDlcCode;
using eduart::sensorring::transport::can::dlcCodeToBytes;
namespace dlc = eduart::sensorring::transport::can::dlc;

TEST_CASE("CanFdDlc: identity for 0..8 bytes", "[CanFdDlc]") {
  for (std::size_t i = 0U; i <= 8U; ++i) {
    REQUIRE(bytesToDlcCode(i) == static_cast<std::uint8_t>(i));
    REQUIRE(dlcCodeToBytes(static_cast<std::uint8_t>(i)) == i);
  }
}

TEST_CASE("CanFdDlc: rounds up to next supported frame size", "[CanFdDlc]") {
  // Edges of every CAN FD bucket above 8.
  REQUIRE(bytesToDlcCode(9)  == dlc::BYTES_12);
  REQUIRE(bytesToDlcCode(12) == dlc::BYTES_12);
  REQUIRE(bytesToDlcCode(13) == dlc::BYTES_16);
  REQUIRE(bytesToDlcCode(16) == dlc::BYTES_16);
  REQUIRE(bytesToDlcCode(17) == dlc::BYTES_20);
  REQUIRE(bytesToDlcCode(20) == dlc::BYTES_20);
  REQUIRE(bytesToDlcCode(21) == dlc::BYTES_24);
  REQUIRE(bytesToDlcCode(24) == dlc::BYTES_24);
  REQUIRE(bytesToDlcCode(25) == dlc::BYTES_32);
  REQUIRE(bytesToDlcCode(32) == dlc::BYTES_32);
  REQUIRE(bytesToDlcCode(33) == dlc::BYTES_48);
  REQUIRE(bytesToDlcCode(48) == dlc::BYTES_48);
  REQUIRE(bytesToDlcCode(49) == dlc::BYTES_64);
  REQUIRE(bytesToDlcCode(64) == dlc::BYTES_64);
}

TEST_CASE("CanFdDlc: saturates past 64 bytes", "[CanFdDlc]") {
  REQUIRE(bytesToDlcCode(65)  == dlc::BYTES_64);
  REQUIRE(bytesToDlcCode(128) == dlc::BYTES_64);
  REQUIRE(bytesToDlcCode(1024) == dlc::BYTES_64);
}

TEST_CASE("CanFdDlc: dlcCodeToBytes covers all 16 codes", "[CanFdDlc]") {
  constexpr std::size_t expected[16] = {
    0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64
  };
  for (std::uint8_t code = 0U; code < 16U; ++code) {
    REQUIRE(dlcCodeToBytes(code) == expected[code]);
  }
}

TEST_CASE("CanFdDlc: dlcCodeToBytes returns 0 for codes outside 4 bits", "[CanFdDlc]") {
  REQUIRE(dlcCodeToBytes(16) == 0U);
  REQUIRE(dlcCodeToBytes(255) == 0U);
}

TEST_CASE("CanFdDlc: round-trip for payloads that fit exactly", "[CanFdDlc]") {
  for (std::size_t bytes : {0U, 1U, 4U, 8U, 12U, 16U, 20U, 24U, 32U, 48U, 64U}) {
    const auto code = bytesToDlcCode(bytes);
    REQUIRE(dlcCodeToBytes(code) == bytes);
  }
}

TEST_CASE("CanFdDlc: constexpr-evaluable", "[CanFdDlc]") {
  // These would fail to compile if the helpers were not actually constexpr.
  static_assert(bytesToDlcCode(0)  == dlc::BYTES_0,  "");
  static_assert(bytesToDlcCode(8)  == dlc::BYTES_8,  "");
  static_assert(bytesToDlcCode(9)  == dlc::BYTES_12, "");
  static_assert(bytesToDlcCode(64) == dlc::BYTES_64, "");
  static_assert(dlcCodeToBytes(dlc::BYTES_12) == 12U, "");
  static_assert(dlcCodeToBytes(dlc::BYTES_64) == 64U, "");
}
