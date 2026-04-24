/**
 * @file   CanFdDlc.hpp
 * @brief  CAN FD Data Length Code (DLC) helpers (ISO 11898-1).
 *
 * The 4-bit DLC field in a CAN FD frame encodes payload sizes using a
 * non-linear mapping: codes 0..8 represent their own byte count, codes 9..15
 * map to 12, 16, 20, 24, 32, 48 and 64 bytes respectively. Controller HALs
 * typically expose the same 4-bit value behind vendor-specific macros; this
 * header provides a transport-agnostic API that consumers can translate into
 * their local wire-format representation with a single cast.
 */

#pragma once

#include <cstddef>
#include <cstdint>

namespace eduart {
namespace sensorring {
namespace transport {
namespace can {

/**
 * @brief Standard CAN FD DLC codes (ISO 11898-1).
 *
 * Values 0..8 encode their own byte count; values 9..15 encode 12, 16, 20,
 * 24, 32, 48 and 64 bytes respectively.
 */
namespace dlc {
constexpr std::uint8_t BYTES_0  = 0U;
constexpr std::uint8_t BYTES_1  = 1U;
constexpr std::uint8_t BYTES_2  = 2U;
constexpr std::uint8_t BYTES_3  = 3U;
constexpr std::uint8_t BYTES_4  = 4U;
constexpr std::uint8_t BYTES_5  = 5U;
constexpr std::uint8_t BYTES_6  = 6U;
constexpr std::uint8_t BYTES_7  = 7U;
constexpr std::uint8_t BYTES_8  = 8U;
constexpr std::uint8_t BYTES_12 = 9U;
constexpr std::uint8_t BYTES_16 = 10U;
constexpr std::uint8_t BYTES_20 = 11U;
constexpr std::uint8_t BYTES_24 = 12U;
constexpr std::uint8_t BYTES_32 = 13U;
constexpr std::uint8_t BYTES_48 = 14U;
constexpr std::uint8_t BYTES_64 = 15U;
} // namespace dlc

/**
 * @brief Smallest CAN FD DLC code that can carry @p bytes payload octets.
 *
 * Inputs larger than 64 are clamped to @ref dlc::BYTES_64. The returned value
 * is a 4-bit code in the range [0, 15] and can be fed directly into a
 * controller HAL field that expects the raw CAN FD DLC encoding.
 *
 * @param bytes Payload size in bytes.
 * @return CAN FD DLC code rounded up to the next supported frame size.
 */
constexpr std::uint8_t bytesToDlcCode(std::size_t bytes) {
  if (bytes <= 8U)  return static_cast<std::uint8_t>(bytes); // codes 0..8 == byte count
  if (bytes <= 12U) return dlc::BYTES_12;
  if (bytes <= 16U) return dlc::BYTES_16;
  if (bytes <= 20U) return dlc::BYTES_20;
  if (bytes <= 24U) return dlc::BYTES_24;
  if (bytes <= 32U) return dlc::BYTES_32;
  if (bytes <= 48U) return dlc::BYTES_48;
  return dlc::BYTES_64;
}

/**
 * @brief Number of frame bytes encoded by a CAN FD DLC code.
 *
 * Codes outside the 4-bit range [0, 15] return 0.
 *
 * @param dlcCode CAN FD DLC code (0..15).
 * @return Payload size in bytes.
 */
constexpr std::size_t dlcCodeToBytes(std::uint8_t dlcCode) {
  switch (dlcCode) {
    case dlc::BYTES_0:  return 0U;
    case dlc::BYTES_1:  return 1U;
    case dlc::BYTES_2:  return 2U;
    case dlc::BYTES_3:  return 3U;
    case dlc::BYTES_4:  return 4U;
    case dlc::BYTES_5:  return 5U;
    case dlc::BYTES_6:  return 6U;
    case dlc::BYTES_7:  return 7U;
    case dlc::BYTES_8:  return 8U;
    case dlc::BYTES_12: return 12U;
    case dlc::BYTES_16: return 16U;
    case dlc::BYTES_20: return 20U;
    case dlc::BYTES_24: return 24U;
    case dlc::BYTES_32: return 32U;
    case dlc::BYTES_48: return 48U;
    case dlc::BYTES_64: return 64U;
    default:            return 0U;
  }
}

} // namespace can
} // namespace transport
} // namespace sensorring
} // namespace eduart
