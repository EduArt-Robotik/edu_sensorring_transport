/**
 * @file   Protocol.hpp
 * @brief  Sensor Ring transport protocol definitions and TransportFrame structure.
 *
 * This header is shared between the host-side C++ library and the microcontroller firmware.
 */

#pragma once

#include <cstdint>
#include <vector>

namespace eduart {
namespace sensorring {
namespace transport {
namespace protocol {

/**
 * @defgroup ProtocolDefines Protocol Constants
 * @{
 */

/// Device byte identifiers (byte 1 of protocol header)
namespace devbyte {
constexpr std::uint8_t BOARD    = 0x00;
constexpr std::uint8_t VL53L8CX = 0x01;
constexpr std::uint8_t HTPA32   = 0x02;
constexpr std::uint8_t WS2812B  = 0x03;
constexpr std::uint8_t TMF8829  = 0x04;
} // namespace devbyte

/// Fragment type encoding (bits 7-6 of fragment byte)
namespace fragment {
constexpr std::uint8_t MIDDLE = 0x00; ///< 0b00 -- continuation
constexpr std::uint8_t FIRST  = 0x40; ///< 0b01 -- starts multi-frame transfer
constexpr std::uint8_t LAST   = 0x80; ///< 0b10 -- ends multi-frame transfer
constexpr std::uint8_t SINGLE = 0xC0; ///< 0b11 -- complete single-frame message

constexpr std::uint8_t TYPE_MASK  = 0xC0;
constexpr std::uint8_t COUNT_MASK = 0x3F;
} // namespace fragment

/// Protocol header size (fragment + device + command)
constexpr std::uint8_t HEADER_SIZE = 3;

/** @} */

/**
 * @defgroup CommandDefines Sensor Ring Commands
 * @{
 */

/// Sensor board commands
namespace sensor_board {
constexpr std::uint8_t ACTIVE_DEVICE_REQUEST  = 0x01;
constexpr std::uint8_t ACTIVE_DEVICE_RESPONSE = 0x02;
constexpr std::uint8_t RESET                  = 0x03;
constexpr std::uint8_t BOOTLOADER_START       = 0x04;
constexpr std::uint8_t BOOTLOADER_START_ACK   = 0x05;
constexpr std::uint8_t BOOTLOADER_REQUEST     = 0x06;
constexpr std::uint8_t BOOTLOADER_RESPONSE    = 0x07;
} // namespace sensor_board

/// VL53L8CX commands
namespace vl53l8cx {
constexpr std::uint8_t MEASUREMENT_REQUEST               = 0x01;
constexpr std::uint8_t MEASUREMENT_RESPONSE              = 0x02;
constexpr std::uint8_t MEASUREMENT_TRANSMISSION_REQUEST  = 0x03;
constexpr std::uint8_t MEASUREMENT_TRANSMISSION_RESPONSE = 0x04;
} // namespace vl53l8cx

/// HTPA32 commands
namespace htpa32 {
constexpr std::uint8_t MEASUREMENT_REQUEST               = 0x01;
constexpr std::uint8_t MEASUREMENT_RESPONSE              = 0x02;
constexpr std::uint8_t MEASUREMENT_TRANSMISSION_REQUEST  = 0x03;
constexpr std::uint8_t MEASUREMENT_TRANSMISSION_RESPONSE = 0x04;
constexpr std::uint8_t EEPROM_TRANSMISSION_REQUEST       = 0x05;
constexpr std::uint8_t EEPROM_TRANSMISSION_RESPONSE      = 0x06;
} // namespace htpa32

/// WS2812B commands
namespace ws2812b {
constexpr std::uint8_t SYNCHRONIZE           = 0x01;
constexpr std::uint8_t SET_ORIENTATION       = 0x02;
constexpr std::uint8_t SET_LED_MODE          = 0x03;
constexpr std::uint8_t SET_LEDS_INDIVIDUALLY = 0x04;

namespace mode {
constexpr std::uint8_t BEAT             = 1;  // Heartbeat of IOTShield for synchronizing attached devices
constexpr std::uint8_t LIGHTS_OFF       = 2;  // Lights off
constexpr std::uint8_t DIM_LIGHT        = 3;  // Dimmed headlight
constexpr std::uint8_t HIGH_BEAM        = 4;  // high beam headlight
constexpr std::uint8_t FLASH_ALL        = 5;  // Flash lights
constexpr std::uint8_t FLASH_LEFT       = 6;  // Flash lights to the left
constexpr std::uint8_t FLASH_RIGHT      = 7;  // Flash lights to the right
constexpr std::uint8_t PULSATION        = 8;  // Pulsation
constexpr std::uint8_t ROTATION         = 9;  // Rotating light
constexpr std::uint8_t RUNNING          = 10; // Running light
constexpr std::uint8_t DISTANCE_MAP     = 11; // 0xb  // Map distance to color
constexpr std::uint8_t FIXED_COLOR      = 12; // 0xc  // Set LEDs to a fixed color
constexpr std::uint8_t PULSATION_COLOR  = 13; // 0xd  // Pulsate with a given color
constexpr std::uint8_t INDIVIDUAL_COLOR = 14; // 0xe  // Set every led individually
} // namespace mode
} // namespace ws2812b

/// TMF8829 commands
namespace tmf8829 {
constexpr std::uint8_t MEASUREMENT_REQUEST               = 0x01;
constexpr std::uint8_t MEASUREMENT_RESPONSE              = 0x02;
constexpr std::uint8_t MEASUREMENT_TRANSMISSION_REQUEST  = 0x03;
constexpr std::uint8_t MEASUREMENT_TRANSMISSION_RESPONSE = 0x04;
constexpr std::uint8_t PARAMETER_SET_RESOLUTION          = 0x20;
} // namespace tmf8829
/** @} */

} // namespace protocol

/**
 * @enum Direction
 * @brief Communication direction from the board's perspective.
 */
enum class Direction : std::uint8_t {
  Input     = 0, ///< Host -> board (board receives)
  Broadcast = 0, ///< Alias for Input; used with broadcast board address
  Output    = 1  ///< Board -> host (board sends)
};

/**
 * @struct TransportFrame
 * @brief Transport-agnostic protocol frame.
 *
 * Contains all routing information plus the payload for one protocol frame.
 * The assembly/reassembly layers work exclusively with this structure.
 * Transport codecs (CAN, USB, ...) map between TransportFrame and their wire format.
 */
struct TransportFrame {
  Direction direction;            ///< Communication direction
  std::uint8_t boardAddress;      ///< 0x00 = broadcast, 0x01-0x7E = individual boards
  std::uint8_t deviceId;          ///< Device byte (devbyte::*)
  std::uint8_t command;           ///< Command byte
  std::uint8_t fragmentType;      ///< Fragment type (fragment::*)
  std::uint8_t fragmentCount;     ///< Bits 5-0: data length (single/last), or count-width in bytes (first)
  std::vector<std::uint8_t> data; ///< Payload data bytes

  /// Broadcast address constant.
  static constexpr std::uint8_t BROADCAST = 0x00;

  /// Wildcard: matches any board address during dispatch.
  static constexpr std::uint8_t ANY_BOARD = 0xFF;
};

} // namespace transport
} // namespace sensorring
} // namespace eduart