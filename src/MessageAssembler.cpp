#include "sensorring_transport/MessageAssembler.hpp"

#include <algorithm>

using namespace eduart::sensorring::transport::protocol;

namespace eduart {
namespace sensorring {
namespace transport {

MessageAssembler::MessageAssembler(std::uint16_t maxPayloadPerFrame)
    : _maxPayloadPerFrame(maxPayloadPerFrame) {
}

std::vector<TransportFrame> MessageAssembler::assemble(Direction direction, std::uint8_t boardAddress, std::uint8_t deviceId, std::uint8_t command, const std::vector<std::uint8_t>& data) const {
  std::vector<TransportFrame> frames;

  if (data.size() <= _maxPayloadPerFrame) {
    // Single frame
    TransportFrame frame;
    frame.direction     = direction;
    frame.boardAddress  = boardAddress;
    frame.deviceId      = deviceId;
    frame.command       = command;
    frame.fragmentType  = fragment::SINGLE;
    frame.fragmentCount = static_cast<std::uint8_t>(data.size() & fragment::COUNT_MASK);
    frame.data          = data;
    frames.push_back(std::move(frame));
    return frames;
  }

#ifdef SENSORRING_TRANSPORT_NO_MULTIFRAME
  // Multi-frame support is compiled out. Oversized payloads are a programming
  // error on this build; signal failure to the caller by returning no frames.
  (void)direction;
  (void)boardAddress;
  (void)deviceId;
  (void)command;
  return frames;
#else
  // Multi-frame: calculate total fragments needed.
  // The FIRST frame's data starts with the total-fragment-count encoded in
  // the minimum number of bytes (big-endian).  The 6-bit fragmentCount field
  // of the FIRST frame carries that byte width (1, 2, or 3).
  //
  // Step 1: Determine the count-width that the FIRST frame will carry.
  //         We need to iterate because the width itself reduces the FIRST
  //         frame's available payload, which can change the total.
  std::uint8_t countWidth = 1;
  std::size_t totalFrames = 0;

  for (int pass = 0; pass < 3; ++pass) {
    std::size_t firstPayload          = _maxPayloadPerFrame - countWidth;
    std::size_t remaining_after_first = (data.size() > firstPayload) ? data.size() - firstPayload : 0;
    totalFrames                       = 1; // the FIRST frame
    if (remaining_after_first > 0) {
      totalFrames += (remaining_after_first + _maxPayloadPerFrame - 1) / _maxPayloadPerFrame;
    }

    std::uint8_t neededWidth;
    if (totalFrames <= 0xFF) {
      neededWidth = 1;
    } else if (totalFrames <= 0xFFFF) {
      neededWidth = 2;
    } else {
      neededWidth = 3;
    }

    if (neededWidth == countWidth)
      break;
    countWidth = neededWidth;
  }

  // Step 2: Build the FIRST frame with the count prefix.
  std::size_t offset = 0;
  {
    TransportFrame frame;
    frame.direction     = direction;
    frame.boardAddress  = boardAddress;
    frame.deviceId      = deviceId;
    frame.command       = command;
    frame.fragmentType  = fragment::FIRST;
    frame.fragmentCount = countWidth;

    // Encode totalFrames as big-endian in countWidth bytes
    for (int i = countWidth - 1; i >= 0; --i) {
      frame.data.push_back(static_cast<std::uint8_t>((totalFrames >> (i * 8)) & 0xFF));
    }

    // Fill remaining space with payload data
    std::size_t firstDataLen = std::min(data.size(), static_cast<std::size_t>(_maxPayloadPerFrame - countWidth));
    frame.data.insert(frame.data.end(), data.begin(), data.begin() + firstDataLen);
    offset += firstDataLen;

    frames.push_back(std::move(frame));
  }

  // Step 3: Build MIDDLE and LAST frames (unchanged payload layout).
  std::size_t remaining = data.size() - offset;

  while (remaining > 0) {
    std::size_t chunkSize = std::min(remaining, static_cast<std::size_t>(_maxPayloadPerFrame));

    TransportFrame frame;
    frame.direction    = direction;
    frame.boardAddress = boardAddress;
    frame.deviceId     = deviceId;
    frame.command      = command;
    frame.data.assign(data.begin() + offset, data.begin() + offset + chunkSize);

    bool isLast = (offset + chunkSize >= data.size());
    if (isLast) {
      frame.fragmentType  = fragment::LAST;
      frame.fragmentCount = static_cast<std::uint8_t>(chunkSize & fragment::COUNT_MASK);
    } else {
      frame.fragmentType  = fragment::MIDDLE;
      frame.fragmentCount = 0;
    }

    frames.push_back(std::move(frame));
    offset += chunkSize;
    remaining -= chunkSize;
  }

  return frames;
#endif // SENSORRING_TRANSPORT_NO_MULTIFRAME
}

} // namespace transport
} // namespace sensorring
} // namespace eduart