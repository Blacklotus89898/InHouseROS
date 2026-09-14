#pragma once

#include <cstdint>
#include <cstddef>
#include <span>
#include <vector>

namespace ihrs::transport::framing {

/// Length-prefixed message framing for stream transports (TCP).
///
/// Wire format: [uint32 payload length, little-endian][payload bytes].
/// write_frame() produces a frame on the sending side; FrameExtractor
/// reassembles frames from an arbitrary byte stream on the receiving side.

inline constexpr std::size_t kHeaderSize = 4;
inline constexpr std::uint32_t kMaxFrameSize = 4u * 1024u * 1024u; // sanity guard

/// Appends one complete frame (header + payload) to `out`.
inline void write_frame(std::vector<std::byte>& out, std::span<const std::byte> payload) {
    const std::uint32_t len = static_cast<std::uint32_t>(payload.size());
    out.push_back(static_cast<std::byte>(len & 0xFFu));
    out.push_back(static_cast<std::byte>((len >> 8) & 0xFFu));
    out.push_back(static_cast<std::byte>((len >> 16) & 0xFFu));
    out.push_back(static_cast<std::byte>((len >> 24) & 0xFFu));
    out.insert(out.end(), payload.begin(), payload.end());
}

/// Incremental reassembler: feed raw stream bytes, get whole frames back.
/// The payload passed to the callback is valid until the next feed() call.
class FrameExtractor {
public:
    /// Feeds raw bytes; invokes on_frame(payload_view) for every complete
    /// frame that becomes available. Oversized frames (corrupt stream) are
    /// discarded together with the buffer (re-sync is impossible without
    /// application support), and false is returned.
    template <typename OnFrame>
    bool feed(std::span<const std::byte> bytes, OnFrame&& on_frame) {
        buffer_.insert(buffer_.end(), bytes.begin(), bytes.end());
        std::size_t offset = 0;
        bool ok = true;

        while (buffer_.size() - offset >= kHeaderSize) {
            const std::uint32_t len = read_le32(buffer_.data() + offset);
            if (len > kMaxFrameSize) {
                ok = false; // corrupted stream; drop everything buffered
                buffer_.clear();
                offset = 0;
                break;
            }
            if (buffer_.size() - offset - kHeaderSize < len) {
                break; // partial frame: wait for more bytes
            }
            const std::byte* start = buffer_.data() + offset + kHeaderSize;
            on_frame(std::span(start, len));
            offset += kHeaderSize + len;
        }

        if (offset > 0 && !buffer_.empty()) {
            buffer_.erase(buffer_.begin(), buffer_.begin() + static_cast<std::ptrdiff_t>(offset));
        }
        return ok;
    }

    /// Bytes currently buffered (partial frame data waiting for more).
    [[nodiscard]] std::size_t pending() const noexcept { return buffer_.size(); }

private:
    static std::uint32_t read_le32(const std::byte* p) {
        return static_cast<std::uint32_t>(static_cast<unsigned char>(p[0])) |
               (static_cast<std::uint32_t>(static_cast<unsigned char>(p[1])) << 8) |
               (static_cast<std::uint32_t>(static_cast<unsigned char>(p[2])) << 16) |
               (static_cast<std::uint32_t>(static_cast<unsigned char>(p[3])) << 24);
    }

    std::vector<std::byte> buffer_;
};

} // namespace ihrs::transport::framing
