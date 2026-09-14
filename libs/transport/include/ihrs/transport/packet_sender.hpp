#pragma once

#include <cstddef>
#include <span>
#include <string>
#include <string_view>

namespace ihrs::transport {

/// Common interface for outbound packet transports (TCP, UDP, ...).
///
/// Contract:
/// - connect() prepares the transport; it must be called before send().
/// - send() hands the full payload to the socket or returns false and
///   records the errno in last_error(); no partial sends escape the
///   interface. Payloads are binary-safe (may contain any byte values).
/// - close() is idempotent and called by the destructor.
class PacketSender {
public:
    virtual ~PacketSender() = default;

    /// Establishes the transport to `ip:port`. Returns false on failure.
    virtual bool connect(const std::string& ip, int port) = 0;

    /// Sends the payload. Returns true only if the full payload was sent.
    virtual bool send(std::span<const std::byte> data) = 0;

    /// Convenience overload for text/binary byte ranges.
    bool send(std::string_view data) {
        return send(std::span(reinterpret_cast<const std::byte*>(data.data()), data.size()));
    }

    /// Releases the underlying socket. Idempotent.
    virtual void close() = 0;

    /// errno of the most recent failed socket operation (0 if none).
    [[nodiscard]] virtual int last_error() const noexcept = 0;
};

} // namespace ihrs::transport
