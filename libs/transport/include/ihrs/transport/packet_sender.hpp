#pragma once

#include <string>

namespace ihrs::transport {

/// Common interface for outbound packet transports (TCP, UDP, ...).
///
/// Contract:
/// - connect() prepares the transport; it must be called before send().
/// - send() returns false if the payload could not be handed to the
///   socket in full; no partial sends escape the interface.
/// - close() is idempotent and called by the destructor.
class PacketSender {
public:
    virtual ~PacketSender() = default;

    /// Establishes the transport to `ip:port`. Returns false on failure.
    virtual bool connect(const std::string& ip, int port) = 0;

    /// Sends the payload. Returns true only if the full payload was sent.
    virtual bool send(const std::string& data) = 0;

    /// Releases the underlying socket. Idempotent.
    virtual void close() = 0;
};

} // namespace ihrs::transport
