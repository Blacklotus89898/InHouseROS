#pragma once

#include "ihrs/core/broker.hpp"

namespace ihrs::core {

/// Owns a Broker and is the composition root for a middleware instance.
///
/// A Context replaces global-state lookups: publishers, subscribers and
/// executors can be bound to a specific context (dependency injection in
/// tests, isolated buses in one process). Context::global() preserves the
/// historical default: Broker::instance() and all default-constructed
/// endpoints share the global context's broker.
class Context {
public:
    Context() = default;

    Context(const Context&) = delete;
    Context& operator=(const Context&) = delete;

    /// The broker owned by this context.
    [[nodiscard]] Broker& broker() noexcept { return broker_; }

    /// Process-wide default context (the one Broker::instance() returns).
    [[nodiscard]] static Context& global() noexcept {
        static Context g;
        return g;
    }

private:
    Broker broker_;
};

} // namespace ihrs::core
