#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <string_view>
#include <utility>

#include "ihrs/core/broker.hpp"
#include "ihrs/core/message.hpp"
#include "ihrs/core/type_support.hpp"

namespace ihrs::core {

/// Quality-of-service options for typed endpoints.
///
/// Stage 2 scope: `depth` bounds queueing wherever the delivery path has a
/// real queue (bridge/SHM/net). In-process delivery inherits the broker's
/// global pending bound; per-subscription local queues are not implemented.
struct QoS {
    std::size_t depth = 1;

    QoS& with_depth(std::size_t d) noexcept {
        depth = d;
        return *this;
    }
};

namespace detail {

/// Type-tagged message envelope travelling through the broker.
template <typename T>
class TypedMessage final : public Message {
public:
    TypedMessage(std::uint64_t hash, const T& v) : type_hash(hash), value(v) {}

    std::uint64_t type_hash;
    T value;
};

} // namespace detail

/// Typed publisher for topic `topic`.
///
/// Binding lifetime: the constructor registers the payload type for the
/// topic (throws TypeMismatchError on conflict) and the destructor releases
/// it — a topic becomes rebindable once its last endpoint disappears.
/// Movable; not copyable.
template <typename T>
class Pub final {
    static_assert(is_wire_compatible_v<T>,
                  "ihrs::core: payload type must be trivially copyable or provide a "
                  "Codec<T> specialization (see ihrs/core/type_support.hpp)");

public:
    Pub(std::string topic, const QoS& qos, Broker& broker)
        : topic_(std::move(topic)), qos_(qos), broker_(broker), hash_(type_hash<T>()) {
        broker_.register_topic_type(topic_, hash_);
    }

    ~Pub() {
        if (hash_ != 0) {
            broker_.release_topic_type(topic_, hash_);
        }
    }

    Pub(const Pub&) = delete;
    Pub& operator=(const Pub&) = delete;

    Pub(Pub&& other) noexcept
        : topic_(std::move(other.topic_)),
          qos_(other.qos_),
          broker_(other.broker_),
          hash_(other.hash_) {
        other.hash_ = 0; // moved-from: releases nothing
    }

    Pub& operator=(Pub&& other) noexcept {
        if (this != &other) {
            if (hash_ != 0) {
                broker_.release_topic_type(topic_, hash_);
            }
            topic_ = std::move(other.topic_);
            qos_ = other.qos_;
            hash_ = other.hash_;
            other.hash_ = 0;
        }
        return *this;
    }

    /// Enqueues a copy of `value` for all subscribers. Non-blocking.
    void publish(const T& value) {
        broker_.publish(topic_, std::make_shared<detail::TypedMessage<T>>(hash_, value));
    }

    [[nodiscard]] const std::string& topic() const noexcept { return topic_; }
    [[nodiscard]] const QoS& qos() const noexcept { return qos_; }

private:
    std::string topic_;
    QoS qos_;
    Broker& broker_;
    std::uint64_t hash_;
};

/// Typed subscription for topic `topic` (RAII; see Pub for binding rules).
///
/// The callback runs on the thread calling Broker::spinOnce() and receives
/// `const T&` valid only for the duration of the call. Messages of foreign
/// types arriving on the same topic (untyped-API mixing) are ignored.
template <typename T>
class Sub final {
    static_assert(is_wire_compatible_v<T>,
                  "ihrs::core: payload type must be trivially copyable or provide a "
                  "Codec<T> specialization (see ihrs/core/type_support.hpp)");

public:
    using Callback = std::function<void(const T&)>;

    Sub(std::string topic, Callback cb, const QoS& qos, Broker& broker)
        : topic_(std::move(topic)),
          broker_(broker),
          hash_(type_hash<T>()),
          callback_(std::move(cb)),
          qos_(qos) {
        broker_.register_topic_type(topic_, hash_);
        const std::uint64_t hash = hash_;
        const Callback& user_cb = callback_;
        token_ = broker_.subscribe(
            topic_, [cb = user_cb, hash](MessagePtr msg) {
                const auto typed = std::dynamic_pointer_cast<detail::TypedMessage<T>>(msg);
                if (typed == nullptr || typed->type_hash != hash) {
                    return; // foreign/untyped message on this topic: ignore
                }
                cb(typed->value);
            });
    }

    ~Sub() {
        if (token_ != Broker::kInvalidToken) {
            broker_.unsubscribe(topic_, token_);
        }
        if (hash_ != 0) {
            broker_.release_topic_type(topic_, hash_);
        }
    }

    Sub(const Sub&) = delete;
    Sub& operator=(const Sub&) = delete;

    Sub(Sub&& other) noexcept
        : topic_(std::move(other.topic_)),
          broker_(other.broker_),
          hash_(other.hash_),
          callback_(std::move(other.callback_)),
          qos_(other.qos_),
          token_(other.token_) {
        other.hash_ = 0;
        other.token_ = Broker::kInvalidToken;
    }

    Sub& operator=(Sub&& other) noexcept {
        if (this != &other) {
            if (token_ != Broker::kInvalidToken) {
                broker_.unsubscribe(topic_, token_);
            }
            if (hash_ != 0) {
                broker_.release_topic_type(topic_, hash_);
            }
            topic_ = std::move(other.topic_);
            hash_ = other.hash_;
            callback_ = std::move(other.callback_);
            qos_ = other.qos_;
            token_ = other.token_;
            other.hash_ = 0;
            other.token_ = Broker::kInvalidToken;
        }
        return *this;
    }

    [[nodiscard]] const std::string& topic() const noexcept { return topic_; }
    [[nodiscard]] const QoS& qos() const noexcept { return qos_; }

private:
    std::string topic_;
    Broker& broker_;
    std::uint64_t hash_;
    Callback callback_;
    QoS qos_;
    Broker::Token token_ = Broker::kInvalidToken;
};

/// Named handle bound to a context; manufactures typed endpoints.
/// Created via Context::create_node(). Copyable view, cheap to pass.
class NodeHandle {
public:
    template <typename T>
    [[nodiscard]] std::shared_ptr<Pub<T>> create_publisher(std::string_view topic,
                                                           const QoS& qos = {}) const {
        return std::make_shared<Pub<T>>(std::string(topic), qos, broker_);
    }

    template <typename T>
    [[nodiscard]] std::shared_ptr<Sub<T>> create_subscriber(
        std::string_view topic, std::function<void(const T&)> cb, const QoS& qos = {}) const {
        return std::make_shared<Sub<T>>(std::string(topic), std::move(cb), qos, broker_);
    }

    [[nodiscard]] std::string_view name() const noexcept { return name_; }

private:
    friend class Context;

    NodeHandle(Broker& broker, std::string name)
        : broker_(broker), name_(std::move(name)) {}

    Broker& broker_;
    std::string name_;
};

} // namespace ihrs::core
