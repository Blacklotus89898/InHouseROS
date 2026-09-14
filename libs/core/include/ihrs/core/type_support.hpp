#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string_view>
#include <type_traits>
#include <utility>

namespace ihrs::core {

/// Compile-time type identity for topic payloads.
///
/// Hashes the compiler-generated function signature that embeds T's mangled
/// name (FNV-1a). Equal for the same type within one binary; different types
/// collide with negligible probability. The guarantee is same-binary only —
/// hashes must not be persisted across ABI boundaries.
template <typename T>
[[nodiscard]] consteval std::uint64_t type_hash() noexcept {
#if defined(_MSC_VER)
    constexpr std::string_view kSignature = __FUNCSIG__;
#else
    constexpr std::string_view kSignature = __PRETTY_FUNCTION__;
#endif
    std::uint64_t hash = 14695981039346656037ull; // FNV-1a offset basis
    for (const char c : kSignature) {
        hash ^= static_cast<std::uint64_t>(static_cast<unsigned char>(c));
        hash *= 1099511628211ull; // FNV-1a prime
    }
    return hash;
}

/// Wire codec for topic payloads.
///
/// Primary template: absent. Payloads that are not trivially copyable must
/// provide a specialization with exactly these members:
///
/// @code
/// template <> struct ihrs::core::Codec<MyType> {
///     static std::size_t encoded_size(const MyType&);
///     static void encode(const MyType&, std::byte* out);   // encoded_size bytes
///     static MyType   decode(const std::byte* in);
/// };
/// @endcode
template <typename T, typename = void>
struct Codec {};

// Auto codec: trivially copyable payloads travel as raw bytes (zero-copy
// capable once SHM v2 lands).
template <typename T>
struct Codec<T, std::enable_if_t<std::is_trivially_copyable_v<T>>> {
    static std::size_t encoded_size(const T&) noexcept { return sizeof(T); }
    static void encode(const T& value, std::byte* out) {
        std::memcpy(out, &value, sizeof(T));
    }
    static T decode(const std::byte* in) {
        T value;
        std::memcpy(&value, in, sizeof(T));
        return value;
    }
};

/// True when a payload has a usable wire codec (auto or user-specialized).
template <typename T, typename = void>
struct has_codec : std::false_type {};

template <typename T>
struct has_codec<T, std::void_t<decltype(Codec<T>::encoded_size(std::declval<const T&>())),
                                decltype(Codec<T>::encode(std::declval<const T&>(),
                                                          std::declval<std::byte*>()))>>
    : std::true_type {};

/// Compile-time gate used by Pub<T>/Sub<T>.
template <typename T>
inline constexpr bool is_wire_compatible_v = has_codec<T>::value;

} // namespace ihrs::core
