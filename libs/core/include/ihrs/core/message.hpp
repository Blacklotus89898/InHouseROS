#pragma once

#include <memory>

namespace ihrs::core {

/// Polymorphic message base. Derive and add your payload fields:
///
/// @code
/// struct Pose2DMsg : ihrs::core::Message { double x, y, theta; };
/// @endcode
///
/// Messages travel as shared_ptr and are dispatched on the spinning
/// thread; keep them cheap to copy and free of side effects in the
/// destructor.
struct Message {
  virtual ~Message() = default;
};

using MessagePtr = std::shared_ptr<Message>;

} // namespace ihrs::core
