#include "ihrs/core/context.hpp"

#include "ihrs/core/typed.hpp"

namespace ihrs::core {

NodeHandle Context::create_node(std::string name) {
    return NodeHandle(broker(), std::move(name));
}

} // namespace ihrs::core
