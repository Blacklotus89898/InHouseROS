#pragma once

namespace ihrs::core {

/// Base class for units of robot behavior driven by an Executor.
///
/// Contract:
/// - onInit() is called once when the node is added to an Executor.
/// - onUpdate(dt) is called periodically; dt is the requested cycle
///   period in seconds (not the measured elapsed time).
///
/// Nodes are plain objects: the Executor stores a raw pointer, so a node
/// must outlive the executor loop it is registered with.
class Node {
public:
  virtual ~Node() = default;

  Node() = default;
  Node(const Node&) = default;
  Node& operator=(const Node&) = default;
  Node(Node&&) = default;
  Node& operator=(Node&&) = default;

  /// Called once by Executor::addNode().
  virtual void onInit() {}

  /// Called once per executor cycle with the requested period (seconds).
  virtual void onUpdate(double /*dt*/) {}
};

} // namespace ihrs::core
