#pragma once

class Node {
public:
  virtual ~Node() = default;
  virtual void onInit() {}
  virtual void onUpdate(double /*dt*/) {}
};

