#include "robotics_algo/decision/fsm/state_machine.hpp"
#include <iostream>

namespace robotics::decision {

    void StateMachine::addState(const std::string& name, std::shared_ptr<State> state) {
        states_[name] = state;
        state->machine = this;
    }

    void StateMachine::changeState(const std::string& name) {
        if (states_.find(name) == states_.end()) {
            std::cerr << "[FSM] Error: State " << name << " does not exist!" << std::endl;
            return;
        }

        // 1. Exit old state
        if (current_state_) {
            current_state_->onExit();
        }

        // 2. Swap
        current_state_ = states_[name];
        current_state_name_ = name;

        // 3. Enter new state
        if (current_state_) {
            std::cout << "[FSM] Entering: " << name << std::endl;
            current_state_->onEnter();
        }
    }

    void StateMachine::update(double dt) {
        if (!current_state_) return;

        // Run logic
        std::string next_state = current_state_->update(dt);

        // Check for transition
        if (!next_state.empty() && next_state != current_state_name_) {
            changeState(next_state);
        }
    }
}
