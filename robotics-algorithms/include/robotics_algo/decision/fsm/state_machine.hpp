#pragma once
#include <string>
#include <memory>
#include <unordered_map>
#include <functional>

namespace robotics::decision {

    // Forward declaration
    class StateMachine;

    // Abstract Base Class for a State
    class State {
    public:
        virtual ~State() = default;

        // Called once when state becomes active
        virtual void onEnter() {}

        // Called every frame. Returns name of next state (or empty string to stay)
        virtual std::string update(double dt) = 0;

        // Called once when state is left
        virtual void onExit() {}

        // Helper to access the machine (injected later)
        StateMachine* machine = nullptr;
    };

    class StateMachine {
    public:
        void addState(const std::string& name, std::shared_ptr<State> state);
        
        // Switch to a specific state immediately
        void changeState(const std::string& name);

        // Run the current state's logic
        void update(double dt);

        // Shared data for states to access (e.g., Robot Position)
        // In a real engine, this might be a blackboard or context pointer.
        // For simplicity, we assume states have access to external variables.
        
        std::string getCurrentStateName() const { return current_state_name_; }

    private:
        std::unordered_map<std::string, std::shared_ptr<State>> states_;
        std::shared_ptr<State> current_state_ = nullptr;
        std::string current_state_name_;
    };
}
