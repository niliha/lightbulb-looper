#include <memory>

class State {
 public:
    /// Executed once when the state is entered.
    virtual void enter() {
    }

    /// Execute the state logic.
    /// If a state transistion is required, an unique_ptr to the next state is returned, otherwise nullptr.
    virtual std::unique_ptr<State> execute() = 0;

    // Executed once when the state is exited.
    virtual void exit() {
    }

    virtual ~State() {
    }
};

class StateMachine {
 public:
    void setCurrentState(std::unique_ptr<State> state) {
        assert(state != nullptr);

        if (currentState) {
            currentState->exit();
        }

        currentState = std::move(state);  // Transfer ownership
        currentState->enter();
    }

    void update() {
        assert(currentState != nullptr);
        std::unique_ptr<State> newState = currentState->execute();
        if (newState) {
            // Move semantics to transfer ownership
            setCurrentState(std::move(newState));
        }
    }

 private:
    std::unique_ptr<State> currentState = nullptr;
};