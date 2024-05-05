#include <esp_log.h>
#include <memory>

class State {
 public:
    /// Executed once when the state is entered.
    virtual void enter() {
    }

    /// Execute the state logic.
    /// If a state transistion is required, a pointer to the next state is returned, otherwise nullptr.
    virtual State *execute() = 0;

    virtual std::string getName() = 0;

    // Executed once when the state is exited.
    virtual void exit() {
    }

    virtual ~State() {
    }
};

class StateMachine {
 public:
    void setNextState(State *nextState) {
        assert(nextState != nullptr);

        State *discardedState;
        if (xQueueReceiveFromISR(nextStateQueue_, &discardedState, nullptr) == pdTRUE) {
            delete discardedState;
        }

        if (xQueueSend(nextStateQueue_, (const void *)&nextState, 0) == errQUEUE_FULL) {
            ESP_LOGW("StateMachine", "Failed to set next state (%s) since there is already a State waiting",
                     nextState->getName().c_str());
        }
    }

    void setNextStateFromIsr(State *nextState) {
        assert(nextState != nullptr);

        State *discardedState;
        if (xQueueReceiveFromISR(nextStateQueue_, &discardedState, nullptr) == pdTRUE) {
            delete discardedState;
        }

        xQueueSendFromISR(nextStateQueue_, (const void *)&nextState, nullptr);
    }

    std::string getCurrentStateName() {
        return currentState_ != nullptr ? currentState_->getName() : "nullptr";
    }

    void update() {
        State *nextState;
        if (xQueueReceive(nextStateQueue_, &nextState, 0) == pdTRUE) {
            ESP_LOGI("StateMachine", "Transitioning states from %s to %s...",
                     currentState_ != nullptr ? currentState_->getName().c_str() : "nullptr",
                     nextState->getName().c_str());

            if (currentState_ != nullptr) {
                currentState_->exit();
                delete currentState_;
            }

            currentState_ = nextState;
            currentState_->enter();
        }

        nextState = currentState_->execute();
        if (nextState != nullptr) {
            setNextState(nextState);
        }
    }

 private:
    State *currentState_ = nullptr;
    QueueHandle_t nextStateQueue_ = xQueueCreate(1, sizeof(State *));
};