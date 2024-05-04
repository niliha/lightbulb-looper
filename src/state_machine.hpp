#include <esp_log.h>
#include <memory>

class State {
 public:
    /// Executed once when the state is entered.
    virtual void enter() {
    }

    /// Execute the state logic.
    /// If a state transistion is required, an pointer to the next state is returned, otherwise nullptr.
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
        if (xQueueReceiveFromISR(nextStateQueue, &discardedState, nullptr) == pdTRUE) {
            delete discardedState;
        }

        xQueueSendFromISR(nextStateQueue, (const void *)&nextState, nullptr);
    }

    std::string getCurrentStateName() {
        return currentState != nullptr ? currentState->getName() : "nullptr";
    }

    void update() {
        State *nextState;
        if (xQueueReceive(nextStateQueue, &nextState, 0) == pdTRUE) {
            ESP_LOGI("StateMachine", "Transitioning states from %s to %s...",
                     currentState != nullptr ? currentState->getName().c_str() : "nullptr",
                     nextState->getName().c_str());

            if (currentState != nullptr) {
                currentState->exit();
                delete currentState;
            }

            currentState = nextState;
            currentState->enter();
        }

        nextState = currentState->execute();
        if (nextState != nullptr) {
            setNextState(nextState);
        }
    }

 private:
    State *currentState = nullptr;
    QueueHandle_t nextStateQueue = xQueueCreate(1, sizeof(State *));
};