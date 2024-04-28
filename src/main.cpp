#include <Arduino.h>

#include "dimmer/AcDimmer.hpp"
#include <FastLED.h>
#include <dimmer/PixelFrame.hpp>
#include <state_machine.hpp>

// Configuration
const int CHANNEL_COUNT = 6;
const std::array<int, CHANNEL_COUNT> FADE_TRIGGER_PINS = {5, 18, 19, 21, 22, 23};
const std::array<int, CHANNEL_COUNT> FADE_SPEED_PINS = {34, 35, 14, 2, 4, 32};
const int RECORDING_SWITCH_PIN = 33;  // TODO: Adapt pin
const int BUTTON_DEBOUNCE_MS = 50;
const int UPDATE_INTERVAL_MS = 33;
const int MIN_BRIGHTNESS = 0;
const int MAX_BRIGHTNESS = 255;
const int MIN_FADE_DURATION_MS = 100;
const int MAX_FADE_DURATION_MS = 10000;
const float MIN_FADE_STEP_PER_INTERVAL = (float)MAX_BRIGHTNESS / MAX_FADE_DURATION_MS * UPDATE_INTERVAL_MS;
const float MAX_FADE_STEP_PER_INTERVAL = (float)MAX_BRIGHTNESS / MIN_FADE_DURATION_MS * UPDATE_INTERVAL_MS;

// State
std::array<unsigned long, CHANNEL_COUNT> lastFadeTriggerTimes = {0};
std::array<CRGB, CHANNEL_COUNT> fastLedBuffer = {0};
std::array<float, CHANNEL_COUNT> channels = {0.0};  // [0, 255]

StateMachine stateMachine;

struct PlaybackEvent {
    const unsigned long millis;
    const std::array<float, CHANNEL_COUNT> channels;

    PlaybackEvent(unsigned long millis, std::array<float, CHANNEL_COUNT> channels)
        : millis(millis), channels(channels) {
    }
};
std::vector<PlaybackEvent> playbackEvents;

float mapFloat(float value, float fromLow, float fromHigh, float toLow, float toHigh) {
    return (value - fromLow) * (toHigh - toLow) / (fromHigh - fromLow) + toLow;
}

bool prepareChannels() {
    bool isWriteRequired = false;

    for (int channelIndex = 0; channelIndex < CHANNEL_COUNT; channelIndex++) {
        int fadeSpeed = analogRead(FADE_SPEED_PINS[channelIndex]);  // ADC value is between 0 and 4095
        float fadeStep = mapFloat(fadeSpeed, 0, 4095, MIN_FADE_STEP_PER_INTERVAL, MAX_FADE_STEP_PER_INTERVAL);

        // Fade down if fade trigger is currently not pressed, otherwise fade up
        if (digitalRead(FADE_TRIGGER_PINS[channelIndex]) == LOW &&
            millis() - lastFadeTriggerTimes[channelIndex] > UPDATE_INTERVAL_MS) {
            fadeStep *= -1;
        }

        float &currentBrightness = channels[channelIndex];
        float newBrightness = constrain(currentBrightness + fadeStep, MIN_BRIGHTNESS, MAX_BRIGHTNESS);

        // Write is only required if at least one channel has changed its integer value, since
        // this is the minimum resolution of the dimmer and the LED strip
        if ((uint8_t)newBrightness != currentBrightness) {
            isWriteRequired = true;
        }
        currentBrightness = newBrightness;
    }

    return isWriteRequired;
}

void writeChannels() {
    // LED strip visualizer
    std::transform(channels.begin(), channels.end(), fastLedBuffer.begin(),
                   [](float brightness) { return CRGB(brightness, 0, 0); });
    FastLED.show();

    // Light bulbs
    // AcDimmer::write(channels);
}

class InitialState : public State {
 public:
    std::unique_ptr<State> execute() override {
        bool isWriteRequired = prepareChannels();
        if (isWriteRequired) {
            writeChannels();
        }

        delay(UPDATE_INTERVAL_MS);
        return nullptr;
    }
};

class RecordingState : public State {
 public:
    void enter() override {
        playbackEvents.clear();
        recordingStartMillis = millis();
    }

    std::unique_ptr<State> execute() override {
        bool isWriteRequired = prepareChannels();
        if (isWriteRequired) {
            playbackEvents.emplace_back(millis() - recordingStartMillis, channels);
            writeChannels();
        }

        delay(UPDATE_INTERVAL_MS);
        return nullptr;
    }

 private:
    unsigned long recordingStartMillis = 0;
};

class PlaybackState : public State {
 public:
    void enter() override {
        playbackStartMillis = millis();
    }

    std::unique_ptr<State> execute() override {
        auto &event = playbackEvents[currentPlaybackEventIndex];
        delay(event.millis - (millis() - playbackStartMillis));

        channels = event.channels;
        writeChannels();

        currentPlaybackEventIndex++;
        if (currentPlaybackEventIndex >= playbackEvents.size()) {
            return std::make_unique<PlaybackState>();
        } else {
            return nullptr;
        }
    }

 private:
    int currentPlaybackEventIndex = 0;
    unsigned long playbackStartMillis = 0;
};

void IRAM_ATTR onFadeTrigger(void *param) {
    int channelIndex = reinterpret_cast<int>(param);
    // Store the time of the last fade trigger event to avoid missing it while polling
    lastFadeTriggerTimes[channelIndex] = millis();
}

void IRAM_ATTR onRecordSwitchChange() {
    static unsigned long lastRecordButtonEventTimeMs = 0;

    auto currentMillis = millis();
    if (currentMillis - lastRecordButtonEventTimeMs < BUTTON_DEBOUNCE_MS) {
        return;
    } else {
        lastRecordButtonEventTimeMs = currentMillis;
    }

    if (digitalRead(RECORDING_SWITCH_PIN) == LOW) {
        stateMachine.setCurrentState(std::make_unique<RecordingState>());
    } else {
        stateMachine.setCurrentState(std::make_unique<PlaybackState>());
    }
}

extern "C" void app_main() {
    initArduino();
    Serial.begin(115200);

    // AcDimmer::init(CHANNEL_COUNT, 5, 0);
    // AcDimmer::testLights();

    for (int channelIndex = 0; channelIndex < CHANNEL_COUNT; channelIndex++) {
        pinMode(FADE_TRIGGER_PINS[channelIndex], INPUT);
        attachInterruptArg(digitalPinToInterrupt(FADE_TRIGGER_PINS[channelIndex]), onFadeTrigger,
                           reinterpret_cast<void *>(channelIndex), FALLING);
    }

    pinMode(RECORDING_SWITCH_PIN, INPUT_PULLUP);
    attachInterrupt(RECORDING_SWITCH_PIN, onRecordSwitchChange, CHANGE);

    FastLED.addLeds<WS2812B, 13, GRB>(fastLedBuffer.data(), CHANNEL_COUNT);
    FastLED.showColor(CRGB::Black);

    stateMachine.setCurrentState(std::make_unique<InitialState>());

    while (true) {
        stateMachine.update();
    }
}