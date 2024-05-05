#include <Arduino.h>

#include "dimmer/AcDimmer.hpp"
#include <FastLED.h>
#include <dimmer/PixelFrame.hpp>
#include <esp_log.h>
#include <numeric>
#include <state_machine.hpp>

static const char *TAG = "main";

// Configuration
const int CHANNEL_COUNT = 6;
const std::array<int, CHANNEL_COUNT> FADE_TRIGGER_PINS = {5, 18, 19, 21, 22, 23};
const std::array<int, CHANNEL_COUNT> FADE_SPEED_PINS = {34, 35, 14, 2, 4, 32};
const int RECORDING_SWITCH_PIN = 27;
const int RANDOMIZE_SWITCH_PIN = 28;  // TODO: Assign correct pin number

const int BUTTON_DEBOUNCE_MS = 100;
const int UPDATE_INTERVAL_MS = 33;
const int MIN_BRIGHTNESS = 0;
const int MAX_BRIGHTNESS = 255;
const int MIN_FADE_DURATION_MS = 100;
const int MAX_FADE_DURATION_MS = 10000;
const float MIN_FADE_STEP_PER_INTERVAL = (float)MAX_BRIGHTNESS / MAX_FADE_DURATION_MS * UPDATE_INTERVAL_MS;
const float MAX_FADE_STEP_PER_INTERVAL = (float)MAX_BRIGHTNESS / MIN_FADE_DURATION_MS * UPDATE_INTERVAL_MS;

// State
std::array<unsigned long, CHANNEL_COUNT> lastFadeTriggerTimes_ = {0};
std::array<CRGB, CHANNEL_COUNT> fastLedBuffer_ = {0};
std::array<float, CHANNEL_COUNT> channels_ = {0.0};  // [0, 255]

StateMachine stateMachine_;

struct PlaybackEvent {
    const unsigned long millis;
    const std::array<float, CHANNEL_COUNT> channels;

    PlaybackEvent(unsigned long millis, std::array<float, CHANNEL_COUNT> channels)
        : millis(millis), channels(channels) {
    }
};
std::vector<PlaybackEvent> playbackEvents_;

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
            millis() - lastFadeTriggerTimes_[channelIndex] > UPDATE_INTERVAL_MS) {
            fadeStep *= -1;
        }

        float &currentBrightness = channels_[channelIndex];
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
    std::transform(channels_.begin(), channels_.end(), fastLedBuffer_.begin(),
                   [](float brightness) { return CRGB(brightness, 0, 0); });
    FastLED.show();

    // Light bulbs
    // AcDimmer::write(channels);
}

class LiveState : public State {
 public:
    State *execute() override {
        bool isWriteRequired = prepareChannels();
        if (isWriteRequired) {
            writeChannels();
        }

        delay(UPDATE_INTERVAL_MS);
        return nullptr;
    }

    std::string getName() override {
        return "LiveState";
    }
};

class RecordingState : public State {
 public:
    void enter() override {
        playbackEvents_.clear();
        lastFadeTriggerTimes_.fill(0);
        channels_.fill(0);
        recordingStartMillis = millis();
    }

    State *execute() override {
        bool isWriteRequired = prepareChannels();
        if (isWriteRequired) {
            auto eventMillis = millis() - recordingStartMillis;
            playbackEvents_.emplace_back(eventMillis, channels_);
            writeChannels();
            ESP_LOGI("RecordingState", "Recorded %dth event at %lu ms", playbackEvents_.size(), eventMillis);
        }

        delay(UPDATE_INTERVAL_MS);
        return nullptr;
    }

    std::string getName() override {
        return "RecordingState";
    }

 private:
    unsigned long recordingStartMillis = 0;
};

class PlaybackState : public State {
 public:
    void enter() override {
        playbackStartMillis_ = millis();

        std::iota(channelMapping_.begin(), channelMapping_.end(), 0);
        if (digitalRead(RANDOMIZE_SWITCH_PIN) == HIGH) {
            ESP_LOGI("PlaybackState", "Randomizing playback events...");
            std::random_shuffle(channelMapping_.begin(), channelMapping_.end());
        }
    }

    State *execute() override {
        if (playbackEvents_.empty()) {
            ESP_LOGW("PlaybackState", "No playback events available. Transitioning to LiveState...");
            return new LiveState();
        }

        auto &event = playbackEvents_[currentPlaybackEventIndex_];
        ESP_LOGI("PlaybackState", "Executing playback event %d of %d...", currentPlaybackEventIndex_ + 1,
                 playbackEvents_.size());

        auto millisSincePlaybackStart = millis() - playbackStartMillis_;
        if (event.millis > millisSincePlaybackStart) {
            delay(event.millis - millisSincePlaybackStart);
        }

        channels_ = event.channels;
        std::transform(channelMapping_.begin(), channelMapping_.end(), channels_.begin(),
                       [](int channelIndex) { return channels_[channelIndex]; });

        writeChannels();

        currentPlaybackEventIndex_++;
        if (currentPlaybackEventIndex_ >= playbackEvents_.size()) {
            return new PlaybackState();
        } else {
            return nullptr;
        }
    }

    std::string getName() override {
        return "PlaybackState";
    }

 private:
    int currentPlaybackEventIndex_ = 0;
    unsigned long playbackStartMillis_ = 0;
    std::array<int, CHANNEL_COUNT> channelMapping_;
};

void IRAM_ATTR onFadeTrigger(void *param) {
    int channelIndex = reinterpret_cast<int>(param);
    // Store the time of the last fade trigger event to avoid missing it while polling
    lastFadeTriggerTimes_[channelIndex] = millis();
}

void IRAM_ATTR onRecordSwitchChange() {
    static unsigned long lastRecordButtonEventTimeMs = 0;

    auto currentMillis = millis();
    if (currentMillis - lastRecordButtonEventTimeMs < BUTTON_DEBOUNCE_MS) {
        return;
    } else {
        lastRecordButtonEventTimeMs = currentMillis;
    }

    if (stateMachine_.getCurrentStateName() == "RecordingState") {
        stateMachine_.setNextStateFromIsr(new PlaybackState());
    } else {
        stateMachine_.setNextStateFromIsr(new RecordingState());
    }
}

extern "C" void app_main() {
    initArduino();
    Serial.begin(115200);

    esp_log_level_set("gpio", ESP_LOG_WARN);
    // AcDimmer::init(CHANNEL_COUNT, 5, 0);
    // AcDimmer::testLights();

    for (int channelIndex = 0; channelIndex < CHANNEL_COUNT; channelIndex++) {
        pinMode(FADE_TRIGGER_PINS[channelIndex], INPUT);
        attachInterruptArg(digitalPinToInterrupt(FADE_TRIGGER_PINS[channelIndex]), onFadeTrigger,
                           reinterpret_cast<void *>(channelIndex), FALLING);
    }

    pinMode(RECORDING_SWITCH_PIN, INPUT_PULLDOWN);
    pinMode(RANDOMIZE_SWITCH_PIN, INPUT_PULLDOWN);
    attachInterrupt(RECORDING_SWITCH_PIN, onRecordSwitchChange, CHANGE);

    FastLED.addLeds<WS2812B, 13, GRB>(fastLedBuffer_.data(), CHANNEL_COUNT);
    FastLED.showColor(CRGB::Black);

    stateMachine_.setNextState(new LiveState());

    ESP_LOGI(TAG, "Initialization complete. Starting state machine loop...");

    while (true) {
        stateMachine_.update();
    }
}