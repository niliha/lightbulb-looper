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
// const std::array<int, CHANNEL_COUNT> FADE_SPEED_PINS = {34, 35, 14, 2, 4, 32};
const int FADE_SPEED_PIN = 34;
const int RECORDING_BUTTON_PIN = 27;
const int RECORDING_LED_PIN = 16;
const int RANDOMIZE_SWITCH_PIN = 28;  // TODO: Assign correct pin number

const int BUTTON_DEBOUNCE_MS = 500;
const int UPDATE_INTERVAL_MS = 33;
const int MIN_BRIGHTNESS = 0;
const int MAX_BRIGHTNESS = 255;
const int MIN_FADE_DURATION_MS = 500;
const int MAX_FADE_DURATION_MS = 5000;
const float MIN_FADE_STEP_PER_INTERVAL = (float)MAX_BRIGHTNESS / MAX_FADE_DURATION_MS * UPDATE_INTERVAL_MS;
const float MAX_FADE_STEP_PER_INTERVAL = (float)MAX_BRIGHTNESS / MIN_FADE_DURATION_MS * UPDATE_INTERVAL_MS;

// State
std::array<unsigned long, CHANNEL_COUNT> lastFadeTriggerTimes_ = {0};
std::array<CRGB, CHANNEL_COUNT> fastLedBuffer_ = {0};
std::array<float, CHANNEL_COUNT> channels_ = {0.0};  // [0, 255]
std::array<float, CHANNEL_COUNT> fadeSteps_ = {0.0};

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
        // Fade down if fade trigger is currently not pressed, otherwise fade up
        int fadeSign = -1;
        if (digitalRead(FADE_TRIGGER_PINS[channelIndex]) == HIGH) {
            fadeSign = 1;

            if (millis() - lastFadeTriggerTimes_[channelIndex] > 3 * UPDATE_INTERVAL_MS) {
                ESP_LOGI("prepareChannels", "Detected fade trigger after absence for channel %d", channelIndex);
                int fadeSpeed = analogRead(FADE_SPEED_PIN);  // ADC value is between 0 and 4095
                fadeSteps_[channelIndex] =
                    mapFloat(fadeSpeed, 0, 4095, MIN_FADE_STEP_PER_INTERVAL, MAX_FADE_STEP_PER_INTERVAL);
            }

            lastFadeTriggerTimes_[channelIndex] = millis();
        }

        float &currentBrightness = channels_[channelIndex];
        float newBrightness =
            constrain(currentBrightness + fadeSign * fadeSteps_[channelIndex], MIN_BRIGHTNESS, MAX_BRIGHTNESS);

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
    std::string getName() override {
        return "LiveState";
    }

    State *execute() override {
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
    std::string getName() override {
        return "RecordingState";
    }

    void enter() override {
        playbackEvents_.clear();
        lastFadeTriggerTimes_.fill(0);
        channels_.fill(0);
        writeChannels();

        recordingStartMillis = millis();
        digitalWrite(RECORDING_LED_PIN, HIGH);
    }

    State *execute() override {
        bool isWriteRequired = prepareChannels();
        if (isWriteRequired) {
            auto eventMillis = millis() - recordingStartMillis;
            playbackEvents_.emplace_back(eventMillis, channels_);
            writeChannels();
        }

        delay(UPDATE_INTERVAL_MS);
        return nullptr;
    }

    void exit() override {
        digitalWrite(RECORDING_LED_PIN, LOW);
    }

 private:
    unsigned long recordingStartMillis = 0;
};

class PlaybackState : public State {
 public:
    std::string getName() override {
        return "PlaybackState";
    }

    void enter() override {
        playbackStartMillis_ = millis();

        std::iota(channelMapping_.begin(), channelMapping_.end(), 0);
        // if (digitalRead(RANDOMIZE_SWITCH_PIN) == HIGH) {
        if (false) {
            ESP_LOGI("PlaybackState", "Randomizing playback events...");
            std::random_shuffle(channelMapping_.begin(), channelMapping_.end());
            printf("Channel mapping: ");
            for (int i = 0; i < CHANNEL_COUNT; i++) {
                printf("%d ", channelMapping_[i]);
            }
            printf("\n");
        }
    }

    State *execute() override {
        if (playbackEvents_.empty()) {
            ESP_LOGW("PlaybackState", "No playback events available. Transitioning to LiveState...");
            return new LiveState();
        }

        auto &event = playbackEvents_[currentPlaybackEventIndex_];
        // ESP_LOGI("PlaybackState", "Executing playback event %d of %d...", currentPlaybackEventIndex_ + 1,
        //          playbackEvents_.size());

        auto millisSincePlaybackStart = millis() - playbackStartMillis_;
        if (event.millis > millisSincePlaybackStart) {
            delay(event.millis - millisSincePlaybackStart);
        }

        channels_ = event.channels;
        auto channels = channels_;
        std::transform(channelMapping_.begin(), channelMapping_.end(), channels.begin(),
                       [](int channelIndex) { return channels_[channelIndex]; });
        channels_ = channels;
        writeChannels();

        currentPlaybackEventIndex_++;
        if (currentPlaybackEventIndex_ >= playbackEvents_.size()) {
            return new PlaybackState();
        } else {
            return nullptr;
        }
    }

 private:
    int currentPlaybackEventIndex_ = 0;
    unsigned long playbackStartMillis_ = 0;
    std::array<int, CHANNEL_COUNT> channelMapping_;
};

void IRAM_ATTR onRecordButtonPressed() {
    static volatile unsigned long lastRecordButtonMillis = 0;

    auto currentMillis = millis();
    if (currentMillis - lastRecordButtonMillis < BUTTON_DEBOUNCE_MS) {
        return;
    }
    lastRecordButtonMillis = currentMillis;

    if (stateMachine_.getCurrentStateName() != "RecordingState") {
        stateMachine_.setNextStateFromIsr(new RecordingState());
    } else {
        stateMachine_.setNextStateFromIsr(new PlaybackState());
    }
}

extern "C" void app_main() {
    initArduino();
    Serial.begin(115200);

    delay(1000);

    esp_log_level_set("gpio", ESP_LOG_WARN);
    // AcDimmer::init(CHANNEL_COUNT, 5, 0);
    // AcDimmer::testLights();

    for (int channelIndex = 0; channelIndex < CHANNEL_COUNT; channelIndex++) {
        pinMode(FADE_TRIGGER_PINS[channelIndex], INPUT);
    }

    pinMode(RECORDING_LED_PIN, OUTPUT);
    digitalWrite(RECORDING_LED_PIN, LOW);
    pinMode(RECORDING_BUTTON_PIN, INPUT_PULLUP);
    // pinMode(RANDOMIZE_SWITCH_PIN, INPUT_PULLDOWN);
    attachInterrupt(RECORDING_BUTTON_PIN, onRecordButtonPressed, FALLING);

    FastLED.addLeds<WS2812B, 13, GRB>(fastLedBuffer_.data(), CHANNEL_COUNT);
    FastLED.showColor(CRGB::Black);

    stateMachine_.setNextState(new LiveState());

    ESP_LOGI(TAG, "Initialization complete. Starting state machine loop...");

    while (true) {
        stateMachine_.update();
    }
}