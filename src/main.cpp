#include <Arduino.h>

#include "StateMachine.hpp"
#include "dimmer/AcDimmer.hpp"
#include <Adafruit_MPR121.h>
#include <FastLED.h>
#include <dimmer/PixelFrame.hpp>
#include <esp_log.h>
#include <numeric>

static const char *TAG = "main";

// Configuration
const int CHANNEL_COUNT = 8;
const int FADE_SPEED_PIN = 34;
const int RECORDING_BUTTON_PIN = 27;
const int RECORDING_LED_PIN = 16;
const int LED_STRIP_PIN = 19;
const int ZERO_CROSSING_PIN = 4;

const int BUTTON_DEBOUNCE_MS = 500;
const int UPDATE_INTERVAL_MS = 33;
const int MIN_BRIGHTNESS = 0;
const int MAX_BRIGHTNESS = 100;
const int MIN_FADE_DURATION_MS = 500;
const int MAX_FADE_DURATION_MS = 2500;
const float MIN_FADE_STEP_PER_INTERVAL = (float)MAX_BRIGHTNESS / MAX_FADE_DURATION_MS * UPDATE_INTERVAL_MS;
const float MAX_FADE_STEP_PER_INTERVAL = (float)MAX_BRIGHTNESS / MIN_FADE_DURATION_MS * UPDATE_INTERVAL_MS;

// State Machine context
std::array<unsigned long, CHANNEL_COUNT> lastFadeTriggerTimes_ = {0};
std::array<CRGB, CHANNEL_COUNT> fastLedBuffer_ = {0};
std::array<float, CHANNEL_COUNT> channels_ = {0.0};  // [0, 255]
std::array<float, CHANNEL_COUNT> fadeSteps_ = {0.0};
StateMachine stateMachine_;
Adafruit_MPR121 touchSensor_ = Adafruit_MPR121();

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
    uint16_t touchData = touchSensor_.touched();

    for (int channelIndex = 0; channelIndex < CHANNEL_COUNT; channelIndex++) {
        // Fade up if trigger is pressed, otherwise fade down
        int fadeSign = -1;
        if (touchData & (1 << channelIndex)) {
            fadeSign = 1;

            if (millis() - lastFadeTriggerTimes_[channelIndex] > 3 * UPDATE_INTERVAL_MS) {
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
    std::vector<uint8_t> channelsInt(CHANNEL_COUNT);
    std::transform(channels_.begin(), channels_.end(), channelsInt.begin(),
                   [](float brightness) { return (uint8_t)brightness; });
    AcDimmer::write(channelsInt);
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
    unsigned long MAX_RECORDING_DURATION_MS = 1000 * 60 * 5;  // 5 minutes
    unsigned long recordingStartMillis = 0;
};

class PlaybackState : public State {
 public:
    std::string getName() override {
        return "PlaybackState";
    }

    void enter() override {
        playbackStartMillis_ = millis();
    }

    State *execute() override {
        if (playbackEvents_.empty()) {
            ESP_LOGW("PlaybackState", "No playback events available. Transitioning to LiveState...");
            return new LiveState();
        }

        auto &event = playbackEvents_[currentPlaybackEventIndex_];

        auto millisSincePlaybackStart = millis() - playbackStartMillis_;
        if (event.millis > millisSincePlaybackStart) {
            delay(event.millis - millisSincePlaybackStart);
        }

        channels_ = event.channels;
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
};

void IRAM_ATTR onRecordButtonPressed() {
    static volatile unsigned long lastRecordButtonMillis = 0;

    auto currentMillis = millis();
    if (currentMillis - lastRecordButtonMillis < BUTTON_DEBOUNCE_MS) {
        return;
    }
    lastRecordButtonMillis = currentMillis;

    State *nextState;
    if (stateMachine_.getCurrentStateName() != "RecordingState") {
        nextState = new RecordingState();
    } else {
        nextState = new PlaybackState();
    }

    stateMachine_.setNextStateFromIsr(nextState);
}

extern "C" void app_main() {
    initArduino();

    Serial.begin(115200);
    esp_log_level_set("gpio", ESP_LOG_WARN);
    delay(1000);

    AcDimmer::init(CHANNEL_COUNT, ZERO_CROSSING_PIN, 0);
    AcDimmer::testLights();

    if (!touchSensor_.begin(0x5A)) {
        ESP_LOGE(TAG, "Failed to initialize MPR121 touch sensor");
        ESP.restart();
    }
    ESP_LOGI(TAG, "MPR121 touch sensor initialized");

    pinMode(RECORDING_LED_PIN, OUTPUT);
    digitalWrite(RECORDING_LED_PIN, LOW);
    pinMode(RECORDING_BUTTON_PIN, INPUT_PULLUP);
    attachInterrupt(RECORDING_BUTTON_PIN, onRecordButtonPressed, FALLING);

    FastLED.addLeds<WS2812B, LED_STRIP_PIN, GRB>(fastLedBuffer_.data(), CHANNEL_COUNT);
    FastLED.showColor(CRGB::Black);
    FastLED.setBrightness(126);

    stateMachine_.setNextState(new LiveState());

    ESP_LOGI(TAG, "Initialization complete. Starting state machine loop...");

    while (true) {
        stateMachine_.update();
    }
}