#include <Arduino.h>

#include "dimmer/AcDimmer.hpp"
#include <FastLED.h>
#include <dimmer/PixelFrame.hpp>

const int CHANNEL_COUNT = 1;
// const std::array<int, CHANNEL_COUNT> FADE_TRIGGER_PINS = {18, 19, 21, 22, 23, 25};
// const std::array<int, CHANNEL_COUNT> FADE_SPEED_PINS = {23, 33, 34, 35, 36, 39};
const std::array<int, CHANNEL_COUNT> FADE_TRIGGER_PINS = {18};
const std::array<int, CHANNEL_COUNT> FADE_SPEED_PINS = {34};
const int UPDATE_INTERVAL_MS = 33;

const int MIN_BRIGHTNESS = 0;
const int MAX_BRIGHTNESS = 255;
const int MIN_FADE_DURATION_MS = 100;
const int MAX_FADE_DURATION_MS = 10000;
const float MIN_FADE_STEP_PER_INTERVAL = (float)MAX_BRIGHTNESS / MAX_FADE_DURATION_MS * UPDATE_INTERVAL_MS;
const float MAX_FADE_STEP_PER_INTERVAL = (float)MAX_BRIGHTNESS / MIN_FADE_DURATION_MS * UPDATE_INTERVAL_MS;

std::array<CRGB, CHANNEL_COUNT> fastLedBuffer;

std::array<unsigned long, CHANNEL_COUNT> lastFadeTriggerTimes = {0};
std::array<float, CHANNEL_COUNT> brightnesses = {0.0};  // [0, 255]

void IRAM_ATTR onFadeTrigger(void *param) {
    int channelIndex = reinterpret_cast<int>(param);
    lastFadeTriggerTimes[channelIndex] = millis();
}

float mapFloat(float value, float fromLow, float fromHigh, float toLow, float toHigh) {
    return (value - fromLow) * (toHigh - toLow) / (fromHigh - fromLow) + toLow;
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

    FastLED.addLeds<WS2812B, 13, GRB>(fastLedBuffer.data(), CHANNEL_COUNT);
    FastLED.showColor(CRGB::Black);

    while (true) {
        for (int channelIndex = 0; channelIndex < CHANNEL_COUNT; channelIndex++) {
            int fadeSpeed = analogRead(FADE_SPEED_PINS[channelIndex]);  // ADC value is between 0 and 4095
            float fadeStep = mapFloat(fadeSpeed, 0, 4095, MIN_FADE_STEP_PER_INTERVAL, MAX_FADE_STEP_PER_INTERVAL);
            float &currentBrightness = brightnesses[channelIndex];
            // Serial.printf("fadeSpeed: %d, fadeStep: %f, currentBrightness: %f\n", fadeSpeed, fadeStep,
            //               currentBrightness);

            if (digitalRead(FADE_TRIGGER_PINS[channelIndex]) != HIGH &&
                millis() - lastFadeTriggerTimes[channelIndex] > UPDATE_INTERVAL_MS) {
                fadeStep *= -1;
            }

            currentBrightness = constrain(currentBrightness + fadeStep, MIN_BRIGHTNESS, MAX_BRIGHTNESS);
        }

        std::transform(brightnesses.begin(), brightnesses.end(), fastLedBuffer.begin(),
                       [](float brightness) { return CRGB(brightness, 0, 0); });
        FastLED.show();

        PixelFrame frame(CHANNEL_COUNT);
        std::transform(brightnesses.begin(), brightnesses.end(), frame.begin(),
                       [](float brightness) { return Pixel(brightness, 0, 0); });
        // AcDimmer::write(frame);

        delay(UPDATE_INTERVAL_MS);
    }
}