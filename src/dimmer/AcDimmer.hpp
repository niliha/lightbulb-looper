#pragma once

#include "PixelFrame.hpp"

namespace AcDimmer {

void init(const int channelCount, const int zeroCrossingPin, const int triacTaskCore = 1);
void write(const std::vector<uint8_t> &channels);
void testLights();

}  // namespace AcDimmer
