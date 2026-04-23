#pragma once

#include <cmath>

namespace theglue {
namespace dsp {

// Zero-attack soft clipper (peak clip stage).
// Models a fixed waveshaper with infinite ratio and medium knee.
// Pure C++17, no JUCE dependency.

struct PeakClipper
{
    // Soft clip using tanh. The ceiling is at ~0 dBFS.
    // Input beyond +/- 1.0 is smoothly limited.
    [[nodiscard]] static double processSample(double input)
    {
        return std::tanh(input);
    }
};

} // namespace dsp
} // namespace theglue
