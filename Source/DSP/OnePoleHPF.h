#pragma once

#include <cmath>

namespace theglue {
namespace dsp {

// Trapezoidal-integrated one-pole high-pass filter (-6dB/oct).
// Used for the sidechain HPF in the SSL bus compressor topology.
// Pure C++17, no JUCE dependency.

struct OnePoleHPF
{
    double s[2] = {};  // Integrator state per channel

    void reset()
    {
        s[0] = s[1] = 0.0;
    }

    // Process one sample. Returns high-passed signal.
    // cutoffHz: 0 = bypass (returns input unchanged)
    // sr: sample rate (accounting for any oversampling)
    [[nodiscard]] double processSample(int channel, double input,
                                       double cutoffHz, double sr)
    {
        if (cutoffHz < 1.0)
            return input;

        // Trapezoidal integration: g = tan(pi * fc / fs)
        const double g = std::tan(M_PI * std::min(cutoffHz, sr * 0.499) / sr);

        // One-pole lowpass tick
        const double v = (input - s[channel]) * g / (1.0 + g);
        const double lp = s[channel] + v;

        // Trapezoidal state update: s = 2*lp - s
        s[channel] = lp + v;

        // High-pass = input - lowpass
        return input - lp;
    }
};

} // namespace dsp
} // namespace theglue
