#pragma once

#include <cmath>
#include <algorithm>
#include <atomic>
#include "DualDiodeEnvelope.h"
#include "OnePoleHPF.h"
#include "PeakClipper.h"

namespace theglue {
namespace dsp {

// Complete SSL 4000 E-series bus compressor engine.
// Feedback VCA topology with analog-modeled dual-diode envelope follower.
//
// Pure C++17, no JUCE dependency. All processing at double precision.
// Designed for per-sample processing (call processSample per sample per channel pair).

struct GlueCompressor
{
    // Exact parameter values matching The Glue
    static constexpr double kAttackTimesMs[7] = {
        0.01, 0.1, 0.3, 1.0, 3.0, 10.0, 30.0
    };

    static constexpr double kRatios[3] = { 2.0, 4.0, 10.0 };

    // Knee width per ratio: softer at 2:1, harder at 10:1
    static constexpr double kKneeWidthDb[3] = { 12.0, 6.0, 1.0 };

    // RMS smoothing time constant (~300us for SSL-style detection)
    static constexpr double kRmsTimeConstantSec = 0.0003;

    struct Parameters
    {
        int    ratioIndex   = 1;       // 0=2:1, 1=4:1, 2=10:1
        int    attackIndex  = 3;       // 0-6, index into kAttackTimesMs
        double releaseSec   = 0.4;     // 0.1 to 1.2
        bool   releaseAuto  = false;   // Program-dependent release
        double thresholdDb  = 0.0;     // -40 to 0
        double rangeDb      = -82.0;   // -82 to 0 (0 = no range limit)
        double makeupDb     = 0.0;     // 0 to 40
        double mix          = 1.0;     // 0 to 1 (dry/wet)
        double scHpfHz      = 0.0;     // 0 (off) to 2000
        bool   peakClipOn   = false;
        double stereoLink   = 1.0;    // 0 = dual mono, 1 = linked
    };

    void prepare(double sampleRate, int numChannels);
    void reset();

    // Process a stereo pair of samples. Handles the full signal flow:
    // feedback detection -> sidechain HPF -> stereo link -> RMS ->
    // gain computer -> diode envelope -> range -> VCA -> makeup -> peak clip
    void processStereoSample(double& left, double& right, const Parameters& params);

    // Get current gain reduction for metering (positive dB)
    double getGainReductionDb(int channel) const { return currentGrDb[channel]; }

private:
    DualDiodeEnvelope envelope;
    OnePoleHPF        scHpf;

    double prevOutput[2]  = {};   // Feedback detection state
    double currentGrDb[2] = {};   // Current gain reduction for metering
    double rmsState[2]    = {};   // RMS detector state
    double rmsCoeff       = 0.0;  // RMS smoothing coefficient
    double sampleRate_    = 44100.0;
    int    numChannels_   = 2;

    // Gain computer: threshold + ratio + ratio-dependent knee
    static double gainComputer(double inputDb, double thresholdDb,
                               double ratio, double kneeWidthDb);

    // Clamp GR to range limit
    static double applyRange(double grDb, double rangeDb);

    // Convert gain to dB (safe log)
    static double toDb(double gain)
    {
        return 20.0 * std::log10(std::max(gain, 1e-20));
    }

    // Convert dB to linear gain
    static double fromDb(double db)
    {
        return std::pow(10.0, db / 20.0);
    }
};

} // namespace dsp
} // namespace theglue
