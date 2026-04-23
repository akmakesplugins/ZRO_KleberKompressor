#pragma once

#include <cmath>
#include <algorithm>

namespace theglue {
namespace dsp {

// Analog-modeled dual-diode envelope follower for the SSL 4000 bus compressor.
//
// Models the diode-capacitor envelope detector circuit using trapezoidal
// integration and Newton-Raphson iteration, following the Cytomic methodology.
//
// The diode creates nonlinear charge/discharge behavior:
// - Fast charging (attack): diode conducts when input > envelope
// - Slow discharging (release): through resistor when input < envelope
// - The diode's exponential I-V curve creates smooth, musical transitions
//
// Pure C++17, no JUCE dependency. Double precision throughout.

struct DualDiodeEnvelope
{
    // 1N4148 silicon signal diode parameters
    static constexpr double kIs   = 2.52e-9;    // Saturation current (A)
    static constexpr double kVt   = 0.02585;    // Thermal voltage at 20C (V)
    static constexpr double kN    = 1.752;      // Ideality factor
    static constexpr double kNVt  = kN * kVt;   // ~0.04529

    static constexpr int    kMaxIter  = 8;       // Newton-Raphson max iterations
    static constexpr double kTol      = 1e-7;    // Convergence tolerance
    static constexpr double kExpClamp = 50.0;    // Clamp exp() argument

    // Auto-release dual time constants
    static constexpr double kAutoReleaseFastSec = 0.1;   // 100ms fast
    static constexpr double kAutoReleaseSlowSec = 1.0;   // 1000ms slow
    static constexpr double kAutoReleaseBlendDepthDb = 6.0; // Full blend at 6dB GR

    // Per-channel state (max 2 channels)
    double state[2]  = {};    // Trapezoidal integrator state
    double envOut[2] = {};    // Current envelope output (dB of GR, positive)

    // Precomputed conductances
    double gAtk = 0.0;
    double gRel = 0.0;

    // Auto-release conductances
    double gRelFast = 0.0;
    double gRelSlow = 0.0;

    double sampleRate = 44100.0;

    void prepare(double sr)
    {
        sampleRate = sr;
        reset();
    }

    void reset()
    {
        state[0] = state[1] = 0.0;
        envOut[0] = envOut[1] = 0.0;
    }

    // Update conductances from time constants.
    // attackSec: attack time in seconds
    // releaseSec: release time in seconds (used when autoRelease is off)
    void updateCoefficients(double attackSec, double releaseSec)
    {
        // Trapezoidal integration: G = 2 / (tau * fs)
        // where tau is the RC time constant
        gAtk = 2.0 / (std::max(attackSec, 1e-6) * sampleRate);
        gRel = 2.0 / (std::max(releaseSec, 1e-4) * sampleRate);

        // Auto-release conductances (always precomputed)
        gRelFast = 2.0 / (kAutoReleaseFastSec * sampleRate);
        gRelSlow = 2.0 / (kAutoReleaseSlowSec * sampleRate);
    }

    // Process one sample through the analog diode envelope model.
    // targetGrDb: target gain reduction from gain computer (positive = more reduction)
    // autoRelease: if true, uses program-dependent dual time constant
    // Returns: smoothed gain reduction in dB (positive)
    [[nodiscard]] double processSample(int channel, double targetGrDb, bool autoRelease)
    {
        const double x = targetGrDb;  // Input to detector
        double v = envOut[channel];   // Initial guess = previous output
        const double s = state[channel];

        // Select conductance based on direction and auto-release mode
        double g;
        if (x > v)
        {
            // Attack: input wants more compression than current envelope
            g = gAtk;
        }
        else
        {
            // Release: input wants less compression
            if (autoRelease)
            {
                // Program-dependent: blend fast/slow based on compression depth
                const double depth = std::abs(envOut[channel]);
                const double blend = std::min(depth / kAutoReleaseBlendDepthDb, 1.0);
                g = gRelFast + blend * (gRelSlow - gRelFast);
            }
            else
            {
                g = gRel;
            }
        }

        // Newton-Raphson to solve: F(v) = G*(v - s) + Is*(exp((x - v)/(n*Vt)) - 1) = 0
        //
        // The nodal equation represents KCL at the envelope node:
        //   - G*(v - s): current through the capacitor (trapezoidal discretization)
        //   - Is*(exp(...) - 1): current through the diode (Shockley equation)
        //
        // The diode conducts when x > v (charging the capacitor = attack).
        // When x < v, the diode is reverse-biased and only leakage current flows,
        // so the capacitor discharges slowly through the conductance (release).

        for (int iter = 0; iter < kMaxIter; ++iter)
        {
            const double diff = (x - v) / kNVt;
            const double clampedDiff = std::clamp(diff, -kExpClamp, kExpClamp);
            const double expVal = std::exp(clampedDiff);

            const double fv = g * (v - s) + kIs * (expVal - 1.0);
            const double fpv = g + (kIs / kNVt) * expVal;

            const double delta = fv / fpv;
            v -= delta;

            if (std::abs(delta) < kTol)
                break;
        }

        // Update trapezoidal state: s_new = 2*v - s_old
        state[channel] = 2.0 * v - s;
        envOut[channel] = v;

        return std::max(v, 0.0);  // GR is always non-negative
    }
};

} // namespace dsp
} // namespace theglue
