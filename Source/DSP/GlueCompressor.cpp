#include "GlueCompressor.h"

namespace theglue {
namespace dsp {

void GlueCompressor::prepare(double sampleRate, int numChannels)
{
    sampleRate_ = sampleRate;
    numChannels_ = numChannels;

    envelope.prepare(sampleRate);
    scHpf.reset();

    // RMS smoothing coefficient: single-pole IIR
    const double rmsTimeSec = std::max(kRmsTimeConstantSec, 1e-6);
    rmsCoeff = std::exp(-1.0 / (rmsTimeSec * sampleRate));

    reset();
}

void GlueCompressor::reset()
{
    envelope.reset();
    scHpf.reset();

    for (int ch = 0; ch < 2; ++ch)
    {
        prevOutput[ch] = 0.0;
        currentGrDb[ch] = 0.0;
        rmsState[ch] = 0.0;
    }
}

void GlueCompressor::processStereoSample(double& left, double& right,
                                          const Parameters& params)
{
    // Clamp parameter indices
    const int ratioIdx  = std::clamp(params.ratioIndex, 0, 2);
    const int attackIdx = std::clamp(params.attackIndex, 0, 6);

    const double ratio    = kRatios[ratioIdx];
    const double kneeDb   = kKneeWidthDb[ratioIdx];
    const double attackMs = kAttackTimesMs[attackIdx];

    // Update envelope coefficients
    envelope.updateCoefficients(attackMs * 0.001, params.releaseSec);

    // Save dry signal for mix
    const double dryL = left;
    const double dryR = right;

    // ---------------------------------------------------------------
    // FEEDBACK DETECTION: use previous output, not input
    // This is the defining characteristic of the SSL bus comp topology.
    // The compressor monitors its own output, creating a self-regulating
    // feedback loop that produces smooth, transparent compression.
    // ---------------------------------------------------------------
    double detL = prevOutput[0];
    double detR = prevOutput[1];

    // ---------------------------------------------------------------
    // SIDECHAIN HPF: -6dB/oct, reduces low-frequency pumping
    // ---------------------------------------------------------------
    if (params.scHpfHz > 1.0)
    {
        detL = scHpf.processSample(0, detL, params.scHpfHz, sampleRate_);
        detR = scHpf.processSample(1, detR, params.scHpfHz, sampleRate_);
    }

    // ---------------------------------------------------------------
    // STEREO LINK: blend between dual-mono and fully linked detection
    // link=0: each channel uses its own detection (dual mono)
    // link=1: both channels use the max of L/R (preserves stereo image)
    // ---------------------------------------------------------------
    const double absL = std::abs(detL);
    const double absR = std::abs(detR);
    const double linked = std::max(absL, absR);

    const double detMonoL = absL + params.stereoLink * (linked - absL);
    const double detMonoR = absR + params.stereoLink * (linked - absR);

    // ---------------------------------------------------------------
    // RMS SMOOTHING: single-pole IIR (~300us)
    // Smooths the rectified detection signal for more musical response
    // ---------------------------------------------------------------
    rmsState[0] = rmsCoeff * rmsState[0] + (1.0 - rmsCoeff) * (detMonoL * detMonoL);
    rmsState[1] = rmsCoeff * rmsState[1] + (1.0 - rmsCoeff) * (detMonoR * detMonoR);

    const double rmsDbL = toDb(std::sqrt(std::max(rmsState[0], 1e-40)));
    const double rmsDbR = toDb(std::sqrt(std::max(rmsState[1], 1e-40)));

    // ---------------------------------------------------------------
    // GAIN COMPUTER: threshold + ratio + ratio-dependent knee
    // ---------------------------------------------------------------
    const double targetGrL = gainComputer(rmsDbL, params.thresholdDb, ratio, kneeDb);
    const double targetGrR = gainComputer(rmsDbR, params.thresholdDb, ratio, kneeDb);

    // ---------------------------------------------------------------
    // DUAL-DIODE ENVELOPE FOLLOWER: the critical analog-modeled component
    // Newton-Raphson solved per sample for accurate diode behavior
    // ---------------------------------------------------------------
    const double grL = envelope.processSample(0, targetGrL, params.releaseAuto);
    const double grR = envelope.processSample(1, targetGrR, params.releaseAuto);

    // ---------------------------------------------------------------
    // RANGE LIMITER: clamp max gain reduction
    // ---------------------------------------------------------------
    const double clampedGrL = applyRange(grL, params.rangeDb);
    const double clampedGrR = applyRange(grR, params.rangeDb);

    // Store for metering
    currentGrDb[0] = clampedGrL;
    currentGrDb[1] = clampedGrR;

    // ---------------------------------------------------------------
    // LINEAR VCA + MAKEUP GAIN
    // Perfect (linear) VCA — no VCA nonlinearity modeling
    // ---------------------------------------------------------------
    const double gainL = fromDb(-clampedGrL + params.makeupDb);
    const double gainR = fromDb(-clampedGrR + params.makeupDb);

    double wetL = left * gainL;
    double wetR = right * gainR;

    // Store output for feedback detection on next sample
    prevOutput[0] = wetL;
    prevOutput[1] = wetR;

    // ---------------------------------------------------------------
    // PEAK CLIPPER: optional zero-attack soft clipping
    // ---------------------------------------------------------------
    if (params.peakClipOn)
    {
        wetL = PeakClipper::processSample(wetL);
        wetR = PeakClipper::processSample(wetR);
    }

    // ---------------------------------------------------------------
    // DRY/WET MIX: parallel compression
    // ---------------------------------------------------------------
    left  = dryL + params.mix * (wetL - dryL);
    right = dryR + params.mix * (wetR - dryR);
}

double GlueCompressor::gainComputer(double inputDb, double thresholdDb,
                                     double ratio, double kneeWidthDb)
{
    const double halfKnee = kneeWidthDb * 0.5;
    const double overDb = inputDb - thresholdDb;

    if (overDb <= -halfKnee)
        return 0.0;  // Below knee: no compression

    if (overDb >= halfKnee)
        return overDb * (1.0 - 1.0 / ratio);  // Above knee: full ratio

    // Within knee: quadratic interpolation for smooth transition
    const double x = overDb + halfKnee;
    return (1.0 - 1.0 / ratio) * x * x / (2.0 * kneeWidthDb);
}

double GlueCompressor::applyRange(double grDb, double rangeDb)
{
    // rangeDb is negative (e.g., -82 = full range, 0 = no compression allowed)
    // Convert: if range is -82, max GR is 82 dB
    // if range is 0, max GR is 0 dB (no compression)
    const double maxGr = std::abs(rangeDb);
    return std::min(grDb, maxGr);
}

} // namespace dsp
} // namespace theglue
