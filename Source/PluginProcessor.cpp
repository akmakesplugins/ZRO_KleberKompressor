#include "PluginProcessor.h"

TheGlueProcessor::TheGlueProcessor()
    : AudioProcessor(BusesProperties()
          .withInput("Input", juce::AudioChannelSet::stereo(), true)
          .withOutput("Output", juce::AudioChannelSet::stereo(), true)),
      apvts(*this, nullptr, "PARAMETERS", createParameterLayout())
{
    ratioParam       = apvts.getRawParameterValue("ratio");
    attackParam      = apvts.getRawParameterValue("attack");
    releaseParam     = apvts.getRawParameterValue("release");
    releaseAutoParam = apvts.getRawParameterValue("releaseAuto");
    thresholdParam   = apvts.getRawParameterValue("threshold");
    rangeParam       = apvts.getRawParameterValue("range");
    makeupParam      = apvts.getRawParameterValue("makeup");
    mixParam         = apvts.getRawParameterValue("mix");
    scHpfParam       = apvts.getRawParameterValue("scHpf");
    peakClipParam    = apvts.getRawParameterValue("peakClip");
    stereoLinkParam  = apvts.getRawParameterValue("stereoLink");
    oversamplingParam = apvts.getRawParameterValue("oversampling");
}

TheGlueProcessor::~TheGlueProcessor() = default;

juce::AudioProcessorValueTreeState::ParameterLayout
TheGlueProcessor::createParameterLayout()
{
    juce::AudioProcessorValueTreeState::ParameterLayout layout;

    // Ratio: 3 discrete settings (2:1, 4:1, 10:1)
    layout.add(std::make_unique<juce::AudioParameterChoice>(
        juce::ParameterID{"ratio", 1},
        "Ratio",
        juce::StringArray{"2:1", "4:1", "10:1"},
        1));  // Default: 4:1

    // Attack: 7 discrete steps
    layout.add(std::make_unique<juce::AudioParameterChoice>(
        juce::ParameterID{"attack", 1},
        "Attack",
        juce::StringArray{"0.01 ms", "0.1 ms", "0.3 ms", "1 ms", "3 ms", "10 ms", "30 ms"},
        3));  // Default: 1ms

    // Release: 0.1 to 1.2 seconds (continuous)
    layout.add(std::make_unique<juce::AudioParameterFloat>(
        juce::ParameterID{"release", 1},
        "Release",
        juce::NormalisableRange<float>(0.1f, 1.2f, 0.01f),
        0.4f));

    // Auto Release
    layout.add(std::make_unique<juce::AudioParameterBool>(
        juce::ParameterID{"releaseAuto", 1},
        "Auto Release",
        false));

    // Threshold: -40 to 0 dB
    layout.add(std::make_unique<juce::AudioParameterFloat>(
        juce::ParameterID{"threshold", 1},
        "Threshold",
        juce::NormalisableRange<float>(-40.0f, 0.0f, 0.1f),
        0.0f));

    // Range: -82 to 0 dB (0 = no range limit)
    layout.add(std::make_unique<juce::AudioParameterFloat>(
        juce::ParameterID{"range", 1},
        "Range",
        juce::NormalisableRange<float>(-82.0f, 0.0f, 1.0f),
        -82.0f));

    // Makeup Gain: 0 to 40 dB
    layout.add(std::make_unique<juce::AudioParameterFloat>(
        juce::ParameterID{"makeup", 1},
        "Makeup",
        juce::NormalisableRange<float>(0.0f, 40.0f, 0.1f),
        0.0f));

    // Mix: 0 to 100%
    layout.add(std::make_unique<juce::AudioParameterFloat>(
        juce::ParameterID{"mix", 1},
        "Mix",
        juce::NormalisableRange<float>(0.0f, 100.0f, 0.1f),
        100.0f));

    // Sidechain HPF: 0 to 2000 Hz (0 = off)
    layout.add(std::make_unique<juce::AudioParameterFloat>(
        juce::ParameterID{"scHpf", 1},
        "SC HPF",
        juce::NormalisableRange<float>(0.0f, 2000.0f, 1.0f, 0.4f),
        0.0f));

    // Peak Clip
    layout.add(std::make_unique<juce::AudioParameterBool>(
        juce::ParameterID{"peakClip", 1},
        "Peak Clip",
        false));

    // Stereo Link: 0 to 100%
    layout.add(std::make_unique<juce::AudioParameterFloat>(
        juce::ParameterID{"stereoLink", 1},
        "Stereo Link",
        juce::NormalisableRange<float>(0.0f, 100.0f, 1.0f),
        100.0f));

    // Oversampling: 1x, 2x, 4x
    layout.add(std::make_unique<juce::AudioParameterChoice>(
        juce::ParameterID{"oversampling", 1},
        "Oversampling",
        juce::StringArray{"1x", "2x", "4x"},
        1));  // Default: 2x

    return layout;
}

void TheGlueProcessor::prepareToPlay(double sampleRate, int samplesPerBlock)
{
    // Create oversamplers: 1x (passthrough), 2x, 4x
    for (int i = 0; i < 3; ++i)
    {
        oversamplers[i] = std::make_unique<juce::dsp::Oversampling<double>>(
            2,  // stereo
            i,  // order: 0=1x, 1=2x, 2=4x
            juce::dsp::Oversampling<double>::filterHalfBandPolyphaseIIR);

        oversamplers[i]->initProcessing(static_cast<size_t>(samplesPerBlock));
    }

    currentOsIndex = std::clamp(static_cast<int>(oversamplingParam->load()), 0, 2);

    const double osRate = sampleRate * oversamplers[currentOsIndex]->getOversamplingFactor();
    compressor.prepare(osRate, 2);

    doubleBuffer.setSize(2, samplesPerBlock);

    setLatencySamples(static_cast<int>(oversamplers[currentOsIndex]->getLatencyInSamples()));
}

void TheGlueProcessor::releaseResources()
{
    compressor.reset();
}

bool TheGlueProcessor::isBusesLayoutSupported(const BusesLayout& layouts) const
{
    const auto& mainOutput = layouts.getMainOutputChannelSet();
    const auto& mainInput = layouts.getMainInputChannelSet();

    if (mainOutput != juce::AudioChannelSet::mono()
        && mainOutput != juce::AudioChannelSet::stereo())
        return false;

    return mainOutput == mainInput;
}

void TheGlueProcessor::processBlock(juce::AudioBuffer<float>& buffer,
                                     juce::MidiBuffer&)
{
    juce::ScopedNoDenormals noDenormals;

    const auto totalNumInputChannels = getTotalNumInputChannels();
    const auto totalNumOutputChannels = getTotalNumOutputChannels();

    for (auto i = totalNumInputChannels; i < totalNumOutputChannels; ++i)
        buffer.clear(i, 0, buffer.getNumSamples());

    if (buffer.getNumSamples() == 0)
        return;

    const int numSamples = buffer.getNumSamples();
    const int numChannels = std::min(buffer.getNumChannels(), 2);

    // Check if oversampling changed
    const int newOsIndex = std::clamp(static_cast<int>(oversamplingParam->load()), 0, 2);
    if (newOsIndex != currentOsIndex)
    {
        currentOsIndex = newOsIndex;
        const double osRate = getSampleRate() * oversamplers[currentOsIndex]->getOversamplingFactor();
        compressor.prepare(osRate, numChannels);
        setLatencySamples(static_cast<int>(oversamplers[currentOsIndex]->getLatencyInSamples()));
    }

    // Convert float to double
    doubleBuffer.setSize(numChannels, numSamples, false, false, true);
    for (int ch = 0; ch < numChannels; ++ch)
        for (int i = 0; i < numSamples; ++i)
            doubleBuffer.setSample(ch, i, static_cast<double>(buffer.getSample(ch, i)));

    // Build params from APVTS
    theglue::dsp::GlueCompressor::Parameters params;
    params.ratioIndex   = static_cast<int>(ratioParam->load());
    params.attackIndex  = static_cast<int>(attackParam->load());
    params.releaseSec   = static_cast<double>(releaseParam->load());
    params.releaseAuto  = releaseAutoParam->load() > 0.5f;
    params.thresholdDb  = static_cast<double>(thresholdParam->load());
    params.rangeDb      = static_cast<double>(rangeParam->load());
    params.makeupDb     = static_cast<double>(makeupParam->load());
    params.mix          = static_cast<double>(mixParam->load()) / 100.0;
    params.scHpfHz      = static_cast<double>(scHpfParam->load());
    params.peakClipOn   = peakClipParam->load() > 0.5f;
    params.stereoLink   = static_cast<double>(stereoLinkParam->load()) / 100.0;

    // Apply oversampling
    auto& oversampler = *oversamplers[currentOsIndex];
    auto osBlock = juce::dsp::AudioBlock<double>(doubleBuffer);

    if (currentOsIndex > 0)
    {
        auto osOutput = oversampler.processSamplesUp(osBlock);
        const int osSamples = static_cast<int>(osOutput.getNumSamples());

        // Process each sample through the compressor
        auto* leftPtr  = osOutput.getChannelPointer(0);
        auto* rightPtr = (numChannels > 1) ? osOutput.getChannelPointer(1) : leftPtr;

        for (int i = 0; i < osSamples; ++i)
        {
            double l = leftPtr[i];
            double r = rightPtr[i];
            compressor.processStereoSample(l, r, params);
            leftPtr[i] = l;
            if (numChannels > 1)
                rightPtr[i] = r;
        }

        oversampler.processSamplesDown(osBlock);
    }
    else
    {
        // No oversampling: process directly
        auto* leftPtr  = doubleBuffer.getWritePointer(0);
        auto* rightPtr = (numChannels > 1) ? doubleBuffer.getWritePointer(1) : leftPtr;

        for (int i = 0; i < numSamples; ++i)
        {
            double l = leftPtr[i];
            double r = rightPtr[i];
            compressor.processStereoSample(l, r, params);
            leftPtr[i] = l;
            if (numChannels > 1)
                rightPtr[i] = r;
        }
    }

    // Convert double back to float
    for (int ch = 0; ch < numChannels; ++ch)
        for (int i = 0; i < numSamples; ++i)
            buffer.setSample(ch, i, static_cast<float>(doubleBuffer.getSample(ch, i)));

    // Update GR meters (thread-safe)
    gainReductionDbL.store(static_cast<float>(compressor.getGainReductionDb(0)));
    gainReductionDbR.store(static_cast<float>(compressor.getGainReductionDb(1)));
}

void TheGlueProcessor::getStateInformation(juce::MemoryBlock& destData)
{
    auto state = apvts.copyState();
    auto xml = state.createXml();
    copyXmlToBinary(*xml, destData);
}

void TheGlueProcessor::setStateInformation(const void* data, int sizeInBytes)
{
    auto xml = getXmlFromBinary(data, sizeInBytes);
    if (xml && xml->hasTagName(apvts.state.getType()))
        apvts.replaceState(juce::ValueTree::fromXml(*xml));
}

const juce::String TheGlueProcessor::getName() const { return "The Glue"; }
bool TheGlueProcessor::acceptsMidi() const { return false; }
bool TheGlueProcessor::producesMidi() const { return false; }
double TheGlueProcessor::getTailLengthSeconds() const { return 0.0; }
int TheGlueProcessor::getNumPrograms() { return 1; }
int TheGlueProcessor::getCurrentProgram() { return 0; }
void TheGlueProcessor::setCurrentProgram(int) {}
const juce::String TheGlueProcessor::getProgramName(int) { return {}; }
void TheGlueProcessor::changeProgramName(int, const juce::String&) {}
bool TheGlueProcessor::hasEditor() const { return true; }

juce::AudioProcessorEditor* TheGlueProcessor::createEditor()
{
    return new juce::GenericAudioProcessorEditor(*this);
}

juce::AudioProcessor* JUCE_CALLTYPE createPluginFilter()
{
    return new TheGlueProcessor();
}
