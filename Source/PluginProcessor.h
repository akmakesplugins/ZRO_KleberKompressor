#pragma once

#include <JuceHeader.h>
#include "DSP/GlueCompressor.h"

class TheGlueProcessor : public juce::AudioProcessor
{
public:
    TheGlueProcessor();
    ~TheGlueProcessor() override;

    void prepareToPlay(double sampleRate, int samplesPerBlock) override;
    void releaseResources() override;
    bool isBusesLayoutSupported(const BusesLayout& layouts) const override;
    void processBlock(juce::AudioBuffer<float>&, juce::MidiBuffer&) override;

    juce::AudioProcessorEditor* createEditor() override;
    bool hasEditor() const override;

    const juce::String getName() const override;
    bool acceptsMidi() const override;
    bool producesMidi() const override;
    double getTailLengthSeconds() const override;

    int getNumPrograms() override;
    int getCurrentProgram() override;
    void setCurrentProgram(int index) override;
    const juce::String getProgramName(int index) override;
    void changeProgramName(int index, const juce::String& newName) override;

    void getStateInformation(juce::MemoryBlock& destData) override;
    void setStateInformation(const void* data, int sizeInBytes) override;

    juce::AudioProcessorValueTreeState apvts;

    // Thread-safe GR metering for UI
    std::atomic<float> gainReductionDbL { 0.0f };
    std::atomic<float> gainReductionDbR { 0.0f };

    static juce::AudioProcessorValueTreeState::ParameterLayout createParameterLayout();

private:
    theglue::dsp::GlueCompressor compressor;

    // Oversampling: 1x, 2x, 4x
    std::unique_ptr<juce::dsp::Oversampling<double>> oversamplers[3];
    int currentOsIndex = 0;

    // Double-precision working buffer
    juce::AudioBuffer<double> doubleBuffer;

    // Cached raw parameter pointers
    std::atomic<float>* ratioParam       = nullptr;
    std::atomic<float>* attackParam      = nullptr;
    std::atomic<float>* releaseParam     = nullptr;
    std::atomic<float>* releaseAutoParam = nullptr;
    std::atomic<float>* thresholdParam   = nullptr;
    std::atomic<float>* rangeParam       = nullptr;
    std::atomic<float>* makeupParam      = nullptr;
    std::atomic<float>* mixParam         = nullptr;
    std::atomic<float>* scHpfParam       = nullptr;
    std::atomic<float>* peakClipParam    = nullptr;
    std::atomic<float>* stereoLinkParam  = nullptr;
    std::atomic<float>* oversamplingParam = nullptr;

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR(TheGlueProcessor)
};
