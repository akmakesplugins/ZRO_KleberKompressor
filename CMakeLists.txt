cmake_minimum_required(VERSION 3.22)
project(TheGlue VERSION 1.0.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_subdirectory(JUCE)

juce_add_plugin(TheGlue
    PLUGIN_MANUFACTURER_CODE Pitc
    PLUGIN_CODE TgLu
    FORMATS VST3 AU Standalone
    PRODUCT_NAME "The Glue"
    COMPANY_NAME "Pitch Innovations"
    BUNDLE_ID "com.pitchinnovations.theglue"
    IS_SYNTH FALSE
    NEEDS_MIDI_INPUT FALSE
    NEEDS_MIDI_OUTPUT FALSE
    EDITOR_WANTS_KEYBOARD_FOCUS FALSE
    COPY_PLUGIN_AFTER_BUILD TRUE)

juce_generate_juce_header(TheGlue)

target_sources(TheGlue PRIVATE
    Source/PluginProcessor.cpp
    Source/PluginEditor.cpp
    Source/DSP/GlueCompressor.cpp)

target_include_directories(TheGlue PRIVATE Source)

target_compile_definitions(TheGlue PUBLIC
    JUCE_WEB_BROWSER=0
    JUCE_USE_CURL=0
    JUCE_VST3_CAN_REPLACE_VST2=0
    JUCE_DISPLAY_SPLASH_SCREEN=0)

target_link_libraries(TheGlue
    PRIVATE
        juce::juce_audio_utils
        juce::juce_dsp
    PUBLIC
        juce::juce_recommended_config_flags
        juce::juce_recommended_lto_flags
        juce::juce_recommended_warning_flags)
