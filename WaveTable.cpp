#include "daisy_patch_sm.h"
#include "daisysp.h"
#include "wtosc2.h"
#include "WavHeader.h"
#include "SampleVoice.h"
#include <fatfs.h>

using namespace daisy;
using namespace patch_sm;
using namespace daisysp;

DaisyPatchSM hw;
CpuLoadMeter loadMeter;

WTOSC2* wtOsc2;

// for loading WT from sd card
SdmmcHandler   sd;
FatFSInterface fsi;
FIL            SDFile;

#define sampleRate (48000)
#define numSecs (20.0)      /* length of sound file to generate (seconds) */
#define gainMult (0.5)      /* some control over sound file gain */
#define overSamp (2)        /* oversampling factor (positive integer) */
#define baseFrequency (20)  /* starting frequency of first table */
#define constantRatioLimit (99999)    /* set to a large number (greater than or equal to the length of the lowest octave table) for constant table size; set to 0 for a constant oversampling ratio (each higher ocatave table length reduced by half); set somewhere between (64, for instance) for constant oversampling but with a minimum limit */
#define myFloat double

// buffer is twice as big as expected WT size, incase of stereo?
int16_t DSY_SDRAM_BSS wt_sdram_buffer[1024 * 512];
int wt_row_count = 0;
float outputValue = 0;
float freq_with_offset = 0;

float pitchInput = 0;
float pitchFrequency = 0;
float pitchVoltage = 0;
float freq_offset = 0;
// pitch range for 0-5V input CV
const double C1_FREQ = 32.70319566;
const double C6_FREQ = 1046.50226112;

double voltageToFrequency(double voltage) {
    // Clamp input voltage between 0V and 5V
    if (voltage < 0) voltage = 0;
    if (voltage > 5) voltage = 5;
    
    const double baseFreq = C1_FREQ;
    
    // In the 1V/oct standard, each volt represents an octave (frequency doubling)
    // For 5V range, we cover exactly 5 octaves from C0 to C5
    // Calculate the frequency using the exponential relationship
    if (voltage <= 0.04) return C1_FREQ; // thresholding to prevent offset doing weird thingsB
    double frequency = baseFreq * pow(2, voltage + freq_offset);
    
    return frequency;
}

void AudioCallback(AudioHandle::InputBuffer in, AudioHandle::OutputBuffer out, size_t size)
{
    // performance monitoring start
    loadMeter.OnBlockStart();

	hw.ProcessAllControls();
	//float macroTune = hw.GetAdcValue(CV_1);
	//float freq = fmap(macroTune, 20.f, 15000.f, Mapping::LOG);

	float microTune = hw.GetAdcValue(CV_2);
	freq_offset = fmap(microTune, -0.08f, 0.08f, Mapping::LINEAR);
    	
	// constrain potential frequencies regardless
    #if 0
	freq_with_offset = freq + freq_offset;
	if (freq_with_offset < 20) freq_with_offset = 20;
	if (freq_with_offset > 15000) freq_with_offset = 15000;
    #endif

    float tableTune= hw.GetAdcValue(CV_3);
    int tableRowSet = (int)fmap(tableTune, 0, 256, Mapping::LINEAR);

    pitchInput= hw.GetAdcValue(CV_5);
    pitchVoltage = fmap(pitchInput, 0, 5, Mapping::LINEAR);
    pitchFrequency = voltageToFrequency(pitchVoltage);
    double freqVal = (pitchFrequency) / sampleRate;

    float wtIndex = hw.GetAdcValue(CV_6);
    wtIndex = fmap(wtIndex, 0, wt_row_count, Mapping::LINEAR);

	for (size_t i = 0; i < size; i++)
	{		
		wtOsc2->setCurrentRow(wtIndex);
		wtOsc2->setFrequency(freqVal);
        outputValue = wtOsc2->getOutput();      
		OUT_L[i] = outputValue;
		OUT_R[i] = outputValue;
		wtOsc2->updatePhase();
	}
    
    loadMeter.OnBlockEnd();
}

int main(void)
{    
	hw.Init();
    hw.StartLog(true);
    hw.SetAudioBlockSize(4); // number of samples handled per callback
	hw.SetAudioSampleRate(SaiHandle::Config::SampleRate::SAI_48KHZ);
    loadMeter.Init(hw.AudioSampleRate(), hw.AudioBlockSize());
    hw.PrintLine("Starting WaveTable Daisy Patch SM");

    wtOsc2 = new WTOSC2();
	// ----- Get the wavetable data from the SD card -----
    // Init SD Card
    SdmmcHandler::Config sd_cfg;
    sd_cfg.Defaults();
    //sd_cfg.speed = daisy::SdmmcHandler::Speed::SLOW;
    sd.Init(sd_cfg);
    // Links libdaisy i/o to fatfs driver.
    fsi.Init(FatFSInterface::Config::MEDIA_SD);
    // Mount SD Card
    f_mount(&fsi.GetSDFileSystem(), "/", 1);

    SampleVoice sv;
    sv.Init(wt_sdram_buffer, sizeof(wt_sdram_buffer) / sizeof(wt_sdram_buffer[0]));
    char filename[256];
    //strcpy(filename, fsi.GetSDPath());
    strcpy(filename, "wavetable.wav");
    sv.SetSample(filename);
    wt_row_count = sv.GetLength() / 1024;
    wtOsc2->setWaveTable(wt_row_count, wt_sdram_buffer);
    hw.PrintLine("Loaded wavetable with %d rows", wt_row_count);
    // ----- (end)  -----

    #if 0
    // Create an ADC Channel Config object
    AdcChannelConfig adc_config;
    // Set up the ADC config with a connection to pin A0
    adc_config.InitSingle(daisy::patch_sm::DaisyPatchSM::C6);
    // Initialize the ADC peripheral with that configuration
    hw.adc.Init(&adc_config, 1);
    // Start the ADC
    hw.adc.Start();
    #endif

    hw.PrintLine("Starting Audio Callback");
	hw.StartAudio(AudioCallback);
	while(1) {
        // get the current load (smoothed value and peak values)
        const float avgLoad = loadMeter.GetAvgCpuLoad();
        const float maxLoad = loadMeter.GetMaxCpuLoad();
        const float minLoad = loadMeter.GetMinCpuLoad();
        // print it to the serial connection (as percentages)
        hw.Print("Processing Load %:");
        hw.Print("  Max: " FLT_FMT3, FLT_VAR3(maxLoad * 100.0f));
        hw.Print("  Avg: " FLT_FMT3, FLT_VAR3(avgLoad * 100.0f));
        hw.PrintLine("  Min: " FLT_FMT3, FLT_VAR3(minLoad * 100.0f));

        {
            FixedCapStr<32> str("V: ");
            str.AppendFloat(pitchVoltage);
            hw.Print(str);
        }
        {
            FixedCapStr<32> str("   Pi: ");
            str.AppendFloat(pitchInput);
            hw.Print(str);
        }        
        {
            FixedCapStr<32> str("  F+: ");
            str.AppendFloat(pitchFrequency);
            hw.PrintLine(str);
        }
        // don't spam the serial connection too much
        System::Delay(1000);
    }
}
