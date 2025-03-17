#include "daisy_patch_sm.h"
#include "daisysp.h"
#include "WaveTableOsc.h"
#include "WavHeader.h"
#include "SampleVoice.h"
#include <fatfs.h>

using namespace daisy;
using namespace patch_sm;
using namespace daisysp;

DaisyPatchSM hw;
CpuLoadMeter loadMeter;

WaveTableOsc* wtOsc;

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

void fft(int N, myFloat *ar, myFloat *ai)
/* in-place complex fft */
{    
    int i, j, k, L;            /* indexes */
    int M, TEMP, LE, LE1, ip;  /* M = log N */
    int NV2, NM1;
    myFloat t;               /* temp */
    myFloat Ur, Ui, Wr, Wi, Tr, Ti;
    myFloat Ur_old;
    
    // if ((N > 1) && !(N & (N - 1)))   // make sure we have a power of 2
    
    NV2 = N >> 1;
    NM1 = N - 1;
    TEMP = N; /* get M = log N */
    M = 0;
    while (TEMP >>= 1) ++M;
    
    /* shuffle */
    j = 1;
    for (i = 1; i <= NM1; i++) {
        if(i<j) {             /* swap a[i] and a[j] */
            t = ar[j-1];     
            ar[j-1] = ar[i-1];
            ar[i-1] = t;
            t = ai[j-1];
            ai[j-1] = ai[i-1];
            ai[i-1] = t;
        }
        
        k = NV2;             /* bit-reversed counter */
        while(k < j) {
            j -= k;
            k /= 2;
        }
        
        j += k;
    }
    
    LE = 1.;
    for (L = 1; L <= M; L++) {            // stage L
        LE1 = LE;                         // (LE1 = LE/2) 
        LE *= 2;                          // (LE = 2^L)
        Ur = 1.0;
        Ui = 0.; 
        Wr = cos(M_PI/(float)LE1);
        Wi = -sin(M_PI/(float)LE1); // Cooley, Lewis, and Welch have "+" here
        for (j = 1; j <= LE1; j++) {
            for (i = j; i <= N; i += LE) { // butterfly
                ip = i+LE1;
                Tr = ar[ip-1] * Ur - ai[ip-1] * Ui;
                Ti = ar[ip-1] * Ui + ai[ip-1] * Ur;
                ar[ip-1] = ar[i-1] - Tr;
                ai[ip-1] = ai[i-1] - Ti;
                ar[i-1]  = ar[i-1] + Tr;
                ai[i-1]  = ai[i-1] + Ti;
            }
            Ur_old = Ur;
            Ur = Ur_old * Wr - Ui * Wi;
            Ui = Ur_old * Wi + Ui * Wr;
        }
    }
}

//
// if scale is 0, auto-scales
// returns scaling factor (0.0 if failure), and wavetable in ai array
//
float makeWaveTable(WaveTableOsc *osc, int len, myFloat *ar, myFloat *ai, myFloat scale, double topFreq) {
    fft(len, ar, ai);
    
    if (scale == 0.0) {
        // calc normal
        myFloat max = 0;
        for (int idx = 0; idx < len; idx++) {
            myFloat temp = fabs(ai[idx]);
            if (max < temp)
                max = temp;
        }
        scale = 1.0 / max * .999;        
    }
    
    // normalize
    float wave[len];
    for (int idx = 0; idx < len; idx++)
        wave[idx] = ai[idx] * scale;
        
    if (osc->addWaveTable(len, wave, topFreq))
        scale = 0.0;
    
    return scale;
}

void defineSawtooth(int len, int numHarmonics, myFloat *ar, myFloat *ai) {
    if (numHarmonics > (len >> 1))
        numHarmonics = (len >> 1);
    
    // clear
    for (int idx = 0; idx < len; idx++) {
        ai[idx] = 0;
        ar[idx] = 0;
    }

    // sawtooth
    for (int idx = 1, jdx = len - 1; idx <= numHarmonics; idx++, jdx--) {
        myFloat temp = -1.0 / idx;
        ar[idx] = -temp;
        ar[jdx] = temp;
    }
}

void setSawtoothOsc(WaveTableOsc *osc, float baseFreq) {    
    // calc number of harmonics where the highest harmonic baseFreq and lowest alias an octave higher would meet
    int maxHarms = sampleRate / (3.0 * baseFreq) + 0.5;

    // round up to nearest power of two
    unsigned int v = maxHarms;
    v--;            // so we don't go up if already a power of 2
    v |= v >> 1;    // roll the highest bit into all lower bits...
    v |= v >> 2;
    v |= v >> 4;
    v |= v >> 8;
    v |= v >> 16;
    v++;            // and increment to power of 2
    int tableLen = v * 2 * overSamp;  // double for the sample rate, then oversampling

    myFloat ar[tableLen], ai[tableLen];   // for ifft

    double topFreq = baseFreq * 2.0 / sampleRate;
    myFloat scale = 0.0;
    for (; maxHarms >= 1; maxHarms >>= 1) {
        defineSawtooth(tableLen, maxHarms, ar, ai);
        scale = makeWaveTable(osc, tableLen, ar, ai, scale, topFreq);
        topFreq *= 2;
        if (tableLen > constantRatioLimit) // variable table size (constant oversampling but with minimum table size)
            tableLen >>= 1;
    }
}

float outputValue = 0;
void AudioCallback(AudioHandle::InputBuffer in, AudioHandle::OutputBuffer out, size_t size)
{
    loadMeter.OnBlockStart();
	static int cycle = 0;

	hw.ProcessAllControls();
	float macroTune = hw.GetAdcValue(CV_1);
	float freq = fmap(macroTune, 20.f, 15000.f, Mapping::LOG);
#if 0    
	if (cycle % 10000 == 0) 
    {
        FixedCapStr<16> str("F: ");
        str.AppendFloat(freq);
        hw.PrintLine(str);
    }
#endif
	float microTune = hw.GetAdcValue(CV_2);
	float freq_offset = fmap(microTune, -500.f, 500.f, Mapping::LINEAR);
	
	// constrain potential frequencies regardless
	float freq_with_offset = freq + freq_offset;
	if (freq_with_offset < 20) freq_with_offset = 20;
	if (freq_with_offset > 15000) freq_with_offset = 15000;

    FixedCapStr<32> str("R: ");
	for (size_t i = 0; i < size; i++)
	{		
		double freqVal = freq_with_offset / sampleRate;

		wtOsc->setFrequency(freqVal);
        outputValue = wtOsc->getOutput();
#if 0    
        if (cycle % 10000 == 0) 
        {            
            str.AppendFloat(outputValue, 16, true, false);
            hw.PrintLine(str);
            hw.PrintLine(str);
        }
#endif        
		OUT_L[i] = outputValue;
		OUT_R[i] = outputValue;
		wtOsc->updatePhase();
	}
    
    cycle++;
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

	wtOsc = new WaveTableOsc();
	setSawtoothOsc(wtOsc, baseFrequency);    // ----- Get the wavetable data from the SD card -----
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
    // ----- (end)  -----

    for (int i = 0; i < 16; i++)
    {
        hw.PrintLine("wavetable[%d]:%d", i, wt_sdram_buffer[i]);
    }


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

        // get and print the output value
        FixedCapStr<16> str("O: ");
        str.AppendFloat(outputValue);
        hw.PrintLine(str);
        
        // don't spam the serial connection too much
        System::Delay(500);
    }
}
