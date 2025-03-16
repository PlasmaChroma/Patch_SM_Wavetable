#include "SampleVoice.h"

using namespace daisy;
using namespace patch_sm;
using namespace daisysp;

extern DaisyPatchSM hw;
extern FIL          SDFile;

/* adds given file to the buffer. Only supports 16bit PCM 48kHz. 
* If Stereo samples are interleaved left then right.
* return 0: succesful, 1: file read failed, 2: invalid format, 3: file too large
*/ 
int SampleVoice::SetSample(TCHAR *fname) {
    UINT bytesread;
    WavHeader_t wav_data; 

    memset(buffer, 0, bufferLength);
    
    if (f_open(&SDFile, fname, (FA_OPEN_EXISTING | FA_READ)) == FR_OK) {
        // Populate the WAV Info
        hw.PrintLine("Opened file, reading header");
        if (f_read(&SDFile, (void*)&wav_data, sizeof(WavHeader_t), &bytesread) != FR_OK)
        {
            hw.PrintLine("Failed to read WAV header (bytes:%d)", bytesread);
            return 1;
        }
    } else return 1;

    hw.PrintLine("Finished Reading WAV header (bytes:%d)", bytesread);
    hw.PrintLine("SampleRate = %d", wav_data.sampleRate);
    hw.PrintLine("Num Channels = %d", wav_data.numChannels);
    //if (wav_data.sampleRate != 48000 || wav_data.bitsPerSample != 16) return 2;
    //if (wav_data.dataSize > bufferLength || wav_data.numChannels > 2) return 3;
    stereo = wav_data.numChannels == 2;

    hw.PrintLine("Looking for payload of %d bytes", wav_data.dataSize);
    if (f_lseek(&SDFile, sizeof(WavHeader_t)) != FR_OK) return 1;
    if (f_read(&SDFile, buffer, wav_data.dataSize, &bytesread) != FR_OK) return 1;

    length = bytesread / (wav_data.bitsPerSample / 8);

    f_close(&SDFile);
    return 0;
}