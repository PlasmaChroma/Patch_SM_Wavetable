#include "wtosc2.h"

WTOSC2::WTOSC2(void) {
    phasor = 0.0;
    phaseInc = 0.0;
    phaseOfs = 0.5;
    currentRow = 0;
}


WTOSC2::~WTOSC2(void) {
    // nothing to do
}


//
// addWaveTable
//
int WTOSC2::setWaveTable(int rows, int16_t* waveTableIn) {
    waveTable = waveTableIn;
    waveTableRows = rows;
    return 0;
}


//
// getOutput
//
// returns the current oscillator output
//
float WTOSC2::getOutput()
{
    // linear interpolation
    double temp = this->phasor * 1024;
    int intPart = temp;
    double fracPart = temp - intPart;
    float samp0 = waveTable[currentRow * 1024 + intPart] / 32768.0;
    if (++intPart >= 1024)
        intPart = 0;
    float samp1 = waveTable[currentRow * 1024 + intPart] / 32768.0;
    
    return samp0 + (samp1 - samp0) * fracPart;
}

#if 0
//
// getOutputMinusOffset
//
// for variable pulse width: initialize to sawtooth,
// set phaseOfs to duty cycle, use this for osc output
//
// returns the current oscillator output
//
float WTOSC2::getOutputMinusOffset() {
    // grab the appropriate wavetable
    int waveTableIdx = 0;
    while ((this->phaseInc >= this->waveTables[waveTableIdx].topFreq) && (waveTableIdx < (this->numWaveTables - 1))) {
        ++waveTableIdx;
    }
    waveTable *waveTable = &this->waveTables[waveTableIdx];
    
    // linear
    double temp = this->phasor * waveTable->waveTableLen;
    int intPart = temp;
    double fracPart = temp - intPart;
    float samp0 = waveTable->waveTable[intPart];
    if (++intPart >= waveTable->waveTableLen)
        intPart = 0;
    float samp1 = waveTable->waveTable[intPart];
    float samp = samp0 + (samp1 - samp0) * fracPart;
    
    // and linear again for the offset part
    double offsetPhasor = this->phasor + this->phaseOfs;
    if (offsetPhasor > 1.0)
        offsetPhasor -= 1.0;
    temp = offsetPhasor * waveTable->waveTableLen;
    intPart = temp;
    fracPart = temp - intPart;
    samp0 = waveTable->waveTable[intPart];
    if (++intPart >= waveTable->waveTableLen)
        intPart = 0;
    samp1 = waveTable->waveTable[intPart];
    
    return samp - (samp0 + (samp1 - samp0) * fracPart);
}
#endif