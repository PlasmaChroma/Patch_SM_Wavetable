#pragma once

#include <stdint.h>

class WTOSC2 {
protected:
    double phasor;      // phase accumulator
    double phaseInc;    // phase increment
    double phaseOfs;    // phase offset for PWM
    
    // list of wavetables
    int waveTableRows;
    int currentRow;
    int16_t* waveTable;
    
public:
    WTOSC2(void);
    ~WTOSC2(void);
    void setFrequency(double inc);
    void setCurrentRow(int row);
    void setPhaseOffset(double offset);
    void updatePhase(void);
    float getOutput(void);
    //float getOutputMinusOffset(void);
    int setWaveTable(int rows, int16_t* waveTableIn);
};


inline void WTOSC2::setCurrentRow(int row) {
    if (row < waveTableRows)
    {
        currentRow = row;
    }
}

// note: if you don't keep this in the range of 0-1, you'll need to make changes elsewhere
inline void WTOSC2::setFrequency(double inc) {
    phaseInc = inc;
}

// note: if you don't keep this in the range of 0-1, you'll need to make changes elsewhere
inline void WTOSC2::setPhaseOffset(double offset) {
    phaseOfs = offset;
}

inline void WTOSC2::updatePhase() {
    phasor += phaseInc;
    
    if (phasor >= 1.0)
        phasor -= 1.0;
}
