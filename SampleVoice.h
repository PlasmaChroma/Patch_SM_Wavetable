#pragma once

#include <stdio.h>
#include <string.h>
#include <stdio.h>
#include <string.h>
#include <fatfs.h>
#include "daisy_patch_sm.h"
#include "daisysp.h"
#include "WavHeader.h"

class SampleVoice {
    private:
        uint32_t bufferLength;
        size_t length;
        size_t position;
        int16_t *buffer;
        bool stereo;

    public:
        // buffer should be predefined in SDRAM
        void Init(int16_t *buffer, uint32_t bufferLength) {
            this->buffer = buffer;
            this->bufferLength = bufferLength;
            length = 0;
            position = 0;
        }

        size_t GetLength(void) { return length; }
        void SetLength(size_t length) {this->length = length;}
        void *GetBuffer() {return (void *)buffer;}
        uint32_t GetBufferLength() {return bufferLength;}
        bool IsStereo() {return stereo;}
        bool IsMono() {return !stereo;}
        int SetSample(TCHAR *fname);
};