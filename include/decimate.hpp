#ifndef DECIMATE_HPP
#define DECIMATE_HPP

#include "include/cbuf.hpp"
#include <vector>
#include "include/le_defs.h"
// Your system defines
// #define SAMPLE_RATE 48000
// #define SYS_CLK 100000000
// #define OUTPUT_RATE 1000

class Decimator {
private:
    cbuf* circularBuffer;
    int decimationFactor;
    int sampleCounter;
    
public:
    Decimator(int bufferSize) {
        // Calculate decimation factor
        decimationFactor = SAMPLE_RATE / OUTPUT_RATE;
        sampleCounter = 0;
        
        // Create circular buffer (must be power of 2)
        circularBuffer = new cbuf(bufferSize);
    }
    
    ~Decimator() {
        delete circularBuffer;
    }
    
    // Process input buffer and write decimated samples to output buffer
    // Returns number of output samples written
    int decimate(int* inputBuffer, int inputLength, int* outputBuffer);

    int getDecimationFactor() const;
};
#endif