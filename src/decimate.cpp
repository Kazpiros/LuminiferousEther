#include "include/decimate.hpp"
    
int Decimator::decimate(int* inputBuffer, int inputLength, int* outputBuffer) {
    int outputIndex = 0;
    
    for (int i = 0; i < inputLength; i++) {
        // Write each input sample to circular buffer
        circularBuffer->write_cbuf(inputBuffer[i]);
        sampleCounter++;
        
        // Check if we should output a sample
        if (sampleCounter >= decimationFactor) {
            // Read the most recent sample (Xn=1 means 1 sample back)
            outputBuffer[outputIndex++] = circularBuffer->read_cbuf(1);
            sampleCounter = 0;
        }
    }
    
    return outputIndex;
}

int Decimator::getDecimationFactor() const {
    return decimationFactor;
}

// Example usage:
/*
int main() {
    Decimator decimator(64);  // Power of 2 buffer size
    
    int inputBuffer[1000];
    int outputBuffer[1000];
    
    // Fill input buffer with data...
    
    int numOutputSamples = decimator.decimate(inputBuffer, 1000, outputBuffer);
    
    printf("Decimated %d input samples to %d output samples\n", 1000, numOutputSamples);
    
    return 0;
}
*/