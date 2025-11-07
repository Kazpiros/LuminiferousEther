#include <stdio.h>
#include "cbuf.hpp"

/* 
    Used for debugging, Stores data at samplerate S, in circular buffer length 1024,
    transmits serial at datarate U 
*/
class serial_digest
{
    private:
        cbuf cQueue;
        uint32_t SERIAL_BUFFER = 1024;
        
    public:
        serial_digest() : cQueue(SERIAL_BUFFER){}

        /*
            Place data into circular buffer to be read (from ADC)
            Run in a loop
        */
        void storeSerialQueue(uint32_t input_sample)
        {
            cQueue.write_cbuf(input_sample);
        }

        /*
            Reads and transmits data from the circular buffer to Serial
            Run once
        */
        void loadSerialQueue()
        {
            uint32_t temp;
            for(int i = 0; i < SERIAL_BUFFER; i++)
            {
                temp = (uint32_t)cQueue.read_cbuf(0);
                printf("0x%08X\n", temp);
            }
        }
};