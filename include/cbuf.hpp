#ifndef CIRCULAR_BUFFER_HPP
#define CIRCULAR_BUFFER_HPP

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

//#define BUFFER_SIZE 16U
//#define BUFFER_SIZE_MASK (BUFFER_SIZE-1U)
class cbuf
{
    private:
        uint32_t BUFFER_SIZE;
        uint32_t BUFFER_SIZE_MASK = (BUFFER_SIZE-1U);
        
        typedef struct circular_buf{
            int16_t *buffer; //dynamic allocation, should only run once..
            int readIndex;
            int writeIndex;
        } circular_buf;

        circular_buf* tbuf;

        void init();
   
    public:
        cbuf(uint32_t buffer_size)
        {
            BUFFER_SIZE = buffer_size;
            init();
        }

        ~cbuf(void)
        {
            delete[] tbuf->buffer;
            delete tbuf;
        } // make a proper destructor

        void write_cbuf(int16_t val);

        int16_t read_cbuf(unsigned Xn);
};
#endif