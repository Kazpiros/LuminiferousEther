#ifndef CIRCULAR_BUFFER_HPP
#define CIRCULAR_BUFFER_HPP

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

class cbuf
{
    private:
        int BUFFER_SIZE;
        int BUFFER_SIZE_MASK;  // Don't initialize here!
        
        typedef struct circular_buf{
            int *buffer; 
            int readIndex;
            int writeIndex;
        } circular_buf;

        circular_buf* tbuf;

        void init();
   
    public:
        cbuf(int buffer_size)
        {
            BUFFER_SIZE = buffer_size;
            BUFFER_SIZE_MASK = (BUFFER_SIZE - 1);  // Set it here in constructor
            init();
        }

        ~cbuf(void)
        {
            delete[] tbuf->buffer;
            delete tbuf;
        }

        void write_cbuf(int val);

        int read_cbuf(unsigned Xn);
};
#endif