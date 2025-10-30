
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#include "cbuf.hpp"


void cbuf::write_cbuf(int16_t val)
{
    tbuf->buffer[(++tbuf->writeIndex) & BUFFER_SIZE_MASK] = val;
}

int16_t cbuf::read_cbuf(unsigned Xn)
{
    return tbuf->buffer[(tbuf->writeIndex - Xn) & BUFFER_SIZE_MASK];
}

// maybe implement that '=' operator on these classes?
//make class to override cbuf_init to also specify a size
void cbuf::init()
{
    tbuf = new circular_buf;
    tbuf->buffer = new int16_t[BUFFER_SIZE];
    for (unsigned int i = 0; i < BUFFER_SIZE; i++) {
        tbuf->buffer[i] = 0;
        }
        tbuf->writeIndex = 0;
        tbuf->readIndex = 0;
}