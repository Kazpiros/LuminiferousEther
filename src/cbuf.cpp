#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#include "include/cbuf.hpp"


void cbuf::write_cbuf(int val)
{
    tbuf->buffer[(tbuf->writeIndex++) & BUFFER_SIZE_MASK] = val;
}

int cbuf::read_cbuf(unsigned Xn)
{
    return tbuf->buffer[(tbuf->writeIndex - Xn) & BUFFER_SIZE_MASK];
}

// maybe implement that '=' operator on these classes?
//make class to override cbuf_init to also specify a size
void cbuf::init()
{
    tbuf = new circular_buf;
    tbuf->buffer = new int[BUFFER_SIZE];
    for (unsigned int i = 0; i < BUFFER_SIZE; i++) {
        tbuf->buffer[i] = 0;
        }
        tbuf->writeIndex = 0;
        tbuf->readIndex = 0;
}