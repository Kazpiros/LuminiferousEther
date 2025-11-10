#ifndef FILTER_COEF
#define FILTER_COEF

/*
    Moving Average Filter, 2^31, 2's complement
    between -1 and 1 

int movingAve_coeff[10] = 
{
    21014991,
    72226277,
    182313887,
    337989153,
    460197516,
    460197516,
    337989153,
    182313887,
    72226277,
    21014991
};
*/

// spi is 1MHz, ~31ksps
// returns the most recent & processed sample from the ring. 
/*int moving_average(cbuf* inBuffer, cbuf* outBuffer)
{
    int NUM_TAPS = 10;
    int temp = inBuffer->read_cbuf(0);
    for(int i = 0; i < NUM_TAPS; i++)
    {
        temp += inBuffer->read_cbuf(i+1)*movingAve_coeff[i];
    }
    outBuffer->write_cbuf(temp);
    return outBuffer->read_cbuf(0);
}

void downsample()
{
    // need phase accumilator here
}*/


#endif