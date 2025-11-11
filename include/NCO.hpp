#ifndef NCO_HPP
#define NCO_HPP


#include "include/le_defs.h"

#define M_PI 3.14159

//https://zipcpu.com/dsp/2017/12/09/nco.html
class NCO{
    public:
    unsigned m_lglen, m_len, m_mask, m_phase, m_dphase;
	uint *m_table ;
// 128 in length
	
	NCO() {
		const int lgtblsize = 7;
		// We'll use a table 2^(lgtblize) in length.  This is
		// non-negotiable, as the rest of this algorithm depends upon
		// this property.
		m_lglen = lgtblsize;
		m_len = (1<<lgtblsize);

		// m_mask is 1 for any bit used in the index, zero otherwise
		m_mask = m_len - 1;

        m_table = new uint[m_len];
		static const uint m_table[] = {
			514, 539, 564, 589, 614, 639, 663, 687,
			711, 734, 756, 778, 800, 820, 840, 859,
			877, 895, 911, 927, 941, 955, 967, 979,
			989, 998, 1006, 1013, 1018, 1022, 1026, 1027,
			1028, 1027, 1026, 1022, 1018, 1013, 1006, 998,
			989, 979, 967, 955, 941, 927, 911, 895,
			877, 859, 840, 820, 800, 778, 756, 734,
			711, 687, 663, 639, 614, 589, 564, 539,
			514, 489, 464, 439, 414, 389, 365, 341,
			317, 294, 272, 250, 228, 208, 188, 169,
			151, 133, 117, 101, 87, 73, 61, 49,
			39, 30, 22, 15, 10, 6, 2, 1,
			0, 1, 2, 6, 10, 15, 22, 30,
			39, 49, 61, 73, 87, 101, 117, 133,
			151, 169, 188, 208, 228, 250, 272, 294,
			317, 341, 365, 389, 414, 439, 464, 489};

        //m_phase is the variable holding our PHI[n] function from
		// above.
		// We'll initialize our initial phase and frequency to zero
		m_phase = 0;
		m_dphase = 0;    
    }

    // On any object deletion, make sure we delete the table as well
	~NCO(void) {
		//delete[] m_table;
	}

    const float ONE_ROTATION= 2.0 * (1u << (sizeof(unsigned)*8-1));
    void frequency(float f)
    {
        // Convert the frequency to a fractional difference in phase
		m_dphase = (int)(f * ONE_ROTATION / SAMPLE_RATE);
    }
    uint sin_inc(void) //operator ()(void) {
	{
		unsigned index;

		// Increment the phase by an amount dictated by our frequency
		// m_phase was our PHI[n] value above
		m_phase += m_dphase; // PHI[n] = PHI[n-1] + (2^32 * f/fs)

		// Grab the top m_lglen bits of this phase word
		index = m_phase >> (sizeof(unsigned)*8)-m_lglen;

		// Insist that this index be found within 0... (m_len-1)
		index &= m_mask;

		// Finally return the table lookup value
		return m_table[index];
	}

};
#endif