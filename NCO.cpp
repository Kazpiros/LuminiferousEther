#include <cstdlib>

#define M_PI 3.14159
#define SAMPLE_RATE 1.
//https://zipcpu.com/dsp/2017/12/09/nco.html
class NCO{
    public:
    unsigned m_lglen, m_len, m_mask, m_phase, m_dphase;
	float *m_table;

	NCO(const int lgtblsize) {
		// We'll use a table 2^(lgtblize) in length.  This is
		// non-negotiable, as the rest of this algorithm depends upon
		// this property.
		m_lglen = lgtblsize;
		m_len = (1<<lgtblsize);

		// m_mask is 1 for any bit used in the index, zero otherwise
		m_mask = m_len - 1;

        m_table = new float[m_len];
		for(int k=0; k<m_len; k++)
			m_table[k] = sin(2.*M_PI*k / (double)m_len);

        //m_phase is the variable holding our PHI[n] function from
		// above.
		// We'll initialize our initial phase and frequency to zero
		m_phase = 0;
		m_dphase = 0;    
    }

    // On any object deletion, make sure we delete the table as well
	~NCO(void) {
		delete[] m_table;
	}

    const float ONE_ROTATION= 2.0 * (1u << (sizeof(unsigned)*8-1));
    float frequency(float f)
    {
        // Convert the frequency to a fractional difference in phase
		m_dphase = (int)(f * ONE_ROTATION / SAMPLE_RATE);
    }
    float operator ()(void) {
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
