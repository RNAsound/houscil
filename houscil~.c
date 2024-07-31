/* houscil~.c
 * by Aren Akian
 * MUS 177, FA 18
 *
 * The houscil(lator)
 * A tone generator that can produce a seventh chord based on input frequency
 * Left Inlet - sets root note frequency
 * Right inlet - Controls amplitude, ie. amount, of the interval harmonics
 *                = 0, outputs the root frequency only,
 *                  single tone is heard
 *
 *                = 100, outputs the root frequency, + the third, fifth, and seventh
 *                  a chord is heard!
 *
 * Based on Tom Erbe's oscil2~ extern
 */

#include "m_pd.h"
#include <stdlib.h>
#include <math.h>
#ifdef NT
#pragma warning( disable : 4244 )
#pragma warning( disable : 4305 )
#endif

/* ------------------------ houscil~ ----------------------------- */

#define WAVETABLESIZE 1024

static t_class *houscil_class;

typedef struct _houscil
{
    t_object x_obj; 	/* obligatory header */
    t_float x_f;    	/* place to hold inlet's value if it's set by message */
	t_float *wavetable;	/* a place to hold the squared values */
	t_float phase;
	t_float samplerate;
    
    t_float harm_amt; // 0 - 100% control over the harmonics' presence
} t_houscil;


/* handle float arguments */
void houscil_float(t_houscil *x, t_floatarg f)
{
    /* input is limited to the range [0, 100] */
    if      (f > 100){ x->harm_amt = 100; }
    else if (f < 1)  { x->harm_amt = 1; }
    else             { x->harm_amt = f;}
}

// some nice little interpolation routines
static inline float no_interpolate(t_houscil *x)
{
	int x_1 = x->phase * WAVETABLESIZE;

	return(x->wavetable[x_1 % WAVETABLESIZE]);
}
static inline float lin_interpolate(t_houscil *x, float harm)
{
	int x_1 = x->phase * WAVETABLESIZE * harm;
	float y_1 = x->wavetable[x_1 % WAVETABLESIZE];
	float y_2 = x->wavetable[(x_1 + 1) % WAVETABLESIZE];

	return (y_2 - y_1) * ((x->phase * WAVETABLESIZE * harm) - x_1) + y_1;
}

static inline float quad_interpolate(t_houscil *x)
{
	int truncphase = (int) (x->phase * WAVETABLESIZE);
	float fr = (x->phase * WAVETABLESIZE) - ((float) truncphase);
	float inm1 = x->wavetable[(truncphase - 1) % WAVETABLESIZE];
	float in   = x->wavetable[(truncphase + 0) % WAVETABLESIZE];
	float inp1 = x->wavetable[(truncphase + 1) % WAVETABLESIZE];
	float inp2 = x->wavetable[(truncphase + 2) % WAVETABLESIZE];

	return in + 0.5 * fr * (inp1 - inm1 +
	 fr * (4.0 * inp1 + 2.0 * inm1 - 5.0 * in - inp2 +
	 fr * (3.0 * (in - inp1) - inm1 + inp2)));
}

    /* this is the actual performance routine which acts on the samples.
    It's called with a single pointer "w" which is our location in the
    DSP call list.  We return a new "w" which will point to the next item
    after us.  Meanwhile, w[0] is just a pointer to dsp-perform itself
    (no use to us), w[1] and w[2] are the input and output vector locations,
    and w[3] is the number of points to calculate. */
static t_int *houscil_perform(t_int *w)
{
	t_houscil *x = (t_houscil *)(w[1]);
    t_float *freq = (t_float *)(w[2]);
    t_float *out = (t_float *)(w[3]);
    int n = (int)(w[4]);

	// i like counting from zero
	int blocksize = n;
	int i, j, sample = 0;
	float phaseincrement, triflip;
	float findex;
	int	iindex;
    while (n--)
    {
		// first we need to calculate the phase increment from the frequency
		// and sample rate - this is the number of cycles per sample
		// freq = cyc/sec, sr = samp/sec, phaseinc = cyc/samp = freq/sr
		phaseincrement = *(freq+sample)/x->samplerate;
		
		// now, increment the phase and make sure it doesn't go over 1.0
		x->phase += phaseincrement;
		while(x->phase >= 60.0f)
			x->phase -= 60.0f;
		while(x->phase < 0.0f)
			x->phase += 60.0f;
		// now grab the sample from the table
		findex = WAVETABLESIZE * x->phase;
		iindex = (int)findex;
        *(out+sample) = 0.0f;

        
        *(out+sample) += lin_interpolate(x, 1.0f)/2.0f;
        *(out+sample) += lin_interpolate(x, 5.0f/4.0f)/(4.0f / (x->harm_amt / 100) ); //major third
        *(out+sample) += lin_interpolate(x, 3.0f/2.0f)/(6.0f /(x->harm_amt / 100) ); //perfect fifth
        *(out+sample) += lin_interpolate(x, 15.0f/8.0f)/(8.0f / (x->harm_amt / 100) ); //major seventh

        sample++;
    }
    return (w+5);
}

    /* called to start DSP.  Here we call Pd back to add our perform
    routine to a linear callback list which Pd in turn calls to grind
    out the samples. */
static void houscil_dsp(t_houscil *x, t_signal **sp)
{
	// we'll initialize samplerate when starting up
	x->samplerate = sp[0]->s_sr;
    dsp_add(houscil_perform, 4, x, sp[0]->s_vec, sp[1]->s_vec, sp[0]->s_n);
}

static void *houscil_new(t_floatarg f)
{
	float twopi, size;
	int i;
    t_houscil *x = (t_houscil *)pd_new(houscil_class);
    inlet_new(&x->x_obj, &x->x_obj.ob_pd, gensym("float"), gensym("ft1") );

    outlet_new(&x->x_obj, gensym("signal"));
	// initialize variables
    x->x_f = 0.0f;
	x->phase = 0.0f;
	twopi = 8.0f * atanf(1.0f);
	size = (float)WAVETABLESIZE;
    
    x->harm_amt = f;
	
	// space for WAVETABLESIZE samples
	x->wavetable = (t_float *)malloc(WAVETABLESIZE * sizeof(t_float));
	
	// fill it up with a sine wave
	for(i = 0; i < WAVETABLESIZE; i++)
        *(x->wavetable+i) = sinf(twopi * (float)i/size);
    return (x);
}

// since we allocated some memory, we need a delete function
static void houscil_free(t_houscil *x)
{
	free(x->wavetable);
}

    /* this routine, which must have exactly this name (with the "~" replaced
    by "_tilde) is called when the code is first loaded, and tells Pd how
    to build the "class". */
void houscil_tilde_setup(void)
{
    houscil_class = class_new(gensym("houscil~"), (t_newmethod)houscil_new, (t_method)houscil_free,
    	sizeof(t_houscil), 0, A_DEFFLOAT, 0);
	    /* this is magic to declare that the leftmost, "main" inlet
	    takes signals; other signal inlets are done differently... */
    CLASS_MAINSIGNALIN(houscil_class, t_houscil, x_f);
    	/* here we tell Pd about the "dsp" method, which is called back
	when DSP is turned on. */
    class_addmethod(houscil_class, (t_method)houscil_dsp, gensym("dsp"), 0);
    class_addmethod(houscil_class, (t_method)houscil_float, gensym("ft1"), A_FLOAT, 0);
    class_addfloat(houscil_class, houscil_float);
}
