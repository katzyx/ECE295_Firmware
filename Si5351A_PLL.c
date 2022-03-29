/*
 * Si5351A_PLL.c
 *
 * Created: 2021-10-02 3:12:12 PM
 * Author: StewartPearson
 * Adapted from: https://learn.adafruit.com/adafruit-si5351-clock-generator-breakout
 */ 

#include "./Si5351A_PLL.h"


// volatile global variables for the PLLs
volatile bool Si5351_initialised = false;
volatile si5351CrystalFreq_t Si5351_crystalFreq = SI5351_CRYSTAL_FREQ_25MHZ;
volatile si5351CrystalLoad_t Si5351_crystalLoad = SI5351_CRYSTAL_LOAD_10PF;
volatile uint32_t Si5351_crystalPPM = 30;
volatile bool Si5351_plla_configured = false;
volatile uint32_t Si5351_plla_freq = 0;
volatile bool Si5351_pllb_configured = false;
volatile uint32_t Si5351_pllb_freq = 0;
volatile uint8_t Si5351_lastRdivValue[3] = {0,0,0};

// I2C commands (use your own if you'd prefer)
static void write8(uint8_t reg, uint8_t value)
{
	uint8_t data[2] = {reg,value};
	tw_master_transmit(SI5351_ADDRESS, data, sizeof(data), false);
}

static void writeN(uint8_t *data, uint8_t n)
{	
	ASSERT_STATUS(tw_master_transmit(SI5351_ADDRESS, data, sizeof(data), false));
}

static void read8(uint8_t *reg, uint8_t *value) 
{	
	uint8_t len = sizeof(value);
	tw_master_transmit(SI5351_ADDRESS, reg, sizeof(reg), false);
	tw_master_receive(SI5351_ADDRESS, value, len);
}

/**************************************************************************/
/*!
    Configures the PLL module (call this function before
    doing anything else)
*/
/**************************************************************************/
err_t Si5351A_setup(void) {

  /* Do the initialization */

  /* Reset the PLL config fields just in case we call init again */
  Si5351_plla_configured = false;
  Si5351_plla_freq = 0;
  Si5351_pllb_configured = false;
  Si5351_pllb_freq = 0;

  /* All done! */
  Si5351_initialised = true;

  return ERROR_NONE;
}


/**************************************************************************/
/*!
  @brief  Sets the multiplier for the specified PLL using integer values

  @param  pll   The PLL to configure, which must be one of the following:
                - SI5351_PLL_A
                - SI5351_PLL_B
  @param  mult  The PLL integer multiplier (must be between 15 and 90)
*/
/**************************************************************************/
err_t setupPLLInt(si5351PLL_t pll, uint8_t mult) {
  return setupPLL(pll, mult, 0, 1);
}

/**************************************************************************/
/*!
    @brief  Sets the multiplier for the specified PLL

    @param  pll   The PLL to configure, which must be one of the following:
                  - SI5351_PLL_A
                  - SI5351_PLL_B
    @param  mult  The PLL integer multiplier (must be between 15 and 90)
    @param  num   The 20-bit numerator for fractional output (0..1,048,575).
                  Set this to '0' for integer output.
    @param  denom The 20-bit denominator for fractional output (1..1,048,575).
                  Set this to '1' or higher to avoid divider by zero errors.

    @section PLL Configuration

    fVCO is the PLL output, and must be between 600..900MHz, where:

        fVCO = fXTAL * (a+(b/c))

    fXTAL = the crystal input frequency
    a     = an integer between 15 and 90
    b     = the fractional numerator (0..1,048,575)
    c     = the fractional denominator (1..1,048,575)

    NOTE: Try to use integers whenever possible to avoid clock jitter
    (only use the a part, setting b to '0' and c to '1').

    See: http://www.silabs.com/Support%20Documents/TechnicalDocs/AN619.pdf
*/
/**************************************************************************/
err_t setupPLL(si5351PLL_t pll, uint8_t mult, uint32_t num,
                                uint32_t denom) {
  

  return ERROR_NONE;
}

/**************************************************************************/
/*!
    @brief  Configures the Multisynth divider using integer output.

    @param  output    The output channel to use (0..2)
    @param  pllSource	The PLL input source to use, which must be one of:
                      - SI5351_PLL_A
                      - SI5351_PLL_B
    @param  div       The integer divider for the Multisynth output,
                      which must be one of the following values:
                      - SI5351_MULTISYNTH_DIV_4
                      - SI5351_MULTISYNTH_DIV_6
                      - SI5351_MULTISYNTH_DIV_8
*/
/**************************************************************************/
err_t setupMultisynthInt(uint8_t output, si5351PLL_t pllSource,
                                          si5351MultisynthDiv_t div) {
  return setupMultisynth(output, pllSource, div, 0, 1);
}

err_t setupRdiv(uint8_t output, si5351RDiv_t div, uint32_t* freq) {
 
  return ERROR_NONE;
}

/**************************************************************************/
/*!
    @brief  Configures the Multisynth divider, which determines the
            output clock frequency based on the specified PLL input.

    @param  output    The output channel to use (0..2)
    @param  pllSource	The PLL input source to use, which must be one of:
                      - SI5351_PLL_A
                      - SI5351_PLL_B
    @param  div       The integer divider for the Multisynth output.
                      If pure integer values are used, this value must
                      be one of:
                      - SI5351_MULTISYNTH_DIV_4
                      - SI5351_MULTISYNTH_DIV_6
                      - SI5351_MULTISYNTH_DIV_8
                      If fractional output is used, this value must be
                      between 8 and 900.
    @param  num       The 20-bit numerator for fractional output
                      (0..1,048,575). Set this to '0' for integer output.
    @param  denom     The 20-bit denominator for fractional output
                      (1..1,048,575). Set this to '1' or higher to
                      avoid divide by zero errors.

    @section Output Clock Configuration

    The multisynth dividers are applied to the specified PLL output,
    and are used to reduce the PLL output to a valid range (500kHz
    to 160MHz). The relationship can be seen in this formula, where
    fVCO is the PLL output frequency and MSx is the multisynth
    divider:

        fOUT = fVCO / MSx

    Valid multisynth dividers are 4, 6, or 8 when using integers,
    or any fractional values between 8 + 1/1,048,575 and 900 + 0/1

    The following formula is used for the fractional mode divider:

        a + b / c

    a = The integer value, which must be 4, 6 or 8 in integer mode (MSx_INT=1)
        or 8..900 in fractional mode (MSx_INT=0).
    b = The fractional numerator (0..1,048,575)
    c = The fractional denominator (1..1,048,575)

    @note   Try to use integers whenever possible to avoid clock jitter

    @note   For output frequencies > 150MHz, you must set the divider
            to 4 and adjust to PLL to generate the frequency (for example
            a PLL of 640 to generate a 160MHz output clock). This is not
            yet supported in the driver, which limits frequencies to
            500kHz .. 150MHz.

    @note   For frequencies below 500kHz (down to 8kHz) Rx_DIV must be
            used, but this isn't currently implemented in the driver.
*/
/**************************************************************************/
err_t setupMultisynth(uint8_t output, si5351PLL_t pllSource,
                                       uint32_t div, uint32_t num,
                                       uint32_t denom) {

  return ERROR_NONE;
}

/**************************************************************************/
/*!
    @brief  Enables or disables all clock outputs
*/
/**************************************************************************/
err_t enableOutputs(bool enabled) {  
  return ERROR_NONE;
}


/**************************************************************************/
/*!
    @brief  Sets a phase offset for the desired output. One LSB is equivalent
			to a time delay of Tvco/4 where Tvco is the period of the VCO/PLL
			associated with this output.

    @param  output    The output channel to use (0..2)
    @param  phase_off	The phase offset to use. One LSB is equivalent
						to a time delay of Tvco/4 where Tvco is the period 
						of the VCO/PLL associated with this output.
*/
/**************************************************************************/
err_t setPhaseOffset(uint8_t output, uint8_t phase_off) {
	
	 return ERROR_NONE;
};


/**************************************************************************/
/*!
    @brief  Calculates the pll frequency for the given freq

	@param  pll:  choose pll A or B  
	@param  freq: pll frequency to set
	@param  correction: PLL correction registers. Factor calibration value 
						into nominal crystal frequency. Measured in parts-per-billion
	@param  vxco:    voltage controlled oscillator
*/
/**************************************************************************/
uint32_t pll_calc(si5351PLL_t pll, uint32_t freq, int32_t correction, uint8_t vcxo)
{
	freq=0;
	
	return freq;
}

/**************************************************************************/
/*!
    @brief  Calculate the multisynth frequency

    @param  output    The output channel to use (0..2)
    @param  pllSource	The PLL you would like to use for this frequency
    @param  freq	The desired frequency
    @param  pll_freq	The PLL frequency
*/
/**************************************************************************/
uint32_t multisynth_calc(uint8_t output, si5351PLL_t pllSource,	uint32_t freq, uint32_t pll_freq)
{
	freq=0;
	
	return freq;
}

/**************************************************************************/
/*!
    @brief  Configures the Multisynth divider and PLL to output a desired 
			frequency on outputs 0 and 1 (90 deg offset). 

	@param  freq	The desired frequency
	@param  pllSource	The PLL you would like to use for this frequency (A/B)
*/
/**************************************************************************/
err_t setLOF1(uint32_t freq, si5351PLL_t pllSource) 
{	
	uint32_t pll_freq=0;
	uint32_t correction =0;
	uint8_t vxco=0;
	uint32_t phase_off = 0;
	si5351RDiv_t r_div = SI5351_R_DIV_1;  // Rdiv may be needed for freqs<500kHz.
	
	// Get PLL frequency for a given output frequency. Also set appropriate
	// multisynth registers
	pll_freq=multisynth_calc(0,pllSource,freq,0);
	multisynth_calc(1,pllSource,freq,0);
	// Set phase offset for the other output.
	setPhaseOffset(1,phase_off);
	
	// Program PLL registers for required pll_freq as given by the multisynth
	// calculations above. Enable the outputs when you're done.
	pll_calc(pllSource,pll_freq,correction,vxco);
}
/**************************************************************************/
/*!
    @brief  Configures the module to output a desired frequency on a PLL source
			pin for LO_F2 (not required for ECE295)

	@param  freq	The desired frequency
	@param  pllSource	The PLL you would like to use for this frequency
*/
/**************************************************************************/
err_t setLOF2(uint32_t freq, si5351PLL_t pllSource) 
{
	return ERROR_NONE;
}

/**************************************************************************/
/*!
    @brief  You give it the frequency by reference so it picks an rdiv value
			and then modifies the frequency as well to scale it properly.

    @param  freq: the frequency you're aiming for.
*/
/**************************************************************************/
si5351RDiv_t select_r_div(uint32_t *freq)
{
	si5351RDiv_t r_div = SI5351_R_DIV_1;

	return r_div;
}