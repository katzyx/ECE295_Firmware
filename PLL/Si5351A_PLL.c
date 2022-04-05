/*
 * Si5351A_PLL.c
 *
 * Created: 2021-10-02 3:12:12 PM
 * Author: Stewart Pearson, Katherine Zhang
 * Adapted from: https://learn.adafruit.com/adafruit-si5351-clock-generator-breakout
 */ 

#include "Si5351A_PLL.h"

// Function prototypes:
static void write8(uint8_t reg, uint8_t value); // DONE
static void writeN(uint8_t *data, uint8_t n); // DONE
static void read8(uint8_t *reg, uint8_t *value); // DONE
err_t Si5351A_setup(void); // DONE
err_t setupPLLInt(si5351PLL_t pll, uint8_t mult); // DONE
err_t setupPLL(si5351PLL_t pll, uint8_t mult, uint32_t num, uint32_t denom); // DONE
err_t setupMultisynthInt(uint8_t output, si5351PLL_t pllSource, si5351MultisynthDiv_t div); // DONE
err_t setupRdiv(uint8_t output, si5351RDiv_t div, uint32_t* freq);
err_t setupMultisynth(uint8_t output, si5351PLL_t pllSource, uint32_t div, uint32_t num, uint32_t denom); // DONE
err_t enableOutputs(bool enabled); // DONE
err_t setPhaseOffset(uint8_t output, uint8_t phase_off); // NOT USED
uint32_t pll_calc(si5351PLL_t pll, uint32_t freq, int32_t correction, uint8_t vcxo); // NOT USED
uint32_t multisynth_calc(uint8_t output, si5351PLL_t pllSource,	uint32_t freq, uint32_t pll_freq); // NOT USED
err_t setLOF1(uint32_t freq, si5351PLL_t pllSource); // DONE
err_t setLOF2(uint32_t freq, si5351PLL_t pllSource); // NOT USED
si5351RDiv_t select_r_div(uint32_t *freq); // NOT USED

// volatile global variables for the PLLs
volatile bool Si5351_initialised = false;
volatile si5351CrystalFreq_t Si5351_crystalFreq = SI5351_CRYSTAL_FREQ_25MHZ;
volatile si5351CrystalLoad_t Si5351_crystalLoad = SI5351_CRYSTAL_LOAD_10PF;
volatile uint32_t Si5351_crystalPPM = 30;
volatile bool Si5351_plla_configured = false;
volatile uint32_t Si5351_plla_freq = 0;
volatile bool Si5351_pllb_configured = false;
volatile uint32_t Si5351_pllb_freq = 0;
volatile uint8_t Si5351_lastRdivValue[3] = {0,0,0}


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
  /* Disable all outputs setting CLKx_DIS high */
  ASSERT_STATUS(write8(SI5351_REGISTER_3_OUTPUT_ENABLE_CONTROL, 0xFF)); // register 3: output enable control
  /* REGISTER 3: Output Enable Control
      1 = disable CLKx output
  */

  /* Power down all output drivers */
  ASSERT_STATUS(write8(SI5351_REGISTER_16_CLK0_CONTROL, 0x80));
  ASSERT_STATUS(write8(SI5351_REGISTER_17_CLK1_CONTROL, 0x80));
  ASSERT_STATUS(write8(SI5351_REGISTER_18_CLK2_CONTROL, 0x80));
  ASSERT_STATUS(write8(SI5351_REGISTER_19_CLK3_CONTROL, 0x80));
  ASSERT_STATUS(write8(SI5351_REGISTER_20_CLK4_CONTROL, 0x80));
  ASSERT_STATUS(write8(SI5351_REGISTER_21_CLK5_CONTROL, 0x80));
  ASSERT_STATUS(write8(SI5351_REGISTER_22_CLK6_CONTROL, 0x80));
  ASSERT_STATUS(write8(SI5351_REGISTER_23_CLK7_CONTROL, 0x80));
  /* 0x80 = 0b10100000
     REGISTER 16-23:
      bit 7 = CLKx Power Down   --> 1 = powered down
      bit 5 = MSx Source Select --> 1 - select PLLB for MultiSynthx
  */

  /* Set the load capacitance for the XTAL */
  ASSERT_STATUS(write8(SI5351_REGISTER_183_CRYSTAL_INTERNAL_LOAD_CAPACITANCE, Si5351_crystalLoad));
  /* REGISTER 183: Crystal Internal Load Capacitance
     bits 7-6: internal load capciatance for crystal
       00 - reserved (do not select)
       01 - CL = 6 pF
       10 - CL = 8 pF
       11 - CL = 10 pF (default)
     bits 5-0: reserved, should be written 0b010010
  */

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
err_t setupPLL(si5351PLL_t pll, uint8_t mult, uint32_t num, uint32_t denom) {
  uint32_t P1; // PLL config register P1
  uint32_t P2; // PLL config register P2
  uint32_t P3; // PLL config register P3

  // Basic Validation - error check
  ASSERT(Si5351_initialised, ERROR_DEVICENOTINITIALISED);
  ASSERT((mult > 14) && (mult < 91), ERROR_INVALIDPARAMETER); // mult = 15..90
  ASSERT(denom > 0, ERROR_INVALIDPARAMETER);                  // Avoid divide by zero
  ASSERT(num <= 0xFFFFF, ERROR_INVALIDPARAMETER);             // 20-bit limit
  ASSERT(denom <= 0xFFFFF, ERROR_INVALIDPARAMETER);           // 20-bit limit
  
  /* Feedback Multisynth Divider Equation
   *
   * where: a = mult, b = num and c = denom
   *
   * P1 register is an 18-bit value using following formula:
   *
   * 	P1[17:0] = 128 * mult + floor(128*(num/denom)) - 512
   *
   * P2 register is a 20-bit value using the following formula:
   *
   * 	P2[19:0] = 128 * num - denom * floor(128*(num/denom))
   *
   * P3 register is a 20-bit value using the following formula:
   *
   * 	P3[19:0] = denom
   */

  // Set main PLL config registers
  if (num == 0){
    // integer mode
    P1 = 128 * mult - 512;
    P2 = num;
    P3 = denom;
  } 
  else {
    /* Fractional mode */
    P1 =(uint32_t)(128 * mult + floor(128 * ((float)num / (float)denom)) - 512);
    P2 = (uint32_t)(128 * num - denom * floor(128 * ((float)num / (float)denom)));
    P3 = denom;
  }
  
  // Get the appropriate starting point for the PLL registers */
  uint8_t baseaddr = (pll == SI5351_PLL_A ? 26 : 34);

  /* The datasheet is a nightmare of typos and inconsistencies here! */
  ASSERT_STATUS(write8(baseaddr, (P3 & 0x0000FF00) >> 8));
  ASSERT_STATUS(write8(baseaddr + 1, (P3 & 0x000000FF)));
  ASSERT_STATUS(write8(baseaddr + 2, (P1 & 0x00030000) >> 16));
  ASSERT_STATUS(write8(baseaddr + 3, (P1 & 0x0000FF00) >> 8));
  ASSERT_STATUS(write8(baseaddr + 4, (P1 & 0x000000FF)));
  ASSERT_STATUS(write8(baseaddr + 5, ((P3 & 0x000F0000) >> 12) | ((P2 & 0x000F0000) >> 16)));
  ASSERT_STATUS(write8(baseaddr + 6, (P2 & 0x0000FF00) >> 8));
  ASSERT_STATUS(write8(baseaddr + 7, (P2 & 0x000000FF)));

  /* Reset both PLLs */
  ASSERT_STATUS(write8(SI5351_REGISTER_177_PLL_RESET, (1 << 7) | (1 << 5)));

  /* Store the frequency settings for use with the Multisynth helper */
  if (pll == SI5351_PLL_A) {
    float fvco = Si5351_crystalFreq * (mult + ((float)num / (float)denom));
    Si5351_plla_configured = true;
    Si5351_plla_freq = (uint32_t)floor(fvco);
  } 
  else {
    float fvco = Si5351_crystalFreq * (mult + ((float)num / (float)denom));
    Si5351_pllb_configured = true;
    Si5351_pllb_freq = (uint32_t)floor(fvco);
  }

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
err_t setupMultisynthInt(uint8_t output, si5351PLL_t pllSource, si5351MultisynthDiv_t div) {
  return setupMultisynth(output, pllSource, div, 0, 1);
}

err_t setupRdiv(uint8_t output, si5351RDiv_t div, uint32_t* freq) {
  ASSERT(output < 3, ERROR_INVALIDPARAMETER); /* Channel range */

  uint8_t Rreg, regval;

  if (output == 0)
    Rreg = SI5351_REGISTER_44_MULTISYNTH0_PARAMETERS_3;
  if (output == 1)
    Rreg = SI5351_REGISTER_52_MULTISYNTH1_PARAMETERS_3;
  if (output == 2)
    Rreg = SI5351_REGISTER_60_MULTISYNTH2_PARAMETERS_3;

  read8(Rreg, &regval);

  regval &= 0x0F;
  uint8_t divider = div;
  divider &= 0x07;
  divider <<= 4;
  regval |= divider;
  lastRdivValue[output] = divider;


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
err_t setupMultisynth(uint8_t output, si5351PLL_t pllSource, uint32_t div, uint32_t num, uint32_t denom) {
  uint32_t P1; /* Multisynth config register P1 */
  uint32_t P2; /* Multisynth config register P2 */
  uint32_t P3; /* Multisynth config register P3 */

  /* Basic validation */
  ASSERT(Si5351_initialised, ERROR_DEVICENOTINITIALISED);
  ASSERT(output < 3, ERROR_INVALIDPARAMETER);       /* Channel range */
  ASSERT(div > 3, ERROR_INVALIDPARAMETER);          /* Divider integer value */
  ASSERT(div < 2049, ERROR_INVALIDPARAMETER);       /* Divider integer value */
  ASSERT(denom > 0, ERROR_INVALIDPARAMETER);        /* Avoid divide by zero */
  ASSERT(num <= 0xFFFFF, ERROR_INVALIDPARAMETER);   /* 20-bit limit */
  ASSERT(denom <= 0xFFFFF, ERROR_INVALIDPARAMETER); /* 20-bit limit */

  /* Make sure the requested PLL has been initialised */
  if (pllSource == SI5351_PLL_A) {
    ASSERT(Si5351_plla_configured, ERROR_INVALIDPARAMETER);
  } 
  else {
    ASSERT(Si5351_pllb_configured, ERROR_INVALIDPARAMETER);
  }

  /* Output Multisynth Divider Equations
   *
   * where: a = div, b = num and c = denom
   *
   * P1 register is an 18-bit value using following formula:
   *
   * 	P1[17:0] = 128 * a + floor(128*(b/c)) - 512
   *
   * P2 register is a 20-bit value using the following formula:
   *
   * 	P2[19:0] = 128 * b - c * floor(128*(b/c))
   *
   * P3 register is a 20-bit value using the following formula:
   *
   * 	P3[19:0] = c
   */

  /* Set the main PLL config registers */
  if (num == 0) {
    /* Integer mode */
    P1 = 128 * div - 512;
    P2 = 0;
    P3 = denom;
  } else if (denom == 1) {
    /* Fractional mode, simplified calculations */
    P1 = 128 * div + 128 * num - 512;
    P2 = 128 * num - 128;
    P3 = 1;
  } else {
    /* Fractional mode */
    P1 = (uint32_t)(128 * div + floor(128 * ((float)num / (float)denom)) - 512);
    P2 = (uint32_t)(128 * num -
                    denom * floor(128 * ((float)num / (float)denom)));
    P3 = denom;
  }

  /* Get the appropriate starting point for the PLL registers */
  uint8_t baseaddr = 0;
  switch (output) {
  case 0:
    baseaddr = SI5351_REGISTER_42_MULTISYNTH0_PARAMETERS_1;
    break;
  case 1:
    baseaddr = SI5351_REGISTER_50_MULTISYNTH1_PARAMETERS_1;
    break;
  case 2:
    baseaddr = SI5351_REGISTER_58_MULTISYNTH2_PARAMETERS_1;
    break;
  }

  /* Set the MSx config registers */
  /* Burst mode: register address auto-increases */
  uint8_t sendBuffer[9];
  sendBuffer[0] = baseaddr;
  sendBuffer[1] = (P3 & 0xFF00) >> 8;
  sendBuffer[2] = P3 & 0xFF;
  sendBuffer[3] = ((P1 & 0x30000) >> 16) | lastRdivValue[output];
  sendBuffer[4] = (P1 & 0xFF00) >> 8;
  sendBuffer[5] = P1 & 0xFF;
  sendBuffer[6] = ((P3 & 0xF0000) >> 12) | ((P2 & 0xF0000) >> 16);
  sendBuffer[7] = (P2 & 0xFF00) >> 8;
  sendBuffer[8] = P2 & 0xFF;
  ASSERT_STATUS(writeN(sendBuffer, 9));

  /* Configure the clk control and enable the output */
  /* TODO: Check if the clk control byte needs to be updated. */
  uint8_t clkControlReg = 0x0F; /* 8mA drive strength, MS0 as CLK0 source, Clock
                                   not inverted, powered up */
  if (pllSource == SI5351_PLL_B)
    clkControlReg |= (1 << 5); /* Uses PLLB */
  if (num == 0)
    clkControlReg |= (1 << 6); /* Integer mode */
  switch (output) {
  case 0:
    ASSERT_STATUS(write8(SI5351_REGISTER_16_CLK0_CONTROL, clkControlReg));
    break;
  case 1:
    ASSERT_STATUS(write8(SI5351_REGISTER_17_CLK1_CONTROL, clkControlReg));
    break;
  case 2:
    ASSERT_STATUS(write8(SI5351_REGISTER_18_CLK2_CONTROL, clkControlReg));
    break;
  }

  return ERROR_NONE;
}

/**************************************************************************/
/*!
    @brief  Enables or disables all clock outputs
*/
/**************************************************************************/
err_t enableOutputs(bool enabled) {  
  /* Make sure we've called init first */
  ASSERT(Si5351_initialised, ERROR_DEVICENOTINITIALISED);

  /* Enabled desired outputs (see Register 3) */
  ASSERT_STATUS(write8(SI5351_REGISTER_3_OUTPUT_ENABLE_CONTROL, enabled ? 0x00 : 0xFF));
  
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
  phase_off = 90;
  return setPhaseOffset(output, phase_off);
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
uint32_t pll_calc(si5351PLL_t pll, uint32_t freq, int32_t correction, uint8_t vcxo) {
	
  freq = 0;
	
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
uint32_t multisynth_calc(uint8_t output, si5351PLL_t pllSource,	uint32_t freq, uint32_t pll_freq) {
	freq = 0;
	
	return freq;
}

/**************************************************************************/
/*!
  @brief  Configures the Multisynth divider and PLL to output a desired 
      		frequency on outputs 0 and 1 (90 deg offset). 

	@param  freq	The desired frequency
	@param  pllSource	The PLL you would like to use for this frequency (A/B)

  Set CLK0 output ON and to the specified frequency
   - frequency in range 1MHs to 150 MHz
   - ex: setLOF1(10000000); sets output CLK0 to 10MHz

  Sets up PLL A and MultiSynth 0 and procudes output on CLK0

*/
/**************************************************************************/
err_t setLOF1(uint32_t freq, si5351PLL_t pllSource) {	
	/* uint32_t pll_freq = 0;
	uint32_t correction = 0;
	uint8_t vxco = 0;
	uint32_t phase_off = 0;
	si5351RDiv_t r_div = SI5351_R_DIV_1;  // Rdiv may be needed for freqs<500kHz.

	// Get PLL frequency for a given output frequency. Also set appropriate multisynth registers
	pll_freq = multisynth_calc(0,pllSource,freq,0);
	multisynth_calc(1,pllSource,freq,0);

	// Set phase offset for the other output.
	setPhaseOffset(1, phase_off);
	
	// Program PLL registers for required pll_freq as given by the multisynth calculations above. 
  // Enable the outputs when you're done.
	pll_calc(pllSource, pll_freq, correction, vxco); */

  uint32_t pll_freq;
  uint32_t xtal_freq = SI5351_XTAL_FREQ;
  uint32_t l;
  float f;
  uint8_t mult;
  uint32_t num;
  uint32_t denom;
  uint32_t divider;

  // calculate divistion ratio: 900 000 000 is the mex internal
  // PLL frequency: 900MHz
  divider = 900000000 / freq;

  // ensure an even integer division ratio
  if (divider % 2)
    divider--;
  
  // calclate pll frequency: divider * desired output frequency
  pll_freq = divider * freq;
  // determine multiplier to get to required frequency
  mult = pll_freq / xtal_freq; // mult is integer that must be in range 15-90
  l = pll_freq % xtal_freq;
  f = l;
  f *= = 1048575;
  f /= xtal_freq;
  // actual multiplier is mult + num/denom
  num = f;  // num and denom are fractional parts, each is 20 bits (range 0-1048575)
  denom = 1048575; // set denom to maximum

  // set PLL A with calculated multiplicatino ratio
  setupPLL(SI5351_PLL_A, mult, num, denom);

  // set up mutlisynth divider 0 with calculated divider
  // final R division stage can divie by a power of 2 from 1-128
  // represented by constats SI5351_R_DIV_1, SI5351_R_DIV_128
  // to output frequencies below 1MHz, must use final R division stage
  setupMultisynth(SI5351_REGISTER_42_MULTISYNTH0_PARAMETERS_1, SI5351_PLL_A, divider, num, denom);

  // reset the pll causes a glitch in output
  // for small changes to the parameters, don't need to reset PLL and there is no glitch
  tw_master_transmit(SI5351_REGISTER_177_PLL_RESET, 0xA0, 1, false);
  
  // switch on CLK0 output (0x4F) and set multisynth0 input to be PLL A
  tw_master_transmit(SI5351_REGISTER_16_CLK0_CONTROL, 0x4F | SI5351_PLL_A, 1, false);

  return ERROR_NONE;
}
/**************************************************************************/
/*!
    @brief  Configures the module to output a desired frequency on a PLL source
			pin for LO_F2 (not required for ECE295)

	@param  freq	The desired frequency
	@param  pllSource	The PLL you would like to use for this frequency
*/
/**************************************************************************/
err_t setLOF2(uint32_t freq, si5351PLL_t pllSource) {
	return ERROR_NONE;
}

/**************************************************************************/
/*!
    @brief  You give it the frequency by reference so it picks an rdiv value
			and then modifies the frequency as well to scale it properly.

    @param  freq: the frequency you're aiming for.
*/
/**************************************************************************/
si5351RDiv_t select_r_div(uint32_t *freq) {
	si5351RDiv_t r_div = SI5351_R_DIV_1;

	return r_div;
}