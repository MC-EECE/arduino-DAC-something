/*
This sketch illustrates how to set a timer on an SAMD21 based board in Arduino (Feather M0, Arduino Zero should work)
*/
const int dacPIN = DAC0;  //Assign Output Pin to Pin A0
uint32_t sampleRate = 1; //sample rate of the sine wave in Hertz, how many times per second the TC5_Handler() function gets called per second basically


bool state = 0; //just for an example
float Ts; //Declaring Period Variable


const unsigned long period = 100;
float freq = 10;

unsigned long start_time;

void setup() {
  uint16_t Fs = 44100;    //Declaring Sample Rate Variable
  analogWriteResolution(10);  //Setting DAC Resolution to its maximum value for the Arduino Zero
  Ts = 1.0/(float)Fs;         //Calculating Period
  freq = 440.0;
  tcConfigure(freq); //configure the timer to run at <freq>Hertz
  tcStartCounter(); //starts the timer
}

void loop() {
  

    
}

//this function gets called by the interrupt at <sampleRate>Hertz
void TC5_Handler (void) {
  //YOUR CODE HERE 
  
   static uint16_t n = 0;    // Declaring variable n as an unsigned 16-bit integer
   uint16_t sig = (uint16_t)(511.5*(cos(2*PI*freq*n*Ts)+1.0));   //Declaring signal variable sig, assigning a value of cos with the parameters provided in the instructions all added with 1 to give a positive value and multiplied by 2047 (1023?) to properly scale the output for the DAC
   analogWrite(dacPIN, sig);  //Writing the output through the DAC to dacPIN (pin A0)
   n++;                     //Incrementing n
   if (n>65535) n = 0;      //65536 is 2^16, maximum value of a 16-bit number
   
  // END OF YOUR CODE
  
  TC5->COUNT16.INTFLAG.bit.MC0 = 1; //don't change this, it's part of the timer code
}

/* 
 *  TIMER SPECIFIC FUNCTIONS FOLLOW
 *  you shouldn't change these unless you know what you're doing
 */

//Configures the TC to generate output events at the sample frequency.
//Configures the TC in Frequency Generation mode, with an event output once
//each time the audio sample frequency period expires.
 void tcConfigure(int sampleRate)
{
 // Enable GCLK for TCC2 and TC5 (timer counter input clock)
 GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID(GCM_TC4_TC5)) ;
 while (GCLK->STATUS.bit.SYNCBUSY);

 tcReset(); //reset TC5

 // Set Timer counter Mode to 16 bits
 TC5->COUNT16.CTRLA.reg |= TC_CTRLA_MODE_COUNT16;
 // Set TC5 mode as match frequency
 TC5->COUNT16.CTRLA.reg |= TC_CTRLA_WAVEGEN_MFRQ;
 //set prescaler and enable TC5
 TC5->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV1024 | TC_CTRLA_ENABLE;
 //set TC5 timer counter based off of the system clock and the user defined sample rate or waveform
 TC5->COUNT16.CC[0].reg = (uint16_t) (SystemCoreClock / sampleRate / 1024 - 1);
 while (tcIsSyncing());
 
 // Configure interrupt request
 NVIC_DisableIRQ(TC5_IRQn);
 NVIC_ClearPendingIRQ(TC5_IRQn);
 NVIC_SetPriority(TC5_IRQn, 0);
 NVIC_EnableIRQ(TC5_IRQn);

 // Enable the TC5 interrupt request
 TC5->COUNT16.INTENSET.bit.MC0 = 1;
 while (tcIsSyncing()); //wait until TC5 is done syncing 
} 

//Function that is used to check if TC5 is done syncing
//returns true when it is done syncing
bool tcIsSyncing()
{
  return TC5->COUNT16.STATUS.reg & TC_STATUS_SYNCBUSY;
}

//This function enables TC5 and waits for it to be ready
void tcStartCounter()
{
  TC5->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE; //set the CTRLA register
  while (tcIsSyncing()); //wait until snyc'd
}

//Reset TC5 
void tcReset()
{
  TC5->COUNT16.CTRLA.reg = TC_CTRLA_SWRST;
  while (tcIsSyncing());
  while (TC5->COUNT16.CTRLA.bit.SWRST);
}

//disable TC5
void tcDisable()
{
  TC5->COUNT16.CTRLA.reg &= ~TC_CTRLA_ENABLE;
  while (tcIsSyncing());
}
