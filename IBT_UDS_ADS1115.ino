/************************************************************************************************
 2022_04_14  8:30
  IBT demo radio telescope sketch with UDS connection to Radio-SkyPipe.    R3
  This is Version 3 which uses ADS1115 16 bit ADC with softwarwe offset function.
  ADC is +/- & signal is +... So effective = 15 bits
  The UDS command processor came from RadioSky web site.  Added assorted mods.
        Bruce Randall NT4RT  July 10, 2018,  Sept 27, 2018
        Oct 10, 2018  Major I/O schuffle and TIMER1 driven tone process
        Oct 11, 2018 ADC driven by TIMER2 IRQ for accurate timing.
        Oct 12, 2018 added full interpolation to PWR table lookup
        Oct 14, 2018 added "volitile" to all IRQ variables. Removed do nothing code        
        Jun 10, 2019 Change for HW Audio path. D7 now high to select Detector Audio.
        Jul 23, 2019 Add AutoNul on command from push button.
        Mar 07, 2022 New vwersion for ADS1115 ADC & software offset.  New sample rates
                Note: SLOPE IS NEGATIVE from AD8318. Swap ADC to fix on R3.
        Mar 20, 2022 Time slot for task fixes.
        Apr 02  2022 Fixed timing issues adcFlag is volatile now!! Smoother operation of encoder knob.
        Apr 14, 2022 change to ADS1115 drive.  ADS1015 still works with short convert time.          
        
  In UDS PUSH mode, data is sent every 100 milliSeconds.  PREFERED !!!
  In UDS POLL mode, Data request should not be closer than 200 milliSeconds.  
      Extra serial com causes timing slips.
  WARNING:  This sketch does NOT fully decode SkyPipe UDS commands.  Any added commands
  to UDS could cause trouble!!!  Decoder came this way from RadioSky, so must be OK.
  
  Tau Filter Range is 0.1 .. 0.2 .. 0.4 .. 0.8 .... 25.6 Seconds Equiv RC ( 0.1S * 2^N )

  WARNINGS on Timer-Counter usage:
  Timer0  8 bit   Orignal usage Sketch timing & PWM 5 & 6
  Timer1  16 bit  Non-standard!  16 bit tone generation.  NO PWM 9 & 10 or Servo Functions!!!
  Timer2  8 bit   Non-standard!  Does timing for ADC.    NO tone() function or PWM 3 & 11

  Normal Full Scale signal range  in all processes is 0 to 32767
***************************************************************************************************/

#define UDS_debug false         // Set true to debug with a terminal in place of SkyPipe.
#define Sig_debug false         // turns on Serial.print of tstepoints
// ************** WARNING!  SkyPipe will NOT work if either debug is true.  *********************!!!
//                IT CRASHES!!!  REALLY REALLY BAD!!!

#include <EEPROM.h>             // Allow EEPROM usage
#include <Wire.h>               // I2C drivers used by ADS1115 16 bit ADC

#include "Exp_LUT_11R9dB.h"     // Table to convert log detector response to linear temperature responce.

// Define  MCU pins ********** Schuffled for TIMER1 tone output Oct 10, 2018
//                        0     // Serial for IDE
//                        1     // Serial for IDE
//                        2     // DIO, can be IRQ input **NOT USED**
#define EncoderPinA       3     // Input with pullup      Also Port D mask 0x08 
#define EncoderPinB       4     // Input with pullup      Also Port D mask 0x10
#define Meter_pin         5     // PWM output for analog meter.    
#define Offset_pin        6     // PWM for offset voltage.  Used as test point only. Real offset in firmware now.
#define DetSel            7     // Output. High Selects Audio path for detector.  Changed Jun 10, 2019 
#define EncoderPush       8     // Input from push switch on encoder  
#define ToneOutAF         9     // Tone Warning: This prevents pins 9 and 10 PWM
#define AudioSelPush      10    // Inputfrom push button switch 
#define LUT_pin           11    // Input.  High for LUT meter output, low for linear.
#define O_Sync_pin        12     // Output Pin wiggles to sync scope for timing studies.
#define LED_Pin           13    // LED on this  pin on UNO PWB.     Used as test point.
//                        14    // Was used as Analog 0 input to internal Arduino 10 bit ADC.  Now uses ADS1115.
//                        15    // GPIO or A1           **NOT USED**
//                        16    // GPIO or A2           **NOT USED**
#define AutoNulPin_L      17    // Was A3.  Now Dig Input. Hold low to seek half scale on adcFiltOut15. Tunes offset PWN to null. 
//                        18    // Used as I2C was A4
//                        19    // Used as I2C was A5

#define EncA_Mask         0x08 // Bit on port D
#define EncB_Mask         0x10 // Bit on port D

// ****************  Timing stuff sets ADC and output sample rates ************************************************
#define TMR2_Period       78   // OCR value for 5.056 mS cycle.   T = 1024 * ( 78+1 ) / 16000 KHz = 5.056mS   // 255 max!
                                // note ADC convert time = 4.0 mS nominal, 4.4mS max.  5.056  mS cycle is enough margin.
#define Ticks             20    // This is the number of adc reads per out sample in UDS PUSH mode
//  5.008mS thru loop * 20 passes = 100.16mS.  Time slots below must adjust if this changes!!

// * * * * * * * * * * * * * * * * * * * * * * Define Time Slots * * * * * * * * * * * * * * * * * * * * * *
#define TT_OSync     0        // wiggle for scope synch.

#define TT_LogP_A    1        // count 1 & 11 Log Power
#define TT_LogP_B    11
//
#define TT_Tau_A     2        // count 2 or 12 do Tau filter
#define TT_Tau_B     12
//
#define TT_Tone_A     3       // count 3 & 13 Tone output
#define TT_Tone_B     13
//
#define TT_UDS_Cmd_A  4       // Process SkyPipe UDS Command 
#define TT_UDS_Cmd_B  14
//
#define TT_SW_Read    5       // Read user switches
//  
#define TT_PWM        6       // Update PWM for meter output
#define TT_GETD       7       // Send Data to SkyPipe UDS 
#define TT_Auto_Nul   8       // if button pushed, null offset

/* Task list by time slot
 *  5mS
 *     ADC => FiltOut15.  Includes: 
 *          Get ADC data & start next convert
 *          scale and offset for segment 
 *          ADC Tau filter
 *          Read Encoder
 *  50mS
 *     FiltOut15 => dataBus15 
 *          Convert Log signal to Power signal (LUT)
 *          Tua filter
 *          Tone output
 *         
 *  100mS
 *     dataBus15 => dataOut15
 *          PWM for meter
 *          UDS GETD process
 *          Read Switches
 *          Auto Null
 *
 *    UDS I/O process
*/

int count100mS = 0;             // counter for 50mS and 100mS internal tasks and 100mS output

//  ****************  Stuff for ADS1115 16 bit ADC ***********************************************************************
#define ADS_Adr 0x48            // default I2C address of ADS1115 A/D converter. 
                                // ADS sample rate driven by timer. Setup for 250 Hz.  200Hz start Rate from timer.
// ####define ADS_Control 0x8523      // ADS1015: ADS Start, Diff in 0:1,  +/- 2V span, 250 SPS, Triggered conversion.  Actaul=200 SPS.
                                // ADS1015 left for testing with old hardware
#define ADS_Control 0x85A3   // ADS1115: ADS Start, Diff in 0:1,  +/- 2V span, 250 SPS, Triggered conversion.  Actaul=200 SPS.
                             // ADS1015 will sample at 2400 SPS, triggered convert to 200 SPS Actual

// ***********************************************************************************************************************   

boolean poll = true;            // if true then data is polled by RSP using a GETD command
int stat = 0;                   // -1 = we were just stopped by a KILL command 0 = startup state
// 1 = INIT rcvd 2 = Ready to Go 3= Running

byte audioSel = 0;              // Select Audio Path.  Possible values below.  Saved in EEPROM
#define No_AF   0                   // 0 = no audio
#define Tone_AF 1                   // 1 = tone audio
#define Det_AF  2                   // 2 = detector audio
#define DetAFOn HIGH                // port pin "DetSel" to enable detector AF Amp
#define DetAFOff LOW                // port pin "DetSel" to disable detector AF Amp
byte audioSelZ   = 0;               // previous state of audioSel

#define Pitch700 11427              // Load to make 700Hz tone with TIMER1 for beeps. Calc = 16E6/700/2   
#define InitTone 3200               // Zero level tone frequency, Hz * 16.  Must be > 1955 !!        
#define K_T1_FP  128000000          // Freq to Period for 32767 data = about 2048 Hz 
                                    // K_T1_FP = 32767 * 16E6Hz / 2048Hz / 2 = 128E6 

volatile unsigned int t1Period = Pitch700; // Used in TIMER1 tone generation

int sw_Count = 0;                   // counts 100mS ticks for EEPROM write command
#define SW_Detect 12                // number of ticks to activate EEPROM write 

volatile boolean adcFlag = false;
volatile int adcBox = 0;            // holds ADC data from IRQ routine
int adcFiltOut15 = 0;               // Output of IIR filter in ADC routine 15 bits wide positive value. 5mSec task
int dataBus15 = 0;                  // Useed in process.  50mS Task
int dataOut15 =0 ;                  // main data for output devices.  Data 15 bitswide positive value.  50 & 100mSec tasks
#define HalfScale15 16383           // for half scale test of adcFilt15

#define ADCFiltK 4                  // value 4 shifts gives 5.0mS *(2^4) = about 80mS RC

//***  Tau Filter exp2 value.  Sel Value is 2^Sca x Exp2 value so as to update smoother ***
#define TauFil_Sca 2                // 2^Sca is mult beteen stored & used values
#define TauFil_Sel_Min 4            // lowest safe value 2^Sca  gives 50mS *(2^(8/8)) = about 100mS RC 
#define TauFil_Sel_Max 36           // highest value.  NStep *2^Sca  gives 50mS *(2^(36/4)) = about 26 Sec
#define TauMtrStep 25               // first div of 1 ... 10 meter scale.  Counts / Div 
byte tauFil_Sel = TauFil_Sel_Min;   // Range limit MUST be checked! saved in EEPROM.  
byte tauFil_Exp2 = 1;               // Range 1 to 9 goes to filter 

int incomingByte;                   // from serial port
byte af_Sw, af_Sw_Z = HIGH;         // Audio Sel & previous state of it

boolean encoderA, encoderB, encoderZ = 1;  // "Z" is previous state of encoderA
int offsetSet_i = 512;              // 0 ... 1020  saved in EEPROM & PWM as byte after div 4    scaled Dec 02, 2019 for encoder feel. 
int offsetSet_i_FS = 10240;         // Full Scale scaled x20 for use by ADC processing
 
#define OffsetMax 1012              // Max value for offset as 10 bit integer.  Timer as byte.  Lower to avoid ADC math overflow !!!
#define OffsetScale 20              // Scale factor for ADC process.
#define ADCmagic 12308              // Used in ADC process sign fix and PWM offset

// ****************  following now defined in Exp*dB.h lncude file ********************************************
// specific to each long range option
//#define LogRange ...              // segment size of 32767 total to expand 
//#define LogScale ...              // Scale for segment.  Range 2 through 10 Small int, LogRange * LogMult must be < 32767


byte eeData = 0;                    // used in EEPROM process
#define EEADR_Key 0                 // start of EEPROM
#define EE_Key_Value 0x55           // Value in key to establish EE data is valid
#define EEADR_OfstH 1               // EE Address of offset Hi Byte
#define EEADR_OfstL 2               // EE Address of offset Hi Byte
#define EEADR_Filk  3               // EE Address of filter K
#define EEADR_ASel  4               // EE Address of Audio Select

// ************************************************************************************************************
// ************************** Start Programme ****************************************
void setup() {                      //  UDS stat, poll are inialized in assignment
  InitPort();                       // Setup all I/O ports
  Wire.begin();                     // join i2c bus (address optional for master)
  Serial.begin(9600);               // connect to the serial port
  while (!Serial) ;                 // wait for serial port to connect. 
  
  write_ADS( ADS_Adr,  ADS_Control); // kick off ADS1115 first conversion
  delay(5);                         // wait 5mS for first ADS1x15 conversion to complete
 #if Sig_debug 
   Serial.println("startup INIT");  
 #endif

#if UDS_debug
  Serial.println("");
  Serial.print("^^1002 Arduino UDS ");
  Serial.write(255);
  Serial.println("");
#endif
  GetEE();                      // get saved setup from EEPROM
  TIMER_setup();                // Setup TIMER1 IRQ for tone gen. Setup TIMER2 IRQ driven ADC.
  delay(10);                   // may not be needed, but causes no harm
}                               // end of Setup
//***********************************************************************************
//****************************** Timer IRQ ******************************************
// TIMER1 IRQ for tone generation
ISR(TIMER1_COMPA_vect)          // timer compare interrupt service routine
{
  OCR1A = t1Period;             // tone vaue => counter update
}

//***********************************************************************************
// TIMER2 IRQ for IRQ driven ADC.  IRQ is every 5mS. 
ISR(TIMER2_COMPA_vect) // timer compare interrupt service routine
{
  digitalWrite(LED_Pin, HIGH);          // test point to observe timing on OScope
  adcFlag = true;                       // ADC data is ready
  digitalWrite(LED_Pin, LOW);           // test point to observe timing
}
//***********************************************************************************
//******************************* MAIN LOOP ***************************************************************
void loop() {
  /**********************************************************************************
    If we are pushing the data to RSP then we need to establish our timing for
    sending new data.  Here we are doing a delay of 100ms from a timer to get a
    sample rate of 10 samples per second.
  ************************************************************************************/
// * * * * * * * * * * * * * * Task done every 5 mSec * * * * * * * * * * * * * *
// Timing is set by ADC_Process which waits for TIMER2 to trigger ADC.
// --- ADC_Process waits for 5 mSec boundry from timer. -------------------------------------
  while (!adcFlag);                         // WAIT until timer says "run ADC", (adcFlag == true)
  adcFlag = false;                          // clear for next timer event.
  digitalWrite(LED_Pin, HIGH);              // # TESTPOINT for firmware timing.  Timer Tick.

  ADC_Process();                    // get ADC data, Select Segment, Scale, Filter.
                                    // result in global adcFiltOut15 
  ENC_Process();       // check for encoder knob changes & process
  // * * * * * * * * * * End of every 5mS tasks

// ******************************************************************************   
  switch (count100mS) {                        // Assign Time Slots for tasks
    case TT_OSync:
        digitalWrite(O_Sync_pin, HIGH);        // TESTPOINT for firmware timing.
        digitalWrite(O_Sync_pin, LOW);         // O_Scope synch
        break;
// * * * * * * * * * * * * * * Task done every 50mSec * * * * * * * * * * * * * *
     case TT_LogP_A:  // T1 
     case TT_LogP_B:  // T11
        if (digitalRead(LUT_pin) == HIGH) {     // Lookup table select input pin
        dataBus15 = GetPwrData(adcFiltOut15);   // Change LOG data to PWR data, if switch set.
        }
        else dataBus15 = adcFiltOut15;
        break;
        
     case TT_Tau_A:   // T2    
     case TT_Tau_B:   // T12
        dataOut15 = TauFilter(dataBus15);   // do "integrator" RC filter time step
        break;
      
     case TT_Tone_A:  // T3
     case TT_Tone_B:  // T13
        Set_Tone(dataOut15);                // 50mStone update needed for smoothness
        break;
        
      case TT_UDS_Cmd_A:  // T4
      case TT_UDS_Cmd_B:  // T14
        CmdProcess();                       // check for command from Radio SkyPipe 
        break;

 // * * * * * * * * * * * * * * Task done every 100mSec * * * * * * * * * * * * * *
       case TT_SW_Read:    // T5
        ReadSwitches();                       // read push button switches
        break; 

      case TT_PWM:        // T6
        Set_PWM(dataOut15);                            // update PWM's outputs
        break;
         
      case TT_GETD:       // T7
        if (( !poll ) && (stat == 3)) {       // time to PUSH UDS data if PUSH mode
          GETD();                             // send data to PC => SkyPipe UDS
        }
#if Sig_debug  // testpoint
       Serial.println(adcFiltOut15);          // Should take about 7 mSec.
#endif      
        break; // from case TT_GETD:
        
      case TT_Auto_Nul:    // T8
        if (digitalRead(AutoNulPin_L) == LOW) // if AutoNul switch pushed
            autoNul();                        //
        break; 
     
      default: 
        break; 
  }     // End of switch (count100mS)

  
  count100mS++;          
  if (count100mS >= Ticks) count100mS = 0;
  digitalWrite(LED_Pin, LOW);       // test point to observe end of loop timing 
} 
//******************************************************************************* END of MAIN LOOP

// ************** functions used by main programe ******************************************
// *****************  command processor,  From Radio Sky with mods *******************
void CmdProcess() {
  while (Serial.available() > 0) { // very long while loop.
    incomingByte = Serial.read();  // read the oldest byte in the serial buffer:

    // **** if it's a K we stop (KILL):
    if (incomingByte == 'K') {
      DumpChar(3);  //GET PAST THE REST OF THE WORD by Reading it.
      stat = -1;
#if UDS_debug
      Serial.println("^^1002 DEAD"); // Just for troubleshooting
#endif
    }
    // **** if it's an I run the INIT code if any
    if ((incomingByte == 'I') && (stat == 0)) {
      DumpChar(3);  // GET RID OF 'NIT'
      stat = 1 ;
#if UDS_debug
      Serial.println("^^1002 INITIALIZED ");
#endif
    }
    // **** if it's an L RSP will have to POLL for data
    if (incomingByte == 'L') {
      DumpChar(1); // GET RID OF 2nd 'L' in POLL
      poll = true;
#if UDS_debug
      Serial.println("^^1002 POLLING ");
#endif
    }
    //**** if it's an H sets it to push.  Arduino will PUSH data.
    if (incomingByte == 'H') {
      poll = false;
#if UDS_debug
      Serial.println("^^1002 PUSHING ");
#endif
    }

    //**** if it's a C then Radio-SkyPipe is requesting number of CHANnels
    if (incomingByte == 'C') {
      DumpChar(3);  // GET RID OF 'HAN'
      // change the last digit to = digit of channels of data (ex. 1)
      Serial.print("^^20131");
      Serial.write(255);              // print result;
      stat = 2;                       // ready to go
#if UDS_debug
      Serial.println("");
#endif
    }

    //****  if it's an "A" means STAT was requested so send UDS ready message
    if (incomingByte == 'A' ) {
      DumpChar(1);                    // GET RID of final 'T'
      Serial.print("^^1001");
      Serial.write(255);              // print result
      stat = 3;
#if UDS_debug
      Serial.println("");
#endif
    }

    //***** if it's an D we should send data to RSP:
    if ((incomingByte == 'D') && poll && (stat == 3) ) {
      GETD() ;
#if UDS_debug
      Serial.println(" DATA REQUEST RECEIVED ");
#endif
    }

    if (stat == -1) stat = 0;     // clear kill state automatically
  }   //**** End of "while (Serial.available() > 0)"
  // we are finished processing any incoming commands from the PC
}     //**** end of function cmdProcess

/**********************************************************
     Initial setup of all I/O ports
*/
void InitPort() {
  pinMode(ToneOutAF, OUTPUT);
  T1_Tone_Off();                      // turn off TIMER1 output of tone

  // **** Set up rotary encoder pins
  pinMode(EncoderPinA, INPUT_PULLUP);
  pinMode(EncoderPinB, INPUT_PULLUP);
  pinMode(EncoderPush, INPUT_PULLUP);
  
  pinMode(LUT_pin, INPUT_PULLUP);
  pinMode(AudioSelPush, INPUT_PULLUP);
 
  pinMode( AutoNulPin_L, INPUT_PULLUP);
  
  digitalWrite(DetSel, DetAFOff);     // preset to disable state, stop noise burst at powerup.
  pinMode(DetSel, OUTPUT);            // Selects detector audio off at power up 
  pinMode(O_Sync_pin, OUTPUT);
  digitalWrite(O_Sync_pin, LOW);        // test point for firmware timing.
  
  pinMode(LED_Pin, OUTPUT);
}

/**********************************************************
   SETUP TIMERS 1 and 2 for special use
   WARNING:  PWM pins 3, 9, 10, and 11 no linger work.  SERVO library no longer works !!!   
*/
// Set up TIMER1 for High resolution tone .
void TIMER_setup() {
  // Setup timer IRQ for tone generation.
  // WARNING: This steals Timer1 from PWM's on Pins 9 and 10.  Also breaks servo library.
  noInterrupts();               // disable all interrupts
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;

  OCR1A = Pitch700;             // compare register
  TCCR1B |= (1 << WGM12);       // CTC mode
  TCCR1B |= (1 << CS10);        // 1x prescaler

  TIMSK1 |= (1 << OCIE1A);      // enable timer compare interrupt
  TCCR1A |= (1 << COM1A0);      // pin 9 gets timer toggle output

  //   Set up TIMER2 for ADC interupt
  // WARNING: This steals Timer2 from PWM's on Pins 3 and 11.  Also breaks tone() function
  TCCR2A = 0;
  TCCR2B = 0;
  TCNT2  = 0;
  OCR2A  = TMR2_Period;         // compare register
  TCCR2A |= (1 << WGM21);       // CTC mode
  TCCR2B |= ( (1 << CS20) | (1 << CS21) | (1 << CS22));        // 1924x prescaler & start clock
  TIMSK2 |= (1 << OCIE2A);      // enable timer compare interrupt
  interrupts(); // enable all interrupts
}

/**********************************************************
   This is where data is fetched and sent on to RSP.
*/
void GETD() {
  Serial.print("#0");         // # followed by channel number of data
  Serial.print(dataOut15);    // Scale OK,  Fullscale = 32767
  Serial.write(255);
  Serial.print("^^3001");     // This tells RSP to time stamp it
  Serial.write(255);          // all commands end with this character.
#if UDS_debug
  Serial.println("");
#endif
  return;
}

/*********************************************************************
   This pulls characters from serial port buffer & throws them away.
   It is needed because we decode only one character of commands.
*/
void DumpChar(int charCount) {
  int loops;
  for ( loops = 1; loops <= charCount; loops++ ) {
    incomingByte = Serial.read();
  }
  incomingByte = 0;
}

/****************************************************************************************
   ADC_Process to global output variable adcFiltOut15.  Output full scale = 32736.
   ADS1115 runs here after flag from timer 2 kicks it off.
   Gets data from adcBox, and scales as needed for adcFiltOut15 global.
   Filter is 1 pole IIR low pass with shifts making filter K.  About 64mS RC
   Process takes ### uSec after adcFlag from timer becomes true.

   Waits here until flaq from ADC IRQ Process.  NOT BEST BUT WORKS !!!
 ***************************************************************************************/
void ADC_Process() {
  long adcFiltInL;                          // IIR filter processes ADC reads to smoothed value
  static long adcFiltMemL;                  // Memory for IIR filter

   adcBox = read_ADS ( ADS_Adr );            // I2C get data from LAST conversion
  write_ADS( ADS_Adr,  ADS_Control);        // I2C kick off NEXT ADS1x15 conversion.  Range = -32767 to about -4000
  adcBox += ADCmagic;                       // Fix input offset and log segment offset
  adcBox += offsetSet_i_FS;                    // offset fixed now.
  if ( adcBox < 0 ) adcBox = 0;             // log segment Lo limit
  if (adcBox > LogRange) adcBox = LogRange; // log segment Hi limit
  adcBox *= LogScale;                       // Expand segment to 15 bits,  0 to 32767 range

#if Sig_debug
 //     Serial.println(adcBox);            // Just for troubleshooting
#endif
   
//******   Now filter with one pole IIR low pass at about 80mS equiv RC *******************
  adcFiltInL = (long)adcBox;                // get ADC data 32736 full scale
  adcFiltInL = adcFiltInL << 15;            // scale by 2^15 to use long data type
  adcFiltInL -= adcFiltMemL;                // get delta step in
  adcFiltInL = adcFiltInL >> ADCFiltK;      // fixed filter K for ADC done with a shift
  adcFiltMemL += adcFiltInL;                // filtered output in analogFilOut
  adcFiltOut15 = (adcFiltMemL >> 15);       // scale back to fit in int data type output
}                                           // ADC, Segmnet, Scale, Filter Complete

/******************************************************************************************
  Gets power signal from log signal.  Full Scale here is 32767 input and output.
  The exponential output from log input is done with a 1024 length look up table.
  uses linear interpolation for full 15 bit operation.
*/
int GetPwrData(int inp15) {
  int indx, tOut, tOut2, delta, out, testH;
  indx = inp15 >> 5;                       // 0 ... 1023 here is index
  if (indx < 0)    indx = 0;                        // stay in table!!
  if (indx > 1023) indx = 1023;                     // Table in PROGMEM space
  tOut  = pgm_read_word_near(&lookup[indx]);        //  "= lookup[indx]" DOES NOT WORK
  tOut2 = pgm_read_word_near(&lookup[indx + 1]);
  delta = tOut2 - tOut;                             // slope info
  testH = inp15 & 0x1F;                      // get  lsb's for interpolate test
  testH *= delta;
  testH = testH >> 5;
  out = tOut + testH;
  return out;
}

/****************************************************************************************
   TauFilter is basic RC time constant for Radio Telescope. NOTE STATIC MEMORY!!!  NOT reentrant!
   Output full scale = 32736
   Main loop will normally run this about every 50mS.
   Internal calcs are 32 bit to reduce round off errors.  
*/
int TauFilter(int input) {
  long rcFiltIn;
  static long rcFiltMem;              // Memory for IIR filter
  tauFil_Exp2 = tauFil_Sel>> TauFil_Sca;       // Sel is 8 x size for smooth control
  rcFiltIn = input;                   // put input in 32 bits
  rcFiltIn = rcFiltIn << 15;          // scale by 2^15 to use long data type
  rcFiltIn -= rcFiltMem;              // get delta step of input
  rcFiltIn = rcFiltIn >> tauFil_Exp2; // Divide it by typical 32 for filter K of 1/32.
  rcFiltMem += rcFiltIn;              // filtered output in analogFilOut
  return (int)(rcFiltMem >> 15);        // scale back to fit in int data type
}

/****************************************************************
  Read shaft encoder knob and process changes.  
  Changes offfset value for ADC preamp via a PWM.  Offset now in sftware!
  If knob is pushed in,  Changes IIR filter time constant.
  Runs every 5 mS or so. 
  Encoder inputs have 30K to 50K pullups in uP enabled.  Contacts to Gnd.
 Input Pin  DeBounce:
  Enc ---- 3K3 Res ---- 0.033u to Gnd ----- Port Pin.
  Give 100uS / 1000uS for RC Lo vs RC Hi. 
*/
void ENC_Process() {                                
  byte enc_port;
  enc_port = PIND;                         // simultainious pin read                 
 if (enc_port & EncA_Mask) encoderA = true; else encoderA = false;
 if (enc_port & EncB_Mask) encoderB = true; else encoderB = false;
 if (encoderA != encoderZ)     {                // detect any edge change on A
    if (encoderB != encoderA) {   // EncoderB sorts CW vs CCW 
     // Decrease offset or filter K
      if ( digitalRead(EncoderPush) == HIGH)    // NOT pushed
        offsetSet_i--;                           // Change offset down
      else tauFil_Sel--;                       // Change to shorter RC time equiv
    }
    else  {
      // Increase offset or filter K
      if ( digitalRead(EncoderPush) == HIGH)    // NOT pushed
        offsetSet_i++;                           // Change offset up
      else tauFil_Sel++;                       // Change to longer RC time equiv
    }

    // Check limits here for offset size and filter K size
    // Both are byte size values
    if ( offsetSet_i < 0 )   offsetSet_i = 0;
    if ( offsetSet_i > OffsetMax ) offsetSet_i = OffsetMax;

    if (  tauFil_Sel < TauFil_Sel_Min) tauFil_Sel = TauFil_Sel_Min;
    if (  tauFil_Sel > TauFil_Sel_Max) tauFil_Sel = TauFil_Sel_Max;
  }                                             // end of detect edge
  encoderZ = encoderA;            // to catch state change NEXT time
}                                 // end of ENC_Process

/****************************************************************
  Read push button switches and process changes.  In 100mSec loop.
  Note that a pushed switch is LOW !!!
  Measurement process STOPS during EEPROM update!
  AF routing switch.
*/
void ReadSwitches() {
  byte i;
  af_Sw = digitalRead(AudioSelPush);
  if ( (af_Sw == LOW) && (af_Sw_Z == HIGH)) {   // if High to Low edge detected on pin
    audioSelZ = audioSel;                       // to restore after EEPROM write
    audioSel++;
    if (audioSel > Det_AF) audioSel = No_AF;
  }                                             //  x if High to Low edge detected
  af_Sw_Z = af_Sw;                      // setup for NEXT edge detect

  if (af_Sw == HIGH) sw_Count = 0;      // for EEPROM
  if (af_Sw == LOW) {                   // Switch pushed ?
    sw_Count++;
    if (sw_Count > SW_Detect) {         // if a Long Push then do EEPROM write
      audioSel = audioSelZ;             // so EEPROM write dowes not change state
      PutEE();
      for (i = 0; i < 3; i++) {         // Make 3 beeps
        t1Period = Pitch700;            // Need to set pitch for 700 Hz here
        T1_Tone_Off();                  // turn off TIMER1 output of tone
        delay(100);                     // blocking of ADC IRQ by delay is OK here.   update of EEPROM causes lost samples (OK)
        T1_Tone_On();                   // turn off TIMER1 output of tone
        delay(300);                     // blocking of ADC IRQ by delay is OK here.
        T1_Tone_Off();                  // turn off TIMER1 output of tone
      }                                 // x Make 3 beeps
      while (af_Sw == LOW) {            // wait for pin HIGH to exit (Push released)
        af_Sw = digitalRead(AudioSelPush);
      }                                 // x wait for HIGH to exit
    }                                   // x if a Long Push then do EEPROM write
  }                                     // x Switch pushed ?
}                                       // x function ReadSwitches()

/****************************************************************
  Set Tone Pitch from Global dataBus15 value of 32736 Full Scale
  possible range of about 200 to 2200Hz.
  Also Selects audio path.   ### need to check time usage on this !!!
*/
void Set_Tone(int input15) {
  long toneFreq16L, tempIL;
  switch (audioSel) {                         // Do audio path select first
    case No_AF:
      T1_Tone_Off();                          // turn off TIMER1 output of tone
      digitalWrite(DetSel, DetAFOff);         // Power down AF amp
      break;
    case Tone_AF:
      digitalWrite(DetSel, DetAFOff);         // Power down detector to AF amp path
      toneFreq16L = (long)input15 + InitTone;    // make 32 bit & add offset freq
      tempIL = K_T1_FP / toneFreq16L;         // 32 bit calc needed
      t1Period = (unsigned int)tempIL;        // back to 16 bits for timer.  TimerIRQ loads this into timer reg
      T1_Tone_On();
      break;
    case Det_AF:
      T1_Tone_Off();                          // turn off TIMER1 output of tone
      digitalWrite(DetSel, DetAFOn);          // Power up detector AF amplifier
      break;
  }
}
/****************************************************************
  Enable timer1 tone output.
*/
void T1_Tone_On() {
  pinMode(ToneOutAF, OUTPUT);     // turn on TIMER1 output of tone
}

/****************************************************************
  Disable timer1 tone output.
*/
void T1_Tone_Off() {
  pinMode(ToneOutAF, INPUT);      // turn off TIMER1 output of tone
}

/****************************************************************
  Set PWM from Output data value.
*/
void Set_PWM(int input15) {      
  int tempI;
  if ( digitalRead(EncoderPush) == HIGH)  {             // NOT pushed, Output = PWM of signal
    analogWrite(Meter_pin, input15 >> 7);               // 8 bit data to PWM for main output
  }
  else  {                                               // Knob IS pushed, Output filter K displayed
    tauFil_Exp2 = tauFil_Sel >> TauFil_Sca;
    tempI = tauFil_Exp2 * TauMtrStep;    // Display & filt OK   
    analogWrite ( Meter_pin, tempI );                   // shows log2(filt K)
  }
   analogWrite ( Offset_pin, (byte)(offsetSet_i >>2) );   // Scale back to byte for PWM software offset now. ### leave for debug
   offsetSet_i_FS = offsetSet_i * OffsetScale;                // scale for from 10 to 15 bitsADC processing 
   
}

/****************************************************************
  Gets EEPROM data.  If key is set, puts data in proper variables.
  Must match PutEE.
*/
void GetEE() {
  eeData = EEPROM.read(EEADR_Key);
  if (eeData == EE_Key_Value) {                 // if there is data, get it
    offsetSet_i  = ((EEPROM.read(EEADR_OfstH)) << 8);       // MS bits in place.
    offsetSet_i |= (EEPROM.read(EEADR_OfstL));       // or in LS bits.
    tauFil_Sel   =  EEPROM.read(EEADR_Filk);
    if ( tauFil_Sel < TauFil_Sel_Min) tauFil_Sel = TauFil_Sel_Min;  // Check.  Wrong value is a mess!
    if ( tauFil_Sel > TauFil_Sel_Max) tauFil_Sel = TauFil_Sel_Max;
    audioSel = EEPROM.read(EEADR_ASel);
  }
}

/****************************************************************
  Puts data in EEPROM.  Includes key.
  Must match GetEE.
*/
void PutEE() {
  EEPROM.write(EEADR_Key,   EE_Key_Value);
  EEPROM.write(EEADR_OfstH, (byte)(offsetSet_i >> 8));       //
  EEPROM.write(EEADR_OfstL, (byte)(offsetSet_i & 0xff));     //divide by 4 so it fits in byte
  EEPROM.write(EEADR_Filk,  tauFil_Sel);
  EEPROM.write(EEADR_ASel,  audioSel);
}

//**************************** TI ADS1115 I2C drivers ***********************************************
// WARNING:  These two functions are NOT reentrant!!!  Can't be inside timer IRQ or it locks up ???
// They are OK in main loop
// ****  Write word to ADS1115 configuration/command register  ****
static volatile void write_ADS( uint8_t i2C_Adr, uint16_t value) {
  Wire.beginTransmission(i2C_Adr);
  Wire.write((uint8_t)1);                       // Reg 1 is configuration data
  Wire.write((uint8_t)(value >> 8));            // Send Hi Byte
  Wire.write((uint8_t)(value & 0xFF));          // Send Lo Byte
  Wire.endTransmission();
}

// ****   Read word from ADS1015 conversion resuilt register  ****
// returns 16 bit ADC value
static volatile uint16_t read_ADS ( uint8_t i2C_Adr) {
  Wire.beginTransmission(i2C_Adr);
  Wire.write((uint8_t)0);                     // Reg 0 is conversion data
  Wire.endTransmission();
  Wire.requestFrom(i2C_Adr, (uint8_t)2);      // gets 2 bytes
  return ((Wire.read() << 8) | Wire.read());
}

/*********************************************************************************************************
* Called from 100mS task if AutoNul button pushed to request calibration.  
* Evaluates Filtered Scaled ADC value  and adjusts Offset PWM up or down as needed to center ADC.      
* I/O through global variables                */ 
void autoNul() { 
  int nullErr = adcFiltOut15 - HalfScale15;         // Delta Null Error.  Range about +/-16383
  nullErr = nullErr >> 9;                           // Range now +/-31 
  offsetSet_i -= nullErr;
// Check limits here for offset size.  Prevent unsigned byte overflow or underflow
    if ( offsetSet_i < 0 )    offsetSet_i = 0;
    if ( offsetSet_i > OffsetMax ) offsetSet_i = OffsetMax;
}
// ****************** THE END... of programme ***********************************************************
