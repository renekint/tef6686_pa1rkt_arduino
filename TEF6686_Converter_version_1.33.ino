// TEF6686 FM tuner Control module 
// Converter compatible version by PA1RKT (p)2020-2021

// TEF6686 Converter version 1.33 by PA1RKT - changes since 1.32
//  * Fixed a bug that affected STEREO/MONO/MUTE and signal level display (thanks to Arnold/GJ)


// TEF6686 Converter version 1.32 by PA1RKT - changes since 1.31
//  * Changed to hd44780 library for LCD control - expensive in kilobytes but much more modern and flexible
//    * Some displays exposed problems with 1.31 and the libraries used (thanks to PA1POV and PA3FZB)
//  * Various small code optimizations, needed because the hd44780 library uses much more memory ;-)
//
// TEF6686 Converter version 1.32 by PA1RKT - changes since 1.3
//  * Setting manual BW and Squelch improved:
//    * Fixed some race conditions in which a 0kHz BW setting could stick
//    * Switching from manual to auto BW and back is now robust
//    * Squelch is now implemented correctly (hard mute on signal level), squelch range -10..+50 dBuV
//    * During squelch setting, squelch level is shown in place of signal strength and live on your ears ;-)
//    * Squelch level is stored in EEPROM and read at startup
//  * Completely new framework for checking button presses
//    * Non-blocking for interrupts
//    * Waits until button is released (so blocking for code), which is the intention
//    * Anti-bounce code (BUTTON_PRESS_SHORT is the debounce time)
//    * Supports an arbitrary number of distinct press-times (now only short and long, but can be extended easily) 
//  * First refactoring effort of RDS code
//
// TEF6686 Converter version 1.3 by PA1RKT - changes since 1.21
// * Added some comments in the code to make it readable for humans ;-) - this will be continued in future versions
// * Compliant to tight Arduino IDE C++ standards (no warnings etc.)
// * Compatibility with version 1.1.2 of LiquidCrystal_I2C library 
//    * Parameters to lcd.begin() are now mandatory - this might break compatibility with older library versions
// * Moved TEF initialization and filter map constants to tef_init.h
// * Code centered around real TEF frequency (REG_FREQ) (as in 1.0 :-))
//    * Display is corrected with LO frequency
//    * Presets contain real TEF frequency, preventing strange frequencies when changing LO 
// * Choosing a preset works! Code in 1.21 was completely bogus :-)
//    * Long press the encoder button while in operational mode until screen blinks
//    * You are now in PRESET mode, where you van store or retrieve a preset
//    * Scroll (with rotary encoder) to the preset you want to store / retrieve
//    * Short press = retrieve, long press = store current freq. (overwriting what was there)
// * Complete overhaul of displayInfo() routine - for showing info on the LCD - and underpinnings: 
//    * now based on the concept of displayBlocks, max 32 per menuLevel
//    * a displayBlock is a bit in the displayBlocks parameter - bit ON means that the info
//      for this block has changed. displayInfo checks all bits and refreshes only the changed displayBlocks
//    * the bits are set in the code that actually knows anything had changed, so a first
//      'separation of concerns' is implemented - this will be taken further in later versions
//    * this (already in this version) leads to cleaner and better readable code
// * The magic number to mark a preset in memory is now a #define
//    * Cleaner code
//    * Easy to change when we produce a new version, you can reuse your Arduino with a clean preset sheet ;-)
//
// TEF6686 Converter version 1.21 by PA1RKT - minor bug fix
// * Fixed a frequency display <100MHz error (thanks to ODJeetje)
//
// TEF6686 Converter version 1.2 by PA1RKT - changes since 1.1
// * Still pretty simple, but a few advanced features are emerging ;-)
// * Further restructuring and refactoring of original 'stailus'code, much! better than 1.0 - much faster and efficient now
// * Removed all XDR-GTK communications, will be re-programmed in a future version
// * dBuV truncated at 1 dBuV boundaries
// * There are 10 presets to be programmed, but only 0 is used (for startup frequency)
// * LO frequency (for use with converter) is defined by long-press encoder switch at startup screen
//    * Selectable from predefined list (PA3FZB provided LO's): 0, 340, 1200, 2300 MHz
//    * Select LO frequency with encoder and press encoder switch to acknwledge
// * While in operation, a short click of the encoder switch takes you to operational settings:
//    * First short click: "MHz" will flash, to signal that you can now tune in 1 MHz steps instead of 50kHz
//    * Second short click: "kHz" will flash, indicating you can now set a manual FM bandwidth - recieve will fix on this bandwidth
//    * Third short click: "dBuV" will flash, indicating that you can now adjust the squelch level (which is NOT shown)
//    * Last short click: go back to normal operation screen
//    * If you long press when the "MHz" sign flashes, you will go back to the normal operation screen immediately
//    * If you long press when the "kHz" sign flashes, you re-enter automatic bandwidth mode
// * If you long press the encoder switch while in normal operation mode, you will enter the SAVE TO PRESET screen:
//    * Rotate encoder to select the memory preset where you want to store the frequency and click again to save
//    * NOTE: In version 1.2, ONLY preset #0 is really useable - it defines the startup frequency of the tuner
// * I found that this code can be still be refactored for speed and efficiency, ongoing effort
// * RDS acquisition code still not touched and quite original (it shows ;-)

// Version 1.1: changes since 1.0
// * Correct dBuV (including negative numbers), down to .1dBuV accuracy
// * Bandwidth information (between < > in KHz)
// * FM Parameters update thanks to PE1ODJ
// * Removed some XDR-GTK communications
// * Added EEPROM routines to write some startup info to non-volatile memory (starting frequency, converter lo frequency)
// * Starting frequency and converter LO frequency in setup() is read from EEPROM (set by clicking the encoder)
// * LO frequency is selectable with the encoder at startup by pressing the encoder switch briefly, select and press again
// * Starting frequency is set to current frequency by pressing the encoder switch briefly while in operation (screen will flash)
// * Scrolling radio program information


// Version 1.0
// * Very very basic version for 20x4 LCD, based on original code (see below)
// 
// TEF Tuner module control by PA1RKT, suitable for converter use.
// Once upon a time, I found some code on the 'net - see below. That got me started. 
// "
//  TEF6686HN/V102 xdr-gtk控制程序
//  作者 eggplant886
//  rds add by stailus - base from https://github.com/stailus/tef6686_rds
//  niq_ro added some switches using stailus hints
//  niq_ro added encoder for volume
//  niq_ro added LCD2004 (i2c) 
//  niq_ro added RT and PI from RDS data using (again) hints from stailus
//  niq_addded PTY
//  niq_ro added RT from makserge - https://github.com/makserge/tef6686_radio
//"

#include <Wire.h>
#include <EEPROM.h>
#include <Arduino.h>
#include <hd44780.h>
#include <hd44780ioClass/hd44780_I2Cexp.h>  // v1.32: i2c expander i/o class header for MCF8574 controller
                                            
#include "tef_init.h"           // Header file with TEF initialization data and filter map constants

#define FALSE 0
#define TRUE  1

#define STARTUPFREQ_ADDRESS   0x0000  // Memory address start on EEPROM to store startup freq. (4 bytes)

// v1.2 settings EEPROM addresses (0x0000 - 0x0FFF)
#define LOFREQ_ADDRESS        0x0004  // LO freq                  (4 bytes)
#define BAND_EGDE_LO_ADDRESS  0x0008  // CONVERTER_FREQ band edge (4 bytes)
#define BAND_EDGE_HI_ADDRESS  0x000C  // CONVERTER_FREQ band edge (4 bytes)
#define SCAN_EDGE_LO_ADDRESS  0x0010  // Scan edge low frequency  (4 bytes)
#define SCAN_EDGE_HI_ADDRESS  0x0014  // Scan edge high frequency (4 bytes)
#define TUNE_STEP_ADDRESS     0x0018  // Tuning step size (* 1Hz) (4 bytes)
#define SQUELCH_LEVEL_ADDRESS 0x001C  // Squelch level            (2 bytes)
#define PRESET_START_ADDRESS  0x0100  // Start of Presets (MAX_PRESETS * sizeof(preset))

// v1.2 Preset definition
#define PRESET_MAGIC  0xBB  // Magic number as marker for preset in memory
#define MAX_PRESETS   10    // Max number of presets, you can define any number
                            // as long as there is enough EEPROM mem for them
typedef struct 
{
  byte magic = PRESET_MAGIC;      // Needed to check for presence of this preset;
                                  // if this memory place is set to any other value
                                  // there is no preset assumed 
                                  // at this mem location anymore. This way we can 
                                  // have a number of presets that is only limited
                                  // by available memory. Beside, we don't need arrays
                                  // of presets in this code - that would take up much
                                  // sram memory on our precious little arduino ;-)
  uint32_t freq;
  char presetName[16];
  uint32_t tuneStep;
  uint32_t reserved1;
  uint32_t reserved2;
} preset;

preset thisPreset;                // Current preset (to read or write)

short indexPreset, oldIndexPreset;

// v1.3 Display blocks (for displayInfo to check what to draw)
// Bitfield that defines which block(s) have been changed,
// so DisplayInfo(uint32_t displayBlocks) will only refresh screen elements that have changed
// 32 bits means we can define a max of 31 display blocks per menuLevel
//  NOTE: the block definitions are reusable over 
//        menuLevels, but make sure within the menuLevel
//        they do not overlap...that's the weak point now
#define DB_STARTUP_TXT  0x0001    // Startup text screen

#define DB_FREQ         0x0001    // Operational frequency
#define DB_FREQ_UNIT    0x0002    // Unit of operational frequency (MHz or KHz)
#define DB_SIGNAL_LEVEL 0x0004    // Signal level display
#define DB_SIGNAL_UNIT  0x0008    // Signal level unit (dBuV)
#define DB_BW_TXT       0x0010    // Bandwidth text (BW:)
#define DB_BW_LEVEL     0x0020    // Bandwidth
#define DB_BW_UNIT      0x0040    // Bandwidth unit (kHz)
#define DB_STEREO_MONO  0x0080    // Stereo, Mono, Mute signal display
#define DB_RDS_SRVNAME  0x0100    // RDS Service name (name of radiostation)
#define DB_RDS_LONGTXT  0x0200    // RDS long radiotext

#define DB_SETTINGS_TXT 0x0001    // Settings screen text
#define DB_SET_LO_TXT   0x0002    // LO setting text ("Set LO freq" or similar)
#define DB_SET_LO_FREQ  0x0004    // LO frequency
#define DB_SET_LO_UNIT  0x0008    // LO unit (MHz)

#define DB_SET_FREQSTEP 0x0001    // Set frequency step
#define DB_SET_BW_MAN   0x0002    // Manual bandwidth (operational setting)
#define DB_SQL_LVL      0x0004    // Squelch level (operational setting)

#define DB_PRESET_TXT   0x0001    // Store/retrieve preset text
#define DB_PRESET_IDX   0x0002    // Store/retrieve preset index
#define DB_PRESET_INF   0x0004    // Store/retrieve preset info

// Version 1.31 new framework for checking button presses
// See buttonPress() routine below
#define BUTTON_PRESS_LONG     400 // Milliseconds for long button press
#define BUTTON_PRESS_SHORT     10 // Milliseconds for short button press (anti-bounce)

// v1.2 Menu levels for rotary encoder use
// Menu level up/down by button or rotary click?
byte menuLevel = 0; // 0 = No menu level, operational info
                                  // 1 = rotary switch pushed at startup
                                  // 2 = operational settings general
                                  // ...see code for details but you catch the idea
bool  firstStart = TRUE;          // First time we enter a menuLevel, can be handy i.e.
                                  // to display all necessary displayBlocks

// Hardware switches (for future use)
#define sw1 5
#define sw2 6
#define sw3 7
#define sw4 8
#define sw5 9
#define sw6 10

// Rotary encoder
#define ROTARY_DT   2
#define ROTARY_CLK  3
#define ROTARY_SW   4

volatile int counter = 0;
volatile int currentStateCLK;
volatile int lastStateCLK;

int signalLevel = 0, oldSignalLevel = 0;      // Signal level from antenna (.1 dBuV)
int squelch = 0;                              // Squelch threshold (.1 dBuV)

int16_t nDeemphasis, volume;
uint32_t freq;
uint16_t stereoIndicator=0 , 
          oldStereoIndicator=0;             // Stereo indicator:
#define STEREO  0x0001                      // bit 0 = mono / stereo
#define MUTE    0x0002                      // bit 1 = signal muted

//RDS stuff (to be refactored in 2.0)
#define MAX_RDSRADIOTEXT  64            // Size of RDS Radio Text buffer (+1 for \0 char)
                                        // Note: RDS standard is max 64 so maybe problem with long txt
char rdsProgramId[5];
char rdsProgramService[9];
char programServiceUnsafe[9];
uint8_t prevAddress = 3;
uint32_t psErrors = 0xFFFFFFFF;
bool psAB;
uint8_t psCharIsSet = 0;
char unsafePs[2][8];
char rdsProgramServiceUnsafe[9];
char rdsProgramType[17];
char rdsRadioText[MAX_RDSRADIOTEXT];
uint8_t rdsAb;
uint8_t isRdsNewRadioText;

short scrollIndex;
bool blinkOn = TRUE;

#define FREQMIN 6500          // Band /scan edge of the TEF (low) - fixed for now
#define FREQMAX 10800         // Band /scan edge of the TEF (high) - fixed for now

uint32_t loFreqs[] = {0, 34000, 120000, 230000};    // LO freqencies (*10kHz) that we can cycle through (v2.0 menu)
uint32_t LO_FREQ;                                   // Local oscillator in case of front-end converter (0, 34000, 120000, 230000)
short indexLoFreq;                                   // Index in loFreqs array

volatile uint32_t REG_FREQ = 8990;                  // Actual frequency of TEF receiver (the IF frequency if you wish)
volatile uint32_t CONVERTER_FREQ;                   // Frequency shown 
uint32_t displayFrequency;                          // Frequency in display (in case of presets and other virtual frequencies)

uint16_t bandwidth, oldBandwidth;                   // Bandwidth 
bool  autoBandwidth;                                // Auto bandwidth vs manual  
short indexBandwidth = 0;                           // Index in FM filtermap (bandwidth)

uint32_t timer = 0;           // Signal level reporting timer
uint32_t timer_rds = 0;       // RDS reporting timer
uint32_t timer_display = 0;   // Display LCD refresh timer
uint32_t timer_blink = 0;     // Timer for blinking characters  

int8_t current_filter = -1;   // Current FIR filter (-1 is adaptive)
int8_t current_set = -1;      // Current FIR filter (-1 is adaptive)

// Scan variables - why are these global? ;-)
uint16_t  scan_start = 0;
uint16_t  scan_end = 0;
uint16_t  scan_step = 0;
uint8_t   scan_filter = 0;

#define TIMER_INTERVAL            250
#define RDS_TIMER_INTERVAL        250
#define DISPLAY_TIMER_INTERVAL    100
#define BLINK_TIMER_INTERVAL      250

byte DSP_I2C = 0x64;            // I2C interface to TEF6686 tuner module

hd44780_I2Cexp lcd;             // v1.32: Auto-detect i2c address, can be forced with (0x27) as parameter

//-------------------------------------------------------------------------------------------------------------

void Write(uint8_t *buf, uint8_t len)
{
  Wire.beginTransmission(DSP_I2C);
  Wire.setClock (400000);
  for (int i = 0; i < len; i++)
    Wire.write(*buf++);
  Wire.endTransmission();
}

void Read(uint8_t *buf, uint8_t len)
{
  uint8_t lenrec = Wire.requestFrom(DSP_I2C, len);
  for (int i = 0; i < lenrec; i++)
    *buf++ = Wire.read();
}

void EEPROM_write32( uint16_t address, uint32_t value )
{
  byte *p = (byte *)(void *)&value;
  for( int i=0; i<sizeof(value); i++ )
    EEPROM.update( address++, *p++ );
}

uint32_t EEPROM_read32( uint16_t address )
{
  uint32_t value = 0;
  byte *p = (byte *)(void *)&value;
  for( int i=0; i<sizeof(value); i++ )
    *p++ = EEPROM.read( address++ );

  return value;
}

void EEPROM_write16( uint16_t address, uint16_t value )
{
  byte *p = (byte *)(void *)&value;
  for( int i=0; i<sizeof(value); i++ )
    EEPROM.update( address++, *p++ );
}

uint16_t EEPROM_read16( uint16_t address )
{
  uint16_t value = 0;
  byte *p = (byte *)(void *)&value;
  for( int i=0; i<sizeof(value); i++ )
    *p++ = EEPROM.read( address++ );

  return value;
}

void EEPROM_writePreset( byte index, preset myPreset )
{
  uint16_t address = PRESET_START_ADDRESS + (index * sizeof(preset));
  byte *p = (byte *)(void *)&myPreset;
  for( int i=0; i<sizeof(myPreset); i++ )
    EEPROM.update( address++, *p++ );
}

// Read a preset with an index number from EEPROM.
// The code will first look for the magic number at the indexed preset location.
// If it sees the magic number, it assumes there is a preset there.
// With this algorithm we can avoid long arrays of presets in RAM.
bool EEPROM_readPreset( byte index, preset *myPreset )
{
  bool isPresent = FALSE;
  uint16_t address = PRESET_START_ADDRESS + (index * sizeof(preset));
  byte *p = (byte *)(void *)myPreset; 
  if( EEPROM.read(address) == PRESET_MAGIC )    // Look for the magic number (v1.3)
  {
    isPresent = TRUE;
    for( int i=0; i<sizeof(preset); i++ )
      *p++ = EEPROM.read( address++ );
  }
  return isPresent;
}

// A real mod function. Also works for negative numbers.
int mod( int x, int y )
{
   return x<0 ? ((x+1)%y)+y-1 : x%y;
}

uint32_t buttonPress( byte button )
{
  uint32_t startTime;
  uint32_t timeLapse;

  startTime = millis();
  while( digitalRead(button) == LOW );
  timeLapse = millis() - startTime;
  if( timeLapse > BUTTON_PRESS_LONG ) return BUTTON_PRESS_LONG;
  // if( timeLapse > BUTTON_PRESS_MEDIUM ) return BUTTON_PRESS_MEDIUM; // For future purposes
  else if( timeLapse > BUTTON_PRESS_SHORT ) return BUTTON_PRESS_SHORT;
  else return FALSE;
    
}

// Send command to TEF module. Takes any number of arguments,
// as long as you enter the right number of arguments in len
// mdl: module number of TEF (32 for FM)
// cmd: command
// len: number of arguments after len
void Set_Cmd(uint8_t mdl, uint8_t cmd, int len, ...)
{
  uint8_t buf[31];
  uint16_t temp;
  va_list vArgs;
  va_start(vArgs, len);
  buf[0] = mdl;
  buf[1] = cmd;
  buf[2] = 1;
  for (uint8_t i = 0; i< len; i++)
  {
    temp = va_arg(vArgs, uint16_t);
    buf[3 + i * 2] = (uint8_t)(temp >> 8);
    buf[4 + i * 2] = (uint8_t)temp;
  }
  va_end(vArgs);
  Write(buf, len * 2 + 3);
}

// Get info from TEF module
// mdl: module number (32 = FM)
// cmd: command
// receive: buffer to receive data
// len: number of words to receive
void Get_Cmd(uint8_t mdl, uint8_t cmd, int16_t *receive, int len)
{
  uint8_t buf[3];
  buf[0] = mdl;
  buf[1] = cmd;
  buf[2] = 1;
  Write(buf, 3);
  Read((uint8_t*)receive, 2 * len);
  for (uint8_t i = 0; i < len; i++)
  {
    uint16_t newval = (uint8_t)(receive[i] >> 8) | (((uint8_t)(receive[i])) << 8);
    receive[i] = newval;
  }
}

// Write raw data to TEF module
// This is used to initialize the module and send some patch data at startup.
// You can also use this to send operational parameters to the TEF module
// (instead of using the Set_Cmd() method)
//
// NOTE: this routine is based on a string of data that resides in the
// program part of the Arduino's memory (the effect of the PROGMEM keyword)
void dsp_write_data(const uint8_t* data)
{
  uint8_t *pa = (uint8_t *)data;
  uint8_t len, i, first;
  for (;;)
  {
    len = pgm_read_byte_near(pa++);
    first = pgm_read_byte_near(pa);
    if (!len)
      break;
    if (len == 2 && first == 0xff)
    {
      int delaytime = pgm_read_byte_near(++pa);
      delay(delaytime);
      pa++;
    }
    else
    {
      Wire.beginTransmission(DSP_I2C);
      for (int i = 0; i < len; i++)
        Wire.write(pgm_read_byte_near(pa++));
      Wire.endTransmission();
    }
  }
}

// void scan(bool continous)
//{
//  uint32_t freq;
//  uint32_t buffer;
//  Set_Cmd(32, 10, 4, scan_filter == -1 ? 1 : 0, pgm_read_word_near(FMFilterMap + scan_filter), 1000, 1000);
//  Set_Cmd(32, 1, 2, 1, scan_start);
//  do
//  {
//    // Serial.print('U');
//    for (freq = scan_start; freq <= scan_end; freq += scan_step)
//    {
//      Set_Cmd(32, 1, 2, 1, freq);
//      // Serial.print(freq * 10, DEC);  
//      // Serial.print('=');
//      delay(10);
//      int16_t uQuality[2] = { 0 };
//      Get_Cmd(32, 128, uQuality, 2);
//      // Serial.print(uQuality[1] / 10, DEC);
//      // Serial.print(',');
//    }
//    // Serial.print('\n');
//  } while (continous && !Serial.available());
  // Restore previous settings
//  Set_Cmd(32, 10, 4, current_filter == -1 ? 1 : 0, pgm_read_word_near(FMFilterMap + current_filter), 1000, 1000);
//  Set_Cmd(32, 1, 2, 1, REG_FREQ);
//}


// Interrupt routine for rotating the rotary encoder.
// This routine is called as well as turning left or right.
// This routine does as little as possible - only updates a global counter.
// What we do with the counter is up to the rest of the code.
void updateEncoder()
{
   cli();   // Clear all interrupts to make sure we don't rinse repeat :-)
   
  // Read the current state of CLK
  currentStateCLK = digitalRead(ROTARY_CLK);

  // If last and current state of CLK are different, then pulse occurred
  // React to only 1 state change to avoid double count
  if (currentStateCLK != lastStateCLK  && currentStateCLK == 1)
  {
    // If the DT state is different than the CLK state then
    // the encoder is rotating CCW so decrement
    if (digitalRead(ROTARY_DT) != currentStateCLK) 
    {
      counter --;
    } else 
    {
      // Encoder is rotating CW so increment
      counter ++;
    }
  }
 
  // Remember last CLK state
  lastStateCLK = currentStateCLK;
}


void setup()
{
  int16_t uState;
  preset preset0;
  bool loFound = FALSE;

  lcd.begin(20, 4); // initialize the LCD
  lcd.clear(); 
  
  // Print a logo message to the LCD.
  lcd.setCursor(0,0);
  lcd.print("*------PA1RKT------*");
  lcd.setCursor(0,1);
  lcd.print(" NXP TEF6686 Tuner  ");
  lcd.setCursor(0,2);
  lcd.print("       v1.33        ");
  lcd.setCursor(0,3);
  lcd.print("*------------------*");

  Wire.begin();                       // I2C interface (to TEF module and LCD)
  Serial.begin(115200);               // Terminal interface (for debugging)
                                      // Used to be connection to XDR-GTK frontend

  pinMode(sw1, INPUT);
  pinMode(sw2, INPUT);
  pinMode(sw3, INPUT);
  pinMode(sw4, INPUT);
  pinMode(sw5, INPUT);
  pinMode(sw6, INPUT);   
  digitalWrite(sw1, HIGH);
  digitalWrite(sw2, HIGH);
  digitalWrite(sw3, HIGH);
  digitalWrite(sw4, HIGH);
  digitalWrite(sw5, HIGH);
  digitalWrite(sw6, HIGH);


  // Rotary encoder pins
  pinMode(ROTARY_CLK, INPUT_PULLUP);
  pinMode(ROTARY_DT,  INPUT_PULLUP);
  pinMode(ROTARY_SW,  INPUT_PULLUP);

  Get_Cmd(64, 128, &uState, 1);
  if (uState < 2)
  {
    dsp_write_data(DSP_INIT);
  }  
  else if (uState > 2)
  {
    Set_Cmd(64, 1, 1, 1);
  }

  delay(1000);
  lcd.clear();

  clearRDS();

  // Read the initial state of rotary encoder CLK
  lastStateCLK = digitalRead(ROTARY_CLK);
  
  // Setup updateEncoder() as an interrupt handler 
  // for any change on the rotary encoder:
  // on interrupt 0 (pin 2), or interrupt 1 (pin 3)
  attachInterrupt(0, updateEncoder, CHANGE);
  attachInterrupt(1, updateEncoder, CHANGE);

  if (buttonPress(ROTARY_SW))
  {
    lcd.clear();
    firstStart = TRUE;
    menuLevel = 1;
  }

  Set_Cmd(32, 10, 1, 1);                    // Set TEF on auto bandwidth
  autoBandwidth = TRUE;                     // Automatic bandwidth flag
  
  volume = 93;              
  Set_Cmd(48, 10, 1, volume* 7 - 600);      // Set TEF volume

  squelch = EEPROM_read16(SQUELCH_LEVEL_ADDRESS);
  if( squelch<-10 || squelch>50 ) squelch = 0;

  // Read startup frequency from EEPROM
  // Startup frequency is PRESET 0
  if( EEPROM_readPreset( 0, &preset0  ))        // Find regular TEF starting frequency
  {
    REG_FREQ = preset0.freq;
  }
  
  LO_FREQ = EEPROM_read32( LOFREQ_ADDRESS );    // Find LO frequency
  for (byte i=0; i<4; i++)
  {
    if( LO_FREQ == loFreqs[i] ) loFound = TRUE;
  }
  if(loFound == FALSE) LO_FREQ = 0;             // LO No found? Then LO = 0
  
  CONVERTER_FREQ = REG_FREQ + LO_FREQ;          // Calculate converter frequency (for display)
  Set_Cmd(32, 1, 2, 1, REG_FREQ);               // Set TEF to starting frequency
}

void loop()
{  
  byte i;
  bool indexFound;
  uint32_t displayBlocks = 0;
  int16_t uStatus;
  int16_t uQuality[7] = { 0 };
  uint32_t pressTime;
  
  switch (menuLevel)
  { 
    case 0: // Normal operation
      // If this is the first start of th tuner then displayBlocks = 0xFFFF (show all info)
      if(firstStart) displayBlocks = 0xFFFF;
      if(counter !=0 || firstStart )
      {
        // Counter determines change of frequency
        REG_FREQ += 5 * counter; // 50KHz stepsize on FM
        if (REG_FREQ > FREQMAX) REG_FREQ = FREQMIN; // Check TEF band edges
        if (REG_FREQ < FREQMIN) REG_FREQ = FREQMAX;
        CONVERTER_FREQ = REG_FREQ + LO_FREQ;        // Converter freq. calculation
        
        clearRDS();
        Set_Cmd(32, 1, 2, 1, REG_FREQ); // set TEF on freq
        
        displayBlocks |= DB_FREQ;  
        counter = 0;  // reset counter
        if(firstStart) firstStart = FALSE;
      }

      // check signal level, stereo subcarrier and bandwidth every TIMER_INTERVAL
      if ((millis() - timer) >= TIMER_INTERVAL )
      {
        // Check stereo indicator
        Get_Cmd(32, 133, &uStatus, 1);                  // TEF read status
        stereoIndicator = (uStatus & (1 << 15)) >> 15;  // Read stereo/mono bit and shift it for
                                                        // processing as stereoIndicator
                                                        // WARNING: fills up left with 1....
                                                        // No problem yet, but might be in future    
        // Check signal level
        Get_Cmd(32, 128, uQuality, 7);                  // TEF read signal quality
        signalLevel = uQuality[1];                      // Signal level (dBuV)
    
        // Squelch (based on signal level) in 0.1dBuV (-20-120 dBuV)
        if (signalLevel/10 > squelch)                   // Signal level above hard squelch level?
        {
          stereoIndicator &= ~MUTE;                     // clear MUTE bit
          Set_Cmd(48, 11, 1, 0);                        // unmute TEF
        }
        else   
        {
          stereoIndicator |= MUTE;                      // set MUTE bit
          Set_Cmd(48, 11, 1, 1);                        // mute TEF
        }
        if(stereoIndicator != oldStereoIndicator)       // Did the stereo indicator change?
        {  
          displayBlocks |= DB_STEREO_MONO;              // Switch on its displayBlock
          oldStereoIndicator = stereoIndicator;         // (v1.33)
        }
        if(signalLevel != oldSignalLevel)               // Did the signalLevel change?
        {  
          displayBlocks |= DB_SIGNAL_LEVEL;             // Switch on its displayBlock
          oldSignalLevel = signalLevel;                 // (v1.33)
        }
        
        // Bandwidth - TEF provides it in 0.1kHz values (56-311 kHz)
        // see FMFilterMap array
        bandwidth = uQuality[5];
        if(bandwidth != oldBandwidth)                   // Did the bandwidth change?
        {                                         
          displayBlocks |= DB_BW_LEVEL;                 // Switch on its displayBlock
          oldBandwidth = bandwidth;
        }
        timer = millis();                               // Reset this timer
      }
      
      // Get RDS info every RDS timer interval
      if ((millis() - timer_rds) >= RDS_TIMER_INTERVAL)
      {
        get_RDS();
        scrollIndex = mod(scrollIndex+1, MAX_RDSRADIOTEXT-1); // <>still not entirely right for 64 chars, refactoring of RDS needed
        displayBlocks |= DB_RDS_SRVNAME;                // Switch on displayBlock for RDS service name
        displayBlocks |= DB_RDS_LONGTXT;                // Switch on displayBlock for RDS long text (scrolling)
        timer_rds = millis();                           // Reset this timer
      }
      break;

    case 1: // Startup menu
            // Counter determines LO frequency of converter
            // (cycle through array loFreqs[])
      if(firstStart) 
      {
        displayBlocks = 0xFFFF;
      }
      if(counter != 0 || firstStart)
      {
        indexLoFreq = mod(indexLoFreq+counter, 4); // Cycle trough 4 LO frequencies, should be a #define
        LO_FREQ = loFreqs[indexLoFreq];
        counter = 0;
        displayBlocks |= DB_SET_LO_FREQ;
      }
      break;

    case 2: // Operational menu: select preset (read/store) (preset 0 = startup freq.)
      if(firstStart) displayBlocks |= DB_PRESET_TXT;
      if(counter!=0 || firstStart)
      {
        displayBlocks &= ~DB_PRESET_INF;
        indexPreset = mod(indexPreset+counter, MAX_PRESETS);
        
        if(EEPROM_readPreset(indexPreset, &thisPreset)== TRUE)
        {
          displayFrequency = thisPreset.freq + LO_FREQ;
          displayBlocks |= DB_PRESET_INF;
        }        
        displayBlocks |= DB_PRESET_IDX;
        counter = 0;
        
        if(firstStart==TRUE) firstStart = FALSE;
      }
      break;  

    case 3: // Operational: fast tuning (bigger steps)
      if(counter !=0)
      {
        REG_FREQ += (counter * 100);                // 1MHz steps
        if (REG_FREQ > FREQMAX) REG_FREQ = FREQMIN; // Check band edges
        if (REG_FREQ < FREQMIN) REG_FREQ = FREQMAX;
        CONVERTER_FREQ = REG_FREQ + LO_FREQ;        // Calculate CONVERTER_FREQ
        displayBlocks |= DB_SET_FREQSTEP;           // Switch on displayBlocks for fast tuning
        counter = 0;                                // Reset counter
      }
      break;

    case 4: // Operational: set bandwidth
      if(firstStart)
      {
        i=0;
        indexFound = FALSE;
        while( !indexFound && i<16 )      // Find current bandwidth in FMFiltermap
        {                                 // so we can start the manual setting from there
          if( bandwidth != pgm_read_word_near(FMFilterMap + i)) i++;
          else 
          {
            indexBandwidth = i;
            indexFound = TRUE;
          }
        }
        firstStart = FALSE;
        if(!indexFound) bandwidth = 2360;   // Default BW in case we didn't find any matching BW
        displayBlocks |= DB_SET_BW_MAN;     // Switch on displayBlock for manual BW setting
      }
      
      if(counter != 0)
      {
        indexBandwidth = mod(indexBandwidth+counter, 16); 
        bandwidth = pgm_read_word_near(FMFilterMap + indexBandwidth); // Read bandwidth from PROGMEM
        Set_Cmd(32, 10, 4, 0, bandwidth, 1000, 1000);                 // Set TEF to new fixed bandwidth
        displayBlocks |= DB_SET_BW_MAN;                               // Switch on displayBlock for 
                                                                      //  manual BW setting
        counter = 0;                                                  // Reset counter
      }
      
      break;

    case 5: // Operational menu: set squelch level for signal level squelch
      if(counter != 0)
      {
        squelch += counter;                 // Update squelch level from rotary encoder
        if( squelch < -10 ) squelch = -10;  // Check min and max squelch levels (for TEF)
        if( squelch > 50) squelch = 50;

        // Get signal level from TEF
        Get_Cmd(32, 128, uQuality, 7);      // TEF read signal quality
        signalLevel = uQuality[1];          // Signal level (dBuV)
    
        // Squelch (based on signal level) in 0.1dBuV (-20-120 dBuV)
        if (signalLevel/10 > squelch) 
        {
          //stereoIndicator &= ~MUTE;       // clear MUTE bit
          Set_Cmd(48, 11, 1, 0);            // unmute TEF
        }
        if (signalLevel/10 < squelch)
        {
          //stereoIndicator |= MUTE;        // set MUTE bit
          Set_Cmd(48, 11, 1, 1);            // mute TEF
        }
        displayBlocks |= DB_SQL_LVL;        // Switch on displayBlock for squelch level
        counter = 0;                        // Reset counter
      }
      break;
  }

  if( pressTime = buttonPress(ROTARY_SW) )
  {
    switch (menuLevel)
    {
      case 0: // Push rotary switch during normal operation
        if(pressTime == BUTTON_PRESS_LONG)  // Long press
        {
          lcd.clear();
          indexPreset = 0;
          menuLevel = 2;                    // Goto PRESET menu
        }
        else menuLevel = 3;                 // Goto OPERATIONAL SETTINGS menu
        
        firstStart = TRUE;                  
        break; 

      case 1: // finish STORE LO FREQUENCY menu
        lcd.clear();
        EEPROM_write32( LOFREQ_ADDRESS, LO_FREQ );  // Store new LO frequency in memory
        menuLevel = 0;                              // Go to normal operation
        indexLoFreq = 0;                            // <> refactor? is this still necessary?
        firstStart = TRUE;
        break;

      case 2: // finish PRESET menu
        lcd.clear();
        if(pressTime == BUTTON_PRESS_LONG)      // Long press
        {
          thisPreset.magic = PRESET_MAGIC;    // Set magic number as a preset marker
          thisPreset.freq = REG_FREQ;         // Set preset frequency to current freq (TEF freq.)
          EEPROM_writePreset( mod(indexPreset, MAX_PRESETS), thisPreset );  // Write preset to EEPROM
        }
        else if(EEPROM_readPreset( mod(indexPreset, MAX_PRESETS), &thisPreset))
        {
          REG_FREQ = thisPreset.freq;
          CONVERTER_FREQ = REG_FREQ + LO_FREQ;
        }
        menuLevel = 0;
        firstStart = TRUE;
        break;

        case 3: // result of operational Frequency 1MHz step tuning 
          if(pressTime == BUTTON_PRESS_LONG)      // Long press
          {
            menuLevel = 0;                        // Goto operational mode
            lcd.clear();
          }
          else                                    // Short press takes us to operational BW setting
          {
            firstStart = TRUE;
            menuLevel = 4;                        // Go to operational Bandwidth setting
          }
          break;

        case 4: // result of operational Bandwidth setting
          if(pressTime == BUTTON_PRESS_LONG)      // Long press
          {
            autoBandwidth = TRUE;           // Back to AUTO bandwidth
            Set_Cmd(32, 10, 4, 1, bandwidth, 1000, 1000); // Set TEF on auto bandwidth
            menuLevel = 0;                  // Back to operational mode
            firstStart = TRUE;
            lcd.clear();
          }
          else                              // Short press
          {
            menuLevel = 5;                  // Go to operational Squelch mode
          }
          break;

        case 5: // result of operational Squelch setting
          EEPROM_write16(SQUELCH_LEVEL_ADDRESS, squelch);
          firstStart = TRUE;
          menuLevel = 0;
          lcd.clear();
          break;
    }
    delay(400);
  }

  if (pressTime = buttonPress(sw1))
  {
  } 
    
  if (pressTime = buttonPress(sw2))
  {
  } 

  if (pressTime = buttonPress(sw3))
  {
  } 

  if (pressTime = buttonPress(sw4))
  {
  } 

  if (pressTime = buttonPress(sw5))
  {
  } 

  if (pressTime = buttonPress(sw6))
  {
  }


  // Blink characters every blink timer interval
  if ((millis() - timer_blink) >= BLINK_TIMER_INTERVAL)
  {
    if(blinkOn == TRUE) blinkOn = FALSE;
    else blinkOn = TRUE;
    timer_blink = millis();
  }

  displayInfo(displayBlocks);     // Display designated displayBlocks
    
}  // end main loop

void get_RDS()
{
  // https://en.wikipedia.org/wiki/Radio_Data_System#Program_types
  /*
  const char* ptyLUT[51] PROGMEM = { // America
      "      None      ",
      "      News      ",
      "  Information   ",
      "     Sports     ",
      "      Talk      ",
      "      Rock      ",
      "  Classic Rock  ",
      "   Adult Hits   ",
      "   Soft Rock    ",
      "     Top 40     ",
      "    Country     ",
      "     Oldies     ",
      "      Soft      ",
      "   Nostalgia    ",
      "      Jazz      ",
      "   Classical    ",
      "Rhythm and Blues",
      "   Soft R & B   ",
      "Foreign Language",
      "Religious Music ",
      " Religious Talk ",
      "  Personality   ",
      "     Public     ",
      "    College     ",
      " Reserved  -24- ",
      " Reserved  -25- ",
      " Reserved  -26- ",
      " Reserved  -27- ",
      " Reserved  -28- ",
      "     Weather    ",
      " Emergency Test ",
      "  !!!ALERT!!!   ",
      "Current Affairs ",
      "   Education    ",
      "     Drama      ",
      "    Cultures    ",
      "    Science     ",
      " Varied Speech  ",
      " Easy Listening ",
      " Light Classics ",
      "Serious Classics",
      "  Other Music   ",
      "    Finance     ",
      "Children's Progs",
      " Social Affairs ",
      "    Phone In    ",
      "Travel & Touring",
      "Leisure & Hobby ",
      " National Music ",
      "   Folk Music   ",
      "  Documentary   "};
*/
/*
  const char* ptyLUT[51] PROGMEM = {   // Europe
      "      None      ",
      "      News      ",
      " Current Affairs",
      "   Information  ",
      "      Sport     ",
      "    Education   ",
      "     Drama      ",
      "     Culture    ",
      "     Science    ",
      "    Variable    ",
      "    Pop Music   ",
      "   Rock Music   ",
      "  Easy Listening",
      " Light Classical",
      "SeriousClassical",
      "  Other Music   ",
      "     Weather    ",
      "     Finance    ",
      " Childrens Prog ",
      " Social Affairs ",
      " Religious Talk ",
      "  Phone-In Talk ",
      "     Travel     ",
      "    Leisure     ",
      "   Jazz Music   ",
      "  Country Music ",
      "  National Music",
      "  Oldies Music  ",
      "   Folk Music   ",
      "   Documentary  ",
      " Emergency Test ",
      "  !!!ALERT!!!   ",
      "Current Affairs ",
      "   Education    ",
      "     Drama      ",
      "    Cultures    ",
      "    Science     ",
      " Varied Speech  ",
      " Easy Listening ",
      " Light Classics ",
      "Serious Classics",
      "  Other Music   ",
      "    Finance     ",
      "Children's Progs",
      " Social Affairs ",
      "    Phone In    ",
      "Travel & Touring",
      "Leisure & Hobby ",
      " National Music ",
      "   Folk Music   ",
      "  Documentary   "};
*/

  uint8_t rdsBHigh, rdsBLow, rdsCHigh, rdsCLow, rdsDHigh, isReady, rdsDLow;
  int16_t uRds_Data[6] = {0};
  uint8_t type;
  uint8_t version;
  
  Get_Cmd(32, 131, uRds_Data, 8);
  
  if ( bitRead(uRds_Data[0], 15) == 1 )
  {  
    rdsBHigh = (uint8_t)(uRds_Data[2] >> 8);
    rdsBLow = (uint8_t)uRds_Data[2];
    rdsCHigh = (uint8_t)(uRds_Data[3] >> 8);
    rdsCLow = (uint8_t)uRds_Data[3];
    rdsDHigh = (uint8_t)(uRds_Data[4] >> 8);
    rdsDLow = (uint8_t)uRds_Data[4];
    
    uint8_t errA = (uRds_Data[5] & 0b1100000000000000) >> 14;
    uint8_t errB = (uRds_Data[5] & 0b0011000000000000) >> 12;
    uint8_t errC = (uRds_Data[5] & 0b0000110000000000) >> 10;
    uint8_t errD = (uRds_Data[5] & 0b0000001100000000) >> 8;

    type = (rdsBHigh >> 4) & 15;        
    version = bitRead(rdsBHigh, 4);
  }
  // Groups 0A & 0B
  // Basic tuning and switching information only
  if (type == 0) 
  {
    uint8_t address = rdsBLow & 3;
    // Groups 0A & 0B: to extract PS segment we need blocks 1 and 3
    if (address >= 0 && address <= 3) 
    {
      if (rdsDHigh != '\0')
        rdsProgramService[address * 2] = rdsDHigh;
    }  
    if (rdsDLow != '\0') {
      rdsProgramService[address * 2 + 1] = rdsDLow;
    }  
    isReady = (address == 3) ? 1 : 0;
   
    rdsFormatString(rdsProgramService, 8);
  }
  // Groups 2A & 2B
  // Radio Text
  else if (type == 2) 
  {
    uint16_t addressRT = rdsBLow & 15;
    uint8_t ab = bitRead(rdsBLow, 4);
    uint8_t cr = 0;
    uint8_t len = 64;
    if (version == 0) {
      if (addressRT >= 0 && addressRT <= 15) {
        if (rdsCHigh != 0x0D) {
          rdsRadioText[(addressRT*4)%MAX_RDSRADIOTEXT] = rdsCHigh;
        }  
        else {
          len = addressRT * 4;
          cr = 1;
        }
        if (rdsCLow != 0x0D) {
          rdsRadioText[(addressRT * 4 + 1)%MAX_RDSRADIOTEXT] = rdsCLow;
        }  
        else {
          len = addressRT * 4 + 1;
          cr = 1;
        }
        if (rdsDHigh != 0x0D) {
          rdsRadioText[(addressRT * 4 + 2)%MAX_RDSRADIOTEXT] = rdsDHigh;
        }  
        else {
          len = addressRT * 4 + 2;
          cr = 1;
        }
        if (rdsDLow != 0x0D) {
          rdsRadioText[(addressRT * 4 + 3)%MAX_RDSRADIOTEXT] = rdsDLow;
        }
        else {
          len = addressRT * 4 + 3;
          cr = 1;
        }
      }
    }
    else {
      if (addressRT >= 0 && addressRT <= 7) {
        if (rdsDHigh != '\0') {
          rdsRadioText[(addressRT * 2)%MAX_RDSRADIOTEXT] = rdsDHigh;
        }  
        if (rdsDLow != '\0') {
          rdsRadioText[(addressRT * 2 + 1)%MAX_RDSRADIOTEXT] = rdsDLow;
        }
      }
    }
    if (cr) {
      for (uint8_t i = len; i < MAX_RDSRADIOTEXT; i++) {
        rdsRadioText[i%MAX_RDSRADIOTEXT] = ' ';
      }
    }
    if (ab != rdsAb) {      
      for (uint8_t i = 0; i < MAX_RDSRADIOTEXT; i++) {
        rdsRadioText[i%MAX_RDSRADIOTEXT] = ' ';
      }
      rdsRadioText[MAX_RDSRADIOTEXT] = '\0';     
      isRdsNewRadioText = 1;
    }
    else {
      isRdsNewRadioText = 0;
    }
    rdsAb = ab;
    rdsFormatString(rdsRadioText, MAX_RDSRADIOTEXT);
  }
}

//void serial_hex(uint8_t val)
// {
//    Serial.print((val >> 4) & 0xF, HEX);
//    Serial.print(val & 0xF, HEX);
// }

  
void displayInfo(uint32_t displayBlocks)
{  
  switch(menuLevel)
  {
    case 0: //NORMAL OPERATION
      //Frequency unit
      if(displayBlocks & DB_FREQ_UNIT)
      {
        lcd.setCursor(8,0);
        lcd.print("MHz");
        displayBlocks &= ~DB_FREQ_UNIT;
      }
        
      //Signal strength unit
      if(displayBlocks & DB_SIGNAL_UNIT)
      {
        lcd.setCursor(16,0);
        lcd.print("dBuV");
        displayBlocks &= ~DB_SIGNAL_UNIT;
      }

      //Bandwidth part
      if(displayBlocks & DB_BW_TXT)
      {
        lcd.setCursor(0,1);
        lcd.print("BW: ");
        displayBlocks &= ~DB_BW_TXT;
      }
      if(displayBlocks & DB_BW_UNIT)
      {
        lcd.setCursor(8,1);
        lcd.print("kHz");
        displayBlocks &= ~DB_BW_UNIT;
      }
        
      // STEREO/MONO/MUTE indicator
      if(displayBlocks & DB_STEREO_MONO)
      {
        lcd.setCursor(14,1);
        if(stereoIndicator & MUTE) lcd.print("MUTE  ");
        else
        {
          if(stereoIndicator & STEREO) lcd.print("STEREO");
          else lcd.print("MONO  ");
        }
        displayBlocks &= ~DB_STEREO_MONO;
      }

      // Frequency readout
      if(displayBlocks & DB_FREQ)
      {  
        lcd.setCursor(0, 0);
        
        if (CONVERTER_FREQ < 100000) lcd.print(" ");
        if (CONVERTER_FREQ < 10000 ) lcd.print(" ");
        lcd.print(CONVERTER_FREQ/100.0, 2);

        displayBlocks &= ~DB_FREQ;
      }
      
      // Signal level
      if(displayBlocks & DB_SIGNAL_LEVEL)
      {
        lcd.setCursor(12,0);
        if (signalLevel<0) lcd.print("-");
          else lcd.print(" ");
        if ((abs)(signalLevel)<100)
        {
          lcd.print(" ");
        } 
        if((abs)(signalLevel)>1000)
        {
          lcd.setCursor(12,0);
        }
        lcd.print((abs)(signalLevel/10));
        lcd.print(" ");  
        displayBlocks &= ~DB_SIGNAL_LEVEL;
      }

      // Bandwidth
      if(displayBlocks & DB_BW_LEVEL)
      {
        if(bandwidth<1000)
        {
          lcd.setCursor(4,1);
          lcd.print(" ");
        }
        else 
          lcd.setCursor(4,1);        
        lcd.print(bandwidth/10);
        displayBlocks &= ~DB_BW_LEVEL;
      }

      // RDS Service name (radiostation name)
      if(displayBlocks & DB_RDS_SRVNAME)
      {
        lcd.setCursor(6,2);
        lcd.print(rdsProgramService); 
        displayBlocks &= ~DB_RDS_SRVNAME;
      }
      
      // RDS Radiotext - scrolling
      if (displayBlocks & DB_RDS_LONGTXT)
      {
        // Show PTY
        // lcd.setCursor(0,3); 
        // lcd.print("PTY:"); 
        // for (int i = 0; i < 15; i++) 
        // {
        //   lcd.setCursor(i+4,3);
        //   lcd.print(rdsProgramType[i]);   
        // }
        for (int i = 0; i < 20; i++) 
        {
          lcd.setCursor(i,3);
          lcd.print(rdsRadioText[mod(scrollIndex+i, MAX_RDSRADIOTEXT-1)]); // <> RDS refactoring needed
        }
        displayBlocks &= ~DB_RDS_LONGTXT;
      }
      break;

    case 1: // SET LO FREQUENCY 
      if(displayBlocks & DB_SET_LO_TXT)
      {
        lcd.setCursor(0,1);
        lcd.print(" SET  LO FREQUENCY: " );
        displayBlocks &= ~DB_SET_LO_TXT;
      }

      if(displayBlocks & DB_SET_LO_FREQ)
      {
        lcd.setCursor(7,2);
        if( LO_FREQ == 0 )
        {
          lcd.print( " No LO       ");
        }
        else
        {
          lcd.print(LO_FREQ/100.0, 2);
        }
        displayBlocks &= ~DB_SET_LO_FREQ;
      }  
      break;

    case 2: // STORE/RETRIEVE PRESET screen
      if(displayBlocks & DB_PRESET_TXT)
      {
        lcd.setCursor(0,0);
        lcd.print( "-PRESETS READ/WRITE-" );
        displayBlocks &= ~DB_PRESET_TXT;
      }
      if(displayBlocks & DB_PRESET_IDX)
      {
        lcd.setCursor(0,1); 
        lcd.print(indexPreset, DEC);
        lcd.print(":  ");
        displayBlocks &= ~DB_PRESET_IDX;
        if(displayBlocks & DB_PRESET_INF)
        {  
          lcd.setCursor(4,1);
          if (displayFrequency < 100000) lcd.print(" ");
          if (displayFrequency < 10000 ) lcd.print(" ");
          lcd.print(displayFrequency/100.0, 2);
          lcd.print(" MHz   ");

          displayBlocks &= ~DB_PRESET_INF;
        }
        else 
        {
          lcd.setCursor(4,1);
          lcd.print("--------------");
        }
      }
      break;

    case 3: // Operational: 1MHz step tuning
      lcd.setCursor(8,0);
      if(blinkOn)               //<> to be refactored, not efficient
      {
        lcd.print("MHz");
      }
      else
      {
        lcd.print("   ");
      }

      if(displayBlocks & DB_SET_FREQSTEP)
      {
        lcd.setCursor(0,0);
        if (CONVERTER_FREQ < 100000) lcd.print(" ");
        if (CONVERTER_FREQ < 10000 ) lcd.print(" ");
        lcd.print(CONVERTER_FREQ/100.0, 2);
 
        displayBlocks &= ~DB_SET_FREQSTEP;
      }
      break;
      
    case 4: // Set operational BW mode
      lcd.setCursor(8,0);   // Restoring MHz of frequency in case it just blinked off 
      lcd.print("MHz");     // when we came from menuLevel 3 :-) <>should be refactored

      lcd.setCursor(8,1);
      if(blinkOn)
      {
        lcd.print("kHz");
      }
      else
      {
        lcd.print("   ");
      }

      if(displayBlocks & DB_SET_BW_MAN)
      {
        autoBandwidth = FALSE;         // Set to manual bandwidth
        if(bandwidth<1000)
        {
          lcd.setCursor(4,1);
          lcd.print(" ");
        }
        else 
          lcd.setCursor(4,1);
          
        lcd.print(bandwidth/10);
        displayBlocks &= ~DB_SET_BW_MAN;
      }
      break;

      case 5: // Screen for mode operational squelch setting - no visual except a blinking "dBuV" sign
        lcd.setCursor(8,1);   // Make sure the previous BW setting mode (menuLevel 4) 
        lcd.print("kHz");     // did not just blink off ;-)
        
        lcd.setCursor(16,0);  // Blink "dBuV" sign
        if(blinkOn)
        {
          lcd.print("dBuV");
        }
        else
        {
          lcd.print("    ");
        }

        if( displayBlocks & DB_SQL_LVL )
        {
          lcd.setCursor(12,0);
          if (squelch<0) lcd.print("-");
            else lcd.print(" ");
          if ((abs)(squelch)<10)
          {
            lcd.print(" ");
          } 
          lcd.print((abs)(squelch));
          lcd.print(" ");  
          displayBlocks &= ~DB_SQL_LVL;
        }
        break;
  } 
}

void clearRDS() 
{
  byte i;
  
  lcd.setCursor(6,2);
  lcd.print("        ");
  lcd.setCursor(0,3);
  lcd.print("                    ");
  strcpy(rdsProgramService, "        ");
  strcpy(rdsProgramServiceUnsafe, "        ");
  strcpy(rdsProgramType, "                ");
  for(i=0; i<MAX_RDSRADIOTEXT-1; i++)
  {
    rdsRadioText[i] = ' ';
  }
  rdsRadioText[MAX_RDSRADIOTEXT-1] = '\0';
  psErrors = 0xFFFFFFFF;
  psCharIsSet = 0;
}

void rdsFormatString(char* str, uint16_t length) {
  for (uint16_t i = 0; i < length; i++) {
    if ((str[i] != 0 && str[i] < 32) || str[i] > 126 ) {
      str[i] = ' ';
    }
  }
}
