// ###################################################################################
// # Project: Pixel-X v9
// # Engineer:  Mic.Tsai
// # Date:  2 July 2022
// # Objective: Dev.board
// # Usage: ATmega328p
// ###################################################################################

/////////////////////////////////
// Important:
// Only Arduino 1.0.6 can sketch this code
// Check cycle time
/////////////////////////////////

// Requires SdFat library for Arduino:
// http://code.google.com/p/sdfatlib/

// Requires NeoPixel library:
// https://github.com/adafruit/Adafruit_NeoPixel

// This project is base on NeoPixel Painter from Adafruit 
// If you need detail building process please visit here (step by step)
// https://learn.adafruit.com/neopixel-painter?view=all
// https://github.com/adafruit/NeoPixel_Painter
// ###################################################################################

#include <SdFat.h>
#include <avr/pgmspace.h>
#include "./gamma.h"

#include <Adafruit_NeoPixel.h>
#include "WS2812_Definitions.h"
#include <SoftwareSerial.h>
SoftwareSerial BT(6,7);
char val;
byte i;
byte j;
#define PIN 2
#define LED_COUNT 60
Adafruit_NeoPixel leds = Adafruit_NeoPixel(LED_COUNT, PIN, NEO_GRB + NEO_KHZ800);

// CONFIGURABLE STUFF --------------------------------------------------------

//************************************************************
//************************************************************
#define CURRENT_MAX   1800   //Brightness-setting(500-1800) Max current from power supply (mA)
//************************************************************
//************************************************************

#define N_LEDS        60    // Max value is 170 (fits one SD card block)
#define CARD_SELECT    4    // SD card select pin (some shields use #4, not 10)
#define LED_PIN        2    // NeoPixels connect here
#define SPEED         A0    // Speed-setting dial
int BRIGHTNESS =      255;  // Brightness-setting dial /no function
#define TRIGGER       A2    // Playback trigger pin
#define STAR          A2
//************************************************************
//************************************************************

//  int STAR = LOW;               // Set LOW go direct display
                            // Set HIGH need button trigger
//************************************************************
//************************************************************    
                        
// NON-CONFIGURABLE STUFF ----------------------------------------------------

#define OVERHEAD 150 // Extra microseconds for loop processing, etc.

uint8_t           sdBuf[512],  // One SD block (also for NeoPixel color data)
                  pinMask;     // NeoPixel pin bitmask
uint16_t          maxLPS,      // Max playback lines/sec
                  nFrames = 0, // Total # of image files
                  frame   = 0; // Current image # being painted
uint32_t          firstBlock,  // First block # in temp working file
                  nBlocks;     // Number of blocks in file
Sd2Card           card;        // SD card global instance (only one)
SdVolume          volume;      // Filesystem global instance (only one)
SdFile            root;        // Root directory (only one)
volatile uint8_t *port;        // NeoPixel PORT register

// INITIALIZATION ------------------------------------------------------------

void setup() {
  
  BT.begin(9600);
  leds.begin();
  clearLEDs();
  leds.show();
 
  uint8_t  b, startupTrigger, minBrightness;
  char     infile[13], outfile[13];
  boolean  found;
  uint16_t i, n;
  SdFile   tmp;
  uint32_t lastBlock;

  pinMode(13, OUTPUT);
  digitalWrite(TRIGGER, HIGH);           // Enable pullup on trigger button
  
//************************************************************
//************************************************************

  startupTrigger = HIGH; // Set LOW go direct to transfer bmp image process
                        // Set HIGH  --> Skip
                        
//************************************************************
//************************************************************


  
  Serial.begin(9600);
  pinMode(LED_PIN, OUTPUT);              // Enable NeoPixel output
  digitalWrite(LED_PIN, LOW);            // Default logic state = low
  port    = portOutputRegister(digitalPinToPort(LED_PIN));
  pinMask = digitalPinToBitMask(LED_PIN);
  memset(sdBuf, 0, N_LEDS * 3);          // Clear LED buffer
  show();                                // Init LEDs to 'off' state
#ifdef ENCODERSTEPS
  digitalWrite(5, HIGH);                 // Enable pullup on encoder pin
#endif

  Serial.print(F("Initializing SD card..."));
  if(!card.init(SPI_FULL_SPEED, CARD_SELECT)) {
    error(F("failed. Things to check:\n"
            "* is a card is inserted?\n"
            "* Is your wiring correct?\n"
            "* did you edit CARD_SELECT to match the SD shield or module?"));
  }
  Serial.println(F("OK"));

  if(!volume.init(&card)) {
    error(F("Could not find FAT16/FAT32 partition.\n"
            "Make sure the card is formatted."));
  }
  root.openRoot(&volume);

  // This simple application always reads the files 'frameNNN.bmp' in
  // the root directory; there's no file selection mechanism or UI.

  // If button is held at startup, the processing step is skipped, just
  // goes right to playback of the prior converted file(s) (if present).
  if(startupTrigger == LOW) { // button pressed

    // Two passes are made over the input images.  First pass counts the
    // files and estimates the max brightness level that the power supply
    // can sustain...
    minBrightness = 255;
    do {
      sprintf(infile, "frame%03d.bmp", nFrames);
      b = 255;
      if(found = bmpProcess(root, infile, NULL, &b)) { // b modified to safe max
        nFrames++;
        if(b < minBrightness) minBrightness = b;
      }
    } while(found && (nFrames < 1000));

    Serial.print(nFrames);
    Serial.print(" frames\nbrightness = ");
    Serial.println(minBrightness);

    // Read dial, setting brightness between 1 (almost but not quite off)
    // and the previously-estimated safe max.
    b = map(BRIGHTNESS, 0, 1023, 1, minBrightness);
  
    // Second pass now applies brightness adjustment while converting
    // the image(s) from BMP to a raw representation of NeoPixel data
    // (this outputs the file(s) 'frameNNN.tmp' -- any existing file
    // by that name will simply be clobbered, IT DOES NOT ASK).
    for(i=0; i<nFrames; i++) {
      sprintf(infile , "frame%03d.bmp", i);
      sprintf(outfile, "frame%03d.tmp", i);
      b = minBrightness;
      bmpProcess(root, infile, outfile, &b);
    }
    //while(digitalRead(TRIGGER) == HIGH); // Wait for button release

  } else { // Button not held -- use existing data

    do { // Scan for files to get nFrames
      sprintf(infile, "frame%03d.tmp", nFrames);
      if(found = tmp.open(&root, infile, O_RDONLY)) {
        if(tmp.contiguousRange(&firstBlock, &lastBlock)) {
          nFrames++;
        }
        tmp.close();
      }
    } while(found);

  } // end startupTrigger test

#ifdef ENCODERSTEPS
  // To use a rotary encoder rather than timer, connect one output
  // of encoder to T1 pin (digital pin 5 on Arduino Uno).  A small
  // capacitor across the encoder pins may help for debouncing.
  TCCR1A = _BV(WGM11) | _BV(WGM10);
  TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS12) | _BV(CS11);
#else
  // Prepare for playback from file; make a full read pass through the
  // file to estimate block read time (+5% margin) and max playback
  // lines/sec.  Not all SD cards perform the same.  This makes sure a
  // reasonable speed limit is used.
  for(i=0; i<nFrames; i++) { // Scan all files
    sprintf(infile, "frame%03d.tmp", i);
    tmp.open(&root, infile, O_RDONLY);
    tmp.contiguousRange(&firstBlock, &lastBlock);
    nBlocks = tmp.fileSize() / 512;
    tmp.close();
    n = (uint16_t)(1000000L /                         // 1 uSec /
      (((benchmark(firstBlock, nBlocks) * 21) / 20) + // time + 5% +
       (N_LEDS * 30L) + OVERHEAD));                   // 30 uSec/pixel
         Serial.print(F("N: "));
         Serial.println(n);
    if(n > maxLPS) maxLPS = n;
  }
  if(maxLPS > 300) maxLPS = 326; // NeoPixel PWM rate is ~400 Hz //change 326
  Serial.print(F("Max lines/sec: "));
  Serial.println(maxLPS);

  // Set up Timer1 for 64:1 prescale (250 KHz clock source),
  // fast PWM mode, no PWM out.
  TCCR1A = _BV(WGM11) | _BV(WGM10);
  TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS11) | _BV(CS10);
  // Timer1 interrupt is not used; instead, overflow bit is tightly
  // polled.  Very infrequently a block read may inexplicably take longer
  // than normal...using an interrupt, the code would clobber itself.
  // By polling instead, worst case is just a minor glitch where the
  // corresponding row will be a little thicker than normal.
#endif

  // Timer0 interrupt is disabled for smoother playback.
  // This means delay(), millis(), etc. won't work after this.
  TIMSK0 = 0;

  ReadyFlowWhite(1);
}

// Startup error handler; doesn't return, doesn't run loop(), just stops.
static void error(const __FlashStringHelper *ptr) {
    Serial.println(ptr); // Show message    
    for(;;);             // and hang
}


// PLAYBACK LOOP ------------------------------------------------------------------------------------------------------------------------

void loop() 
{
  if(digitalRead(STAR) == LOW)
  {
    Setpoint();  
  }
  if(BT.available())
  {
     val = BT.read();
     switch (val)
      {
        case 'm':      
          Setpoint();          
          break;
        case '1':
          bmpDraw(294); // 1s
          break;
        case '2':
          bmpDraw(1470); //  5s
          break;
        case '3':
          bmpDraw(3078); //  10s
          break;
        case '4':
          bmpDraw(6468); //  20s
          break;
        case '5':
          bmpDraw(9408);  //30s
          break;
        case 'a':
          FlowRed(10);
          break;
        case 's':
          FlowGreen(10);
          break;
        case 'd':
          FlowBlue(10);
          break;
        case 'f':
          FlowPurple(10);
          break;
        case 'g':
          FlowWhite(10);
          break;
        case 'o':
          Rainbow(15);
          break;  
        case 'p':
          DynamicRainbow(10); 
          break;
        case 'q':
          Red(); 
          break;
        case 'w':
          Green(); 
          break;
        case 'e':
          Blue(); 
          break;
        case 'r':
          Purple(); 
          break;
        case 't':
          White(); 
          break;
        case 'z':
          BreatheRed(5); 
          break;
        case 'x':
          BreatheGreen(5); 
          break;
        case 'c':
          BreatheBlue(5); 
          break;
        case 'v':
          BreathePurple(5); 
          break;
        case 'b':
          BreatheWhite(5); 
          break;
      }
  }
}

// PLAYBACK LOOP ------------------------------------------------------------------------------------------------------------------------



void bmpDraw(unsigned long time) {
 
  uint32_t block    = 0;     // Current block # within file
  boolean  stopFlag = false; // If set, stop playback loop
  uint32_t lastBlock;
  char     infile[13];
  SdFile   tmp;

  // Get existing contiguous tempfile info
  sprintf(infile, "frame%03d.tmp", frame);
  if(!tmp.open(&root, infile, O_RDONLY)) {
    error(F("Could not open NeoPixel tempfile for input"));
  }
  if(!tmp.contiguousRange(&firstBlock, &lastBlock)) {
    error(F("NeoPixel tempfile is not contiguous"));
  }
  // Number of blocks needs to be calculated from file size, not the
  // range values.  The contiguous file creation and range functions
  // work on cluster boundaries, not necessarily the actual file size.
  nBlocks = tmp.fileSize() / 512;

  tmp.close(); // File handle is no longer accessed, just block reads

  // Stage first block, but don't display yet -- the loop below
  // does that only when Timer1 overflows.
  card.readBlock(firstBlock, sdBuf);
  // readBlock is used rather than readStart/readData/readEnd as
  // the duration between block reads may exceed the SD timeout.

//  while(digitalRead(TRIGGER) == HIGH);   // Wait for trigger button

#ifdef ENCODERSTEPS
  // Set up for rotary encoder
  TCNT1 = 0;
  OCR1A = ENCODERSTEPS;
#else
  // Set up timer based on dial input
  uint32_t linesPerSec = map(analogRead(SPEED), 0, 1023, 10, maxLPS);
  //Serial.print(F("linesPerSec: "));
  //Serial.println(linesPerSec);
  //Serial.print(F("OCR1A: "));
  OCR1A = (F_CPU / 64) / linesPerSec;          // Timer1 interval
  //Serial.println(OCR1A);
#endif

//  for(;;) 
  for(int x =0 ; x<time ; x++ ) 
  {
    while(!(TIFR1 & _BV(TOV1)));               // Wait for Timer1 overflow
    TIFR1 |= _BV(TOV1);                        // Clear overflow bit

    show();                                    // Display current line
    if(stopFlag) break;                        // Break when done

    if(++block >= nBlocks) 
    {                   // Past last block?
      /*
      if(digitalRead(TRIGGER) == HIGH)
      {       // Trigger released?
        memset(sdBuf, 0, N_LEDS * 3);          // LEDs off on next pass
        stopFlag = true;                       // Stop playback on next pass
        continue;
      }         // Else trigger still held
      */
      block = 0;                               // Loop back to start
    }
    card.readBlock(block + firstBlock, sdBuf); // Load next pixel row
  }
  if(++frame >= nFrames) frame = 0;
}

// BMP->NEOPIXEL FILE CONVERSION ---------------------------------------------

#define BMP_BLUE  0 // BMP and NeoPixels have R/G/B color
#define BMP_GREEN 1 // components in different orders.
#define BMP_RED   2 // (BMP = BGR, Neo = GRB)
#define NEO_GREEN 0
#define NEO_RED   1
#define NEO_BLUE  2

// Convert file from 24-bit Windows BMP format to raw NeoPixel datastream.
// Conversion is bottom-to-top (see notes below)...for horizontal light
// painting, the image is NOT rotated here (the per-pixel file seeking this
// requires takes FOREVER on the Arduino).  Instead, such images should be
// rotated counterclockwise (in Photoshop or other editor) prior to moving
// to SD card.  As currently written, both the input and output files need
// to be in the same directory.  Brightness is set during conversion; there
// aren't enough cycles to do this in realtime during playback.  To change
// brightness, re-process image file using new brightness value.
boolean bmpProcess(
  SdFile  &path,
  char    *inName,
  char    *outName,
  uint8_t *brightness) {

  SdFile    inFile,              // Windows BMP file for input
            outFile;             // NeoPixel temp file for output
  boolean   ok        = false,   // 'true' on valid BMP & output file
            flip      = false;   // 'true' if image stored top-to-bottom
  int       bmpWidth,            // BMP width in pixels
            bmpHeight,           // BMP height in pixels
            bmpStartCol,         // First BMP column to process (crop/center)
            columns,             // Number of columns to process (crop/center)
            row,                 // Current image row (Y)
            column;              // and column (X)
  uint8_t  *ditherRow,           // 16-element dither array for current row
            pixel[3],            // For reordering color data, BGR to GRB
            b = 0,               // 1 + *brightness
            d,                   // Dither value for row/column
            color,               // Color component index (R/G/B)
            raw,                 // 'Raw' R/G/B color value
            corr,                // Gamma-corrected R/G/B
           *ledPtr,              // Pointer into sdBuf (output)
           *ledStartPtr;         // First LED column to process (crop/center)
  uint16_t  b16;                 // 16-bit dup of b
  uint32_t  bmpImageoffset,      // Start of image data in BMP file
            lineMax   = 0L,      // Cumulative brightness of brightest line
            rowSize,             // BMP row size (bytes) w/32-bit alignment
            sum,                 // Sum of pixels in row
            startTime = millis();

  if(brightness)           b = 1 + *brightness; // Wraps around, fun with maths
  else if(NULL == outName) return false; // MUST pass brightness for power est.

  Serial.print(F("Reading file '"));
  Serial.print(inName);
  Serial.print(F("'..."));
  if(!inFile.open(&path, inName, O_RDONLY)) {
    Serial.println(F("error"));
    return false;
  }

  if(inFile.read(sdBuf, 34)             &&    // Load header
    (*(uint16_t *)&sdBuf[ 0] == 0x4D42) &&    // BMP signature
    (*(uint16_t *)&sdBuf[26] == 1)      &&    // Planes: must be 1
    (*(uint16_t *)&sdBuf[28] == 24)     &&    // Bits per pixel: must be 24
    (*(uint32_t *)&sdBuf[30] == 0)) {         // Compression: must be 0 (none)
    // Supported BMP format -- proceed!
    bmpImageoffset = *(uint32_t *)&sdBuf[10]; // Start of image data
    bmpWidth       = *(uint32_t *)&sdBuf[18]; // Image dimensions
    bmpHeight      = *(uint32_t *)&sdBuf[22];
    // That's some nonportable, endian-dependent code right there.

    Serial.print(bmpWidth);
    Serial.write('x');
    Serial.print(bmpHeight);
    Serial.println(F(" pixels"));

    if(outName) { // Doing conversion?  Need outFile.
      // Delete existing outFile file (if any)
      (void)SdFile::remove(&path, outName);
      Serial.print(F("Creating contiguous file..."));
      // NeoPixel working file is always 512 bytes (one SD block) per row
      if(outFile.createContiguous(&path, outName, 512L * bmpHeight)) {
        uint32_t lastBlock;
        outFile.contiguousRange(&firstBlock, &lastBlock);
        // Once we have the first block index, the file handle
        // is no longer needed -- raw block writes are used.
        outFile.close();
        nBlocks = bmpHeight; // See note in setup() re: block calcs
        ok      = true;      // outFile is good; proceed
        Serial.println(F("OK"));
      } else {
        Serial.println(F("error"));
      }
    } else ok = true; // outFile not needed; proceed

    if(ok) { // Valid BMP and contig file (if needed) are ready
      Serial.print(F("Processing..."));

      rowSize = ((bmpWidth * 3) + 3) & ~3; // 32-bit line boundary
      b16     = (int)b;

      if(bmpHeight < 0) {       // If bmpHeight is negative,
        bmpHeight = -bmpHeight; // image is in top-down order.
        flip      = true;       // Rare, but happens.
      }

      if(bmpWidth >= N_LEDS) { // BMP matches LED bar width, or crop image
        bmpStartCol = (bmpWidth - N_LEDS) / 2;
        ledStartPtr = sdBuf;
        columns     = N_LEDS;
      } else {                 // Center narrow image within LED bar
        bmpStartCol = 0;
        ledStartPtr = &sdBuf[((N_LEDS - bmpWidth) / 2) * 3];
        columns     = bmpWidth;
        memset(sdBuf, 0, N_LEDS * 3); // Clear left/right pixels
      }

      for(row=0; row<bmpHeight; row++) { // For each row in image...
        Serial.write('.');
        // Image is converted from bottom to top.  This is on purpose!
        // The ground (physical ground, not the electrical kind) provides
        // a uniform point of reference for multi-frame vertical painting...
        // could then use something like a leaf switch to trigger playback,
        // lifting the light bar like making giant soap bubbles.

        // Seek to first pixel to load for this row...
        inFile.seekSet(
          bmpImageoffset + (bmpStartCol * 3) + (rowSize * (flip ?
          (bmpHeight - 1 - row) : // Image is stored top-to-bottom
          row)));                 // Image stored bottom-to-top
        if(!inFile.read(ledStartPtr, columns * 3))  // Load row
          Serial.println(F("Read error"));

        sum       = 0L;
        ditherRow = (uint8_t *)&dither[row & 0x0F]; // Dither values for row
        ledPtr    = ledStartPtr;
        for(column=0; column<columns; column++) {   // For each column...
          if(b) { // Scale brightness, reorder R/G/B
            pixel[NEO_BLUE]  = (ledPtr[BMP_BLUE]  * b16) >> 8;
            pixel[NEO_GREEN] = (ledPtr[BMP_GREEN] * b16) >> 8;
            pixel[NEO_RED]   = (ledPtr[BMP_RED]   * b16) >> 8;
          } else { // Full brightness, reorder R/G/B
            pixel[NEO_BLUE]  = ledPtr[BMP_BLUE];
            pixel[NEO_GREEN] = ledPtr[BMP_GREEN];
            pixel[NEO_RED]   = ledPtr[BMP_RED];
          }

          d = pgm_read_byte(&ditherRow[column & 0x0F]); // Dither probability
          for(color=0; color<3; color++) {              // 3 color bytes...
            raw  = pixel[color];                        // 'Raw' G/R/B
            corr = pgm_read_byte(&gamma[raw]);          // Gamma-corrected
            if(pgm_read_byte(&bump[raw]) > d) corr++;   // Dither up?
            *ledPtr++ = corr;                           // Store back in sdBuf
            sum      += corr;                           // Total brightness
          } // Next color byte
        } // Next column

        if(outName) {
          if(!card.writeBlock(firstBlock + row, (uint8_t *)sdBuf))
            Serial.println(F("Write error"));
        }
        if(sum > lineMax) lineMax = sum;

      } // Next row
      Serial.println(F("OK"));

      if(brightness) {
        lineMax = (lineMax * 20) / 255; // Est current @ ~20 mA/LED
        if(lineMax > CURRENT_MAX) {
          // Estimate suitable brightness based on CURRENT_MAX
          *brightness = (*brightness * (uint32_t)CURRENT_MAX) / lineMax;
        } // Else no recommended change
      }

      Serial.print(F("Processed in "));
      Serial.print(millis() - startTime);
      Serial.println(F(" ms"));

    } // end 'ok' check
  } else { // end BMP header check
    Serial.println(F("BMP format not recognized."));
  }

  inFile.close();
  return ok; // 'false' on various file open/create errors
}

// MISC UTILITY FUNCTIONS ----------------------------------------------------

// Estimate maximum block-read time for card (microseconds)
static uint32_t benchmark(uint32_t block, uint32_t n) {
  uint32_t t, maxTime = 0L;

  do {
    t = micros();
    card.readBlock(block++, sdBuf);
    if((t = (micros() - t)) > maxTime) maxTime = t;
  } while(--n);

  return maxTime;
}

// NEOPIXEL FUNCTIONS --------------------------------------------------------

// The normal NeoPixel library isn't used by this project.  SD I/O and
// NeoPixels need to occupy the same buffer, there isn't quite an elegant
// way to do this with the existing library that avoids refreshing a longer
// strip than necessary.  Instead, just the core update function for 800 KHz
// pixels on 16 MHz AVR is replicated here; not handling every permutation.

static void show(void) {
  volatile uint16_t
    i   = N_LEDS * 3; // Loop counter
  volatile uint8_t
   *ptr = sdBuf,      // Pointer to next byte
    b   = *ptr++,     // Current byte value
    hi,               // PORT w/output bit set high
    lo,               // PORT w/output bit set low
    next,
    bit = 8;

  noInterrupts();
  hi   = *port |  pinMask;
  lo   = *port & ~pinMask;
  next = lo;

  asm volatile(
   "head20_%=:"                "\n\t"
    "st   %a[port],  %[hi]"    "\n\t"
    "sbrc %[byte],  7"         "\n\t"
     "mov  %[next], %[hi]"     "\n\t"
    "dec  %[bit]"              "\n\t"
    "st   %a[port],  %[next]"  "\n\t"
    "mov  %[next] ,  %[lo]"    "\n\t"
    "breq nextbyte20_%="       "\n\t"
    "rol  %[byte]"             "\n\t"
    "rjmp .+0"                 "\n\t"
    "nop"                      "\n\t"
    "st   %a[port],  %[lo]"    "\n\t"
    "nop"                      "\n\t"
    "rjmp .+0"                 "\n\t"
    "rjmp head20_%="           "\n\t"
   "nextbyte20_%=:"            "\n\t"
    "ldi  %[bit]  ,  8"        "\n\t"
    "ld   %[byte] ,  %a[ptr]+" "\n\t"
    "st   %a[port], %[lo]"     "\n\t"
    "nop"                      "\n\t"
    "sbiw %[count], 1"         "\n\t"
     "brne head20_%="          "\n"
    : [port]  "+e" (port),
      [byte]  "+r" (b),
      [bit]   "+r" (bit),
      [next]  "+r" (next),
      [count] "+w" (i)
    : [ptr]    "e" (ptr),
      [hi]     "r" (hi),
      [lo]     "r" (lo));

  interrupts();
  // There's no explicit 50 uS delay here as with most NeoPixel code;
  // SD card block read provides ample time for latch!
}

//==========================================================================================
//======================================= E F F E C T ======================================
//==========================================================================================

void White ()
{
      //clearLEDs();
      for (int i=0; i<LED_COUNT; i++)
      {
         leds.setPixelColor(i,  50, 50, 50 );
         leds.show();  
         delay(50);
      }     
}

void Red ()
{
      //clearLEDs();
      for (int i=0; i<LED_COUNT; i++)
      {
         leds.setPixelColor(i,  50, 0, 0 );
         leds.show();  
         delay(50);
      }
}

void Green ()
{
      //clearLEDs();
      for (int i=0; i<LED_COUNT; i++)
      {
         leds.setPixelColor(i,  0, 50, 0 );
         leds.show();  
         delay(50);
      } 
}

void Blue ()
{
      //clearLEDs();
      for (int i=0; i<LED_COUNT; i++)
      {
         leds.setPixelColor(i,  0, 0, 50 );
         leds.show();  
         delay(50);
      }
}

void Purple ()
{
      //clearLEDs();
      for (int i=0; i<LED_COUNT; i++)
      {
         leds.setPixelColor(i,  50, 0, 50  );
         leds.show();  
         delay(50);
      }
}

void Rainbow(unsigned int x)
{
    for (; x>0; x--)
    { 
      for (int i=0; i<LED_COUNT; i++)
      {
         leds.setPixelColor(i,  50, 0, 0);
         leds.show();  
      }
      delay(100);
      for (int i=0; i<LED_COUNT; i++)
      {
         leds.setPixelColor(i,  0, 50, 0);
         leds.show();  
      }
      delay(100);
      for (int i=0; i<LED_COUNT; i++)
      {
         leds.setPixelColor(i,  0, 0, 50);
         leds.show();  
      }
      delay(100);
      for (int i=0; i<LED_COUNT; i++)
      {
         leds.setPixelColor(i,  50, 0, 50);
         leds.show();  
      }
      delay(100);
      for (int i=0; i<LED_COUNT; i++)
      {
         leds.setPixelColor(i,  50, 50, 50);
         leds.show();  
      }
    }
}

void ReadyFlowWhite (unsigned int x)
{
  for (; x>0; x--)
  {      
      for (int i=-11; i<LED_COUNT+10; i++)
      {
         clearLEDs();
         leds.setPixelColor(i,  2,2,2 );
         leds.setPixelColor(i+1,  4,4,4 );
         leds.setPixelColor(i+2,  5,5,5 );
         leds.setPixelColor(i+3,  5,5,5 );
         leds.setPixelColor(i+4,  10,10,10 );
         leds.setPixelColor(i+5,  10,10,10 );
         leds.setPixelColor(i+6,  10,10,10 );
         leds.setPixelColor(i+7,  20,20,20 );
         leds.setPixelColor(i+8,  10,10,10 );
         leds.setPixelColor(i+9,  5,5,5 );
         leds.setPixelColor(i+10, 4,4,4 );
         leds.setPixelColor(i+11, 2,2,2 );
         leds.show();  
         
         for(int j=0;j<6;j++)
         {
            Serial.print(j);
         }
         
      }
  } 
  Serial.println(" ");    
  Serial.print("It's show time!");    
}

void FlowWhite (unsigned int x)
{
  for (; x>0; x--)
  {      
      for (int i=-11; i<LED_COUNT+10; i++)
      {
         clearLEDs();
         leds.setPixelColor(i,  20,20,20 );
         leds.setPixelColor(i+1,  40,40,40 );
         leds.setPixelColor(i+2,  50,50,50 );
         leds.setPixelColor(i+3,  70,70,70 );
         leds.setPixelColor(i+4,  100,100,100 );
         leds.setPixelColor(i+5,  100,100,100 );
         leds.setPixelColor(i+6,  130,130,130 );
         leds.setPixelColor(i+7,  WHITE );
         leds.setPixelColor(i+8,  WHITE );
         leds.setPixelColor(i+9,  WHITE );
         leds.setPixelColor(i+10, WHITE );
         leds.setPixelColor(i+11, WHITE );
         leds.show();  
         delay(20);
      }
  }     
}

void FlowRed (unsigned int x)
{
  for (; x>0; x--)
  {      
      for (int i=-11; i<LED_COUNT+10; i++)
      {
         clearLEDs();
         leds.setPixelColor(i,  20,0,0 );
         leds.setPixelColor(i+1,  40,0,0 );
         leds.setPixelColor(i+2,  50,0,0 );
         leds.setPixelColor(i+3,  70,0,0 );
         leds.setPixelColor(i+4,  100,0,0 );
         leds.setPixelColor(i+5,  100,0,0 );
         leds.setPixelColor(i+6,  130,0,0 );
         leds.setPixelColor(i+7,  RED);
         leds.setPixelColor(i+8,  RED );
         leds.setPixelColor(i+9,  RED );
         leds.setPixelColor(i+10, RED );
         leds.setPixelColor(i+11, RED );
         leds.show();  
         delay(20);
      }
  }     
}
void FlowGreen (unsigned int x)
{
  for (; x>0; x--)
  {      
      for (int i=-11; i<LED_COUNT+10; i++)
      {
         clearLEDs();
         leds.setPixelColor(i,  0,20,0 );
         leds.setPixelColor(i+1,  0,40,0 );
         leds.setPixelColor(i+2,  0,50,0 );
         leds.setPixelColor(i+3,  0,70,0 );
         leds.setPixelColor(i+4,  0,100,0 );
         leds.setPixelColor(i+5,  0,100,0 );
         leds.setPixelColor(i+6,  0,130,0 );
         leds.setPixelColor(i+7,  GREEN );
         leds.setPixelColor(i+8,  GREEN );
         leds.setPixelColor(i+9,  GREEN );
         leds.setPixelColor(i+10, GREEN );
         leds.setPixelColor(i+11, GREEN );
         leds.show();  
         delay(20);
      }
  }     
}
void FlowBlue (unsigned int x)
{
  for (; x>0; x--)
  {      
      for (int i=-11; i<LED_COUNT+10; i++)
      {
         clearLEDs();
         leds.setPixelColor(i,  0,0,20 );
         leds.setPixelColor(i+1,  0,0,40 );
         leds.setPixelColor(i+2,  0,0,50 );
         leds.setPixelColor(i+3,  0,0,70 );
         leds.setPixelColor(i+4,  0,0,100 );
         leds.setPixelColor(i+5,  0,0,100 );
         leds.setPixelColor(i+6,  0,0,130 );
         leds.setPixelColor(i+7,  BLUE );
         leds.setPixelColor(i+8,  BLUE );
         leds.setPixelColor(i+9,  BLUE );
         leds.setPixelColor(i+10, BLUE );
         leds.setPixelColor(i+11, BLUE );
         leds.show();  
         delay(20);
      }
  }     
}
void FlowPurple (unsigned int x)
{
  for (; x>0; x--)
  {      
      for (int i=-11; i<LED_COUNT+10; i++)
      {
         clearLEDs();
         leds.setPixelColor(i,  20,0,20 );
         leds.setPixelColor(i+1,  40,0,40 );
         leds.setPixelColor(i+2,  50,0,50 );
         leds.setPixelColor(i+3,  70,0,70 );
         leds.setPixelColor(i+4,  100,0,100 );
         leds.setPixelColor(i+5,  100,0,100 );
         leds.setPixelColor(i+6,  130,0,130 );
         leds.setPixelColor(i+7,  PURPLE );
         leds.setPixelColor(i+8,  PURPLE );
         leds.setPixelColor(i+9,  PURPLE );
         leds.setPixelColor(i+10, PURPLE );
         leds.setPixelColor(i+11, PURPLE );
         leds.show();  
         delay(20);
      }
  }     
}

//=============================================
//=============  B R E A T H  =================
//=============================================

void BreatheWhite(unsigned int x)
{
  for (; x>0; x--)
  {
    for (int BreatheNum = 0; BreatheNum < 100 ; BreatheNum++ )
    {
      for (int i=0; i<LED_COUNT; i++)
      {
        leds.setPixelColor(i, BreatheNum , BreatheNum , BreatheNum );
      }
      leds.show();  
      delay(2);
    }
    for (int BreatheNum = 100; BreatheNum > 0 ; BreatheNum-- )
    {
      for (int i=0; i<LED_COUNT; i++)
      {
        leds.setPixelColor(i, BreatheNum , BreatheNum , BreatheNum );
      }
      leds.show();  
      delay(2);
    }    
  } 
}

void BreatheRed(unsigned int x)
{
  for (; x>0; x--)
  {
    for (int BreatheNum = 0; BreatheNum < 100 ; BreatheNum++ )
    {
      for (int i=0; i<LED_COUNT; i++)
      {
        leds.setPixelColor(i, BreatheNum , 0 , 0 );
      }
      leds.show();  
      delay(2);
    }
    for (int BreatheNum = 100; BreatheNum > 0 ; BreatheNum-- )
    {
      for (int i=0; i<LED_COUNT; i++)
      {
        leds.setPixelColor(i, BreatheNum , 0 , 0 );
      }
      leds.show();  
      delay(2);
    }    
  } 
}

void BreatheGreen(unsigned int x)
{
  for (; x>0; x--)
  {
    for (int BreatheNum = 0; BreatheNum < 100 ; BreatheNum++ )
    {
      for (int i=0; i<LED_COUNT; i++)
      {
        leds.setPixelColor(i, 0 , BreatheNum , 0 );
      }
      leds.show();  
      delay(2);
    }
    for (int BreatheNum = 100; BreatheNum > 0 ; BreatheNum-- )
    {
      for (int i=0; i<LED_COUNT; i++)
      {
        leds.setPixelColor(i, 0 , BreatheNum , 0 );
      }
      leds.show();  
      delay(2);
    }    
  } 
}

void BreatheBlue(unsigned int x)
{
  for (; x>0; x--)
  {
    for (int BreatheNum = 0; BreatheNum < 100 ; BreatheNum++ )
    {
      for (int i=0; i<LED_COUNT; i++)
      {
        leds.setPixelColor(i, 0 , 0 , BreatheNum );
      }
      leds.show();  
      delay(2);
    }
    for (int BreatheNum = 100; BreatheNum > 0 ; BreatheNum-- )
    {
      for (int i=0; i<LED_COUNT; i++)
      {
        leds.setPixelColor(i, 0 , 0 , BreatheNum );
      }
      leds.show();  
      delay(2);
    }    
  } 
}

void BreathePurple(unsigned int x)
{
  for (; x>0; x--)
  {
    for (int BreatheNum = 0; BreatheNum < 100 ; BreatheNum++ )
    {
      for (int i=0; i<LED_COUNT; i++)
      {
        leds.setPixelColor(i, BreatheNum , 0 , BreatheNum );
      }
      leds.show();  
      delay(2);
    }
    for (int BreatheNum = 100; BreatheNum > 0 ; BreatheNum-- )
    {
      for (int i=0; i<LED_COUNT; i++)
      {
        leds.setPixelColor(i, BreatheNum , 0 , BreatheNum );
      }
      leds.show();  
      delay(2);
    }    
  } 
}

//=============================================
//============= R A I N B O W =================
//=============================================

void DynamicRainbow (unsigned int x)
{  
  // Ride the Rainbow Road
  for (; x>0; x--)
  {
    for (int i=0; i<LED_COUNT*5; i++)
    {
      rainbow(i);
      delay(40);  // Delay between rainbow slides
    }
  }
}
//----------------------------------------------------------  
// Sets all LEDs to off, but DOES NOT update the display;
// call leds.show() to actually turn them off after this.
void clearLEDs()
{
  for (int i=0; i<LED_COUNT; i++)
  {
    leds.setPixelColor(i, 0);
  }
}
// Prints a rainbow on the ENTIRE LED strip.
//  The rainbow begins at a specified position. 
// ROY G BIV!
void rainbow(byte startPosition) 
{
  // Need to scale our rainbow. We want a variety of colors, even if there
  // are just 10 or so pixels.
  int rainbowScale = 192 / LED_COUNT;
  
  // Next we setup each pixel with the right color
  for (int i=0; i<LED_COUNT; i++)
  {
    // There are 192 total colors we can get out of the rainbowOrder function.
    // It'll return a color between red->orange->green->...->violet for 0-191.
    leds.setPixelColor(i, rainbowOrder((rainbowScale * (i + startPosition)) % 192));
  }
  // Finally, actually turn the LEDs on:
  leds.show();
}

// Input a value 0 to 191 to get a color value.
// The colors are a transition red->yellow->green->aqua->blue->fuchsia->red...
//  Adapted from Wheel function in the Adafruit_NeoPixel library example sketch
uint32_t rainbowOrder(byte position) 
{
  // 6 total zones of color change:
  if (position < 31)  // Red -> Yellow (Red = FF, blue = 0, green goes 00-FF)
  {
    return leds.Color(0xFF, position * 8, 0);
  }
  else if (position < 63)  // Yellow -> Green (Green = FF, blue = 0, red goes FF->00)
  {
    position -= 31;
    return leds.Color(0xFF - position * 8, 0xFF, 0);
  }
  else if (position < 95)  // Green->Aqua (Green = FF, red = 0, blue goes 00->FF)
  {
    position -= 63;
    return leds.Color(0, 0xFF, position * 8);
  }
  else if (position < 127)  // Aqua->Blue (Blue = FF, red = 0, green goes FF->00)
  {
    position -= 95;
    return leds.Color(0, 0xFF - position * 8, 0xFF);
  }
  else if (position < 159)  // Blue->Fuchsia (Blue = FF, green = 0, red goes 00->FF)
  {
    position -= 127;
    return leds.Color(position * 8, 0, 0xFF);
  }
  else  //160 <position< 191   Fuchsia->Red (Red = FF, green = 0, blue goes FF->00)
  {
    position -= 159;
    return leds.Color(0xFF, 0x00, 0xFF - position * 8);
  }
}

//=============================================
//============= R A I N B O W =================
//=============================================
/*
void delay100m (unsigned int t)
{
	unsigned int a,b;
	for(;t>0;t--)
	for(a=0;a<2000;a++)
	for(b=0;b<1650;b++);
}
*/
//==========================================================================================
//======================================= E F F E C T ======================================
//==========================================================================================


//============================================================================== SET POINT
//===(3)=0.01s
//===(32)=0.1s
//===(326)=1s
//===(3260)=10s
void BBB ()
{
      //clearLEDs();
      for (int i=0; i<LED_COUNT; i++)
      {
         leds.setPixelColor(i,  0, 0, 5 );
         leds.show();  
         delay(50);
      }
}

void Setpoint()
{
    BBB();
    
//    bmpDraw(1000);


    for(int x =0 ; x<2000 ; x++ ) 
    {
          bmpDraw(5000);
    }
}

//============================================================================== SET POINT
