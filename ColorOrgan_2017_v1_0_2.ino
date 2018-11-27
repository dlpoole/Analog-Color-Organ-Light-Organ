//----------------------------------------------------------------------------------------------
//        Digital Color Organ and LED Animator for Arduino DUE
//                and WS2811-based RGB LED strings
//              v1.0.2  7-November-2017  D.L. Poole
//----------------------------------------------------------------------------------------------
// This sketch animates music automatically following the method patented to
// Frederic L. Way as US 3,018,683 and US 3,181,015 in 1962 and 1965.  Way’s first
// patent used filters and a scaled-up meter movement to control a resistive dimmer;
// his second patent used the Silicon Controlled Rectifier first commercialized in 1957.
// Way’s patents were assigned to Mobilcolor, Inc, which leased equipment based on
// them to radio stations including Omaha and Indianapolis, where it was used for
// holiday “Carol Tree” displays.
//
// This implementation adds a fast AGC, digital filtering, full wave peak detection,
// dB scaling, time constants to approximate a bar display, and mapping of the three
// band dB converted results to LED intensities over a given dynamic range.  Cascaded
// first order filters are used instead of Way’s second order filters to make 10KHz
// sampling feasible on an Arduino DUE. The filter crossovers are at 60Hz and 1200Hz
// instead of Way’s 540Hz and 1800Hz, but these and all other “analog” parameters are
// named constants in this sketch so as to be easily modified to taste.
//
// As an alternative to music input, an IR remote can be used to select a uniform color
// of arbitrary hue, saturation and brightness.  Alternately, a rainbow along successive
// LED strings or random colors can be displayed with selectable saturation and brightness.
// The uniform hue can be varied or the rainbow trickled and a sparkle effect can be added
// and sped up or slowed.  These effects can be forked and recompiled and run on an
// Arduino UNO. Only the color organ function requires the greater CPU throughput of
// Arduino DUE.
//
// This sketch requires the FastLED library from http://fastled.io  and a fork of
// the library irlib2 from https://github.com/cyborg5/IRLib2  with unreferenced headers and
// code for IRLLibRecv removed since it throws errors if compiled for SAM3X8E. The forked
// library, renamed IRLibPCISamX, should be carried along with the files for this sketch
// and placed in the /libraries folder associated with the v1.8.4 IDE.  The IDE must then be
// restarted to load it.
//
// Although other decoders are included below, the sketch assumes NEC code such as is
// sent by the Adafruit Mini Remote.  This relies on the autoresume feature which uses
// the repeat 0xffffffff code as an end-of-frame.  The NEC decoder rapidly decodes the
// repeat frames, necessitating a count of 5 repeats before accepting a held key as a repeat.
//
// A problem arises if one attempts to power a genuine DUE from a 7 to 12 volt source via Vin.
// Late shipments of DUE model A000062 used a MPM3620 switching regulator in lieu of the
// LM2734Y of the reference design. The MPM3620 operates at at 2MHz so as to avoid
// a discrete 100uH inductor of the earlier design; however involves substantial currents
// within the regulator circuit which introduce random noise in the low LS 4 to 6 bits
// of the ADC. No way could be found to externally decouple the noise voltages on the 3.3
// and 5V lines, suggesting that regulator loop currents excape the regulator itself and
// flow through or around the CPU itself. The noise produces flicker in the LEDs unless
// AGC_BOOST is reduced substantially, limiting the dynamic range and requiring a 1V RMS
// audio signal.
//
// The DUE reportedly stood discontinued as of this sketch, but several DUE R3 clones were
// purchased which used the LM2734Y and discrete inductor of the reference design. These were
// noise free, but lacked resistor R99 of the reference design. R99 was introduced after the
// first reference schematic was published to ground a floating gate on MOSFET T3 during the
// startup of the SAM3X8E and before the startup of the USB interface which controls it.
// without reworkding to add this resistor, the DUE may start in ERASE mode unless powered
// OFF and ON before the required charge leaks off the floating gate. A discrete 10K resistor
// can be soldered between source and drain of T3 to avoid this.

#include "FastLED.h"
#include <IRLibDecodeBase.h>
#include <IRLib_P01_NEC.h>
#include <IRLib_P02_Sony.h>
#include <IRLib_P07_NECx.h>
#include <IRLibCombo.h>
#include <IRLibRecvPCI.h>

// Hardware Inputs/Outputs

// Pin D2:   Data input from 83KHZ 3.3V IR receiver
// Pin A0:   Analog Audio capacitively coupled with a 16K/16K bias divider from 3.3V
// Pins 25-28... Serial data to LED strings (via CD40109 3.3 to 5V level shifter)
// PWM 12:  Telemetry of sampling rate - should exhibit 100uS half period
// PWM 13:  Telemetry of sampling/filtering and display HIGH=sampling LOW=display
// DAC0:    Analog telemetry of sampling/filtering data
// DAC1:    Analog telemetry of sampling/filtering data

// I/O Pins and Timing Constants

#define REMOTE_PIN 2                 //pin for IR sensor data
#define NUMBER_STRINGS 4             //number of strings used to scale rainbow
#define LEDS_PER_STRING 50           //LEDs per string
#define TOTAL_LEDS  NUMBER_STRINGS * LEDS_PER_STRING    //total LEDS
#define DATA_PIN  25                 //pin for first LED string data line
#define AUDIO_PIN  0                 //pin for audio input
#define DC_BIAS 1.65                 //audio input DC bias in volts
#define MAX_ANALOG  4095.            //maximum raw ADC read or channel brightness
#define BRIGHT_LSB 0.0039            //LSB of LED brightness
#define SATURATION_LIMIT 0.45        //practically white if below this
#define SAMPLE_TIME  100E-6          //time per sample in sec for 10KHz sample rate
#define SAMPLE_SIZE  320             //number of samples taken between LED updates
#define UPDATE_PERIOD 32             //in mS for above SAMPLE_SIZE and SAMPLE_TIME
#define SAMPLING_LOOP_PAD  18        //pad in uS for 100uS sampling loop 10KHz rate

// LED Animation Parameters

#define MIN_ANIMATION_TIME 16       //mS per step
#define MAX_ANIMATION_TIME 4096     //mS per step
#define MIN_SPARKLE_RATE 64         //sparkles per minute
#define MAX_SPARKLE_RATE 4096       //sparkles per minute
#define HUE_STEPS 64                //steps in hue cycle for nudges
#define FAST_ANIMATION_TIME  10     //fast animation rate in mS 

// Color Organ parameters are collected here for ease in tuning. The 3 dB
// corner frequencies are specified in Hz. The filter responses overlap so
// as to form secondary colors with even a pure tone input in the overlap.
//
// Channel gains depend on interplay between filter sections, the green LP
// and HP sections in particular since the passband is formed by the overlap
// between LP and HP stopbands.
//
// DYNAMIC_RANGE (in dB below peak input) is a significant parameter. Too
// high a setting will allow the LEDs to remain ON far down the filter skirts,
// producing mostly pastel shades and potentially introducing flicker due to
// noise levels in the input signal. If the setting is too low, the LEDs will
// be ON only in the passbands, and secondary colors (magenta, cyan. and yellow)
// will be nondisplayed with swept tones and infrequent on spectrally broader
// music. A comparative frequency plot of the filter responses in dB with
// Dynamic Range drawn over them as a horizontal line may be instructive.

// GAINs are also significant.  Set TELEMETRY = true to enable serial dump of
// brightnesses. Set GAINs = 1.0. Set AGC_BOOST = 0.  Applied sine wave test tone
// 1.16 VRMS to obtain MAX_SAMPLE = 2048 as tone swept into each passband. Used
// telemetered brightnesses to adjust respective gains for a target brightness
// of MAX_ANALOG/3 = 1350.  This allows a program audio crest factor of 3 (9.5db)
// before saturating any channel. The values actually used were tested over a
// variety of musical sources and allow for greater crest factor.

#define RED_GAIN 5                   //5 for 9.5dB CF over sine tone
#define GREEN_GAIN 7                 //9 for 9.5dB CF over sine tone
#define BLUE_GAIN 5                  //3 for 9.5dB CF over sine tone
#define BLUE_HP_CORNER  10           //Hz
#define BLUE_LP_CORNER 60            //Hz
#define GREEN_LP_CORNER 300          //Hz
#define GREEN_HP_CORNER  300         //Hz
#define RED_HP_CORNER 1200           //Hz
#define DYNAMIC_RANGE 25             //dB down from ANALOG_MAX to to LED cutoff
#define BRIGHTNESS_TAU  60.           //time const in mS for VU meter-like response
#define DEFAULT_QUIET_HUE 0.          //defaults for color organ's quiet periods
#define DEFAULT_QUIET_SATURATION 0.0  //white
#define DEFAULT_QUIET_BRIGHTNESS 0.0  //default off when quiet
#define TELEMETRY false               //true dumps Gain, Raw, RBG to serial monitor

// AGC parameters: The DUE ADCs' 0-3.3V (AC ±1.65V) linear range is well suited to
// consumer equipment line outputs, though some individual receivers, etc. may
// deliver less.  Nonbroadcast sources are typically uncompressed and have a dynamic
// range in excess of the 8-bits recognized by the led strings and the eye, so a gain
// boost is applied to audio below full scale with a time constant. Audio above full
// scale is corrected instantaneously before a sample is processed. AGC boost
// assures the LEDS will respond to soft passages and prevents them from extinguishing
// before a musical passage has completely decayed.  AGC_BOOST may have to be reduced
// to accompdate a noisy audio source or an original Arduino DUE with the high speed
// regulator.

#define AGC_BOOST 30      //dB of maximum AGC boost
#define AGC_TAU  1.0         //seconds to 63% of AGC_BOOST

// Color Organ globals calculated once in setup() for use in loop()
// Sampling rate is CPU-time critical so everything that can be precomputed is done
// in setup()

float AGCalpha;             //AGC smoothing coefficient
float AGCboost;             //AGC max boost as ratio
float redHPalpha;           //red HP smoothing coefficient - function of time constant
float blueLPalpha;          //blue LP smoothing coefficient - function of time constant
float blueHPalpha;          //blue HP smoothing coefficient - function of time constant
float greenHPalpha;         //green HP smoothing coefficients - function of time constant
float greenLPalpha;         //green LP smoothing coefficients - function of time constant
int   numberStrings;        //actual number of strings deployed
float biasLpAlpha;          //lowpass coefficient for DC bias measurement
float hanningWindow[SAMPLE_SIZE];     //pre-computed Hann(ing) window
float inputBias;            //measured DC input bias

// Globals adjusted by the remote control. Some of these are shared between the fixed
// effects and the color organ

bool  onOff = true;               //default ON
float hue = 0.0;                  //solid hue or rainbow offset; 0 = Blue --> Violet = 1
float saturation = 1.0;           //0 = Hue dimmed to zero; 1=Saturated hue; 4095 = white
float brightness = 1.0;           //0 = OFF; 1.0 = FULL ON
float quietRed;                   //minimum red for quiet periods
float quietBlue;                  //minimum blue for quiet periods
float quietGreen;                 //minimum green for quiet periods
bool  sparkle = false;            //to sparkle or not to sparkle
int   sparklesPerMinute;          //avg number of sparkles per minute
int   slowAnimationTime = 0;      //rate for slow animations
float quietHue;                   //color organ's quiet periods
float quietSaturation;            //color organ's quiet periods
float quietBrightness;            //color organ's quiet periods

// IR Remote Key-handling variables

unsigned int keyValue;      //key handling variables
unsigned int keyCode;
unsigned int repeatCount;
unsigned long keyTimer;     //for held remote key

// The buffer[] is used for non Color Organ effects, such as solid color,
// rainbow, trickles, and sparkle.  Animations are done on buffer[] and
// transferred to the LEDs near the end of loop().  When in color organ
// mode, ShowColorOrgan loads LEDs directly for speed since all LEDs change
// in unison.

struct float_RGB {
  float r;
  float g;
  float b;
};
CRGB leds[TOTAL_LEDS];          //FastLED 3-byte structure for LEDs
float_RGB buffer[TOTAL_LEDS];   //LED buffer[] for animation effects

// Setup IR Remote Receiver

IRrecvPCI myReceiver(REMOTE_PIN);    //instantiate receiver on REMOTE_PIN
IRdecode myDecoder;                  //instantiate decoder

// loop() globals

long animationTimer;                 //10mS timer for sparkle and animations
long slowAnimationTimer;             //100mS for variable hue changes
unsigned long loopStartTime;         //for measuring loop time
int currentState;
int sampleCount;                     //count of audio samples
enum currentState {
  ColorOrgan,
  Rainbow,
  SolidColor,
  RandomColors,
  QuietColor
};

void setup()  {
  Serial.begin(115200);                   //telemetry for debugging and tuning
  Serial.println("v1.0.2  7-November-2017  D.L. Poole");
  myReceiver.enableIRIn();                //start IR receiver

  // Set up WS-2811-based LED strings on individual output pins starting from DATA_PIN
  // The sketch can be used with a varying number of strings by editing NUMBER_STRINGS
  // to properly scale the rainbow to one hue cycle across all strings.  Although fewer
  // strings may be used, six are created here to set the minimum software timing of
  // FastLED.show and hence the sparkle effect.

  FastLED.addLeds < WS2811, DATA_PIN, RGB > (leds, LEDS_PER_STRING);
  FastLED.addLeds < WS2811, DATA_PIN + 1, RGB > (leds + 50, LEDS_PER_STRING);
  FastLED.addLeds < WS2811, DATA_PIN + 2, RGB > (leds + 100, LEDS_PER_STRING);
  FastLED.addLeds < WS2811, DATA_PIN + 3, RGB > (leds + 150, LEDS_PER_STRING);
  FastLED.addLeds < WS2811, DATA_PIN + 4, RGB > (leds + 200, LEDS_PER_STRING);
  FastLED.addLeds < WS2811, DATA_PIN + 5, RGB > (leds + 250, LEDS_PER_STRING);

  // Analog data can be taken out of the sampling section
  // on the DAC0 or DAC1 pins without significantly affecting sampling rate.
  // Measurements can then be made on those pins with an oscilloscope.
  // 12-bit resolution is used, so PWM outputs will range from 0 to 100% duty
  // cycle and DAC0 outputs will range from 0.55 to 2.75V over the 0-0x0fff (4095)
  // data range, corresponding to a sensitivity of .000537V/count

  pinMode(6, OUTPUT);      //Log scaled PWM out for Red LEDs
  pinMode(7, OUTPUT);      //Log scaled PWM out for Green LEDs
  pinMode(8, OUTPUT);      //Log scaled PWM out for Blue LEDs
  pinMode(9, OUTPUT);      //Frame sync for scope
  pinMode(11, OUTPUT);     //telemetry
  pinMode(12, OUTPUT);     //telemetry for sample rate
  pinMode(13, OUTPUT);     //telemetry for LED update time

  // analogReadResolution() is necessary for audio sampling, but the board throws an error
  // error assertion "duty <= pPwm->PWM_CH_NUM[ul_channel].PWM_CPRD" failed:
  // file "../source/pwmc.c", line 272, function: PWMC_SetDutyCycle Exiting with status 1
  // to its serial port when the analogRead is executed unless the analogWriteResolution()
  // statement is also present.  The statement is required for output telemetry anyway

  analogReadResolution(12);     //set input sampling for 12-bit Audio as 0x000 to 0xfff from 0V to +3.3V
  analogWriteResolution(12);    //set output telemetry as 0.25V to 2.75V from 0x000 to 0xfff

  // One time Calculations.  The audio sampling rate is CPU-limited so everything that can
  // be precomputed is done here.  One time calculation of exponential smoothing coefficients
  // is done from the named constant for corner freq in Hz and 1/sampling rate. The sampling
  // rate is execution time dependent, and so should be checked by scoping or half-period
  // counting pin 12 and padding the loop for 100uS to provide a 10KHz sampling rate on which
  // the filter cutoffs depend.  A compile before upload seems to improve the compiler
  // optimization of the 1.8.3 IDE and maximize sampling rate.

  // DUE has sufficient processing power to sample and maintain 13 first order LP and HP
  // filter sections in that time using exponential moving averages.  The corner frequencies of
  // each section are defined as named constants in Hz and pre-converted to equivalent smoothing
  // coefficients for use in the sampling/filtering loop. The AGC and input bias measuring
  // time constants are similarly pre-calculated for 1/display update rate

  redHPalpha = 1 - pow(2.71828, -SAMPLE_TIME * 2 * PI * RED_HP_CORNER);
  greenHPalpha = 1 - pow(2.71828, -SAMPLE_TIME * 2 * PI * GREEN_HP_CORNER);
  greenLPalpha = 1 - pow(2.71828, -SAMPLE_TIME * 2 * PI * GREEN_LP_CORNER);
  blueLPalpha = 1 - pow(2.71828, -SAMPLE_TIME * 2 * PI * BLUE_LP_CORNER);
  blueHPalpha = 1 - pow(2.71828, -SAMPLE_TIME * 2 * PI * BLUE_HP_CORNER);
  AGCboost  = pow(10., AGC_BOOST / 20.);
  inputBias = DC_BIAS / 3.3 * MAX_ANALOG;                              //initial value
  biasLpAlpha = 1 - pow(2.71828, - UPDATE_PERIOD * 0.001 / 1);        //τ= 1 sec fc = 0.1Hz
  AGCalpha = UPDATE_PERIOD * 0.001  / AGC_TAU;

  // Sampling and filtering are performed at the sampling rate in blocks of SAMPLE_SIZE
  // interspersed with brightness calculations and LED updates at the slower update rate.
  // The pre-tabulated Hann(ing) window function controls the sidelobes of this secondary
  // sampling in order to minimize the flicker for steady tones on the filter skirts

  for (int i = 0; i < SAMPLE_SIZE; i++) {
    hanningWindow[i] = 0.5 * (1.0 - cos(2 * PI * i / (SAMPLE_SIZE - 1)));
  }

  // Start out LEDs in string rainbow.  Useful for knowing which string is which

  byte red;
  byte green;
  byte blue;
  for (int j = 0; j < NUMBER_STRINGS; j++) {
    if (j == 0) {
      red = 255;  //violet
      green = 0;
      blue = 255;
    }
    if (j == 1) {
      red = 0;  //blue
      green = 0;
      blue = 255;
    }
    if (j == 2) {
      red = 0;  //green
      green = 255;
      blue = 0;
    }
    if (j == 3) {
      red = 255;  //yellow
      green = 255;
      blue = 0;
    }
    if (j == 5) {
      red = 255;  //orange
      green = 128;
      blue = 0;
    }
    if (j > 6) {
      red = 255;  //red
      green = 0;
      blue = 0;
    }
    for (int i = 0; i < LEDS_PER_STRING; i++) {
      leds[j * LEDS_PER_STRING + i].r = red;
      leds[j * LEDS_PER_STRING + i].g = green;
      leds[j * LEDS_PER_STRING + i].b = blue;
    }
    FastLED.show();
  }
  delay(2000);                                   //hold the init colors 2 seconds
  saturation = DEFAULT_QUIET_SATURATION;         //load the default quiet color
  hue = DEFAULT_QUIET_HUE;
  brightness = DEFAULT_QUIET_BRIGHTNESS;
  UpdateQuietColor();                            //update the quiet RGBs
  slowAnimationTime = 0;                         //default to color organ
  sparkle = false;
  sparklesPerMinute = MAX_SPARKLE_RATE;
  saturation = 1.0;
  brightness = 1.0;
  currentState = ColorOrgan;

}   //end of setup()

void loop() {      //"You can check out any time you like, but you can never leave."  Glenn Frey

  // User Interface. Check for a remote keypress and dispatch them
  // Switch arguments are for NEC codes from Adafruit Mini Remote

  if (millis() - keyTimer > 500) {              //reset repeat count if idle
    repeatCount = 0;
    keyCode = 0;
  }
  if (myReceiver.getResults()) {                //if a frame is pending in the buffer
    myDecoder.decode();                         //decode it
    //myDecoder.dumpResults(true);              //uncomment to dump raw decode - true for detail
    myReceiver.enableIRIn();                    //restart the receiver for immediate frame

    if (myDecoder.value == 0xffffffff) {        //for NEC remote, count repeat frames
      keyTimer = millis();                      //restart idle timer
      if (repeatCount++ > 5) {                  //honor if more than 5 consecutive
        myDecoder.value = keyCode;              //by repeating the last key
      }
    }
    if (myDecoder.protocolNum == 1 && myDecoder.value != 0x00000000
        && myDecoder.value != 0xffffffff ) {    //for NEC codes
      keyCode = myDecoder.value & 0xffff;       //save in case it is to be repeated
      keyTimer = millis();                      //restart idle timer

      if (keyCode == 0x58a7) {                                //On/Off button always checked
        onOff = !onOff;                     //toggle it
        if (onOff == false) {               //if OFF
          for (int i = 0; i < TOTAL_LEDS; i++) { //turn LEDS OFF
            leds[i].r = 0;
            leds[i].g = 0;
            leds[i].b = 0;
          }
          FastLED.show();
        }
      }
      if (onOff == true) {                                   //remaining buttons if ON
        switch (keyCode) {
          case 0x08f7: {                                     // Return to Color Organ
              currentState = ColorOrgan;
            }
            break;
          //----------Show quiet color------------------------------------------------
          case 0x8877:
            currentState = QuietColor;
            slowAnimationTime = 0;
            sparkle = false;
            saturation = quietSaturation;
            hue = quietHue;
            brightness = quietBrightness;
            break;
          //-----------Solid Color----------------------------------------------------
          case 0x48b7:
            if (currentState != SolidColor) {             // first entry from any other mode
              currentState = SolidColor;
              slowAnimationTime = 0;                      // start with a fixed blue
              saturation = 1.0;
              hue = 0.;
              brightness = 1.0;
            }
            else                                              // second entry while non animated
              if (slowAnimationTime == 0) {
                slowAnimationTime = MIN_ANIMATION_TIME;       // start fastest animation
                slowAnimationTimer = millis();
              }
              else {
                slowAnimationTime *= 2;                       // repress or repeat slows animation
                if (slowAnimationTime > MAX_ANIMATION_TIME) {
                  slowAnimationTime = MAX_ANIMATION_TIME;
                  Blink();                                    //warn user its as slow as it goes
                }
              }
            break;
          //------------Rainbow------------------------------------------------------
          case 0x28d7:
            if (currentState != Rainbow) {                    //if entering rainbow from some other mode
              currentState = Rainbow;
              slowAnimationTime = 0;                          //start with blue
              saturation = 1.0;                               //fully saturated
              hue = 0.;
              brightness = 1.0;
            }
            else                                              //start fastest animation
              if (slowAnimationTime == 0) {
                slowAnimationTime = MIN_ANIMATION_TIME;
                slowAnimationTimer = millis();
              }
              else {
                slowAnimationTime *= 2;                       //slow it down
                if (slowAnimationTime > MAX_ANIMATION_TIME) {
                  slowAnimationTime = MAX_ANIMATION_TIME;
                  Blink();                                    //warn user its as slow as it goes
                }
              }
            break;
          //------------Trickle Rainbow/Sparkles--------------------------------------
          case 0xa857:
            currentState = Rainbow;
            saturation = 1.0;
            brightness = 0.25;
            slowAnimationTime = MIN_ANIMATION_TIME + MAX_ANIMATION_TIME / 2;    // mid speed
            sparkle = true;
            break;
          //-------------Random Colors------------------------------------------------
          case 0x6897:
            if (currentState != RandomColors) {               // if coming from some other mode
              currentState = RandomColors;
              slowAnimationTime = 0;                          // start full brightness and sat
              saturation = 1.0;
              brightness = 1.0;
            }
            else                                              // start out fast
              if (slowAnimationTime == 0) {
                slowAnimationTime = MIN_ANIMATION_TIME * 8;
                slowAnimationTimer = millis();
              }
              else {
                slowAnimationTime *= 2;
                if (slowAnimationTime > MAX_ANIMATION_TIME) {
                  slowAnimationTime = MAX_ANIMATION_TIME;
                  Blink();                                    //warn user its slow as it goes
                }
              }
            break;
          //-------------Sparkle ON/Rate Increase----------------------------------------------
          case 0x18e7:
            if (sparkle == false) {
              sparkle = true;
              sparklesPerMinute = MAX_SPARKLE_RATE;
            }
            else {
              sparklesPerMinute /= 2;
              if (sparklesPerMinute < MIN_SPARKLE_RATE) {
                sparklesPerMinute = MIN_SPARKLE_RATE;
              }
            }
            break;
          //-------------Sparkle OFF-----------------------------------------------------------
          case 0x9867:                                           //Sparkle OFF
            sparkle = false;
            break;
          //---------------Next Hue------------------------------------------------------------
          case 0x609f:
            // Hue changes are most easily perceived around yellow.  Third the step
            // size between red and green as an alternative to shifting the ramps
            if (hue > 0.33 && hue < 0.66) {
              hue = MakeHueValid(hue + 1. / HUE_STEPS / 3.0);
            }
            else {
              hue = MakeHueValid(hue + 1. / HUE_STEPS);
            }
            CreateSolidColor(hue, saturation, brightness);
            if (currentState == QuietColor) UpdateQuietColor();
            break;
          //--------------Previous hue---------------------------------------------------------
          case 0x20df:
            // Hue changes are most easily perceived around yellow.  Third the step
            // size between red and green as an alternative to shifting the ramps
            if (hue > 0.33 && hue < 0.66) {
              hue = MakeHueValid(hue - 1. / HUE_STEPS / 3.0);
            }
            else {
              hue = MakeHueValid(hue - 1. / HUE_STEPS);
            }
            CreateSolidColor(hue, saturation, brightness);
            if (currentState == QuietColor) UpdateQuietColor();
            break;
          //-------------Desaturate (towards white)--------------------------------------------
          case 0xa05f:
            saturation *= 0.98;
            if (saturation < SATURATION_LIMIT) {      // a practical limit
              saturation = 0.;
              Blink();
            }
            if (currentState == QuietColor) UpdateQuietColor();
            break;
          //------------Resaturate (deepen color)----------------------------------------------
          case 0xb04f:
            if (saturation == 0) {
              saturation = SATURATION_LIMIT;
            }
            else {
              if (saturation / 0.98 - saturation < BRIGHT_LSB) {
                saturation += BRIGHT_LSB;
              }
              else {
                saturation /= 0.98;
              }
            }
            if (saturation > 1.0 - BRIGHT_LSB) {
              saturation = 1.0;
              Blink();
            }
            break;
          //--------------Brighten-----------------------------------------------------------
          case 0x708f:
            if (brightness == 0) {
              brightness = BRIGHT_LSB;
            }
            else {
              if (brightness / 0.92 - brightness < BRIGHT_LSB) {
                brightness += BRIGHT_LSB;
              }
              else {
                brightness /= 0.92;
              }
            }
            if (brightness > 1.0 - BRIGHT_LSB) {
              brightness = 1.0;
              Blink();
            }
            if (currentState == QuietColor) UpdateQuietColor();
            break;
          //------------Dim--------------------------------------------------------------------
          case 0x30cf:
            if (brightness * (1.0 - 0.9) > BRIGHT_LSB) {
              brightness *= 0.9;
            }
            else {
              if (brightness - BRIGHT_LSB > 0.0) {
                brightness -= BRIGHT_LSB;
              }
              else brightness = 0.;
            }
            if (currentState == QuietColor) UpdateQuietColor();
            break;
          //-----------------------------------------------------------------------------------
          default: {
            }
        }   //end of switch handlers

        // Refresh display in the current mode to execute a remote command including
        // mode changes and hue nudges

        if (currentState == SolidColor || currentState == QuietColor) {
          CreateSolidColor(hue, saturation, brightness);
          ShowBuffer();
        }
        if (currentState == RandomColors) {
          CreateRandom(saturation, brightness);
          ShowBuffer();
        }
        if (currentState == Rainbow) {
          CreateRainbow(hue, saturation, brightness);
          ShowBuffer();
        }
      }
    }  //end of decode validity test
  }     //end of remote command handler

  // If ON, continue color organ or running any fixed animation

  if (onOff == true) {
    if (currentState == ColorOrgan) {
      ShowColorOrgan();                                     //and return after one update
    }

    // Fast animation of sparkles

    if (currentState != ColorOrgan) {
      if (millis() - animationTimer > FAST_ANIMATION_TIME) {
        animationTimer = millis();
        if (sparkle) {
          Sparkle();
        }
      }

      // Slow animations of hue

      if (slowAnimationTime > 0 && millis() - slowAnimationTimer > slowAnimationTime) { //do slow animation
        slowAnimationTimer = millis();
        if (currentState == SolidColor) {
          if (hue > 0.33 && hue < 0.66) {
            hue = MakeHueValid(hue + 1. / HUE_STEPS / 3.0);
          }
          else {
            hue = MakeHueValid(hue + 1. / HUE_STEPS);
          }
          CreateSolidColor(hue, saturation, brightness);
          ShowBuffer();
        }
        if (currentState == Rainbow) {
          hue = MakeHueValid(hue + 1. / HUE_STEPS);
          CreateRainbow(hue, saturation, brightness);
          ShowBuffer();
        }
        if (currentState == RandomColors) {
          CreateRandom(saturation, brightness);
          ShowBuffer();
        }
      }
    } //end of fixed effects
  } //end of ON test
}   //end of loop()

// -------------------Fixed (non-Color Organ) effects-----------------

// Load a rainbow to buffer. Empirically, hue transitions more rapidly around
// green (hue=0.5,) producing only one yellow LED hue.  Reducing the step size
// by a ratio, experimentally to 2 or 3, results in an equal number of primary
// and secondary colors in the rainbow. The statements below implement the
// desired ratio so as to achieve one complete hue cycle over TOTAL_LEDS

void CreateRainbow(float hue, float saturation, float brightness) {
  float red;              //local variables for Hue() to modify
  float green;
  float blue;
  int steps = TOTAL_LEDS;
  int fastSteps = steps * 0.33;
  int slowSteps = steps - fastSteps;
  int ratio = 3;                            //ratio of large to small step
  float stepSize = 1.0 / float(float(slowSteps / ratio) + float(fastSteps));
  for (int i = 0; i < TOTAL_LEDS; i++) {
    if (hue > 0.333 && hue < 0.666) {
      hue = MakeHueValid(hue + stepSize / ratio);
    }
    else {
      hue = MakeHueValid(hue + stepSize);
    }
    Hue(hue, red, green, blue);
    red = (1 - saturation) * MAX_ANALOG + saturation * red;
    green = (1 - saturation) * MAX_ANALOG + saturation * green;
    blue = (1 - saturation) * MAX_ANALOG + saturation * blue;
    red = makeValid(red * brightness);        // now set the brightness
    green = makeValid(green * brightness);
    blue = makeValid(blue * brightness);
    buffer[i].r = red;
    buffer[i].g = green;
    buffer[i].b = blue;
  }
}  //end of Rainbow

// Load a Solid Color into buffer[]

void CreateSolidColor(float hue, float saturation, float brightness) {
  float red;                            // local floats to act upon
  float green;
  float blue;
  Hue(hue, red, green, blue);           // begin with a pure hue
  red = (1 - saturation) * MAX_ANALOG + saturation * red;   //set saturation
  green = (1 - saturation) * MAX_ANALOG + saturation * green;
  blue = (1 - saturation) * MAX_ANALOG + saturation * blue;
  red = makeValid(red * brightness);        // now set the brightness
  green = makeValid(green * brightness);
  blue = makeValid(blue * brightness);
  for (int i = 0; i < TOTAL_LEDS; i++) {
    buffer[i].r = red;
    buffer[i].g = green;
    buffer[i].b = blue;
  }
}   //end of CreateSolidColor

// Create Random Colors

void CreateRandom(float saturation, float brightness) {
  float red;              // local floats for Hue() to act upon
  float green;
  float blue;
  for (int i = 0; i < TOTAL_LEDS; i++) {
    Hue(float(random(0, 10000) / 10000.0), red, green, blue);
    red = (1 - saturation) * MAX_ANALOG + saturation * red;   //set saturation
    green = (1 - saturation) * MAX_ANALOG + saturation * green;
    blue = (1 - saturation) * MAX_ANALOG + saturation * blue;
    red = makeValid(red * brightness);        // now set the brightness
    green = makeValid(green * brightness);
    blue = makeValid(blue * brightness);
    buffer[i].r = red;
    buffer[i].g = green;
    buffer[i].b = blue;
  }
}  //end of CreateRandom

// buffer[] is used in fixed animations to keep the current RGB values.  It is in float
// to maximize the resolution of hue saturation, and brightness. The valid range is
// 0.0 to MAX_ANALOG to match the resolution of the color organ here, the buffer[] values
// are scaled to 8 bits for the WS2812s.

// 12-volt WS2811s flicker red at dim whites above 11.7V DC supply.  The flicker is
// progressively worse if r==g==b<23.  Resolves at lower DC voltage, or if b>r=g by
// at least 1.  Ficker is on/off at 1,1,1 vs 1,1,2, or at 1,1,1 below 11.70 volts
//

void ShowBuffer() {
  for (int i = 0; i < TOTAL_LEDS; i++) {
    leds[i].r = byte(round(buffer[i].r / 16.6));
    leds[i].g = byte(round(buffer[i].g / 16.6));
    leds[i].b = byte(round(buffer[i].b / 16.6));
    if (leds[i].r == leds[i].g && leds[i].g == leds[i].b && leds[i].b < 23 && leds[i].b != 0) {
      leds[i].b++;
    }
  }
  FastLED.show();
}   //end of ShowBuffer

// Execution time of FastLED.show() with six 50 LED strings defined is ≃11 mS/string
// Since the time to accss a given LED in the sequence is the same in both calls to
// FastLED.show, 11mS in WHITE provides a strobe-like flash.

void Sparkle() {
  int i;
  float red;
  float green;
  float blue;
  unsigned long showTime;
  if (random(0, 6000) < sparklesPerMinute) {    //10mS fast animations rate per minute
    i = random(0, TOTAL_LEDS);
    red = leds[i].r;
    green = leds[i].g;
    blue = leds[i].b;
    leds[i] = CRGB::White;
    showTime = micros();
    FastLED.show();
    leds[i].r = red;
    leds[i].g = green;
    leds[i].b = blue;
    FastLED.show();
  }
}

// Make Hue Valid if nudged outside the 0.0 < hue < 1.0 range

float MakeHueValid(float value) {
  if (value > 1.0) {
    value = 0.0;
  }
  if (value < 0) {
    value = 1;
  }
  return value;
}   //end of makeHueValid

// Hues are generated by three ramp functions each one third of the way
// through the hue range. 0=blue; 0.166=magenta; 0.33=red; 0.5=yellow; 0.66=green
// 0.833=cyan; 1.0=blue

void Hue(float hue, float & red, float & green, float & blue) {
  float slope;          //rate of change over phase
  if (hue < 0.0) {    //validate phase
    hue = 0.0;
  }
  if (hue > 1.0) {
    hue = 1.0;
  }

  slope = MAX_ANALOG / 0.33;    //slope around half a phase

  if (hue >= 0.0 && hue <= 0.33 ) {
    red = slope * (hue - 0.0);
  }
  else if (hue > 0.33 &&  hue <= 0.66) {
    red = MAX_ANALOG - slope * (hue - 0.33);
  }
  else {
    red = 0.0;
  }
  if (hue >= 0.33 && hue <= 0.66 ) {
    green = slope * (hue - 0.33);
  }
  else if (hue > 0.66 &&  hue <= 1) {
    green = MAX_ANALOG - slope * (hue - 0.66);
  }
  else {
    green = 0.0;
  }
  if (hue >=  0.66 && hue <= 1.0) {
    blue = slope * (hue - 0.66);
  }
  else if (hue >= 0.0 && hue <= 0.33) {
    blue = MAX_ANALOG - slope * (hue - 0.0);
  }
  else {
    blue = 0.0;
  }
  red = makeValid(red);
  green = makeValid(green);
  blue = makeValid(blue);
}

// Brightnesses and RGB values are stored as floats with 10-bit resolution.
// This allows the use of multiply and divides for scaling, desaturating, hue
// generation, etc without worry about integer overflow or underflow.  Since
// these floats are ultimately converted to byte for loading the LED strings,
// boundary conditions are enforced here

float makeValid(float value) {
  if (value > MAX_ANALOG) {
    value = MAX_ANALOG;
  }
  if (value < 0) {
    value = 0;
  }
  return value;
}   //end of makeValid

// Blink to acknowledge a remote command to a user but warn that the requested change is
// beyond a limit

void Blink() {
  for (int i = 0; i < TOTAL_LEDS; i++) {      // turn em off
    leds[i].r = 0;
    leds[i].g = 0;
    leds[i].b = 0;
  }
  FastLED.show();
  delay(100);                                 // leave off 100mS
  ShowBuffer();                               // restore em
}

// Called when hugh, saturation, or brightness is nudged while the quietColor
// is being displayed. Displayed HSB values are copied to the global HSB quiet values
// which are converted to RGB thresholds for the color organ when below dynamic range
// or no audio is present

void UpdateQuietColor() {
  quietHue = hue;
  quietSaturation = saturation;
  quietBrightness = brightness;
  Hue(quietHue, quietRed, quietGreen, quietBlue);              // begin with the displayed hue
  quietRed = (1 - saturation) * MAX_ANALOG + saturation * quietRed;   //apply saturation
  quietGreen = (1 - saturation) * MAX_ANALOG + saturation * quietGreen;
  quietBlue = (1 - saturation) * MAX_ANALOG + saturation * quietBlue;
  quietRed = makeValid(quietRed * brightness);            //apply brightness
  quietGreen = makeValid(quietGreen * brightness);
  quietBlue = makeValid(quietBlue * brightness);
}

// Color organ

float redBright;       //red brightness  0 = OFF 4095 = FULL ON
float greenBright;     //green brightness  0 = OFF 4095 = FULL ON
float blueBright;      //blue brightness  0 = OFF 4095 = FULL ON
float red;             //red value to LED string
float green;           //green value to LED string
float blue;            //blue value to LED string
float redHP;           //Red channel filter
float redHP2;          //Red second stage
float redHP3;          //Red third stage
float redOut;          //Red filter output
float greenLP;         //Green lowpass filter
float greenLP2;        //Green second stage
float greenLP3;        //Green third stage
float greenHPout;      //Green interstage
float greenHP;         //Green highpass filter
float greenHP2;        //Green second stage
float greenHP3;        //Green third stage
float greenOut;        //Green filter output from lowpass-highpass cascade
float blueLP;          //blue channel filter operates at sample rate
float blueLP2;         //blue second stage
float blueLP3;         //blue third stage
float blueHP;          //blue channel filter
float blueOut;         //blue filter output from lowpass-highpass cascade
int   toggle;          //for telemetry of the audio sampling rate
float audioSample;     //sample & hold for ADC sample = (signed) number with 10-bit resolution
float peakBlue;        //blue channel full-wave rectified/smoothed peak value
float peakRed;         //red channel full-wave rectified/smoothed peak value
float peakGreen;       //green channel full-wave rectified/smoothed peak value
float sampleMax;       //sample and hold for maximum instantaneous sample value
float inputGain;       //initial gain is boost, can be reduced in samplint
float brightnessAlpha; //smoothing coefficient for LED off/on step
float rawSample;       //raw ADC sample reading

// Sammple the input audio for a block of SAMPLE_SIZE while applying the channel
// filters, then compute the brightnesses and load the LED strings.  Return to loop()
// so as to test for remote keypresses; if there have been none, or if the user hasn't
// selected one of the other modes, loop() will execute this continuously

void ShowColorOrgan() {
  sampleCount = 0;
  digitalWrite(13, HIGH);                           //telemeter start of sampling and filtering
  while (sampleCount++ < SAMPLE_SIZE) {            //sample, filter until time to display

    // Audio Sampling and Filtering
    // This loop implements as many first order filters as can be accomodated in 100uS so
    // as to sample at a 10KHz sample rate and support a 5KHz Nyquist limit.  An single
    // pole 7KHz R-C lowpass is used to prevent aliasing of high frequency program content,
    // e.g. producing a blue component to what should display pure red.
    //
    // Component tolerances in the external bias network are compensated to a half-bit
    // by integrating the raw read then subtracting it from the raw read to obtain a
    // signed 12-bit sample value.
    //
    // Sampling and filtering are performed in blocks of SAMPLE_SIZE interspersed with
    // brightness calculations and LED updates. Each block is subjected to a pre-
    // computed Hann(ing) window to minimize brightness flicker due to interactions between
    // a single input frequency and the update rate

    rawSample = float(analogRead(AUDIO_PIN));     //float unsigned read
    inputBias = rawSample * biasLpAlpha +
                inputBias * (1 - biasLpAlpha);    //measure input bias
    rawSample -= inputBias;                       //remove bias to get signed result
    if (abs(rawSample) > sampleMax) {             //measure raw input's peak value
      sampleMax = abs(rawSample);                 //keep peak value
      if (inputGain * sampleMax > MAX_ANALOG / 2.) {  //if current AGC gain would over/underflow
        inputGain = MAX_ANALOG / 2. / sampleMax;      //set a new AGC gain
      }
    }
    audioSample = rawSample * inputGain;          //and apply new or existing AGC gain
    audioSample *= hanningWindow[sampleCount - 1];

    // Each LP and HP prototype is an exponential moving average equivalent to a first
    // filter function.  Each HPF section is the input value minus a LPF prototype.
    // All these variables are floats having a maximum 10-bit value of ±2048 signed.
    // Since the bias value has been removed to produce a signed result, the result of
    // each filter falls towards zero as frequency enters the stopband

    redHP = redHPalpha * audioSample + (1 - redHPalpha) * redHP;    //red LP prototype
    redOut = audioSample - redHP;                                   //transform to HP
    redHP2 = redHPalpha * redOut + (1 - redHPalpha) * redHP2;       //red stage 2
    redOut = redOut - redHP2;
    redHP3 = redHPalpha * redOut + (1 - redHPalpha) * redHP3;       //red stage 3
    redOut = redOut - redHP3;
    greenLP = greenLPalpha * audioSample + (1 - greenLPalpha) * greenLP;  //green LP
    greenLP2 = greenLPalpha * greenLP + (1 - greenLPalpha) * greenLP2;
    greenLP3 = greenLPalpha * greenLP2 + (1 - greenLPalpha) * greenLP3;
    greenHP = greenHPalpha * greenLP3 + (1 - greenHPalpha) * greenHP;    // green HP
    greenHPout = greenLP3 - greenHP;
    greenHP2 = greenHPalpha * greenHPout + (1 - greenHPalpha) * greenHP2;
    greenHPout = greenHPout - greenHP2;
    greenHP3 = greenHPalpha * greenHPout + (1 - greenHPalpha) * greenHP3;
    greenOut = greenHPout - greenHP3;
    blueLP = blueLPalpha * audioSample + ( 1 - blueLPalpha) * blueLP; //blue LP
    blueLP2 = blueLPalpha * blueLP + ( 1 - blueLPalpha) * blueLP2;
    blueLP3 = blueLPalpha * blueLP2 + ( 1 - blueLPalpha) * blueLP3;   //blue HP
    blueHP = blueHPalpha * blueLP3 + (1 - blueHPalpha) * blueHP;
    blueOut = blueLP3 - blueHP;

    // Full-wave rectify the filter outputs to positive floats.  Peak detect and time smooth
    // the result to eliminate DC ripple and keep the LEDs from going OFF too abruptly.
    // The smoothing coefficient (alpha) is a determined from a time constant specified in
    // milliseconds

    brightnessAlpha = 1 - pow(2.71828, - UPDATE_PERIOD * 0.001 / BRIGHTNESS_TAU);
    if (abs(redOut) > peakRed) {
      peakRed = abs(redOut);
    } else {
      peakRed *= (1 - brightnessAlpha);
    }
    if (abs(greenOut) > peakGreen) {
      peakGreen = abs(greenOut);
    }
    else {
      peakGreen *= (1 - brightnessAlpha);
    }
    if (abs(blueOut) > peakBlue) {
      peakBlue = abs(blueOut);
    }
    else {
      peakBlue *= (1 - brightnessAlpha);
    }

    // The DACs can be used diagnostically to remove signed data from the sampling loop
    // without slowing the sampling rate by more than 10uS each.
    // The DACs output from (1/6) x VADVREF to (5/6) x VADVREF, or 0.55V - 2.75V  over the
    // range from zero to 0x0fff.  Any variable can be thus telemetered.  Insert either or
    // both lines below where required, changing the variable to be telemetered
    //
    // analogWrite(DAC0, audioSample + 2047);
    // analogWrite(DAC1, redHP + 2047);

    // Telemeter sample/filter rate.  The desired sampling rate is 10KHz in order to have
    // a 5KHz Nyquist bandwidth.  The time to sample the audio input and update the channel
    // filters is just under 100uS on DUE and the number of first order filters was maximized
    // within that rate. This is telemetered on Pin 12 to permit measureent of the half-period
    // or pulse width with a scope or counter.  The corner frequencies of the filters and
    // time constants of the smoothing are calculated based on a 100uS sample time, so the
    // sampling/filter loop should be padded to 100uS to maintain that. Compiler optimization
    // caused loop time to vary ±10uS from build to build, so for consistent results, this
    // should be checked and the SAMPLING_LOOP_PAD adjusted as a last step after each build.
    // It may be helpful to force a compile before upload with the 1.8.3. IDE

    toggle = !toggle;
    digitalWrite(12, toggle);
    delayMicroseconds(SAMPLING_LOOP_PAD);            //Pad sampling and filtering to 100uS

  } //end of audio sampling and filtering

  // Display Functions are performed after the SAMPLE_SIZE has been sampled.  The
  // SAMPLE_SIZE is chosen to this occurs frequently enough to track the music
  // and avoid flicker

  if (TELEMETRY) {
    digitalWrite(13, LOW); //telemeter either sampling (HIGH) or LED update (LOW)
  }

  // Maintain AGC
  // If sampling has reduced the AGC gain, slowly increase it towards the AGC Boost with
  // the time constant AGC_TAU.  This is implmented as an exponential moving average with
  // an alpha calculated in terms of display update rate

  if (inputGain < AGCboost) {
    inputGain = AGCboost * AGCalpha + (1 - AGCalpha) * inputGain;
  }

  if (TELEMETRY) {
    Serial.print(inputGain);
    Serial.print("\t");
    Serial.print(sampleMax * inputGain);
    Serial.print("\t");
  }

  sampleMax = 0;   //next sampling interval starts over

  // Apply channel gains to derive an unsigned 10-bit brightness value between 0 and 4095.

  redBright = RED_GAIN * peakRed;
  greenBright = GREEN_GAIN * peakGreen;
  blueBright = BLUE_GAIN * peakBlue;

  // Clamp any saturation, intentional or otherwise, to avoid overflows in the
  // integer conversions downstream for the LED strings

  redBright = makeValid(redBright);
  greenBright = makeValid(greenBright);
  blueBright = makeValid(blueBright);

  if (TELEMETRY) {
    Serial.print(redBright);
    Serial.print("\t");
    Serial.print(greenBright);
    Serial.print("\t");
    Serial.print(blueBright);
    Serial.print("\t");
  }

  // WS-2811 based LED strings operate on 8-bit brightness per color
  // Scale the brightness range between MAX_ANALOG and DYNAMIC_RANGE to 255 and 0.
  // Results <0 are below the specified dynamic range and are defaulted to the
  // quiet color and also scaled to 8 bits.

  red = 255. - 255. * 20 * log10(double(redBright / MAX_ANALOG)) / (-DYNAMIC_RANGE);
  green = 255. - 255. * 20 * log10(double(greenBright / MAX_ANALOG)) / (-DYNAMIC_RANGE);
  blue = 255. - 255. * 20 * log10(double(blueBright / MAX_ANALOG)) / (-DYNAMIC_RANGE);

  if (red < 0.) {
    red = byte(round(quietRed / 16.6));
  }
  if (green < 0.) {
    green = byte(round(quietGreen / 16.6));
  }
  if (blue < 0.) {
    blue = byte(round(quietBlue / 16.6));
  } if (red == green && green == blue && blue < 23 && blue != 0) {
    blue++;
  }

  if (TELEMETRY) {
    Serial.print(red);
    Serial.print("\t");
    Serial.print(green);
    Serial.print("\t");
    Serial.println(blue);
  }

  // Convert to byte and send one frame to all string(s) simultaneously

  for (int i = 0; i < TOTAL_LEDS; i++) {
    leds[i].r = (byte)red;
    leds[i].g = (byte)green;
    leds[i].b = (byte)blue;
  }
  FastLED.show();

}     //end of ShowColorOrgan()


