#ifdef PLATFORMIO
  #include <Arduino.h>
#endif

// 6 digit display:
#include <TM1637TinyDisplay6.h>

// eeprom
#include <eeprom.h>


// 2023-11-18: working!
// rotary encoder on a ATTiny84 dev board and 6 digit display,
// to measure distances using a screw thread.
// This is to be used on the box joint jig.

// progress:
//    - reset 0 position: hold button 1 for LONGPRESS ms (5 seconds)
//    - restore the position if accidentally reset by holding for another LONGPRESS
//    - XXX: Battery power: go to deep sleep when not used for a while
//           -> no good, because the optical sensors & screen are still active; provide pwr switch instead
//    - setup params for finger jointing: shortpress of button goes to setup mode.
//      Then the values can be input by moving the jig using the hand wheel.
//      (The correct position of the carriage is maintained during the setup)
//      - select the finger size (e.g kerf/2.5, 5, 6.25 etc)
//      - set saw blade kerf (default is 2.5mm)
//      - set "tightness" by adjusting slot width
//    - timeout so that it reverts to distance display from setup after some time  (20240112)
//    - Save settings to EEPROM (done - but see bugs 20240212)
//    - implement double click for setup; use single click for changing free mode/finger A/finger B
//      (done 20240219)
//    - rejigged button code using a state machine 20240501
//    - Added 2 LEDs to show when a cut should be made (and associated calcs)
//    - flashing LEDs when approaching the start/end of a finger cut
//    - added FW version - display with click+press


// Bugs:
// * There is something screwy happening with the setup occassionally, seems related to min/max values
//   I think it was provoked by going to setup but not changing any values. not reproducable - Not fixed?
// * (fixed) There is a problem with the finger size setting after the kerf has been changed.
//   The minimim finger size is the next highest multiple of increment - setting the minimum=kerf
//   causes the values to become "odd"
// * (fixed) idle timeout only works if no interaction


// TODO:


// ATtiny84 dev board pins
//    PA0/PA1 - used: clock and data for TM1637 6 digit LED display
//    PA2/PA3 - free: white connector pins 3 & 5
//    PA4/5/6 - free: on 6 pin ICSP header (used for programming)
//    PA7 (on board LED) - also white connector pin 6
//    PB0/1 - used: clock crystal - not available for I/O
//    PB2 - used for button : INT0 - on white connector pin 4
//    PB3 - Reset - on ICSP header - not really useable

/* VERSION */
#define FW_VERSION 240523

/* 6 DIGIT DISPLAY */
// For ATTiny84 dev board, use PA0 as data and PA1 as clock for the display
#define DISP_CLK PIN_PA1
#define DISP_DIO PIN_PA0
#define DISP_BRIGHT 3

TM1637TinyDisplay6 display(DISP_CLK, DISP_DIO);

void initDisplay();
void displayPosition();
void displayPositionForce();


/* BUTTON */
#define BTN_PIN PIN_PB2      // ATTiny84 External interrupt INT0; physical pin 5, port B bit 2
uint8_t btnPort = NOT_A_PORT;

#define DEBOUNCE 50          // msec for debounce
#define LONGPRESS 3000       // msec for long press detection
#define DOUBLECLICK 300      // timing window for a double click
#define BTN_PRESSED LOW
#define BTN_RELEASED HIGH

volatile bool btnState;
// store the button click flags to a single byte
volatile uint8_t btnFlags;
#define SINGLE_CLICK_FLAG     B00000001
#define DOUBLE_CLICK_FLAG     B00000010
#define LONG_PRESS_FLAG       B00000100
#define CLICK_PLUS_PRESS_FLAG B00001000
#define BTN_OP(f)             btnFlags & f

// define FiniteStateMachine states for the button
enum stateMachine_t : uint8_t {
  BTN_IDLE = 0,
  BTN_PRESSEDONCE = 1,       // button is down first time
  BTN_RELEASEDONCE = 2,      // button is up first time
  BTN_PRESSEDTWICE = 3,      // button is down second time
  BTN_RELEASEIGNORE = 4      // waiting for release, will be ignored
};

stateMachine_t stateMC = BTN_IDLE;

// events that the state machine processes to transition to a new state
enum smEvent : uint8_t {
  BTN_DOWN = 0,
  BTN_UP = 1,
  BTN_TIMER_EXP = 3
};

void initButton();
uint8_t readButtonState();
bool WasPressed();
bool IsPressed();

void processEvent(smEvent ev);

void handleBtnSingleClick();
void handleBtnDoubleClick();
void handleBtnLongPress();
void handleBtnClickPlusPress();

/* LEDS */
#define _GREEN_LED_    PIN_PA5     // ATTiny84 physical pin 8; port A bit 5 (on the ISCP connector)
#define _RED_LED_      PIN_PA6     // ATTiny84 physical pin 7; port A bit 6 (on the ISCP connector)
#define _ONBOARD_LED_  PIN_PA7     // ATTiny84 physical pin 6; port A bit 7 (on board)

#define LED_ON(x)      PORTA|=(digitalPinToBitMask(x))           // set PAx HIGH
#define LED_OFF(x)     PORTA&=(~(digitalPinToBitMask(x)))        // set PAx LOW
#define LED_TOGGLE(x)  PINA=(digitalPinToBitMask(x))             // PAx Toggle

// define the number of encoder ticks from a boundary when the leds start flashing
#define CLOSE 40
#define VERY_CLOSE 20

void initLEDs();


unsigned long ledUpdateTime;

void updateCutIndicator();
void calculateTicks();

/* TIMER */
void initTimer();
void startTimer(uint16_t msec);
void stopTimer();
void resetTimer();
volatile bool timerRunning;
volatile uint8_t timerIntCount;
volatile bool idleTimerExpired;


/* POSITION ENCODER */
// use PA2 and PA3 as the inputs, and use pin change interrupt to detect movement
#define ENC_A PIN_PA2
#define ENC_B PIN_PA3
#define ENC_BITMASK B00001100;

volatile uint8_t encState;     // temp byte for grabbing the port state
volatile uint8_t encStates;    // current and previous port state as 0B0000CCPP
volatile long encPos;          // absolute count of the encoder's position since init/rezero
long saveEncPos;               // save the encoder position so it can be resored if the gauge is accidentally re-zeroed
long prevEncPos;               // save the encoder position that was last updated.

// increments to the encoder position using encStates as index
const int8_t increments[16] = {0,1,-1,0,-1,0,0,1,1,0,0,-1,0,-1,1,0};

// The thread pitch in mm
#define PITCH 1.25
// The number of "pulses" that the encoder produces per revolution
// The encoder disk has 25 holes
#define ENCODER_PULSES_PER_REV 100
// The number of increments per revolution; used to change a value during setup
#define INCS_PER_REV 10

// convert encoder pulse count to/from mm
double mmPerTick;
double ticksPerMm;

uint16_t kerfTicks;    // the number of encoder ticks corresponding to the blade kerf
uint16_t fingerTicks;  // the number of encoder ticks corresponding to the finger width

void initEncoder();


/* FINGER SETUP AND EEPROM SETTINGS */

// signature 03BC - greek small letter mu
#define SIG_BYTE0 0x03
#define SIG_BYTE1 0xbc
// NOTE: The saw kerf, finger width and increments should all be integral multiples
// of mmPerTick (ie: should correspond exactly to an integral number of ticks)

// the default finger width (must not be less than the saw kerf!)
#define DEF_FINGER 5.0
#define FINGER_INC 0.25
#define MAX_FINGER 25.0
// DO NOT CHANGE!
#define FINGER_DECIMALS 2

// the default saw blade kerf (Also used as the default finger width)
#define DEF_KERF 2.5
#define KERF_INC 0.1
#define MIN_KERF 2.0
#define MAX_KERF 25.0
// DO NOT CHANGE!
#define KERF_DECIMALS 1

#define TIGHTNESS_INC 1
#define MIN_TIGHTNESS -10.0
#define MAX_TIGHTNESS  10.0
// DO NOT CHANGE!
#define TIGHTNESS_DECIMALS 0

#define IDLETIMEOUT 10000  // msec for dropping out of setup after idle
uint8_t setupState;

void initSettings();
void doFingerSetup();    // setup the finger jig parameters: finger width, kerf, tightness
float getEncoderInput(float currentValue, float increments, float min, float max, uint8_t decimals);
float roundUp(float kerf, float increment);  // round kerf up, to the next highest multiple of increment

struct eeprom_image_t {
  // we can check the signature to see if our settings are in eeprom
  unsigned char sig[2];    // by default: use the signature 03BC - greek small letter mu

  float fingerWidth;       // in mm
  float kerf;              // in mm
  int8_t tightness;        // in encoder ticks, default range is -10 to +10
} eeprom_image;


/* MISC */

#define FLASH_DELAY 200
#define FAST_FLASH_DELAY 100

void doRezero();

// The operating modes are 0 = free mode (not finger jointing), 1=finger A, 2=finger B
enum opMode_t : uint8_t { 
   normal = 0,
   fingerA = 1,
   fingerB = 2
} opMode;

void switchOpMode();

void showFWVersion();

void setup() {

  // initialize hardware devices etc
  initEncoder();

  initDisplay();

  initLEDs();

  initButton();

  // Timer is used to detect long press and double click of button and
  // to time-out from the setup routine
  initTimer();

  initSettings();

  mmPerTick = (double)PITCH / (double)ENCODER_PULSES_PER_REV;    // 1.25 / 100 or 0.0125
  ticksPerMm = (double)ENCODER_PULSES_PER_REV / (double)PITCH;   // 100 / 1.25 or 80

  // convert the kerf and finger width to encoder ticks
  calculateTicks ();

  setupState = 0;   // see doFingerSetup(); state 0 is "not in setup"

  opMode = normal;   // free mode - not finger joints

}


void loop() {

  // handle any button presses that might have occurred
  if (btnFlags != 0) {
    if (BTN_OP(SINGLE_CLICK_FLAG)) {
      handleBtnSingleClick();              // change op mode
    } else if (BTN_OP(DOUBLE_CLICK_FLAG)) {
      handleBtnDoubleClick();              // setup finger params
    } else if (BTN_OP(LONG_PRESS_FLAG)) {
      handleBtnLongPress();                // reset zero
    } else if (BTN_OP(CLICK_PLUS_PRESS_FLAG)) {
      handleBtnClickPlusPress();           // show version info
    }

    // reset all the button click flags
    btnFlags = 0;
    
    displayPositionForce();

  }else {
    // display the carriage position (if it's changed)
    displayPosition();
  }

  // update cut indicator LEDS for finger modes
  updateCutIndicator();
}


/* 6 DIGIT DISPLAY */
void initDisplay() {
  display.begin();
  display.setBrightness(DISP_BRIGHT);
  // force the display to update
  displayPositionForce();
  ledUpdateTime = 0;
}


// The thread used on the jig is 8mm metric, which has 1.25mm pitch.

// The encoder wheel has 25 holes, which will give 100 "transitions" or encoder ticks per revolution.
// This gives a resolution of 0.0125 mm (or about 0.5 thou)

// The jig can move ~400mm, or ~300 rotations
// So the max value we can get from getPosition is 30000 - long int for encPos is more than big enough.

void displayPosition() {
  // check if we moved from the previous time we updated the display
  if (encPos != prevEncPos) {
    displayPositionForce();
  }  // else do nothing - we are already displaying the correct reading

}


// unconditionally update the display
void displayPositionForce() {
  // We want to show thousandths of a mm; Fortunately the display library does the rounding for us so just
  // convert the encPos to mm and tell the display we want 3 decimals:
  display.showNumber (((double)encPos * mmPerTick ), 3);
  // save the last updated position to avoid unnecessary updates
  prevEncPos = encPos;

}


/* BUTTON */
void initButton() {

  // Check if the pin is actually a pin on a port
  btnPort = digitalPinToPort(BTN_PIN);

  if (btnPort != NOT_A_PORT) {
    pinMode(BTN_PIN, INPUT_PULLUP);

    if (IsPressed()) {
      // button is pressed during initialization: ignore the next release
      stateMC = BTN_RELEASEIGNORE;
      //dbg_info = 2;
    } else {
      stateMC = BTN_IDLE;
      //dbg_info = 3;
    }

    cli();

    // set up the external interrupt for button

    // INT0 Int sense control settings
    //             ISC01  ISC00
    // logic low -   0      0
    // change    -   0      1
    // falling   -   1      0
    // rising    -   1      1

    MCUCR |= (1 << ISC00);  // use "change" - we want to detect press AND release
    // Enable external interrupts for INT0
    GIMSK |= (1 << INT0);

    // enable global interrupts
    sei();

    // reset all the button click flags
    btnFlags = 0;

  }
}


uint8_t readButtonState() {

  if (btnPort == NOT_A_PORT) {
    // either the pin is not part of a port (?!?) or the initButton() method hasn't been called.
    return BTN_RELEASED;
  } else {
    return (*portInputRegister(btnPort) & digitalPinToBitMask(BTN_PIN));
  }
}


bool WasPressed() {
  return (btnState == BTN_PRESSED);
}


bool IsPressed() {
  return (readButtonState() == BTN_PRESSED);
}


// processEvent() is called from within the button and timer interrupts, so make it snappy!
void processEvent(smEvent ev) {

  // debug: show current state and the event
  //display.showString_P(PSTR("S"), 1, 0);
  //display.showNumber(stateMC, 0, 1, 1);
  //display.showString_P(PSTR("E"), 1, 2);
  //display.showNumber(ev, 0, 1, 3);


  switch (stateMC) {

    case BTN_IDLE:
      // when IDLE: all we can do is respond to a button down
      // If we are in finger setup mode: move to BTN_RELEASEIGNORE state and signal the single click
      // In normal operation: move to BTN_PRESSEDONCE
      // anything else requires no action
      if (ev == BTN_DOWN) {
        if (setupState != 0 ) {
          stateMC=BTN_RELEASEIGNORE;
          btnFlags |= SINGLE_CLICK_FLAG;
        } else {
          stateMC=BTN_PRESSEDONCE;
          // start the timer to check for a long press
          startTimer(LONGPRESS);
        }
      }
      break;

    case BTN_PRESSEDONCE:
      // after being pressed for the first time, we can action a button release or a timer expired (long press)
      if (ev == BTN_UP) {
        // this is a single click, but we need to wait and see if this turns into a double click:
        stateMC=BTN_RELEASEDONCE;
        stopTimer();  // stop the long press timer
        startTimer(DOUBLECLICK);
      } else if (ev == BTN_TIMER_EXP) {
        // this is a long press
        stopTimer();  // stop the long press timer (else it will keep firing)
        // we can ignore the next button release
        stateMC=BTN_RELEASEIGNORE;
        // Signal the long press
        btnFlags |= LONG_PRESS_FLAG;
      } 
      break;

    case BTN_RELEASEDONCE:
      // after being released for the first time, we can action a button press or
      // a timer expired (double click timeout, which means it's a single click)
      if (ev == BTN_DOWN) {
        // potentially either a double click or single click followed by a long press
        // next state is PRESSEDTWICE
        stateMC=BTN_PRESSEDTWICE;
        stopTimer();  // stop the double click timer
        // start the timer to check for a long press
        startTimer(LONGPRESS);
      } else if (ev == BTN_TIMER_EXP) {
        // this is a single click (the double click timer expired before another button down)
        // next state is idle
        stateMC=BTN_IDLE;
        stopTimer();  // stop the double click timer (else it will keep firing)
        // Signal the single click
        btnFlags |= SINGLE_CLICK_FLAG;
      }
      break;

    case BTN_PRESSEDTWICE:
      // after being pressed for the second time, we can action a button release (double click)
      // or a timer expired (long press)
      if (ev == BTN_UP) {
        // this is a double click
        stateMC=BTN_IDLE;
        stopTimer();  // stop the long press timer
        // Signal the double click
        btnFlags |= DOUBLE_CLICK_FLAG;
      } else if (ev == BTN_TIMER_EXP) {
        // this is a single click followed by a long press
        stopTimer();  // stop the long press timer (else it will keep firing)
        // we can ignore the next button release
        stateMC=BTN_RELEASEIGNORE;
        // Signal the click+long press
        btnFlags |= CLICK_PLUS_PRESS_FLAG;
      } 
      break;

    case BTN_RELEASEIGNORE:
        // we are here because a long press has previously occurred.
        // When the button is released, move to IDLE
        if (ev == BTN_UP) {
          stateMC=BTN_IDLE;
        }
      break;
  }

  // debug - show state after processing
  //display.showString_P(PSTR("S"), 1, 4);
  //display.showNumber(stateMC, 0, 1, 5);

}


void handleBtnSingleClick() {
  switchOpMode();
  // debug: display.showString_P(PSTR("SC"));
}


void handleBtnDoubleClick() {
  doFingerSetup();
  // debug: display.showString_P(PSTR("DC"));
}


void handleBtnLongPress() {
  doRezero();
  // debug: display.showString_P(PSTR("LP"));
}


void handleBtnClickPlusPress() {
  showFWVersion();
  // debug: display.showString_P(PSTR("SC LP"));
}



// INT0 handler for the button
ISR(INT0_vect) {

  static unsigned long last_interrupt_time = 0;
  unsigned long interrupt_time = millis();

  // Debounce by ignoring anything within DEBOUNCE msec of the last interrupt
  // This didn't work all that well until a cap was added across the switch
  if ((interrupt_time - last_interrupt_time) > DEBOUNCE ) {

    // is the button pressed or released?
    if (!WasPressed() && IsPressed()) {
      // wasn't pressed, but now *is* pressed
      processEvent(BTN_DOWN);
      btnState = BTN_PRESSED;
      LED_ON(_ONBOARD_LED_);

    } else if (WasPressed() && !IsPressed()) {
      // was pressed but now *not* pressed (i.e. released)
      processEvent(BTN_UP);
      btnState = BTN_RELEASED;
      LED_OFF(_ONBOARD_LED_);
      //LED_OFF(_GREEN_LED_);
      //LED_OFF(_RED_LED_);
    }

    last_interrupt_time = interrupt_time;
  }
}


/* LEDS */
void initLEDs() {

  pinMode(_GREEN_LED_, OUTPUT);
  pinMode(_RED_LED_, OUTPUT);
  pinMode(_ONBOARD_LED_, OUTPUT);

  // test LEDS
  for (int i=0; i<2; i++){
    LED_TOGGLE(_GREEN_LED_);
    delay(100);
    LED_TOGGLE(_RED_LED_);
    delay(100);
    LED_TOGGLE(_ONBOARD_LED_);
    delay(100);
  }
}


// Update the display (6 digit display &/or separate LEDs) to indicate when cuts should be made
void updateCutIndicator() {

  if (opMode == normal) {
    // extinguish both the LEDS when not doing finger joint cuts
    LED_OFF(_GREEN_LED_);
    LED_OFF(_RED_LED_);
  } else {

    // FINGER A
    // Cut when 0 <= [position MOD(2Fing)] <= Fing-Kerf
    // FINGER B
    // Cut when Fing <= [position MOD(2Fing)] <= 2*Fing-Kerf

    // take the modulus of the position wrt the finger+void
    long modPos = encPos%(2*fingerTicks);

    // upper and lower bounds
    // absolute lowerBound = 0.0
    // cut upper and lower bounds refer to the "left" of the saw blade
    const uint16_t cutLowerBoundTicks = (opMode==fingerB ? fingerTicks : 0);

    //Tightness neds to be subtracted from the upper bound of the cut (making the void narrower)
    const uint16_t cutUpperBoundTicks = cutLowerBoundTicks + fingerTicks - kerfTicks - eeprom_image.tightness;

    const uint16_t fingerUpperBoundTicks = (opMode==fingerB ? fingerTicks : 2 * fingerTicks);

    // absolute upper bound = 2 * eeprom_image.fingerWidth - eeprom_image.kerf;

    // When close to a finger/void boundary flash the appropriate LED
    if (cutLowerBoundTicks <= modPos && modPos <= cutUpperBoundTicks) {
      // in a void between fingers: cut!
      LED_ON(_GREEN_LED_);
      // Are we approaching the end of the void?
      // Defaults are close = < 40 ticks (.5mm), very close < 0.25mm (20 ticks)
      if (cutUpperBoundTicks - modPos < VERY_CLOSE) {
        // we are very close to the last cut in a  void
        if (millis() - ledUpdateTime > FAST_FLASH_DELAY) {
          // fast flash
          LED_TOGGLE(_RED_LED_);
          ledUpdateTime = millis();
        }
      } else if (cutUpperBoundTicks - modPos < CLOSE) {
        // we are close to the last cut in a void
        if (millis() - ledUpdateTime > FLASH_DELAY) {
          // slow flash
          LED_TOGGLE(_RED_LED_);
          ledUpdateTime = millis();
        }
      } else {
        LED_OFF(_RED_LED_);
      }
    } else {
      // finger: don't cut
      // Are we approaching the end of the finger?
      if (fingerUpperBoundTicks - modPos < VERY_CLOSE) {  // 20 ticks = .25mm
        // we are very close to the last "don't cut" on a finger,
        // and getting very close to the first cut of the next void
        if (millis() - ledUpdateTime > FAST_FLASH_DELAY) {
          // fast flash
          LED_TOGGLE(_GREEN_LED_);
          ledUpdateTime = millis();
        }

      } else if (fingerUpperBoundTicks - modPos < CLOSE) {  // 40 ticks = .5mm
        // we are close to the last "don't cut" on a finger,
        // and getting close to the first cut of the next void
        if (millis() - ledUpdateTime > FLASH_DELAY) {
          // slow flash
          LED_TOGGLE(_GREEN_LED_);
          ledUpdateTime = millis();
        }
      } else {
        LED_OFF(_GREEN_LED_);
      }
      LED_ON(_RED_LED_);
    }
  }

}


void calculateTicks() {
  kerfTicks = eeprom_image.kerf * ticksPerMm;
  fingerTicks =  eeprom_image.fingerWidth * ticksPerMm ;
}



/* TIMER */
void initTimer() {

  // ATTiny84 uses timer1

  // This is the default value for TCCR1A according to the
  // datasheet...however: it does not work without this!
  // Possibly the value is manipulated by a library or framework.  *shrug*
  TCCR1A = 0;

  // enable the output compare interrupt A
  TIMSK1 = ( 1 << OCIE1A );

  // for now, keep the timer off - it will be started as needed
  // (e.g. with a button press, or "last input" during setup)
  stopTimer();

}


void startTimer(uint16_t msec = 0) {

  // Set the counter for a pre-scale factor of 1024
  // However we are further scaling in the interrupt routine by a factor of 256
  // This is to allow for longer delays without exceeding the 8 or 16 bit capacity of
  // the output compare register A  (OCR1A for ATTiny 84, OCR2A for UNO)

  // set the interrupt counter to 0
  timerIntCount = 0;

  // Timer1 - 16 bit: max msec delay of 65535 should be ok
  // turn the msec parameter into a count for the output compare register A
  // The 262144 value is the prescale (1024) times the count of interrupts that are needed
  // before the timer signals that it has expired (256)
  //  i.e. 262144 = 1024 * 256

  OCR1A = (msec>0 ? msec : LONGPRESS) * ((float)(F_CPU/1000) / (float)262144);

  // force the timer1 count to 0
  TCNT1 = 0;

  // Set WGM12 for CTC mode, CS12 & CS10 for prescale = 1024
  // Setting the clock divisor also starts the timer running
  TCCR1B = (1 << WGM12) | (1 << CS12) | (1 << CS10);

  // debugging: turn on the LED when the timer is started
  //LED_ON(_GREEN_LED_);

  timerRunning = true;
}


void stopTimer() {

  // force the interrupt counter to 0
  timerIntCount = 0;
  // clear the expired flag
  idleTimerExpired = false;
  // debugging: turn off the LEDs when the timer is stopped
  //LED_OFF(_GREEN_LED_);
  //LED_OFF(_RED_LED_);

  timerRunning = false;

  // Setting the clock divisor to 000 stops the timer
  TCCR1B = 0;

  // force the timer1 counter and the "output compare register A" value to 0
  TCNT1 = 0;
  OCR1A = 0;
}


void resetTimer() {
  // force the interrupt counter to 0
  timerIntCount = 0;
}


// timer interrupt to detect press/hold of buttons, or for idle timeout
ISR(TIMER1_COMPA_vect) {

  // Ignore any interrupts when timer is not suppposed to be running.
  // When the 8 bit timer_int_cnt overflows back to 0 we set timer_expired
  // This has the effect of dividing the timer interrupt frequency by 256,
  // allowing for longer delays
  if (timerRunning) {

    // debugging - toggle the red LED on PA6 each time an interrupt occurs (when the timer is running)
    //LED_TOGGLE(_RED_LED_);

    if ((++timerIntCount) == 0) {

      // Check if we are in setup; if so, set the timer expired flag
      // else nudge the button state machine
      if (setupState != 0) {
        // during setup, timer expired means "idle timeout"
        idleTimerExpired = true;
      } else {
        // not in setup
        processEvent(BTN_TIMER_EXP);
      }

      // debugging: turn on the LED when the timer expires
      //LED_ON(_RED_LED_);
    }
  }
}


/* POSITION ENCODER */
void initEncoder() {
  // Set the encoder pins as inputs
  pinMode(ENC_A, INPUT);
  pinMode(ENC_B, INPUT);

  encPos = 0;
  saveEncPos = 0;
  prevEncPos = 0;

  // initialise the encoder state variable
  // read pins PA2 and PA3 as a single byte (but mask so only the encoder pins come through)
  encState = PINA & ENC_BITMASK;

  // Duplicate the initial value into bits 1/0 so that a false "change" isn't detected 
  encStates = ((encState >> 2) & B00000011) | encState;

  // setup the pin change interrupt:
  // disable interrupts while setting the register values
  cli();

  // enable PCINT3 & PCINT2 (ATTiny84 pins 11,10 or PA2/PA3)
  bitSet(PCMSK0, PCINT3);
  bitSet(PCMSK0, PCINT2);

  // Enable pin change interrupts on port A (PCIE0)
  bitSet(GIMSK, PCIE0);

  // enable interrupts
  sei();
}


// Pin change interrupt handler for the position encoder
ISR(PCINT0_vect) {
  // read the current encoder state
  encState = PINA & ENC_BITMASK;

  // shift the previous state from bits 3/2 -> 1/0 and put the new state into bits 3/2
  encStates = ((encStates >> 2) & B00000011) | encState;
  // use the states to index into the increments array, and adjust the absolute position accordingly
  encPos += increments[encStates];
}


/* FINGER SETUP AND EEPROM SETTINGS */

void initSettings() {

  // attempt to restore settings from EEPROM
  EEPROM.get(0, eeprom_image);

  // check the signature - by default, look for 03BC - the greek small letter mu
  if (!(eeprom_image.sig[0] == SIG_BYTE0 && eeprom_image.sig[1] == SIG_BYTE1)) {
     // signature not found: use default values
    eeprom_image.sig[0] = SIG_BYTE0;
    eeprom_image.sig[1] = SIG_BYTE1;
    eeprom_image.kerf = DEF_KERF;
    // finger cannot be less than kerf
    eeprom_image.fingerWidth = ( eeprom_image.kerf > DEF_FINGER ? eeprom_image.kerf : DEF_FINGER);
    eeprom_image.tightness = 0;                        // in encoder ticks (0.0125mm)

    // initialise eeprom by saving the values
    EEPROM.put(0, eeprom_image);
  }
  // debug:
  //else {
  // signature found: use the eeprom values we just read
  //display.showString_P(PSTR("Sig OK"));
  //delay(1000);
  //}

}


void doFingerSetup() {

  // The states are 0=setup inactive (normal operation), 1=finger size, 2=kerf, 3=tightness
  // Upon entry, the state transitions from 0->1 (set finger size)
  // Then 1->2 (kerf), then 2->3 (tightness), then back to setup inactive (normal operation)

  // Use the encoder to input the values

  // Start the idle timer so we drop out of setup after IDLETIMEOUT msec (default: 10 secs)
  startTimer(IDLETIMEOUT);

  while ((setupState < 3) && !idleTimerExpired) {

    // reset the single click flag
    btnFlags &= !SINGLE_CLICK_FLAG;

    // go to next state:
    setupState++;

    // debug
    //display.showString_P(PSTR("SS"));
    //display.showNumber(setupState, 0, 1, 3);
    //delay(1000);

    if (setupState == 1) {
      // Get the finger width
      display.showString_P(PSTR("F"));
      display.showNumber(eeprom_image.fingerWidth, 2, 4, 2);

      // The minimum finger width the next multiple of increment >= kerf
      eeprom_image.fingerWidth = getEncoderInput(eeprom_image.fingerWidth, FINGER_INC, roundUp(eeprom_image.kerf, FINGER_INC), MAX_FINGER, FINGER_DECIMALS);

    } else if (setupState == 2) {
      // Get the blade width (kerf)
      display.showString_P(PSTR("b"));
      display.showNumber(eeprom_image.kerf, 1, 4, 2);

      // The MIN_KERF default assumes minimum saw blade is 2.00 mm thick
      // Max of 25mm is perhaps for a dado stack - probably more than is possible?
      eeprom_image.kerf = getEncoderInput(eeprom_image.kerf, KERF_INC, MIN_KERF, MAX_KERF, KERF_DECIMALS);

      // Make sure the fingers are at least as wide as the kerf, and a multiple of the finger increment
      if (eeprom_image.fingerWidth < eeprom_image.kerf) {
        eeprom_image.fingerWidth = roundUp(eeprom_image.kerf, FINGER_INC);
      }

    } else if (setupState == 3) {
      // Get the tightness (the number of encoder "ticks" to add or subtract on each finger)
      // min/max is +/- 10 ticks (+/-0.125mm)
      // +ve values are tighter; -ve values are looser
      display.showString_P(PSTR("t"));
      display.showNumber((float)eeprom_image.tightness, 0, 4, 2);

      eeprom_image.tightness = (int8_t) getEncoderInput(eeprom_image.tightness, TIGHTNESS_INC, MIN_TIGHTNESS, MAX_TIGHTNESS, TIGHTNESS_DECIMALS);
    }
  }

  // Only save the new values if the setup was completed - not if it times out
  if (!idleTimerExpired) {
    // save the values to EEPROM (put() uses update() so will only write changed values)
    EEPROM.put(0, eeprom_image);

    // convert the kerf and finger width to encoder ticks
    calculateTicks();
  }

  // stop the idle timer and exit setup mode
  stopTimer();
  setupState = 0;

}


float getEncoderInput(float currentValue, float increment, float min, float max, uint8_t decimals) {

  // Use the encoder to input values
  // Grab the encoder value now, so we can detect input as a difference
  long refEncPos = encPos;        // ticks
  long prevEncPos = refEncPos;    // ticks
  long encDiff = 0;               // ticks
  float newValue = currentValue;  // parameter value between min and max

  // convert the currentvalue to encoder ticks
  //long currEncVal = MM_ENCPOS(currentValue);

  // work out the encoder ticks per increment, and map the min/max so that we are only
  // dealing with +ve values (else rounding doesn't work well!)
  uint8_t ticks_per_inc = (ENCODER_PULSES_PER_REV / INCS_PER_REV);

  // the number of possible values between min/max stepping by the increment, times the ticks per inc
  // uint16_t tick_range = ((max - min + increments)/increments)  * ticks_per_inc;

  // min and max values mapped to encoder ticks
  // map_min = 0 always
  uint16_t map_max = ((((max - min)/increment) + 1) * ticks_per_inc) - 1;

  // convert the current value to a tick range value
  // add 1/2 increment so that the value is mapped to the middle of the tick range for that value
  long map_current = ( (currentValue - min) / increment ) * ticks_per_inc + (ticks_per_inc/2);

  // loop until the button is pressed again or idle timeout
  while (!BTN_OP(SINGLE_CLICK_FLAG) && !idleTimerExpired) {

    // debug
    //if (timer_expired) {
    //  display.showString_P(PSTR("E3"));
    //}

    // check for encoder movement
    encDiff = encPos - refEncPos;

    // Update the display only when we find a movement
    if (encPos != prevEncPos) {
      // The encoder has moved
      // Reset the time-out
      resetTimer();

      // Calculate and display a new value
      newValue = min + increment * ((map_current + encDiff)/((int8_t)ticks_per_inc));

      // If the encoder is wound way beyond the min/max limits, then it would have to
      // be wound back into range before further adjustment is possible
      // To fix this: Adjust refEncPos
      if (newValue < min) {
          newValue = min;
          refEncPos = refEncPos + (map_current + encDiff);
      } else if (newValue > max) {
          newValue = max;
          refEncPos = refEncPos + (map_current + encDiff - map_max);
      }

      display.showNumber(newValue, decimals, 4, 2);

      // store the encPos so we only calculate the update once
      prevEncPos=encPos;
    }
  }

  if (BTN_OP(SINGLE_CLICK_FLAG)) {
    // Button has been pressed to confirm value - reset flag, since we handled the button click here
    btnFlags &= ! SINGLE_CLICK_FLAG;
    // Reset the time-out, since we have received input
    resetTimer();
    return newValue;
  } else {
    // an idle timeout must have occured: send back the original value
    return currentValue;
  }
}


// Round up the kerf to the next highest multiple of increment
// Used to determine the minimum finger size
float roundUp(float kerf, float increment) {
  if (increment < 0.01) {
    return kerf;
  } else {
    return (int)((kerf + increment - 0.001)/increment) * increment;
  }
}


/* MISC */
void doRezero() {

  // cannot rezero or restore position while in setup mode:
  if (setupState == 0) {
    if (saveEncPos != 0 && encPos == 0) {
      // we have a saved position and the carriage is at the 0 position
      // => restore the saved position:
      encPos = saveEncPos;
      saveEncPos = 0;
    } else {
      // save the current encPos in case we need to restore it
      saveEncPos = encPos;
      // rezero the display
      encPos = 0;
    }

    // show the new position and flash the display a few times
    bool disp_on = true;
    // the loop counter must be an even number else the display will be OFF at the end
    for (uint8_t k = 0; k < 6; k++) {
      disp_on = !disp_on;
      display.setBrightness(DISP_BRIGHT, disp_on);  // Turn display off/on
      displayPosition();       // Display visibility only changes when data is refreshed
      delay(FLASH_DELAY);      // using delay() here is probably ok, since we're resetting anyway?
    }
  }
}


void switchOpMode() {
  // modes are 0=free (not fingering), 1= finger A, 2= finger B

  if (opMode == normal) {
    opMode = fingerA;
    display.showString_P(PSTR("F=A"));
  } else if (opMode == fingerA) {
    opMode = fingerB;
    display.showString_P(PSTR("F=B"));
  } else {  // fingerB
    opMode = normal;
    display.showString_P(PSTR("FrEE"));
  }

  // update the cut indicator LEDS
  updateCutIndicator();

  delay(500);  // pause for a moment
}


void showFWVersion() {
  // display the version of this firmware
  display.showNumber(FW_VERSION);
  delay(1000);  // pause for a moment
}