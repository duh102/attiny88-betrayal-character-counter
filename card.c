#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>
#include <avr/interrupt.h>
#include "ws2812.h"
#include "hsv_rgb.h"
#include <stdbool.h>
#include <avr/pgmspace.h>

// Some LED placement constants
#define MAX_LED 36
#define SPEED_MIN 0
#define MIGHT_MIN 9
#define SANITY_MIN 18
#define KNOWLEDGE_MIN 27
// We have 6 possible characters
#define CHARACTER_MAX 6
// Each attribute has at max 9 levels, 0-8
#define MAX_STAT 8
// Red: Darrin "Flash" Williams, Ox Bellows
#define RED_R 50
#define RED_G 0
#define RED_B 0
// Green: Brandon Jaspers, Peter Akimoto
#define GREEN_R 0
#define GREEN_G 50
#define GREEN_B 00
// Blue: Vivan Lopez, Madame Zostra
#define BLUE_R 0
#define BLUE_G 0
#define BLUE_B 50
// Purple: Jenny LeClerc, Heather Granville
#define PURP_R 30
#define PURP_G 0
#define PURP_B 50
// Yellow: Missy Dubourde, Zoe Ingstrom
#define YELLOW_R 35
#define YELLOW_G 50
#define YELLOW_B 0
// White: Fr. Rhinehardt, Prof. Longfellow
#define WHITE_R 40
#define WHITE_G 40
#define WHITE_B 40

#define PIN_LED PA0
#define DDR_LED DDRA

// Button constants
#define BT_CHAR PC0
#define BT_SPD_DN PD0
#define BT_SPD_UP PD1
#define BT_MI_DN PD2
#define BT_MI_UP PD3
#define BT_SN_DN PD4
#define BT_SN_UP PD5
#define BT_KN_DN PD6
#define BT_KN_UP PD7

// How long between loops (in ms)
#define LOOP_TIME 100
// How many iterations before we continue a button-down
#define BUTTON_TIMEOUT 3

// The pin change interrupt stuff for the character button
#define CHAR_PCIN_IE PCIE1
#define CHAR_PCIN_REG PCMSK1
#define CHAR_PCIN_INT PCINT8
#define CHAR_PCIN_VECT PCINT1_vect
// The pin change interrupt stuff for the stats
#define STAT_PCIN_IE PCIE2
#define STAT_PCIN_REG PCMSK2
// we don't need to define an INT because we're using the whole 8 bit register, so it's just 0xff
#define STAT_PCIN_VECT PCINT2_vect

// Registers for controlling the inputs
#define DDR_CHAR DDRC
#define PORT_CHAR PORTC
#define PIN_CHAR PINC
#define DDR_STATS DDRD
#define PORT_STATS PORTD
#define PIN_STATS PIND

volatile uint8_t speed = 4, might = 4, sanity = 4, knowledge = 4;
volatile uint8_t character = 0;
uint8_t r = 40, g = 0, b = 40;
const uint8_t character_settings[CHARACTER_MAX][3] PROGMEM = {
  {RED_R, RED_G, RED_B},
  {GREEN_R, GREEN_G, GREEN_B},
  {BLUE_R, BLUE_G, BLUE_B},
  {PURP_R, PURP_G, PURP_B},
  {YELLOW_R, YELLOW_G, YELLOW_B},
  {WHITE_R, WHITE_G, WHITE_B},
};

// Doing quick little ceil
#define STATE_SIZE (MAX_LED+7)/8
const uint8_t state_size = STATE_SIZE;
// Boolean state of the LEDs
uint8_t ledState[STATE_SIZE];

// Display/button state
volatile bool displayChanged = true, checkChar = false, checkStats = false;
uint8_t checkCharCount = 0, checkStatCount = 0;

// reserving a byte for loop variant
uint8_t curLed;

void updateDisplay();
void loop();
void incrementCharacter();
void updateRGB();
void doStatsButtons();

ISR(CHAR_PCIN_VECT) {
  checkChar = true;
}

ISR(STAT_PCIN_VECT) {
  checkStats = true;
}

int main() {
  CLKPR = 1<<CLKPCE;   // allow writes to CLKPR
  CLKPR = 0;   // disable system clock prescaler (run at full 8MHz)

  // Enable the LED output pin as an output, the rest of the pins on A as inputs
  DDR_LED = (1<<PIN_LED);

  // Enable the requisite pin change interrupts
  PCICR = (1<<CHAR_PCIN_IE) | (1<<STAT_PCIN_IE);
  // Now, enable all the pins that we want in the proper pin enable registers
  CHAR_PCIN_REG |= (1<<CHAR_PCIN_INT);
  STAT_PCIN_REG = 0xff;

  // Set the inputs to be inputs (DDRx 0, PORTx 1 for pullups)
  DDR_CHAR = 0;
  DDR_STATS = 0;
  PORT_CHAR = 0Xff;
  PORT_STATS = 0xff;

  //make sure we initialize the colors for the first character
  updateRGB();

  // Enable the display
  ws2812_init();

  while(1) {
    loop();
  }
}

void loop() {
  _delay_ms(LOOP_TIME);
  // Check the character flag
  if(checkChar && ((~PIN_CHAR) & (1<<BT_CHAR)) ) {
    if(checkCharCount == 0) {
      incrementCharacter();
    }
    checkCharCount = (checkCharCount+1) % BUTTON_TIMEOUT;
  } else {
    checkCharCount = 0;
  }
  // Check the stats flag
  if(checkStats && (~PIN_STATS) ) {
    if(checkStatCount == 0) {
      doStatsButtons();
    }
    checkStatCount = (checkStatCount+1) % BUTTON_TIMEOUT;
  } else {
    checkStatCount = 0;
  }

  // After all that, update the display if we need to
  if(displayChanged) {
    updateDisplay();
  }
}

void updateDisplay() {
  displayChanged = false;

  for(uint8_t place = 0; place < state_size; place++) {
    ledState[place] = 0;
  }
  ledState[(SPEED_MIN+speed)/8] |= 1<<((SPEED_MIN+speed)%8);
  ledState[(MIGHT_MIN+might)/8] |= 1<<((MIGHT_MIN+might)%8);
  ledState[(SANITY_MIN+sanity)/8] |= 1<<((SANITY_MIN+sanity)%8);
  ledState[(KNOWLEDGE_MIN+knowledge)/8] |= 1<<((KNOWLEDGE_MIN+knowledge)%8);

  cli();
  for(curLed = 0; curLed < MAX_LED; curLed++) {
    if( ledState[curLed/8] & 1<<(curLed%8) ) {
      ws2812_set_single(r, g, b);
    } else {
      ws2812_set_single(0, 0, 0);
    }
  }
  sei();
}

void incrementCharacter() {
  character = (character + 1) % CHARACTER_MAX;
  updateRGB();
  displayChanged = true;
}

void updateRGB() {
  r = pgm_read_byte(&character_settings[character][0]);
  g = pgm_read_byte(&character_settings[character][1]);
  b = pgm_read_byte(&character_settings[character][2]);
}

void doStatsButtons() {
  uint8_t enabled = ~PIN_STATS;

  // Speed
  if(enabled & (1<<BT_SPD_DN) ) {
    speed = speed == 0? 0 : speed - 1;
  } else if (enabled & (1<<BT_SPD_UP)) {
    speed = speed == MAX_STAT? MAX_STAT : speed + 1;
  }

  // Might
  if(enabled & (1<<BT_MI_DN) ) {
    might = might == 0? 0 : might - 1;
  } else if (enabled & (1<<BT_MI_UP)) {
    might = might == MAX_STAT? MAX_STAT : might + 1;
  }

  // Sanity
  if(enabled & (1<<BT_SN_DN) ) {
    sanity = sanity == 0? 0 : sanity - 1;
  } else if (enabled & (1<<BT_SN_UP)) {
    sanity = sanity == MAX_STAT? MAX_STAT : sanity + 1;
  }

  // Knowledge
  if(enabled & (1<<BT_KN_DN) ) {
    knowledge = knowledge == 0? 0 : knowledge - 1;
  } else if (enabled & (1<<BT_KN_UP)) {
    knowledge = knowledge == MAX_STAT? MAX_STAT : knowledge + 1;
  }

  displayChanged = true;
}
