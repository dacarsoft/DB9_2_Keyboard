
/*
Project:  DB9 Joysticks to PS/2 adapter firmware
Author:   David Carrión - 2023

PS2Dev library: https://github.com/Harvie/ps2dev
SoftwareI2C library: https://github.com/Seeed-Studio/Arduino_Software_I2C
*/



#include "ps2dev.h" // PS/2 protocol
#include "Keyboard.h" // USB protocol
#include "src/u8g2/U8g2lib.h" // OLED display - See modified U8x8lib.h where some optimizations where done by changing: #define U8X8_USE_ARDUINO_AVR_SW_I2C_OPTIMIZATION, #define U8X8_DO_NOT_SET_WIRE_CLOCK, #define U8X8_HAVE_HW_I2C and #define U8X8_HAVE_HW_SPI highlighted with "///////" at the end of lines
#include "SoftwareI2C.h"
#include <EEPROM.h>



/* *********** Board model compilation selection *********** */
#define AVILLENA_BOARD
//#define PROMICRO_BOARD
/* ********************************************************* */







/* *********** START - Antonio Villena's board declarations *********** */
#ifdef AVILLENA_BOARD
/*
Antonio Villena's ATmega 32U4 board pinout:
Arduino 18 -> DB9_1 pin 1: Up
Arduino 19 -> DB9_1 pin 2: Down
Arduino 20 -> DB9_1 pin 3: Left
Arduino 21 -> DB9_1 pin 4: Right
Arduino +  -> DB9_1 pin 5: VCC 5V
Arduino 14 -> DB9_1 pin 6: A / Primary fire
Arduino 7  -> DB9_1 pin 7: Selection signal (allows more buttons)
Arduino -  -> DB9_1 pin 8: GND
Arduino 15 -> DB9_1 pin 9: SELECT / Secondary fire
*/
const uint8_t PS2_PINS[2] = { 12, 30 }; // clock PD6 12, data PD5 30 (TXLED)

const uint8_t DB9_1_PINS[6] = { 18, 19, 20, 21, 14, 15 }; // up A0, down A1, left A2, right A3, button1 PB3, button2 PB1 - Exclude pin 7 for Sega controllers
const uint8_t DB9_1_SELECT = 7; // select E6

const uint8_t DB9_2_PINS[6] = { 1, 0, 2, 3, 4, 6 }; // up PD3, down PD2, left PD1, right PD0, button1 PD4, button2 PD7 - Exclude pin 7 for Sega controllers
const uint8_t DB9_2_SELECT = 5; // select PC6

const uint8_t OLED_PINS[2] = { 3, 2 }; // SDA (DB9_2 pin 3), SCL (DB9_2 pin 4) - OLED screen pins
#endif
/* ***********  END  - Antonio Villena's board declarations *********** */



/* *********** START - Arduino Pro Micro board declarations *********** */
#ifdef PROMICRO_BOARD
/*
Arduino Pro Micro ATmega 32U4 board pinout:
Arduino 2 -> DB9_1 pin 1: Up
Arduino 3 -> DB9_1 pin 2: Down
Arduino 4 -> DB9_1 pin 3: Left
Arduino 5 -> DB9_1 pin 4: Right
Arduino + -> DB9_1 pin 5: VCC 5V
Arduino 6 -> DB9_1 pin 6: A / Primary fire
Arduino 7 -> DB9_1 pin 7: Selection signal (allows more buttons)
Arduino - -> DB9_1 pin 8: GND
Arduino 8 -> DB9_1 pin 9: SELECT / Secondary fire
*/
const uint8_t PS2_PINS[2] = { 1, 0 }; // clock PD3 1, data PD2 0  // clock PD5 24, data PD6

const uint8_t DB9_1_PINS[6] = { 2, 3, 4, 5, 6, 8 }; // up PD1, down PD0, left PD4, right PC6, button1 PD7, button2 PB4 - Exclude pin 7 for Sega controllers
const uint8_t DB9_1_SELECT = 7; // select PE6

const uint8_t DB9_2_PINS[6] = { 10, 16, 14, 15, 18, 20 }; // up PB6, down PB2, left PB3, right PB1, button1 PF7, button2 PF5 - Exclude pin 7 for Sega controllers
const uint8_t DB9_2_SELECT = 19; // select PF6

//const uint8_t LED_ONBOARD_1 = LED_BUILTIN_RX; // RXLED 17
//const uint8_t LED_ONBOARD_2 = LED_BUILTIN_TX; // TXLED 30
const uint8_t LED_ONBOARD_1 = 17; // RXLED 17
const uint8_t LED_ONBOARD_2 = 30; // TXLED 30

const uint8_t OLED_PINS[2] = { 21, 9 }; // SDA, SCL - OLED screen pins
#endif
/* ***********  END  - Arduino Pro Micro board declarations *********** */







/* *********** START - Keyboard declarations *********** */
PS2dev keyboard(PS2_PINS[0], PS2_PINS[1]);  // PS/2 - clock, data
//char lastkeycode; // PS/2 - Keycode to be sent again when something fails // Not needed anymore? ///////
//uint8_t lastkeycodestatus; // PS/2 - Keydown or keyrelease status for the last keycode sent
//unsigned char keyboardleddata; // PS/2 - Needed for keyboard reading handle code

bool USB_AVAILABLE = false; // USB - Needed to disable USB functionality after a timeout if it is not available
/* ***********  END  - Keyboard declarations *********** */







/* *********** START - DB9 joysticks declarations *********** */
const uint8_t DB9_1_TOTALPINS = sizeof(DB9_1_PINS);

const char DB9_1_MAP_PS2[5][12] = { // Keycode PS2 maps for the first controller
{0x42, 0x43, 0x40, 0x41, 0x47, 0x48, 0x46, 0x44, 0x4B, 0x4A, 0x49, 0x45}, // ESPectrum special keycodes ///////
{PS2dev::UP_ARROW, PS2dev::DOWN_ARROW, PS2dev::LEFT_ARROW, PS2dev::RIGHT_ARROW, PS2dev::RIGHT_ALT, PS2dev::ENTER, PS2dev::ESCAPE, PS2dev::F1, PS2dev::Z, PS2dev::Y, PS2dev::X, PS2dev::M}, // ZX-Spectrum emulators using F1 and keyboard cursors
{PS2dev::SEVEN, PS2dev::SIX, PS2dev::FIVE, PS2dev::EIGHT, PS2dev::ZERO, PS2dev::ENTER, PS2dev::ESCAPE, PS2dev::F1, PS2dev::Z, PS2dev::Y, PS2dev::X, PS2dev::M}, // ZX-Spectrum emulators using F1 and Sinclair rubber keyboard cursors
{PS2dev::UP_ARROW, PS2dev::DOWN_ARROW, PS2dev::LEFT_ARROW, PS2dev::RIGHT_ARROW, PS2dev::ZERO, PS2dev::ENTER, PS2dev::ESCAPE, PS2dev::F5, PS2dev::Z, PS2dev::Y, PS2dev::X, PS2dev::M}, // ZX-UNO
{PS2dev::UP_ARROW, PS2dev::DOWN_ARROW, PS2dev::LEFT_ARROW, PS2dev::RIGHT_ARROW, PS2dev::SPACE, PS2dev::SPACE, PS2dev::SPACE, PS2dev::F5, PS2dev::SPACE, PS2dev::SPACE, PS2dev::SPACE, PS2dev::SPACE}
};

const char DB9_1_MAP_USB[5][12] = { // Keycode USB maps for the first controller
{KEY_UP_ARROW, KEY_DOWN_ARROW, KEY_LEFT_ARROW, KEY_RIGHT_ARROW, KEY_RETURN, KEY_RIGHT_ALT, KEY_ESC, KEY_F12, 'z', 'y', 'x', 'm'}, // MiSTer
{KEY_UP_ARROW, KEY_DOWN_ARROW, KEY_LEFT_ARROW, KEY_RIGHT_ARROW, 'x', 'c', 'z', KEY_ESC, KEY_RETURN, 'c', 'x', KEY_SCROLL_LOCK}, // Pico-8 player 1
{KEY_UP_ARROW, KEY_DOWN_ARROW, KEY_LEFT_ARROW, KEY_RIGHT_ARROW, KEY_RETURN, '0', KEY_ESC, KEY_F5, 'z', 'y', 'x', 'm'}, // F5 emulators map
{'7', '6', '5', '8', '0', KEY_RETURN, KEY_ESC, KEY_F12, 'z', 'y', 'x', 'm'}, // ZX-Spectrum emulators using F12 and Sinclair rubber keyboard cursors
{KEY_UP_ARROW, KEY_DOWN_ARROW, KEY_LEFT_ARROW, KEY_RIGHT_ARROW, '4', '5', '6', '7', '8', '9', '0', 'A'}
};

uint8_t DB9_1_MAP_ACTIVE = 0;
uint8_t DB9_1_PRESSCOUNT[12] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
bool DB9_1_STATUS[12] = { false, false, false, false, false, false, false, false, false, false, false, false };


const uint8_t DB9_2_TOTALPINS = sizeof(DB9_2_PINS);

const char DB9_2_MAP_PS2[5][12] = { // Keycode PS2 maps for the second controller
{0x4E, 0x4F, 0x4C, 0x4D, 0x53, 0x54, 0x52, 0x50, 0x57, 0x56, 0x55, 0x51}, // ESPectrum special keycodes ///////
{PS2dev::UP_ARROW, PS2dev::DOWN_ARROW, PS2dev::LEFT_ARROW, PS2dev::RIGHT_ARROW, PS2dev::RIGHT_ALT, PS2dev::ENTER, PS2dev::ESCAPE, PS2dev::F1, PS2dev::Z, PS2dev::Y, PS2dev::X, PS2dev::M},
{PS2dev::SEVEN, PS2dev::SIX, PS2dev::FIVE, PS2dev::EIGHT, PS2dev::ZERO, PS2dev::ENTER, PS2dev::ESCAPE, PS2dev::F1, PS2dev::Z, PS2dev::Y, PS2dev::X, PS2dev::M},
{PS2dev::UP_ARROW, PS2dev::DOWN_ARROW, PS2dev::LEFT_ARROW, PS2dev::RIGHT_ARROW, PS2dev::ZERO, PS2dev::ENTER, PS2dev::ESCAPE, PS2dev::F5, PS2dev::Z, PS2dev::Y, PS2dev::X, PS2dev::M},
{PS2dev::UP_ARROW, PS2dev::DOWN_ARROW, PS2dev::LEFT_ARROW, PS2dev::RIGHT_ARROW, PS2dev::SPACE, PS2dev::SPACE, PS2dev::SPACE, PS2dev::F5, PS2dev::SPACE, PS2dev::SPACE, PS2dev::SPACE, PS2dev::SPACE}
};

const char DB9_2_MAP_USB[5][12] = { // Keycode USB maps for the second controller
{'q', 'a', 'o', 'p', 'm', KEY_RETURN, KEY_ESC, KEY_F12, 'z', 'y', 'x', 'c'},
{'e', 'd', 's', 'f', 'q', KEY_TAB, KEY_LEFT_SHIFT, KEY_ESC, KEY_RETURN, KEY_TAB, 'q', KEY_SCROLL_LOCK}, // Pico-8 player 2
{KEY_UP_ARROW, KEY_DOWN_ARROW, KEY_LEFT_ARROW, KEY_RIGHT_ARROW, KEY_RETURN, '0', KEY_ESC, KEY_F5, 'z', 'y', 'x', 'm'},
{'7', '6', '5', '8', '0', KEY_RETURN, KEY_ESC, KEY_F12, 'z', 'y', 'x', 'm'},
{KEY_UP_ARROW, KEY_DOWN_ARROW, KEY_LEFT_ARROW, KEY_RIGHT_ARROW, '4', '5', '6', '7', '8', '9', '0', 'A'}
};

uint8_t DB9_2_MAP_ACTIVE = 0;
uint8_t DB9_2_PRESSCOUNT[12] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
bool DB9_2_STATUS[12] = { false, false, false, false, false, false, false, false, false, false, false, false };


const uint8_t DB9_CYCLES_PRESS = 1; // DEBOUNCE CYCLES TO VALIDATE PRESS
const uint8_t DB9_CYCLES_WAIT = 3;  // RELAX CYCLES NEEDED
uint8_t DB9_CYCLES_WAITCOUNT = 0;   // RELAX CYCLES PENDING


//bool JOYSTICK_3BUTTON = false;
uint8_t JOYSTICK_BUTTONS[2] = { 0, 0 };
/* ***********  END  - DB9 joysticks declarations *********** */







/* *********** START - OLED screen declarations *********** */
//U8G2_SSD1306_128X64_NONAME_F_SW_I2C U8G2(U8G2_R0, OLED_PINS[0], OLED_PINS[1], U8X8_PIN_NONE); // OLED display declaration
U8G2_SSD1306_128X64_NONAME_1_SW_I2C U8G2(U8G2_R0, OLED_PINS[0], OLED_PINS[1], U8X8_PIN_NONE); // OLED display declaration
//U8G2_SSD1306_128X64_NONAME_1_HW_I2C U8G2(U8G2_R0, U8X8_PIN_NONE); // OLED display declaration

int8_t OLED_CURRENT_MODE = -1;   // OLED menu section identifier - 0 : Initialization, 1 : Status info, 2 : On-screen keyboard
uint16_t OLED_BUTTON_COUNTER[2] = { 0, 0 };   // OLED button menu press control for each joystick
int8_t OLED_KEYSEL_X = 0;
int8_t OLED_KEYSEL_Y = 0;
//bool OLED_KEYSEL_FLASH = false;
const char* OLED_KEYBOARD_BUTTONS[10][10] = { // Array[Rows][Cols] of visible composed buttons to obtain strings as the buttons may be composed of more than one chars
{"1","2","3","4","5","6","7","8","9","0"},
{"Q","W","E","R","T","Y","U","I","O","P"},
{"A","S","D","F","G","H","J","K","L",""},
{"Z","X","C","V","B","N","M","","bksp",""},
{"","esc","","","space","","","","enter",""},
{"F1","","F2","","F3","","F4","","F5",""},
{"F6","","F7","","F8","","F9","","F10",""},
{"","F11","","F12","","","Lsh","","Rsh",""},
{"","Lct","","alt","","","agr","","Rct",""},
{"Cu","","Cd","","Cl","","Cr","","tab",""}
};
const char OLED_KEYBOARD_MAP_PS2[10][10] = { // Array of PS/2 scancodes to send from OLED virtual keyboard
{PS2dev::ONE,PS2dev::TWO,PS2dev::THREE,PS2dev::FOUR,PS2dev::FIVE,PS2dev::SIX,PS2dev::SEVEN,PS2dev::EIGHT,PS2dev::NINE,PS2dev::ZERO},
{PS2dev::Q,PS2dev::W,PS2dev::E,PS2dev::R,PS2dev::T,PS2dev::Y,PS2dev::U,PS2dev::I,PS2dev::O,PS2dev::P},
{PS2dev::A,PS2dev::S,PS2dev::D,PS2dev::F,PS2dev::G,PS2dev::H,PS2dev::J,PS2dev::K,PS2dev::L,' '},
{PS2dev::Z,PS2dev::X,PS2dev::C,PS2dev::V,PS2dev::B,PS2dev::N,PS2dev::M,' ',PS2dev::BACKSPACE,' '},
{' ',PS2dev::ESCAPE,' ',' ',PS2dev::SPACE,' ',' ',' ',PS2dev::ENTER,' '},
{PS2dev::F1,' ',PS2dev::F2,' ',PS2dev::F3,' ',PS2dev::F4,' ',PS2dev::F5,' '},
{PS2dev::F6,' ',PS2dev::F7,' ',PS2dev::F8,' ',PS2dev::F9,' ',PS2dev::F10,' '},
{' ',PS2dev::F11,' ',PS2dev::F12,' ',' ',PS2dev::LEFT_SHIFT,' ',PS2dev::RIGHT_SHIFT,' '},
{' ',PS2dev::LEFT_CONTROL,' ',PS2dev::LEFT_ALT,' ',' ',PS2dev::RIGHT_ALT,' ',PS2dev::RIGHT_CONTROL,' '},
{PS2dev::UP_ARROW,' ',PS2dev::DOWN_ARROW,' ',PS2dev::LEFT_ARROW,' ',PS2dev::RIGHT_ARROW,' ',PS2dev::TAB,' '}
};
const char OLED_KEYBOARD_MAP_USB[10][10] = { // Array of USB scancodes to send from OLED virtual keyboard
{'1','2','3','4','5','6','7','8','9','0'},
{'q','w','e','r','t','y','u','i','o','p'},
{'a','s','d','f','g','h','j','k','l',' '},
{'z','x','c','v','b','n','m',' ',KEY_BACKSPACE,' '},
{' ',KEY_ESC,' ',' ',' ',' ',' ',' ',KEY_RETURN,' '},
{KEY_F1,' ',KEY_F2,' ',KEY_F3,' ',KEY_F4,' ',KEY_F5,' '},
{KEY_F6,' ',KEY_F7,' ',KEY_F8,' ',KEY_F9,' ',KEY_F10,' '},
{' ',KEY_F11,' ',KEY_F12,' ',' ',KEY_LEFT_SHIFT,' ',KEY_RIGHT_SHIFT,' '},
{' ',KEY_LEFT_CTRL,' ',KEY_LEFT_ALT,' ',' ',KEY_RIGHT_ALT,' ',KEY_RIGHT_CTRL,' '},
{KEY_UP_ARROW,' ',KEY_DOWN_ARROW,' ',KEY_LEFT_ARROW,' ',KEY_RIGHT_ARROW,' ',KEY_TAB,' '}
};
/* ***********  END  - OLED screen declarations *********** */







void setup() {
  
  //DDRD=0; // Help to disable UART to use pins PD3 and PD2 (1 and 0)

  #ifdef PROMICRO_BOARD
  pinMode(LED_ONBOARD_1, OUTPUT); // Onboard LED 1 
  digitalWrite(LED_ONBOARD_1, HIGH);
  pinMode(LED_ONBOARD_2, OUTPUT); // Onboard LED 2 
  digitalWrite(LED_ONBOARD_2, HIGH);
  #endif

  // Check for an OLED screen available
  OLEDcheck();

  // Keyboard connection mode detection with delay between tries
  uint8_t USB_tries = 6;
  while (USB_AVAILABLE == false && USB_tries > 0) {
    USB_tries--;
    delay(500); // Waits 1/2 second for USB host assignment request check
    USBcheck(true); // USB connection check and USB keyboard initialization when found but using "false" to avoid an OLED screen update showing the default keyboard map
  }

  // Maps management and PS/2 intialization depending on connection mode found
  if (USB_AVAILABLE == false) {
    // Recover the PS/2 keyboard maps previously stored here
    DB9_1_MAP_ACTIVE = EEPROM.read(0); // Recover the previously stored active map index for DB9-1
    if (DB9_1_MAP_ACTIVE > (sizeof(DB9_1_MAP_PS2) / sizeof(DB9_1_MAP_PS2[0]))) { DB9_1_MAP_ACTIVE = 0; } // Ensure the value recovered is valid just in case it was not previously stored
    DB9_2_MAP_ACTIVE = EEPROM.read(1); // Recover the previously stored active map index for DB9-2
    if (DB9_2_MAP_ACTIVE > (sizeof(DB9_2_MAP_PS2) / sizeof(DB9_2_MAP_PS2[0]))) { DB9_2_MAP_ACTIVE = 0; } // Ensure the value recovered is valid just in case it was not previously stored
  
    keyboard.keyboard_init(); // PS2 keyboard initialization when no USB was found (notice the k in lowercase)
  }
  
  // Joystick in DB9 port 1 initialization
  for ( uint8_t i = 0; i < sizeof(DB9_1_PINS); ++i ) {
    pinMode(DB9_1_PINS[i], INPUT_PULLUP); // Setup joystick 1 pins as inputs for buttons presses read
  }
  pinMode(DB9_1_SELECT, OUTPUT); // Pin to do signal selection for extra buttons in Sega controllers
  digitalWrite(DB9_1_SELECT, HIGH);
  
  /*
  // Sega 6 button detection
  digitalWrite(DB9_1_SELECT, LOW);  // State 0
  digitalWrite(DB9_1_SELECT, HIGH); // State 1
  digitalWrite(DB9_1_SELECT, LOW);  // State 2
  digitalWrite(DB9_1_SELECT, HIGH); // State 3
  */
  if ((digitalRead(DB9_1_PINS[0]) == LOW) && (digitalRead(DB9_1_PINS[1]) == HIGH)) { // Check press for desired map change
    DB9_1_MAP_ACTIVE = 1;
  }
  if ((digitalRead(DB9_1_PINS[1]) == LOW) && (digitalRead(DB9_1_PINS[0]) == HIGH)) { // Check press for desired map change
    DB9_1_MAP_ACTIVE = 2;
  }
  if ((digitalRead(DB9_1_PINS[2]) == LOW) && (digitalRead(DB9_1_PINS[3]) == HIGH)) { // Check press for desired map change
    DB9_1_MAP_ACTIVE = 3;
  }
  if ((digitalRead(DB9_1_PINS[3]) == LOW) && (digitalRead(DB9_1_PINS[2]) == HIGH)) { // Check press for desired map change
    DB9_1_MAP_ACTIVE = 4;
  }
  if (digitalRead(DB9_1_PINS[4]) == LOW) { // Check press for desired map change
    DB9_1_MAP_ACTIVE = 0;
  }
  /*
  digitalWrite(DB9_1_SELECT, LOW);  // State 4
  if ((digitalRead(DB9_1_PINS[0]) == LOW) && (digitalRead(DB9_1_PINS[1]) == LOW)) { // Detect if UP and DOWN are sent as pressed at the same time to check 6 buttons mode
    DB9_1_6BUTTON = true;
    
    #ifdef PROMICRO_BOARD
    digitalWrite(LED_ONBOARD_2, LOW);
    #endif
    
    digitalWrite(DB9_1_SELECT, HIGH); // State 5
    digitalWrite(DB9_1_SELECT, LOW);  // State 6
    digitalWrite(DB9_1_SELECT, HIGH); // State 7
    delay(15);
    //digitalWrite(DB9_1_SELECT, LOW);  // State 0
    //digitalWrite(DB9_1_SELECT, HIGH); // State 1
    //DB9_1_SELECT_TIME = millis();
  }
  */

  
  
  // Joystick in DB9 port 2 initialization
  for ( uint8_t i = 0; i < sizeof(DB9_2_PINS); ++i ) {
    pinMode(DB9_2_PINS[i], INPUT_PULLUP); // Setup joystick 2 pins as inputs for buttons presses read /////// OLED HW TEST
  }
  pinMode(DB9_2_SELECT, OUTPUT); // Pin to do signal selection for extra buttons in Sega controllers /////// OLED HW TEST
  digitalWrite(DB9_2_SELECT, HIGH);
  
  /*
  // Sega 6 button detection
  digitalWrite(DB9_2_SELECT, LOW);    // State 0
  digitalWrite(DB9_2_SELECT, HIGH);   // State 1
  digitalWrite(DB9_2_SELECT, LOW);    // State 2
  digitalWrite(DB9_2_SELECT, HIGH);   // State 3
  */
  if ((digitalRead(DB9_2_PINS[0]) == LOW) && (digitalRead(DB9_2_PINS[1]) == HIGH)) { // Check press for desired map change
    DB9_2_MAP_ACTIVE = 1;
  }
  if ((digitalRead(DB9_2_PINS[1]) == LOW) && (digitalRead(DB9_2_PINS[0]) == HIGH)) { // Check press for desired map change
    DB9_2_MAP_ACTIVE = 2;
  }
  if ((digitalRead(DB9_2_PINS[2]) == LOW) && (digitalRead(DB9_2_PINS[3]) == HIGH)) { // Check press for desired map change
    DB9_2_MAP_ACTIVE = 3;
  }
  if ((digitalRead(DB9_2_PINS[3]) == LOW) && (digitalRead(DB9_2_PINS[2]) == HIGH)) { // Check press for desired map change
    DB9_2_MAP_ACTIVE = 4;
  }
  if (digitalRead(DB9_2_PINS[4]) == LOW) { // Check press for desired map change
    DB9_2_MAP_ACTIVE = 0;
  }
  /*
  digitalWrite(DB9_2_SELECT, LOW);    // State 4
  if ((digitalRead(DB9_2_PINS[0]) == LOW) && (digitalRead(DB9_2_PINS[1]) == LOW)) { // Detect if UP and DOWN are sent as pressed at the same time to check 6 buttons mode
    DB9_2_6BUTTON = true;
    
    #ifdef PROMICRO_BOARD
    digitalWrite(LED_ONBOARD_2, LOW);
    #endif
    
    digitalWrite(DB9_2_SELECT, HIGH); // State 5
    digitalWrite(DB9_2_SELECT, LOW);  // State 6
    digitalWrite(DB9_2_SELECT, HIGH); // State 7
    delay(15);
    //digitalWrite(DB9_2_SELECT, LOW);  // State 0
    //digitalWrite(DB9_2_SELECT, HIGH); // State 1
    //DB9_2_SELECT_TIME = millis();
  }
  */

  // Store the map index change for PS/2 or USB only if the new value is different as the EEPROM update library function already takes care about it
  if (USB_AVAILABLE == false) {
    EEPROM.update(0, DB9_1_MAP_ACTIVE);
    EEPROM.update(1, DB9_2_MAP_ACTIVE);
  } else {
    EEPROM.update(2, DB9_1_MAP_ACTIVE);
    EEPROM.update(3, DB9_2_MAP_ACTIVE);
  }
  

  //OLEDmanage(1);

}



void loop() {
  /*  // PS/2 Error management - call to read if there are errors pending
  readPS2handle(keyboardleddata);
  */

  // Retain the previous values for the joysticks buttons number which are stored inside the "joystickProcess" for each joystick
  uint8_t joystick1buttonsfound = JOYSTICK_BUTTONS[0];
  uint8_t joystick2buttonsfound = JOYSTICK_BUTTONS[1];
  
  joystickProcess(DB9_1_PINS, DB9_1_TOTALPINS, DB9_1_SELECT, DB9_1_STATUS, DB9_1_PRESSCOUNT, DB9_1_MAP_PS2, DB9_1_MAP_USB, DB9_1_MAP_ACTIVE, 0);
  joystickProcess(DB9_2_PINS, DB9_2_TOTALPINS, DB9_2_SELECT, DB9_2_STATUS, DB9_2_PRESSCOUNT, DB9_2_MAP_PS2, DB9_2_MAP_USB, DB9_2_MAP_ACTIVE, 1);

  // Update the OLED screen showing the joystick buttons info when first detected or a hot swap is done by the user
  if ((joystick1buttonsfound != JOYSTICK_BUTTONS[0]) || (joystick2buttonsfound != JOYSTICK_BUTTONS[1])) { // Check if OLED screen update is needed by compairing with the buttons detected values previously found
    OLEDmanage(1); // Update the last proper value found to be used in next iterations and the OLED screen joystick info shown
  }

  // Check again for USB availability just in case a slow USB host could need more time to wake up USB device detection
  USBcheck(false);
  
  /* // Flash effect
  if (OLED_CURRENT_MODE == 2) {
    OLEDmanage(3);
  }
  */
}







void joystickProcess(const uint8_t JOYSTICK_PINS[6], const uint8_t JOYSTICK_TOTALPINS, const uint8_t JOYSTICK_SELECT, bool JOYSTICK_STATUS[12], uint8_t JOYSTICK_PRESSCOUNT[12], const char JOYSTICK_MAP_PS2[4][12], const char JOYSTICK_MAP_USB[4][12], uint8_t JOYSTICK_MAP_ACTIVE, uint8_t JOYSTICK_INDEX) {

  //JOYSTICK_3BUTTON = false;
  JOYSTICK_BUTTONS[JOYSTICK_INDEX] = 2; // Joystick buttons info flag reset
  
  for (uint8_t i = 0; i < 4; i++) { // Read steps by changing SELECT pin level
    digitalWrite(JOYSTICK_SELECT, LOW);
    if ((digitalRead(JOYSTICK_PINS[0]) == LOW) && (digitalRead(JOYSTICK_PINS[1]) == LOW)) {
      digitalWrite(JOYSTICK_SELECT, HIGH);
      // Now we can try to read the buttons sequence: (UP pin) Z - (DOWN pin) Y - (LEFT pin) X - (RIGHT pin) - Mode
      buttonsProcess(0, 4, 8, JOYSTICK_PINS, JOYSTICK_STATUS, JOYSTICK_PRESSCOUNT, JOYSTICK_MAP_PS2, JOYSTICK_MAP_USB, JOYSTICK_MAP_ACTIVE, JOYSTICK_INDEX);
      //JOYSTICK_3BUTTON = true;
      JOYSTICK_BUTTONS[JOYSTICK_INDEX] = 6; // Joystick buttons info flag update
      break;
    }
    if ((digitalRead(JOYSTICK_PINS[2]) == LOW) && (digitalRead(JOYSTICK_PINS[3]) == LOW)) {
      //JOYSTICK_3BUTTON = true;
      JOYSTICK_BUTTONS[JOYSTICK_INDEX] = 3; // OLED screen joystick buttons info flag update
    }
    digitalWrite(JOYSTICK_SELECT, HIGH);
  }

  if (JOYSTICK_BUTTONS[JOYSTICK_INDEX] > 2) { // JOYSTICK_3BUTTON == true
    for (uint8_t i = 0; i < 4; i++) { // Read steps by changing SELECT pin level
      digitalWrite(JOYSTICK_SELECT, LOW);
      if ((digitalRead(JOYSTICK_PINS[2]) == LOW) && (digitalRead(JOYSTICK_PINS[3]) == LOW)) {
        // Now we can try to read the buttons sequence: (B pin) A - (C pin) Start
        buttonsProcess(4, JOYSTICK_TOTALPINS, 2, JOYSTICK_PINS, JOYSTICK_STATUS, JOYSTICK_PRESSCOUNT, JOYSTICK_MAP_PS2, JOYSTICK_MAP_USB, JOYSTICK_MAP_ACTIVE, JOYSTICK_INDEX);
        digitalWrite(JOYSTICK_SELECT, HIGH); 
        break;
      }
      digitalWrite(JOYSTICK_SELECT, HIGH); 
    }
  }
  
  // Now we can try to read the common buttons sequence: UP - DOWN - LEFT - RIGHT - B - C
  buttonsProcess(0, JOYSTICK_TOTALPINS, 0, JOYSTICK_PINS, JOYSTICK_STATUS, JOYSTICK_PRESSCOUNT, JOYSTICK_MAP_PS2, JOYSTICK_MAP_USB, JOYSTICK_MAP_ACTIVE, JOYSTICK_INDEX);


  
  /* // Previous code
  
  JOYSTICK_SELECT_STATUS = true; // We use true value here as it is needed to start from false or LOW value after the reference flag will be changed below
  JOYSTICK_3BUTTON = false;
  
  for (uint8_t i = 0; i < 4; i++) {
    JOYSTICK_SELECT_STATUS = !JOYSTICK_SELECT_STATUS; // Change the reference flag here to control the SELECT pin level to use
    if (JOYSTICK_SELECT_STATUS == false) { digitalWrite(JOYSTICK_SELECT, LOW); } else { digitalWrite(JOYSTICK_SELECT, HIGH); } // Put the proper level for SELECT pin
    
    // Specific Sega 3 buttons processing  // State 2 (also 0)  // digitalWrite(JOYSTICK_SELECT, LOW)
    if ((JOYSTICK_SELECT_STATUS == false) && ((digitalRead(JOYSTICK_PINS[2]) == LOW) && (digitalRead(JOYSTICK_PINS[3]) == LOW)) && !((digitalRead(JOYSTICK_PINS[0]) == LOW) && (digitalRead(JOYSTICK_PINS[1]) == LOW))) { // Detect if only LEFT and RIGHT are sent as pressed at the same time to check 3 buttons mode discarding simultaneous UP and DOWN which indicates 6 button mode
      // Remember we found a 3 buttons compatible controller
      JOYSTICK_3BUTTON = true;
      // Now we can try to read the buttons sequence: (B pin) A - (C pin) Start
      buttonsProcess(4, JOYSTICK_TOTALPINS, 2, JOYSTICK_PINS, JOYSTICK_STATUS, JOYSTICK_PRESSCOUNT, JOYSTICK_MAP_PS2, JOYSTICK_MAP_USB, JOYSTICK_MAP_ACTIVE, JOYSTICK_INDEX);
      // Break the loop to continue processing 6 buttons detection
      break;
    } else { // Standard common processing  // Also may be: State 0 or 1 when previous digitalWrite(JOYSTICK_SELECT, HIGH)
      // Looks processing logic does not achieve this point never when joystick is in 3 button mode - PENDING CHECK ///////
      // Now we can try to read the buttons sequence: UP - DOWN - LEFT - RIGHT - B - C
      buttonsProcess(0, JOYSTICK_TOTALPINS, 0, JOYSTICK_PINS, JOYSTICK_STATUS, JOYSTICK_PRESSCOUNT, JOYSTICK_MAP_PS2, JOYSTICK_MAP_USB, JOYSTICK_MAP_ACTIVE, JOYSTICK_INDEX);
    }
    
  }
  
  // Specific Sega 6 buttons processing  // From state 2
  if (JOYSTICK_3BUTTON == true) {

    digitalWrite(JOYSTICK_SELECT, HIGH);  // State 3
    // Now we can try to read the buttons sequence: UP - DOWN - LEFT - RIGHT - B - C
    buttonsProcess(0, JOYSTICK_TOTALPINS, 0, JOYSTICK_PINS, JOYSTICK_STATUS, JOYSTICK_PRESSCOUNT, JOYSTICK_MAP_PS2, JOYSTICK_MAP_USB, JOYSTICK_MAP_ACTIVE, JOYSTICK_INDEX);
    
    digitalWrite(JOYSTICK_SELECT, LOW);  // State 4
    
    if ((digitalRead(JOYSTICK_PINS[0]) == LOW) && (digitalRead(JOYSTICK_PINS[1]) == LOW)) { // Detect if UP and DOWN are sent as pressed at the same time to check 6 buttons mode
      
      digitalWrite(JOYSTICK_SELECT, HIGH); // State 5
      
      // Now we can try to read the buttons sequence: (UP pin) Z - (DOWN pin) Y - (LEFT pin) X - (RIGHT pin) Mode
      buttonsProcess(0, 4, 8, JOYSTICK_PINS, JOYSTICK_STATUS, JOYSTICK_PRESSCOUNT, JOYSTICK_MAP_PS2, JOYSTICK_MAP_USB, JOYSTICK_MAP_ACTIVE, JOYSTICK_INDEX);
      
      digitalWrite(JOYSTICK_SELECT, LOW);  // State 6
      digitalWrite(JOYSTICK_SELECT, HIGH); // State 7
      delay(2); // Just to ensure the internal counter of a Sega 6 button controller is reset to zero cause we use a higher timeout than 1.5ms
      
    }
  
  }
  
  */
  
}



void buttonsProcess(const uint8_t FROM_JOYSTICK_PIN, const uint8_t TO_JOYSTICK_PIN, const uint8_t DIFF_JOYSTICK_MAP, const uint8_t JOYSTICK_PINS[6], bool JOYSTICK_STATUS[12], uint8_t JOYSTICK_PRESSCOUNT[12], const char JOYSTICK_MAP_PS2[4][12], const char JOYSTICK_MAP_USB[4][12], uint8_t JOYSTICK_MAP_ACTIVE, uint8_t JOYSTICK_INDEX) {
  
  if (DB9_CYCLES_WAITCOUNT > 0) { DB9_CYCLES_WAITCOUNT--; }
  
  for ( uint8_t i = FROM_JOYSTICK_PIN; i < TO_JOYSTICK_PIN; i++ ) {
    if (digitalRead(JOYSTICK_PINS[i]) == LOW) {  // Button is pressed
      
      // *********************************************************************************************************************************
      // Check if OLED screen is available
      if (OLED_CURRENT_MODE > -1) {
        // Check special case for OLED keyboard menu button press (START)
        if (((i+DIFF_JOYSTICK_MAP) == 7) && (JOYSTICK_STATUS[7] == false)) {  // digitalRead(JOYSTICK_PINS[4]) == LOW  // (JOYSTICK_3BUTTON == true) ///////
          if (OLED_BUTTON_COUNTER[JOYSTICK_INDEX] < 8000) {  // Use a counter to differentiate between long press and short press
            OLED_BUTTON_COUNTER[JOYSTICK_INDEX]++;
          } else if (OLED_CURRENT_MODE != 2) {  // Activate the OLED on-screen keyboard mode only when a long press was sent and OLED mode was not already in on-screen mode
            JOYSTICK_STATUS[7] = true;  // The button status map is used as flag to detect that a long press was done
            OLED_BUTTON_COUNTER[JOYSTICK_INDEX] = 0;
            OLEDmanage(2);
          }
          continue;  // Avoid the START button will be processed
        }
        // Check if the OLED on-screen keyboard is waiting
        if ((OLED_CURRENT_MODE == 2) && ((i+DIFF_JOYSTICK_MAP) != 7)) { // Check if OLED on-screen keyboard is active and SELECT is not being pressed
          // Avoid the processing of button scancodes but allow the press or release logic management
          if (!((digitalRead(JOYSTICK_PINS[2]) == LOW) && (digitalRead(JOYSTICK_PINS[3]) == LOW))) { // Avoid false readings OLED generates by sending SELECT pulses while refreshing
            OLEDbutton((i+DIFF_JOYSTICK_MAP), true, JOYSTICK_STATUS);
            //OLEDbutton(((i+DIFF_JOYSTICK_MAP) * (-1)), JOYSTICK_STATUS); // Release ///////
          }
          continue;
        }
      }
      // *********************************************************************************************************************************
      
      if (JOYSTICK_PRESSCOUNT[i+DIFF_JOYSTICK_MAP] < DB9_CYCLES_PRESS) {  // Debounce margin code
        
        JOYSTICK_PRESSCOUNT[i+DIFF_JOYSTICK_MAP]++;
        
      } else if ((JOYSTICK_STATUS[i+DIFF_JOYSTICK_MAP] == false) && (DB9_CYCLES_WAITCOUNT == 0)) {  // Data sending margin code

        JOYSTICK_STATUS[i+DIFF_JOYSTICK_MAP] = true;
        DB9_CYCLES_WAITCOUNT = DB9_CYCLES_WAIT;

        // Check connection protocol and send the button key press according to the proper map
        if (USB_AVAILABLE == false) {
          if (JOYSTICK_MAP_ACTIVE == 0) { // Only needed for ESPectrum keycodes ///////
            keyboard.write(0xe2);
            keyboard.keyboard_press(JOYSTICK_MAP_PS2[JOYSTICK_MAP_ACTIVE][i+DIFF_JOYSTICK_MAP]);
          } else {
            sendPS2keypress(JOYSTICK_MAP_PS2[JOYSTICK_MAP_ACTIVE][i+DIFF_JOYSTICK_MAP]);
          }
        } else {
          Keyboard.press(JOYSTICK_MAP_USB[JOYSTICK_MAP_ACTIVE][i+DIFF_JOYSTICK_MAP]);
        }
        
        #ifdef PROMICRO_BOARD
        if (JOYSTICK_INDEX == 0) { digitalWrite(LED_ONBOARD_1, LOW); }
        if (JOYSTICK_INDEX == 1) { digitalWrite(LED_ONBOARD_2, LOW); }
        #endif
      }
    } else {  // Button is not pressed

      // *********************************************************************************************************************************
      // Check if OLED screen is available
      if (OLED_CURRENT_MODE > -1) {
        // Check special case for OLED keyboard menu button release (START)
        if (((i+DIFF_JOYSTICK_MAP) == 7) && (OLED_BUTTON_COUNTER[JOYSTICK_INDEX] > 0)) {  // (JOYSTICK_3BUTTON == true) ///////
  
          OLED_BUTTON_COUNTER[JOYSTICK_INDEX] = 0;
  
          if (OLED_CURRENT_MODE == 2) {  // Check if OLED on-screen keyboard is active
            if (JOYSTICK_STATUS[7] == true) {  // Check if joystick START button was pressed and released before exiting the OLED on-screen keyboard mode
              JOYSTICK_STATUS[7] = false;  // Restore the button status map used as flag to detect that a long press was done
            } else {  // If joystick START button was released once OLED on-screen was activated then exit from the OLED on-screen keyboard mode
              OLEDmanage(2);
            }
          } else {  // Manage joystick START button keypress on button release when a short press was sent and OLED on-screen keyboard is not active
            JOYSTICK_STATUS[7] = true;
            DB9_CYCLES_WAITCOUNT = DB9_CYCLES_WAIT;
    
            // Check connection protocol and send the START key press according to the proper map
            if (USB_AVAILABLE == false) {
              if (JOYSTICK_MAP_ACTIVE == 0) { // Only needed for ESPectrum keycodes ///////
                keyboard.write(0xe2);
                keyboard.keyboard_press(JOYSTICK_MAP_PS2[JOYSTICK_MAP_ACTIVE][7]);
              } else {
                sendPS2keypress(JOYSTICK_MAP_PS2[JOYSTICK_MAP_ACTIVE][7]);
              }
              
            } else {
              Keyboard.press(JOYSTICK_MAP_USB[JOYSTICK_MAP_ACTIVE][7]);
            }
            delay(75); // Wait an average minimum time a keypress can have to avoid some programs ignore the keypress because a fast keypress and inmediate keyrelease
          }
          continue;  // Avoid the START button will be processed
        }
        // Check if the OLED on-screen keyboard is waiting
        if ((OLED_CURRENT_MODE == 2) && ((i+DIFF_JOYSTICK_MAP) != 7)) { // Check if OLED on-screen keyboard is active and START is not being released
          // Avoid the processing of button scancodes but allow the press or release logic management
          OLEDbutton((i+DIFF_JOYSTICK_MAP), false, JOYSTICK_STATUS); ///////
          continue;
        }
      }
      // *********************************************************************************************************************************
      
      if ((JOYSTICK_PRESSCOUNT[i+DIFF_JOYSTICK_MAP] > 0) && (DB9_CYCLES_WAITCOUNT == 0)) {  // Data sending margin code
        
        JOYSTICK_PRESSCOUNT[i+DIFF_JOYSTICK_MAP]--;
        /* // Error management - duplicate keycode sending to ensure key release data will reach the host
        if ((JOYSTICK_STATUS[i+DIFF_JOYSTICK_MAP] == 1) && (JOYSTICK_PRESSCOUNT[i+DIFF_JOYSTICK_MAP] < 3)) {
          DB9_CYCLES_WAITCOUNT = DB9_CYCLES_WAIT;
          while (sendPS2keyrelease(JOYSTICK_MAP_PS2[JOYSTICK_MAP_ACTIVE][i+DIFF_JOYSTICK_MAP]) != 0) {}
          //sendPS2keyrelease(JOYSTICK_MAP_PS2[JOYSTICK_MAP_ACTIVE][i+2DIFF_JOYSTICK_MAP]);
        }
        */
        
      } else if ((JOYSTICK_STATUS[i+DIFF_JOYSTICK_MAP] == true) && (DB9_CYCLES_WAITCOUNT == 0)) {  // Debounce margin code
        
        JOYSTICK_STATUS[i+DIFF_JOYSTICK_MAP] = false;
        DB9_CYCLES_WAITCOUNT = DB9_CYCLES_WAIT;

        // Check connection protocol and send the key release according to the proper map
        if (USB_AVAILABLE == false) { // Only needed for ESPectrum keycodes ///////
          if (JOYSTICK_MAP_ACTIVE == 0) {
            keyboard.write(0xe2);
            keyboard.keyboard_release(JOYSTICK_MAP_PS2[JOYSTICK_MAP_ACTIVE][i+DIFF_JOYSTICK_MAP]);
          } else {
            sendPS2keyrelease(JOYSTICK_MAP_PS2[JOYSTICK_MAP_ACTIVE][i+DIFF_JOYSTICK_MAP]);
          }
        } else {
          Keyboard.release(JOYSTICK_MAP_USB[JOYSTICK_MAP_ACTIVE][i+DIFF_JOYSTICK_MAP]);
        }
        
        #ifdef PROMICRO_BOARD
        if (JOYSTICK_INDEX == 0) { digitalWrite(LED_ONBOARD_1, HIGH); }
        if (JOYSTICK_INDEX == 1) { digitalWrite(LED_ONBOARD_2, HIGH); }
        #endif
      }
    }
  }
  
}







uint8_t sendPS2keypress(char keycode) {
  /*  // PS/2 Error management - keycode to be sent
  lastkeycode = keycode;
  lastkeycodestatus = 1;
  */
  if (keycode == PS2dev::UP_ARROW || keycode == PS2dev::DOWN_ARROW || keycode == PS2dev::LEFT_ARROW || keycode == PS2dev::RIGHT_ARROW || keycode == PS2dev::RIGHT_ALT) {
    //while (digitalRead(PS2_PINS[0]) == LOW || digitalRead(PS2_PINS[1]) == LOW) {}  // Ensure clock is ready
    return keyboard.keyboard_press_special(keycode);
  } else {
    //while (digitalRead(PS2_PINS[0]) == LOW || digitalRead(PS2_PINS[1]) == LOW) {}  // Ensure clock is ready
    return keyboard.keyboard_press(keycode);
  }
}



uint8_t sendPS2keyrelease(char keycode) {
  /*  // PS/2 Error management - keycode to be sent
  lastkeycode = keycode;
  lastkeycodestatus = 0;
  */
  if (keycode == PS2dev::UP_ARROW || keycode == PS2dev::DOWN_ARROW || keycode == PS2dev::LEFT_ARROW || keycode == PS2dev::RIGHT_ARROW || keycode == PS2dev::RIGHT_ALT) {
    //while (digitalRead(PS2_PINS[0]) == LOW || digitalRead(PS2_PINS[1]) == LOW) {}  // Ensure clock is ready
    return keyboard.keyboard_release_special(keycode);
  } else {
    //while (digitalRead(PS2_PINS[0]) == LOW || digitalRead(PS2_PINS[1]) == LOW) {}  // Ensure clock is ready
    return keyboard.keyboard_release(keycode);
  }
}



/*  // PS/2 Error management - function code
uint8_t readPS2handle(unsigned char &leddata) {
  if (keyboard.keyboard_handle(&leddata) == 2) {
    if (lastkeycodestatus == 1) { sendPS2keypress(lastkeycode); }
    if (lastkeycodestatus == 0) { sendPS2keyrelease(lastkeycode); }
    #ifdef PROMICRO_BOARD
    //digitalWrite(LED_ONBOARD_2, LOW);
    //delay(2000);
    //digitalWrite(LED_ONBOARD_2, HIGH);
    #endif
  }
}
*/


  
  



void USBcheck(bool booting){
  if ((USB_AVAILABLE == false) && (UDADDR & _BV(ADDEN))) {
    // Recover the USB keyboard maps previously stored here
    DB9_1_MAP_ACTIVE = EEPROM.read(2); // Recover the previously stored active map index for DB9-1
    if (DB9_1_MAP_ACTIVE > (sizeof(DB9_1_MAP_USB) / sizeof(DB9_1_MAP_USB[0]))) { DB9_1_MAP_ACTIVE = 0; } // Ensure the value recovered is valid just in case it was not previously stored
    DB9_2_MAP_ACTIVE = EEPROM.read(3); // Recover the previously stored active map index for DB9-2
    if (DB9_2_MAP_ACTIVE > (sizeof(DB9_2_MAP_USB) / sizeof(DB9_2_MAP_USB[0]))) { DB9_2_MAP_ACTIVE = 0; } // Ensure the value recovered is valid just in case it was not previously stored

    Keyboard.begin(); // USB keyboard initialization (notice the K in uppercase)
    
    USB_AVAILABLE = true; // Update the global variable used as flag

    // Update the OLED screen only after the board boot process to avoid showing the default keyboard map on the OLED screen for a moment when the host wakes up the USB so late
    if (booting == false) {
      // Update the OLED screen
      OLEDmanage(1);
    }

    /*
    #ifdef PROMICRO_BOARD
    digitalWrite(LED_ONBOARD_2, LOW);
    #endif
    */
  }  
}







void OLEDcheck() {

  SoftwareI2C softwarei2c; // Wire replacement declaration
  
  bool OLEDfound = false;
  //OLED_CURRENT_MODE = -1;
  
  // Put the proper pin configurations for OLED screen when sharing pins with a joystick
  pinMode(DB9_2_SELECT, OUTPUT);  // Needed this for SELECT pin to avoid data collision caused by Sega 3 or 6 button controllers
  digitalWrite(DB9_2_SELECT, LOW);  // Needed this for SELECT pin to avoid data collision caused by Sega 3 or 6 button controllers

  softwarei2c.begin(OLED_PINS[1], OLED_PINS[0]);
  
  //for (unsigned char i = 1; i < 128; i++) {
  if (softwarei2c.beginTransmission(60)){ // 60 is the same as 0x3C in hex format which is the direction for the 0.96inch 128x64 OLED screen used
    OLEDfound = true;
  }
  softwarei2c.endTransmission();
  //}

  if (OLEDfound == true) {
    OLEDmanage(0);
  }
  
}



void OLEDmanage(const uint8_t MENU_MODE){  // 0 == init | 1 == status | 2 == keyboard change | 3 == refresh keyboard selection

  if ((OLED_CURRENT_MODE == -1) && (MENU_MODE > 0)) { return; } // Exit if OLED was not detected previously
  
  // Put the proper pin configurations for OLED screen when sharing pins with a joystick
  pinMode(DB9_2_SELECT, OUTPUT);  // Needed this for SELECT pin to avoid data collision caused by Sega 3 or 6 button controllers
  digitalWrite(DB9_2_SELECT, LOW);  // Needed this for SELECT pin to avoid data collision caused by Sega 3 or 6 button controllers - Recommended to be used before pinMode for certain chip models ???
  pinMode(OLED_PINS[0], OUTPUT);
  pinMode(OLED_PINS[1], OUTPUT);

  //U8G2.clearBuffer();
  //U8G2.clearDisplay();

  

  if ((MENU_MODE == 2) && (OLED_CURRENT_MODE == 2)) {
    OLED_CURRENT_MODE = 1;
  } else if (MENU_MODE != 3) {
    OLED_CURRENT_MODE = MENU_MODE;
  }

  if (OLED_CURRENT_MODE == 0) { // Init
    U8G2.setI2CAddress(0x78); // 0x78 // 0x3C
    U8G2.setBusClock(400000); ///////
    
    U8G2.begin();
    //U8G2.setPowerSave(0);
    U8G2.setContrast(100); // 0 - 255
    U8G2.setFlipMode(0); // No rotation
    //U8G2.setFont(u8g2_font_amstrad_cpc_extended_8f);
    U8G2.setFont(u8g2_font_chroma48medium8_8r);
    //U8G2.setFont(u8g2_font_tinyunicode_tf);
    //U8G2.setFont(u8g2_font_6x12_t_symbols);
    
    // Draw screen
    U8G2.firstPage(); ///////
    do { ///////
      U8G2.drawButtonUTF8(64, 32, U8G2_BTN_HCENTER|U8G2_BTN_BW0, 62,  0,  0, "CONNECTING..." );
    } while ( U8G2.nextPage() ); ///////
    //U8G2.sendBuffer();
    //U8G2.updateDisplay();
  }



  if (OLED_CURRENT_MODE == 1) { // Status
    U8G2.firstPage();
    do {
      // Connection info
      if (USB_AVAILABLE == true) {
        U8G2.drawButtonUTF8(64, 6, U8G2_BTN_HCENTER|U8G2_BTN_BW0, 128,  0,  0, "USB MODE" );
      } else {
        U8G2.drawButtonUTF8(64, 6, U8G2_BTN_HCENTER|U8G2_BTN_BW0, 128,  0,  0, "PS/2 MODE" );
      }
      // Joystick 1 buttons info
      if (JOYSTICK_BUTTONS[0] == 2) { U8G2.drawButtonUTF8(33, 16, U8G2_BTN_HCENTER|U8G2_BTN_BW0/*|U8G2_BTN_INV*/, 62,  0,  0, "1 ATARI" ); }
      if (JOYSTICK_BUTTONS[0] == 3) { U8G2.drawButtonUTF8(33, 16, U8G2_BTN_HCENTER|U8G2_BTN_BW0/*|U8G2_BTN_INV*/, 62,  0,  0, "1 SEGA3" ); }
      if (JOYSTICK_BUTTONS[0] == 6) { U8G2.drawButtonUTF8(33, 16, U8G2_BTN_HCENTER|U8G2_BTN_BW0/*|U8G2_BTN_INV*/, 62,  0,  0, "1 SEGA6" ); }
      // Joystick 2 buttons info
      if (JOYSTICK_BUTTONS[1] == 2) { U8G2.drawButtonUTF8(96, 16, U8G2_BTN_HCENTER|U8G2_BTN_BW0/*|U8G2_BTN_INV*/, 62,  0,  0, "2 ATARI" ); }
      if (JOYSTICK_BUTTONS[1] == 3) { U8G2.drawButtonUTF8(96, 16, U8G2_BTN_HCENTER|U8G2_BTN_BW0/*|U8G2_BTN_INV*/, 62,  0,  0, "2 SEGA3" ); }
      if (JOYSTICK_BUTTONS[1] == 6) { U8G2.drawButtonUTF8(96, 16, U8G2_BTN_HCENTER|U8G2_BTN_BW0/*|U8G2_BTN_INV*/, 62,  0,  0, "2 SEGA6" ); }
      // Draw some divider lines
      U8G2.drawLine(64, 8, 64, 64); // Middle
      U8G2.drawLine(0, 8, 0, 64); // Left
      U8G2.drawLine(127, 8, 127, 64); // Right
      U8G2.drawLine(0, 8, 127, 8); // Horizontal
      //U8G2.drawBox(0,8,64,56);
      //U8G2.drawBox(64,8,128,64);

      // Keycodes to be sent for each button info
      //const uint8_t buttons_coords[12][2] = {{32,24},{32,40},{11,32},{53,32},{11,48},{32,48},{53,48},{11,56},{32,56},{53,56},{16,64},{48,64}}; // Coordinates for buttons to be drawn on the OLED screen in order 1 2 3 4 - 5 6 7 - 8 9 10 - 11 12
      const uint8_t buttons_coords[12][2] = {{32,24},{32,40},{11,32},{53,32},{32,48},{53,48},{11,48},{16,64},{53,56},{32,56},{11,56},{48,64}}; // Coordinates for buttons to be drawn on the OLED screen in order 1 2 3 4 - 6 7 5 - 11 10 9 - 8 12
      uint8_t oy; // OLED vertical coordinate to draw
      uint8_t ox; // OLED horizontal coordinate to draw
      char obtn[3]; // OLED button keycode message
      // Rescue keycodes and draw the needed buttons according to joysticks connected and scancodes for the connection protocol in use
      if (USB_AVAILABLE == false) {
        // Avoid show the keycodes info for the custom PS/2 maps 0 showing a custom message instead (ESPectrum emulator special scancodes maps)
        if (DB9_1_MAP_ACTIVE == 0) {
          U8G2.drawButtonUTF8(32, 44, U8G2_BTN_HCENTER|U8G2_BTN_BW0, 20,  0,  0, "CUSTOM");
        } else {
          // Draw the DB9_1 joytick buttons equivalence on the OLED screen according to the current active PS/2 map
          for ( uint8_t mx = 0; mx < (sizeof(DB9_1_MAP_PS2[0])); mx++ ) { // Loop to check buttons in current DB9_1 PS/2 map which should have the same size as the buttons available from previous array (12)
            if ((JOYSTICK_BUTTONS[0] == 2) && (mx > 3) && (mx != 4) && (mx != 7)) { continue; } // Discard not used keycodes info for Atari joysticks
            if ((JOYSTICK_BUTTONS[0] == 3) && (mx > 7)) { continue; } // Discard not used keycodes info for Sega 3 buttons joysticks
            
            ox = buttons_coords[mx][0]; // Obtain OLED "X" coordinate according to the const array
            oy = buttons_coords[mx][1]; // Obtain OLED "Y" coordinate according to the const array
            for ( uint8_t ky = 0; ky < (sizeof(OLED_KEYBOARD_MAP_PS2) / (sizeof(OLED_KEYBOARD_MAP_PS2[0]))); ky++ ) { // Check OLED keyboard map rows
              for ( uint8_t kx = 0; kx < (sizeof(OLED_KEYBOARD_MAP_PS2[0])); kx++ ) { // Check OLED keyboard map cols
                if (DB9_1_MAP_PS2[DB9_1_MAP_ACTIVE][mx] == OLED_KEYBOARD_MAP_PS2[ky][kx]) { // Check if keycode in current loop position is the same in the OLED on-screen keyboard map position checked
                  obtn[0] = OLED_KEYBOARD_BUTTONS[ky][kx][0];
                  obtn[1] = OLED_KEYBOARD_BUTTONS[ky][kx][1];
                  obtn[2] = '\0'; // End of string is needed to allow the drawButtonUTF8 function detect a proper ending and calculate the center position
                  U8G2.drawButtonUTF8(ox, oy, U8G2_BTN_HCENTER|U8G2_BTN_BW0, 20,  0,  0, obtn); // Draw the button description at the proper coordinate position on the OLED screen
                }
              }
            }
          }
        }
        // Avoid show the keycodes info for the custom PS/2 maps 0 showing a custom message instead (ESPectrum emulator special scancodes maps)
        if (DB9_2_MAP_ACTIVE == 0) {
          U8G2.drawButtonUTF8(96, 44, U8G2_BTN_HCENTER|U8G2_BTN_BW0, 20,  0,  0, "CUSTOM");
        } else {
          // Draw the DB9_2 joytick buttons equivalence on the OLED screen according to the current active PS/2 map
          for ( uint8_t mx = 0; mx < (sizeof(DB9_2_MAP_PS2[0])); mx++ ) { // Loop to check buttons in current DB9_1 PS/2 map which should have the same size as the buttons available from previous array (12)
            if ((JOYSTICK_BUTTONS[1] == 2) && (mx > 3) && (mx != 4) && (mx != 7)) { continue; } // Discard not used keycodes info for Atari joysticks
            if ((JOYSTICK_BUTTONS[1] == 3) && (mx > 7)) { continue; } // Discard not used keycodes info for Sega 3 buttons joysticks
            
            ox = (buttons_coords[mx][0] + 64); // Obtain OLED "X" coordinate according to the const array adding the proper pixels margin for the "Joy 2" OLED drawing coordinates
            oy = buttons_coords[mx][1]; // Obtain OLED "Y" coordinate according to the const array
            for ( uint8_t ky = 0; ky < (sizeof(OLED_KEYBOARD_MAP_PS2) / (sizeof(OLED_KEYBOARD_MAP_USB[0]))); ky++ ) { // Check OLED keyboard map rows
              for ( uint8_t kx = 0; kx < (sizeof(OLED_KEYBOARD_MAP_PS2[0])); kx++ ) { // Check OLED keyboard map cols
                if (DB9_2_MAP_PS2[DB9_2_MAP_ACTIVE][mx] == OLED_KEYBOARD_MAP_PS2[ky][kx]) { // Check if keycode in current loop position is the same in the OLED on-screen keyboard map position checked
                  obtn[0] = OLED_KEYBOARD_BUTTONS[ky][kx][0];
                  obtn[1] = OLED_KEYBOARD_BUTTONS[ky][kx][1];
                  obtn[2] = '\0'; // End of string is needed to allow the drawButtonUTF8 function detect a proper ending and calculate the center position
                  U8G2.drawButtonUTF8(ox, oy, U8G2_BTN_HCENTER|U8G2_BTN_BW0, 20,  0,  0, obtn); // Draw the button description at the proper coordinate position on the OLED screen
                }
              }
            }
          }
        }
      } else {
        // Draw the DB9_1 joytick buttons equivalence on the OLED screen according to the current active USB map
        for ( uint8_t mx = 0; mx < (sizeof(DB9_1_MAP_USB[0])); mx++ ) { // Loop to check buttons in current DB9_1 USB map which should have the same size as the buttons available from previous array (12)
          if ((JOYSTICK_BUTTONS[0] == 2) && (mx > 3) && (mx != 4) && (mx != 7)) { continue; } // Discard not used keycodes info for Atari joysticks
          if ((JOYSTICK_BUTTONS[0] == 3) && (mx > 7)) { continue; } // Discard not used keycodes info for Sega 3 buttons joysticks
          
          ox = buttons_coords[mx][0]; // Obtain OLED "X" coordinate according to the const array
          oy = buttons_coords[mx][1]; // Obtain OLED "Y" coordinate according to the const array
          for ( uint8_t ky = 0; ky < (sizeof(OLED_KEYBOARD_MAP_USB) / (sizeof(OLED_KEYBOARD_MAP_USB[0]))); ky++ ) { // Check OLED keyboard map rows
            for ( uint8_t kx = 0; kx < (sizeof(OLED_KEYBOARD_MAP_USB[0])); kx++ ) { // Check OLED keyboard map cols
              if (DB9_1_MAP_USB[DB9_1_MAP_ACTIVE][mx] == OLED_KEYBOARD_MAP_USB[ky][kx]) { // Check if keycode in current loop position is the same in the OLED on-screen keyboard map position checked
                obtn[0] = OLED_KEYBOARD_BUTTONS[ky][kx][0];
                obtn[1] = OLED_KEYBOARD_BUTTONS[ky][kx][1];
                obtn[2] = '\0'; // End of string is needed to allow the drawButtonUTF8 function detect a proper ending and calculate the center position
                U8G2.drawButtonUTF8(ox, oy, U8G2_BTN_HCENTER|U8G2_BTN_BW0, 20,  0,  0, obtn); // Draw the button description at the proper coordinate position on the OLED screen
              }
            }
          }
        }
        // Draw the DB9_2 joytick buttons equivalence on the OLED screen according to the current active USB map
        for ( uint8_t mx = 0; mx < (sizeof(DB9_2_MAP_USB[0])); mx++ ) { // Loop to check buttons in current DB9_1 USB map which should have the same size as the buttons available from previous array (12)
          if ((JOYSTICK_BUTTONS[1] == 2) && (mx > 3) && (mx != 4) && (mx != 7)) { continue; } // Discard not used keycodes info for Atari joysticks
          if ((JOYSTICK_BUTTONS[1] == 3) && (mx > 7)) { continue; } // Discard not used keycodes info for Sega 3 buttons joysticks
          
          ox = (buttons_coords[mx][0] + 64); // Obtain OLED "X" coordinate according to the const array
          oy = buttons_coords[mx][1]; // Obtain OLED "Y" coordinate according to the const array
          for ( uint8_t ky = 0; ky < (sizeof(OLED_KEYBOARD_MAP_USB) / (sizeof(OLED_KEYBOARD_MAP_USB[0]))); ky++ ) { // Check OLED keyboard map rows
            for ( uint8_t kx = 0; kx < (sizeof(OLED_KEYBOARD_MAP_USB[0])); kx++ ) { // Check OLED keyboard map cols
              if (DB9_2_MAP_USB[DB9_2_MAP_ACTIVE][mx] == OLED_KEYBOARD_MAP_USB[ky][kx]) { // Check if keycode in current loop position is the same in the OLED on-screen keyboard map position checked
                obtn[0] = OLED_KEYBOARD_BUTTONS[ky][kx][0];
                obtn[1] = OLED_KEYBOARD_BUTTONS[ky][kx][1];
                obtn[2] = '\0'; // End of string is needed to allow the drawButtonUTF8 function detect a proper ending and calculate the center position
                U8G2.drawButtonUTF8(ox, oy, U8G2_BTN_HCENTER|U8G2_BTN_BW0, 20,  0,  0, obtn); // Draw the button description at the proper coordinate position on the OLED screen
              }
            }
          }
        }
      }

      /*
      if (DB9_1_MAP_ACTIVE == 0) {
          
          //U8G2.drawStr(0,24,"CURSORS");
          //U8G2.drawStr(0,32,"Enter");
          //U8G2.drawStr(0,40,"AltGr");
          //U8G2.drawStr(0,48,"Esc");
          //U8G2.drawStr(0,56,"F1");
          //U8G2.drawStr(0,64,"Z Y X M");
          
          U8G2.drawButtonUTF8(33, 24, U8G2_BTN_HCENTER|U8G2_BTN_BW0, 62,  0,  0, "SPECIAL" );
          U8G2.drawButtonUTF8(33, 32, U8G2_BTN_HCENTER|U8G2_BTN_BW0, 62,  0,  0, "Enter  " );
          U8G2.drawButtonUTF8(33, 40, U8G2_BTN_HCENTER|U8G2_BTN_BW0, 62,  0,  0, "AltGr  " );
          U8G2.drawButtonUTF8(33, 48, U8G2_BTN_HCENTER|U8G2_BTN_BW0, 62,  0,  0, "Esc    " );
          U8G2.drawButtonUTF8(33, 56, U8G2_BTN_HCENTER|U8G2_BTN_BW0, 62,  0,  0, "F1     " );
          U8G2.drawButtonUTF8(33, 64, U8G2_BTN_HCENTER|U8G2_BTN_BW0, 62,  0,  0, "Z Y X M" );
      }
      if (DB9_1_MAP_ACTIVE == 1) {
          U8G2.drawButtonUTF8(33, 24, U8G2_BTN_HCENTER|U8G2_BTN_BW0, 62,  0,  0, "Q A O P" );
          U8G2.drawButtonUTF8(33, 32, U8G2_BTN_HCENTER|U8G2_BTN_BW0, 62,  0,  0, "Enter  " );
          U8G2.drawButtonUTF8(33, 40, U8G2_BTN_HCENTER|U8G2_BTN_BW0, 62,  0,  0, "M      " );
          U8G2.drawButtonUTF8(33, 48, U8G2_BTN_HCENTER|U8G2_BTN_BW0, 62,  0,  0, "Esc    " );
          U8G2.drawButtonUTF8(33, 56, U8G2_BTN_HCENTER|U8G2_BTN_BW0, 62,  0,  0, "F1     " );
          U8G2.drawButtonUTF8(33, 64, U8G2_BTN_HCENTER|U8G2_BTN_BW0, 62,  0,  0, "Z Y X M" );
      }
      if (DB9_1_MAP_ACTIVE == 2) {
          U8G2.drawButtonUTF8(33, 24, U8G2_BTN_HCENTER|U8G2_BTN_BW0, 62,  0,  0, "CURSORS" );
          U8G2.drawButtonUTF8(33, 32, U8G2_BTN_HCENTER|U8G2_BTN_BW0, 62,  0,  0, "Enter  " );
          U8G2.drawButtonUTF8(33, 40, U8G2_BTN_HCENTER|U8G2_BTN_BW0, 62,  0,  0, "0      " );
          U8G2.drawButtonUTF8(33, 48, U8G2_BTN_HCENTER|U8G2_BTN_BW0, 62,  0,  0, "Esc    " );
          U8G2.drawButtonUTF8(33, 56, U8G2_BTN_HCENTER|U8G2_BTN_BW0, 62,  0,  0, "F5     " );
          U8G2.drawButtonUTF8(33, 64, U8G2_BTN_HCENTER|U8G2_BTN_BW0, 62,  0,  0, "Z Y X M" );
      }
      if (DB9_1_MAP_ACTIVE == 3) {
          U8G2.drawButtonUTF8(33, 24, U8G2_BTN_HCENTER|U8G2_BTN_BW0, 62,  0,  0, "7 6 5 8" );
          U8G2.drawButtonUTF8(33, 32, U8G2_BTN_HCENTER|U8G2_BTN_BW0, 62,  0,  0, "Enter  " );
          U8G2.drawButtonUTF8(33, 40, U8G2_BTN_HCENTER|U8G2_BTN_BW0, 62,  0,  0, "0      " );
          U8G2.drawButtonUTF8(33, 48, U8G2_BTN_HCENTER|U8G2_BTN_BW0, 62,  0,  0, "Esc    " );
          U8G2.drawButtonUTF8(33, 56, U8G2_BTN_HCENTER|U8G2_BTN_BW0, 62,  0,  0, "F1     " );
          U8G2.drawButtonUTF8(33, 64, U8G2_BTN_HCENTER|U8G2_BTN_BW0, 62,  0,  0, "Z Y X M" );
      }
      if (DB9_1_MAP_ACTIVE == 4) {
          U8G2.drawButtonUTF8(33, 24, U8G2_BTN_HCENTER|U8G2_BTN_BW0, 62,  0,  0, "CURSORS" );
          U8G2.drawButtonUTF8(33, 32, U8G2_BTN_HCENTER|U8G2_BTN_BW0, 62,  0,  0, "Space  " );
          U8G2.drawButtonUTF8(33, 40, U8G2_BTN_HCENTER|U8G2_BTN_BW0, 62,  0,  0, "Space  " );
          U8G2.drawButtonUTF8(33, 48, U8G2_BTN_HCENTER|U8G2_BTN_BW0, 62,  0,  0, "Space  " );
          U8G2.drawButtonUTF8(33, 56, U8G2_BTN_HCENTER|U8G2_BTN_BW0, 62,  0,  0, "Space  " );
          U8G2.drawButtonUTF8(33, 64, U8G2_BTN_HCENTER|U8G2_BTN_BW0, 62,  0,  0, "Space  " );
      }
      
  
      if (DB9_2_MAP_ACTIVE == 0) {
          U8G2.drawButtonUTF8(96, 24, U8G2_BTN_HCENTER|U8G2_BTN_BW0, 62,  0,  0, "SPECIAL" );
          U8G2.drawButtonUTF8(96, 32, U8G2_BTN_HCENTER|U8G2_BTN_BW0, 62,  0,  0, "  Enter" );
          U8G2.drawButtonUTF8(96, 40, U8G2_BTN_HCENTER|U8G2_BTN_BW0, 62,  0,  0, "  AltGr" );
          U8G2.drawButtonUTF8(96, 48, U8G2_BTN_HCENTER|U8G2_BTN_BW0, 62,  0,  0, "    Esc" );
          U8G2.drawButtonUTF8(96, 56, U8G2_BTN_HCENTER|U8G2_BTN_BW0, 62,  0,  0, "     F1" );
          U8G2.drawButtonUTF8(96, 64, U8G2_BTN_HCENTER|U8G2_BTN_BW0, 62,  0,  0, "Z Y X M" );
      }
      if (DB9_2_MAP_ACTIVE == 1) {
          U8G2.drawButtonUTF8(96, 24, U8G2_BTN_HCENTER|U8G2_BTN_BW0, 62,  0,  0, "CURSORS" );
          U8G2.drawButtonUTF8(96, 32, U8G2_BTN_HCENTER|U8G2_BTN_BW0, 62,  0,  0, "  Enter" );
          U8G2.drawButtonUTF8(96, 40, U8G2_BTN_HCENTER|U8G2_BTN_BW0, 62,  0,  0, "  AltGr" );
          U8G2.drawButtonUTF8(96, 48, U8G2_BTN_HCENTER|U8G2_BTN_BW0, 62,  0,  0, "    Esc" );
          U8G2.drawButtonUTF8(96, 56, U8G2_BTN_HCENTER|U8G2_BTN_BW0, 62,  0,  0, "     F1" );
          U8G2.drawButtonUTF8(96, 64, U8G2_BTN_HCENTER|U8G2_BTN_BW0, 62,  0,  0, "Z Y X M" );
      }
      if (DB9_2_MAP_ACTIVE == 2) {
          U8G2.drawButtonUTF8(96, 24, U8G2_BTN_HCENTER|U8G2_BTN_BW0, 62,  0,  0, "CURSORS" );
          U8G2.drawButtonUTF8(96, 32, U8G2_BTN_HCENTER|U8G2_BTN_BW0, 62,  0,  0, "  Enter" );
          U8G2.drawButtonUTF8(96, 40, U8G2_BTN_HCENTER|U8G2_BTN_BW0, 62,  0,  0, "      0" );
          U8G2.drawButtonUTF8(96, 48, U8G2_BTN_HCENTER|U8G2_BTN_BW0, 62,  0,  0, "    Esc" );
          U8G2.drawButtonUTF8(96, 56, U8G2_BTN_HCENTER|U8G2_BTN_BW0, 62,  0,  0, "     F5" );
          U8G2.drawButtonUTF8(96, 64, U8G2_BTN_HCENTER|U8G2_BTN_BW0, 62,  0,  0, "Z Y X M" );
      }
      if (DB9_2_MAP_ACTIVE == 3) {
          U8G2.drawButtonUTF8(96, 24, U8G2_BTN_HCENTER|U8G2_BTN_BW0, 62,  0,  0, "7 6 5 8" );
          U8G2.drawButtonUTF8(96, 32, U8G2_BTN_HCENTER|U8G2_BTN_BW0, 62,  0,  0, "  Enter" );
          U8G2.drawButtonUTF8(96, 40, U8G2_BTN_HCENTER|U8G2_BTN_BW0, 62,  0,  0, "      0" );
          U8G2.drawButtonUTF8(96, 48, U8G2_BTN_HCENTER|U8G2_BTN_BW0, 62,  0,  0, "    Esc" );
          U8G2.drawButtonUTF8(96, 56, U8G2_BTN_HCENTER|U8G2_BTN_BW0, 62,  0,  0, "     F1" );
          U8G2.drawButtonUTF8(96, 64, U8G2_BTN_HCENTER|U8G2_BTN_BW0, 62,  0,  0, "Z Y X M" );
      }
      if (DB9_2_MAP_ACTIVE == 4) {
          U8G2.drawButtonUTF8(96, 24, U8G2_BTN_HCENTER|U8G2_BTN_BW0, 62,  0,  0, "CURSORS" );
          U8G2.drawButtonUTF8(96, 32, U8G2_BTN_HCENTER|U8G2_BTN_BW0, 62,  0,  0, "Space  " );
          U8G2.drawButtonUTF8(96, 40, U8G2_BTN_HCENTER|U8G2_BTN_BW0, 62,  0,  0, "Space  " );
          U8G2.drawButtonUTF8(96, 48, U8G2_BTN_HCENTER|U8G2_BTN_BW0, 62,  0,  0, "Space  " );
          U8G2.drawButtonUTF8(96, 56, U8G2_BTN_HCENTER|U8G2_BTN_BW0, 62,  0,  0, "Space  " );
          U8G2.drawButtonUTF8(96, 64, U8G2_BTN_HCENTER|U8G2_BTN_BW0, 62,  0,  0, "Space  " );
      }
      */
      
    } while ( U8G2.nextPage() );
    //U8G2.sendBuffer();
    //U8G2.updateDisplay();
  }


  // Draw the on screen keyboard
  if ((OLED_CURRENT_MODE == 2) && ((MENU_MODE == 2) || (MENU_MODE == 3))) { // Keyboard mode
    u8g2_uint_t U8G2_BORDER;
    const uint8_t Y_KEYSEL_FROM = ((OLED_KEYSEL_Y > 4) ? (OLED_KEYSEL_Y - 4) : (0)); // We have space to show only 5 rows of keys on the OLED screen
    //uint8_t y_counter;
    // Draw screen
    U8G2.firstPage();
    do {
      U8G2.drawButtonUTF8(64,  8, U8G2_BTN_HCENTER|U8G2_BTN_BW0, 62,  0,  0, "KEYBOARD MODE" );
      //U8G2.drawButtonUTF8(64, 24, U8G2_BTN_HCENTER|U8G2_BTN_BW0, 62,  0,  0, "1234567890" );
      //U8G2.drawButtonUTF8(64, 32, U8G2_BTN_HCENTER|U8G2_BTN_BW0, 62,  0,  0, "QWERTYUIOP" );
      //U8G2.drawButtonUTF8(64, 40, U8G2_BTN_HCENTER|U8G2_BTN_BW0, 62,  0,  0, "ASDFGHJKL " );
      //U8G2.drawButtonUTF8(64, 48, U8G2_BTN_HCENTER|U8G2_BTN_BW0, 62,  0,  0, "ZXCVBNM   " );
      //for ( uint8_t y = 0; y < (sizeof(OLED_KEYBOARD_BUTTONS) / sizeof(OLED_KEYBOARD_BUTTONS[0])); y++ ) { // Rows
      for ( uint8_t y = Y_KEYSEL_FROM; y < (Y_KEYSEL_FROM + 5); y++ ) { // Rows according to the 5 rows of keys on the OLED screen limit
        for ( uint8_t x = 0; x < (sizeof(OLED_KEYBOARD_BUTTONS[0]) / sizeof(OLED_KEYBOARD_BUTTONS[0][0])); x++ ) { // Cols
          if ((OLED_KEYSEL_Y == y) && (OLED_KEYSEL_X == x)) { U8G2_BORDER = U8G2_BTN_BW1; } else { U8G2_BORDER = U8G2_BTN_BW0; }
          U8G2.drawButtonUTF8(10 + (x * 12), 25 + ((y - Y_KEYSEL_FROM) * 9), U8G2_BTN_HCENTER|U8G2_BORDER, sizeof(OLED_KEYBOARD_BUTTONS[y][x]), 1, 1, OLED_KEYBOARD_BUTTONS[y][x]);
        }
      }

      /* // Flash effect
      if (OLED_KEYSEL_FLASH == false) {
        U8G2_BORDER = U8G2_BTN_BW0;
      } else {
        U8G2_BORDER = U8G2_BTN_BW1;
      }
      U8G2.drawButtonUTF8(11 + (OLED_KEYSEL_X * 12), 23 + (OLED_KEYSEL_Y * 9), U8G2_BTN_HCENTER|U8G2_BORDER, sizeof(OLED_KEYBOARD_BUTTONS[OLED_KEYSEL_Y][OLED_KEYSEL_X]),  0,  0, OLED_KEYBOARD_BUTTONS[OLED_KEYSEL_Y][OLED_KEYSEL_X]);
      */ ///////
      //U8G2.drawButtonUTF8(11 + (OLED_KEYSEL_X * 12), 23 + (OLED_KEYSEL_Y * 9), U8G2_BTN_HCENTER|U8G2_BTN_BW1, sizeof(OLED_KEYBOARD_BUTTONS[OLED_KEYSEL_Y][OLED_KEYSEL_X]),  0,  0, OLED_KEYBOARD_BUTTONS[OLED_KEYSEL_Y][OLED_KEYSEL_X]);
    } while ( U8G2.nextPage() );
    //OLED_KEYSEL_FLASH = !(OLED_KEYSEL_FLASH); // Flash effect
  }

  

  // Restore the proper pin configurations for joytick when sharing pins with the OLED screen
  pinMode(DB9_2_SELECT, OUTPUT);  // Needed this for SELECT pin to avoid data collision caused by Sega 3 or 6 button controllers
  pinMode(OLED_PINS[0], INPUT_PULLUP);
  pinMode(OLED_PINS[1], INPUT_PULLUP);
  
}



void OLEDbutton(uint8_t BUTTON_INDEX, const bool BUTTON_PRESS, bool JOYSTICK_STATUS[12]) {
  //if (OLED_KEYSEL_X >= ((sizeof(OLED_KEYBOARD_BUTTONS[0]) / sizeof(OLED_KEYBOARD_BUTTONS[0][0])) / 2)) { OLED_KEYSEL_X++; } else { OLED_KEYSEL_X--; }
  
  if (JOYSTICK_STATUS[BUTTON_INDEX] != BUTTON_PRESS) {
    JOYSTICK_STATUS[BUTTON_INDEX] = BUTTON_PRESS;
    
    if (BUTTON_PRESS == true) {
      if (BUTTON_INDEX > 3) {
        if (USB_AVAILABLE == false) {
          sendPS2keypress(OLED_KEYBOARD_MAP_PS2[OLED_KEYSEL_Y][OLED_KEYSEL_X]);
        } else {
          Keyboard.press(OLED_KEYBOARD_MAP_USB[OLED_KEYSEL_Y][OLED_KEYSEL_X]);
        }
      } else {
        do {
          // Check if an empty key was found in previous iteration while precessing up or down direction and try to move the focus to the nearest proper key in the new current row
          if ((OLED_KEYBOARD_BUTTONS[OLED_KEYSEL_Y][OLED_KEYSEL_X] == "") && (BUTTON_INDEX == 0 || BUTTON_INDEX == 1)) {
            if ((OLED_KEYSEL_X > (sizeof(OLED_KEYBOARD_BUTTONS[0]) / 2)) || (OLED_KEYSEL_X == 0)) { // If it is in a position higher than the middle or just in the left border
              BUTTON_INDEX = 3; // Force move in right direction
              continue; // Process the new direction change needed for the new row
            }
            if ((OLED_KEYSEL_X <= (sizeof(OLED_KEYBOARD_BUTTONS[0]) / 2)) || (OLED_KEYSEL_X == sizeof(OLED_KEYBOARD_BUTTONS[0]))) { // If it is in a position lower or equal than the middle or just in the right border
              BUTTON_INDEX = 2; // Force move in left direction
              continue; // Process the new direction change needed for the new row
            }
          }
          
          // Manage normal keypress to move the focus in the direction desired
          if (BUTTON_INDEX == 3) { // Right move
            OLED_KEYSEL_X++;
          }
          
          if (BUTTON_INDEX == 2) { // Left move
            OLED_KEYSEL_X--;
          }
          
          if (BUTTON_INDEX == 0) { // Up move
            OLED_KEYSEL_Y--;
          }
          
          if (BUTTON_INDEX == 1) { // Down move
            OLED_KEYSEL_Y++;
          }

          // Check if we are at the end of a col or row and move the focus to the next proper key
          if (OLED_KEYSEL_Y == (sizeof(OLED_KEYBOARD_BUTTONS) / sizeof(OLED_KEYBOARD_BUTTONS[0]))) { OLED_KEYSEL_Y = 0; }
          if (OLED_KEYSEL_Y < 0) { OLED_KEYSEL_Y = ((sizeof(OLED_KEYBOARD_BUTTONS) / sizeof(OLED_KEYBOARD_BUTTONS[0])) - 1); }
          if (OLED_KEYSEL_X == (sizeof(OLED_KEYBOARD_BUTTONS[0]) / sizeof(OLED_KEYBOARD_BUTTONS[0][0]))) { OLED_KEYSEL_X = 0; }
          if (OLED_KEYSEL_X < 0) { OLED_KEYSEL_X = ((sizeof(OLED_KEYBOARD_BUTTONS[0]) / sizeof(OLED_KEYBOARD_BUTTONS[0][0])) - 1); }
        } while (OLED_KEYBOARD_BUTTONS[OLED_KEYSEL_Y][OLED_KEYSEL_X] == ""); // Continue the direction movement processing until a valid key will be found

        OLEDmanage(3);
      }
    } else {
      if (BUTTON_INDEX > 3) {
        if (USB_AVAILABLE == false) {
          sendPS2keyrelease(OLED_KEYBOARD_MAP_PS2[OLED_KEYSEL_Y][OLED_KEYSEL_X]);
        } else {
          Keyboard.release(OLED_KEYBOARD_MAP_USB[OLED_KEYSEL_Y][OLED_KEYSEL_X]);
        }
      }
    }
    
  }
}







/*
 // ESPectrumSpecialScanCodes
    ESP_JOY1LEFT = 0x40,    // MY_COMPUTER
    ESP_JOY1RIGHT = 0x41,   // COMMA
    ESP_JOY1UP = 0x42,      // K
    ESP_JOY1DOWN = 0x43,    // I
    ESP_JOY1START = 0x44,   // O
    ESP_JOY1MODE = 0x45,    // ZERO
    ESP_JOY1A = 0x46,       // NINE
    ESP_JOY1B = 0x47,       // 
    ESP_JOY1C = 0x48,       // EMAIL
    ESP_JOY1X = 0x49,       // PERIOD
    ESP_JOY1Y = 0x4a,       // SLASH
    ESP_JOY1Z = 0x4b,       // L
    ESP_JOY2LEFT = 0x4c,    // SEMI_COLON
    ESP_JOY2RIGHT = 0x4d,   // NEXT_TRACK
    ESP_JOY2UP = 0x4e,      // MINUS
    ESP_JOY2DOWN = 0x4f,    // 
    ESP_JOY2START = 0x50,   // MEDIA_SELECT
    ESP_JOY2MODE = 0x51,    // 
    ESP_JOY2A = 0x52,       // TICK_MARK
    ESP_JOY2B = 0x53,       // 
    ESP_JOY2C = 0x54,       // OPEN_BRACKET
    ESP_JOY2X = 0x55,       // EQUAL
    ESP_JOY2Y = 0x56,       // 
    ESP_JOY2Z = 0x57        // 

 // DB9_1_MAP_PS2
    {PS2dev::ESP_JOY1UP, PS2dev::ESP_JOY1DOWN, PS2dev::ESP_JOY1LEFT, PS2dev::ESP_JOY1RIGHT, PS2dev::ESP_JOY1B, PS2dev::ESP_JOY1C, PS2dev::ESP_JOY1A, PS2dev::ESP_JOY1START, PS2dev::ESP_JOY1Z, PS2dev::ESP_JOY1Y, PS2dev::ESP_JOY1X, PS2dev::ESP_JOY1MODE},
    {0x42, 0x43, 0x40, 0x41, 0x47, 0x48, 0x46, 0x44, 0x4B, 0x4A, 0x49, 0x45},
  
 // DB9_2_MAP_PS2
    {PS2dev::ESP_JOY2UP, PS2dev::ESP_JOY2DOWN, PS2dev::ESP_JOY2LEFT, PS2dev::ESP_JOY2RIGHT, PS2dev::ESP_JOY2B, PS2dev::ESP_JOY2C, PS2dev::ESP_JOY2A, PS2dev::ESP_JOY2START, PS2dev::ESP_JOY2Z, PS2dev::ESP_JOY2Y, PS2dev::ESP_JOY2X, PS2dev::ESP_JOY2MODE},
    {0x4E, 0x4F, 0x4C, 0x4D, 0x53, 0x54, 0x52, 0x50, 0x57, 0x56, 0x55, 0x51},
    
 // Needed to send
    write(0xe2);
    //write(0xf0); // Only for release
    write(code); 
*/







/* // Pending ...

PS2dev mouse(PD2,PD3); // 3 data 2clock
o
char buttons[3] = {0,0,0};

int delta_x = 0;
int delta_y = 0;

//we start off not enabled
int enabled = 0;

//ack a host command
void ack() {
  while (mouse.write(0xFA));
}

void write_packet() {
  char overflowx =0;
  char overflowy =0;
  char data[3];
  int x,y;

  if (delta_x > 255) {
    overflowx =1;
    x=255;
  }
  if (delta_x < -255) {
    overflowx = 1;
    x=-255;
  }
  if (delta_y > 255) {
    overflowy =1;
    y=255;
  }
  if (delta_y < -255) {
    overflowy = 1;
    y=-255;
  }

  data[0] = ((overflowy & 1) << 7) |
    ( (overflowx & 1) << 6) |
    ( (((delta_y &0x100)>>8) & 1) << 5) |
    ( ( ((delta_x &0x100)>>8)& 1) << 4) |
    ( ( 1) << 3) |
    ( ( buttons[1] & 1) << 2) |
    ( ( buttons[2] & 1) << 1) |
    ( ( buttons[0] & 1) << 0) ;

  data[1] = delta_x & 0xff;
  data[2] = delta_y & 0xff;

  mouse.write(data[0]);
  mouse.write(data[1]);

  mouse.write(data[2]);

  delta_x = 0;
  delta_y = 0;
}

int mousecommand(int command) {
  unsigned char val;

  //This implements enough mouse commands to get by, most of them are
  //just acked without really doing anything

  switch (command) {
  case 0xFF: //reset
    ack();
    //the while loop lets us wait for the host to be ready
    while (mouse.write(0xAA)!=0);
    while (mouse.write(0x00)!=0);
    break;
  case 0xFE: //resend
    ack();
    break;
  case 0xF6: //set defaults
    //enter stream mode
    ack();
    break;
  case 0xF5:  //disable data reporting
    //FM
    ack();
    break;
  case 0xF4: //enable data reporting
    //FM
    enabled = 1;
    ack();
    break;
  case 0xF3: //set sample rate
    ack();
    mouse.read(&val); // for now drop the new rate on the floor
    //      Serial.println(val,HEX);
    ack();
    break;
  case 0xF2: //get device id
    ack();
    mouse.write(00);
    break;
  case 0xF0: //set remote mode
    ack();
    break;
  case 0xEE: //set wrap mode
    ack();
    break;
  case 0xEC: //reset wrap mode
    ack();
    break;
  case 0xEB: //read data
    ack();
    write_packet();
    break;
  case 0xEA: //set stream mode
    ack();
    break;
  case 0xE9: //status request
    ack();
    //      send_status();
    break;
  case 0xE8: //set resolution
    ack();
    mouse.read(&val);
    //    Serial.println(val,HEX);
    ack();
    break;
  case 0xE7: //set scaling 2:1
    ack();
    break;
  case 0xE6: //set scaling 1:1
    ack();
    break;
  }
}

int xcenter;
int ycenter;

int xsum = 0;
int ysum = 0;

void setup() {
  unsigned char val;

  // send the mouse start up
  mouse.write(0xAA);
  mouse.write(0x00);
}

void loop() {
  unsigned char c;
  if ((digitalRead(3) == LOW) || (digitalRead(2) == LOW)) {
    while (mouse.read(&c));
    mousecommand(c);
  }

  if (enabled) {
    // move the mouse diagonally
    delta_x = 1;
    delta_y = 1;
    write_packet();
  }
  delay(50);
}

*/
