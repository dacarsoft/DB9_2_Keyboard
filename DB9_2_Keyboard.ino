/*
Project:  DB9 Joysticks to PS/2 adapter firmware
Author:   David Carrión - 2023
PS2Dev library: https://github.com/Harvie/ps2dev
*/


//#include <ps2dev.h>
#include "ps2dev.h"

#define AVILLENA_BOARD
//#define BLUEPILL_BOARD

#ifdef BLUEPILL_BOARD
/*
Bluepill ATmega 32U4 board pinout:
Arduino 2 -> DB9 pin 1: Up
Arduino 3 -> DB9 pin 2: Down
Arduino 4 -> DB9 pin 3: Left
Arduino 5 -> DB9 pin 4: Right
Arduino + -> DB9 pin 5: VCC 5V
Arduino 6 -> DB9 pin 6: A / Primary fire
Arduino 7 -> DB9 pin 7: Selection signal (allows more buttons)
Arduino - -> DB9 pin 8: GND
Arduino 8 -> DB9 pin 9: SELECT / Secondary fire
*/
const uint8_t PS2_PINS[2] = { 1, 0 }; // clock 1 PD3, data 0 PD2  // clock PD5 24, data PD6

const uint8_t DB9_1_PINS[6] = { 2, 3, 4, 5, 6, 8 }; // PD1, PD0, PD4, PC6, PD7, PB4 - Exclude pin 7 for Sega controllers
const uint8_t DB9_1_SELECT = 7; // PE6

const uint8_t DB9_2_PINS[6] = { 10, 16, 14, 15, 18, 20 }; // PB6, PB2, PB3, PB1, PF7, PF5 - Exclude pin 7 for Sega controllers
const uint8_t DB9_2_SELECT = 19; // PF6

//const uint8_t LED_ONBOARD_1 = LED_BUILTIN_RX; // RXLED 17
//const uint8_t LED_ONBOARD_2 = LED_BUILTIN_TX; // TXLED 30
const uint8_t LED_ONBOARD_1 = 17; // RXLED 17
const uint8_t LED_ONBOARD_2 = 30; // TXLED 30
#endif

#ifdef AVILLENA_BOARD
/*
Bluepill ATmega 32U4 board pinout:
Arduino A0 -> DB9 pin 1: Up
Arduino A1 -> DB9 pin 2: Down
Arduino A2 -> DB9 pin 3: Left
Arduino A3 -> DB9 pin 4: Right
Arduino +  -> DB9 pin 5: VCC 5V
Arduino PB3-> DB9 pin 6: A / Primary fire
Arduino 7  -> DB9 pin 7: Selection signal (allows more buttons)
Arduino -  -> DB9 pin 8: GND
Arduino PB1-> DB9 pin 9: SELECT / Secondary fire
*/
const uint8_t PS2_PINS[2] = { 12, 30 }; // clock PD6 12, data PD5 TXLED

const uint8_t DB9_1_PINS[6] = { 18, 19, 20, 21, 14, 15 }; // A0, A1, A2, A3, PB3, PB1 - Exclude pin 7 for Sega controllers
const uint8_t DB9_1_SELECT = 7; // E6

const uint8_t DB9_2_PINS[6] = { 1, 0, 2, 3, 4, 6 }; // PD3, PD2, PD1, PD0, PD4, PD7 - Exclude pin 7 for Sega controllers
const uint8_t DB9_2_SELECT = 5; // PC6
#endif



PS2dev keyboard(PS2_PINS[0], PS2_PINS[1]);  // clock, data
char lastkeycode; // Keycode to be sent again when something fails
uint8_t lastkeycodestatus; // Keydown or keyrelease status for the last keycode sent
unsigned char keyboardleddata; // Needed for keyboard reading handle code



const uint8_t DB9_1_TOTALPINS = sizeof(DB9_1_PINS);
const char DB9_1_MAP[12][12] = {
{PS2dev::UP_ARROW, PS2dev::DOWN_ARROW, PS2dev::LEFT_ARROW, PS2dev::RIGHT_ARROW, PS2dev::ENTER, PS2dev::RIGHT_ALT, PS2dev::ESCAPE, PS2dev::F1, PS2dev::Z, PS2dev::Y, PS2dev::X, PS2dev::M},
{PS2dev::Q, PS2dev::A, PS2dev::O, PS2dev::P, PS2dev::ENTER, PS2dev::M, PS2dev::ESCAPE, PS2dev::F1, PS2dev::Z, PS2dev::Y, PS2dev::X, PS2dev::M},
{PS2dev::SEVEN, PS2dev::SIX, PS2dev::FIVE, PS2dev::EIGHT, PS2dev::ENTER, PS2dev::ZERO, PS2dev::ESCAPE, PS2dev::F1, PS2dev::Z, PS2dev::Y, PS2dev::X, PS2dev::M}
}; // Keycode maps for the first controller

uint8_t DB9_1_MAP_ACTIVE = 0;
uint8_t DB9_1_PRESSCOUNT[12] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
uint8_t DB9_1_STATUS[12] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };



const uint8_t DB9_2_TOTALPINS = sizeof(DB9_2_PINS);
const char DB9_2_MAP[12][12] = {
{PS2dev::Q, PS2dev::A, PS2dev::O, PS2dev::P, PS2dev::ENTER, PS2dev::M, PS2dev::ESCAPE, PS2dev::F1, PS2dev::Z, PS2dev::Y, PS2dev::X, PS2dev::M},
{PS2dev::UP_ARROW, PS2dev::DOWN_ARROW, PS2dev::LEFT_ARROW, PS2dev::RIGHT_ARROW, PS2dev::ENTER, PS2dev::RIGHT_ALT, PS2dev::ESCAPE, PS2dev::F1, PS2dev::Z, PS2dev::Y, PS2dev::X, PS2dev::M},
{PS2dev::SEVEN, PS2dev::SIX, PS2dev::FIVE, PS2dev::EIGHT, PS2dev::ENTER, PS2dev::ZERO, PS2dev::ESCAPE, PS2dev::F1, PS2dev::Z, PS2dev::Y, PS2dev::X, PS2dev::M}
}; // Keycode maps for the first controller

uint8_t DB9_2_MAP_ACTIVE = 0;
uint8_t DB9_2_PRESSCOUNT[12] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
uint8_t DB9_2_STATUS[12] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };



const uint8_t DB9_CYCLES_PRESS = 1; // DEBOUNCE CYCLES TO VALIDATE PRESS
const uint8_t DB9_CYCLES_WAIT = 3; // RELAX CYCLES NEEDED
uint8_t DB9_CYCLES_WAITCOUNT = 0; // RELAX CYCLES PENDING



//unsigned long sega_timer = 0; // Needed to track delay after SEGA checked its buttons



//unsigned long timecount = 0;



void setup() {
  //pinMode(PS2_PINS[0], INPUT);
  //pinMode(PS2_PINS[1], INPUT);
  //DDRD=0; // Help to disable UART to use pins PD3 and PD2 (1 and 0)
  
  keyboard.keyboard_init();

  #ifdef BLUEPILL_BOARD
  pinMode(LED_ONBOARD_1, OUTPUT); // Led 1 
  digitalWrite(LED_ONBOARD_1, HIGH);
  pinMode(LED_ONBOARD_2, OUTPUT); // Led 2 
  digitalWrite(LED_ONBOARD_2, HIGH);
  #endif

  
  
  pinMode(DB9_1_SELECT, OUTPUT); // Pin to do signal selection for extra buttons in Sega controllers
  digitalWrite(DB9_1_SELECT, HIGH);
  
  for ( uint8_t i = 0; i < sizeof(DB9_1_PINS); ++i ) {
    pinMode(DB9_1_PINS[i], INPUT_PULLUP); // Setup joystick press data pins
  }

  if (digitalRead(DB9_1_PINS[0]) == LOW) {
    DB9_1_MAP_ACTIVE = 1;
  }
  if (digitalRead(DB9_1_PINS[1]) == LOW) {
    DB9_1_MAP_ACTIVE = 2;
  }
  
  
  
  pinMode(DB9_2_SELECT, OUTPUT); // Pin to do signal selection for extra buttons in Sega controllers
  digitalWrite(DB9_2_SELECT, HIGH);
  
  for ( uint8_t i = 0; i < sizeof(DB9_2_PINS); ++i ) {
    pinMode(DB9_2_PINS[i], INPUT_PULLUP); // Setup joystick press data pins
  }

  if (digitalRead(DB9_2_PINS[0]) == LOW) {
    DB9_2_MAP_ACTIVE = 1;
  }
  if (digitalRead(DB9_2_PINS[1]) == LOW) {
    DB9_2_MAP_ACTIVE = 2;
  }
}



void loop() {
  
  readPS2handle(keyboardleddata);
  
  joystickProcess(DB9_1_PINS, DB9_1_TOTALPINS, DB9_1_SELECT, DB9_1_STATUS, DB9_1_PRESSCOUNT, DB9_1_MAP, DB9_1_MAP_ACTIVE, 1);
  joystickProcess(DB9_2_PINS, DB9_2_TOTALPINS, DB9_2_SELECT, DB9_2_STATUS, DB9_2_PRESSCOUNT, DB9_2_MAP, DB9_2_MAP_ACTIVE, 2);
  
}



uint8_t sendPS2keypress(char keycode) {
  lastkeycode = keycode;
  lastkeycodestatus = 1;
  if (keycode == PS2dev::UP_ARROW || keycode == PS2dev::DOWN_ARROW || keycode == PS2dev::LEFT_ARROW || keycode == PS2dev::RIGHT_ARROW || keycode == PS2dev::RIGHT_ALT) {
    //while (digitalRead(PS2_PINS[0]) == LOW || digitalRead(PS2_PINS[1]) == LOW) {}
    return keyboard.keyboard_press_special(keycode);
  } else {
    //while (digitalRead(PS2_PINS[0]) == LOW || digitalRead(PS2_PINS[1]) == LOW) {}
    return keyboard.keyboard_press(keycode);
  }
}

uint8_t sendPS2keyrelease(char keycode) {
  lastkeycode = keycode;
  lastkeycodestatus = 0;
  if (keycode == PS2dev::UP_ARROW || keycode == PS2dev::DOWN_ARROW || keycode == PS2dev::LEFT_ARROW || keycode == PS2dev::RIGHT_ARROW || keycode == PS2dev::RIGHT_ALT) {
    //while (digitalRead(PS2_PINS[0]) == LOW || digitalRead(PS2_PINS[1]) == LOW) {}
    return keyboard.keyboard_release_special(keycode);
  } else {
    //while (digitalRead(PS2_PINS[0]) == LOW || digitalRead(PS2_PINS[1]) == LOW) {}
    return keyboard.keyboard_release(keycode);
  }
}

uint8_t readPS2handle(unsigned char &leddata) {
  if (keyboard.keyboard_handle(&leddata) == 2) {
    if (lastkeycodestatus == 1) { sendPS2keypress(lastkeycode); }
    if (lastkeycodestatus == 0) { sendPS2keyrelease(lastkeycode); }
    #ifdef BLUEPILL_BOARD
    //digitalWrite(LED_ONBOARD_2, LOW);
    //delay(2000);
    //digitalWrite(LED_ONBOARD_2, HIGH);
    #endif
  }
}



void joystickProcess(const uint8_t JOYSTICK_PINS[6], const uint8_t JOYSTICK_TOTALPINS, const uint8_t JOYSTICK_SELECT, uint8_t JOYSTICK_STATUS[12], uint8_t JOYSTICK_PRESSCOUNT[12], const char JOYSTICK_MAP[12][12], uint8_t JOYSTICK_MAP_ACTIVE, uint8_t JOYSTICK_INDEX) {

  bool sendPS2keyError;
  
  if (DB9_CYCLES_WAITCOUNT > 0) { DB9_CYCLES_WAITCOUNT--; }

  digitalWrite(JOYSTICK_SELECT, HIGH);
  //delayMicroseconds(20);

  for ( uint8_t i = 0; i < JOYSTICK_TOTALPINS; i++ ) {
    if (digitalRead(JOYSTICK_PINS[i]) == LOW) {
      if (JOYSTICK_PRESSCOUNT[i] < DB9_CYCLES_PRESS) {
        JOYSTICK_PRESSCOUNT[i]++;
      } else if ((JOYSTICK_STATUS[i] == 0) && (DB9_CYCLES_WAITCOUNT == 0)) {
        JOYSTICK_STATUS[i] = 1;
        DB9_CYCLES_WAITCOUNT = DB9_CYCLES_WAIT;
        /*
        do {
          sendPS2keyError = false;
          if (sendPS2keypress(JOYSTICK_MAP[JOYSTICK_MAP_ACTIVE][i]) != 0) {digitalWrite(LED_ONBOARD_2, LOW); delay(1); sendPS2keyError = true;}
        } while (sendPS2keyError == true);
        digitalWrite(LED_ONBOARD_2, HIGH);
        */
        sendPS2keypress(JOYSTICK_MAP[JOYSTICK_MAP_ACTIVE][i]);
        //delay(20);
        #ifdef BLUEPILL_BOARD
        if (JOYSTICK_INDEX == 1) { digitalWrite(LED_ONBOARD_1, LOW); }
        if (JOYSTICK_INDEX == 2) { digitalWrite(LED_ONBOARD_2, LOW); }
        #endif
      }
    } else {
      if ((JOYSTICK_PRESSCOUNT[i] > 0) && (DB9_CYCLES_WAITCOUNT == 0)) {
        JOYSTICK_PRESSCOUNT[i]--;
        /*
        if ((JOYSTICK_STATUS[i] == 1) && (JOYSTICK_PRESSCOUNT[i] < 3)) {
          DB9_CYCLES_WAITCOUNT = DB9_CYCLES_WAIT;
          while (sendPS2keyrelease(JOYSTICK_MAP[JOYSTICK_MAP_ACTIVE][i]) != 0) {}
          //sendPS2keyrelease(JOYSTICK_MAP[JOYSTICK_MAP_ACTIVE][i]);
        }
        */
      } else if ((JOYSTICK_STATUS[i] == 1) && (DB9_CYCLES_WAITCOUNT == 0)) {
        JOYSTICK_STATUS[i] = 0;
        DB9_CYCLES_WAITCOUNT = DB9_CYCLES_WAIT;
        /*
        do {
          sendPS2keyError = false;
          if (sendPS2keyrelease(JOYSTICK_MAP[JOYSTICK_MAP_ACTIVE][i]) != 0) {digitalWrite(LED_ONBOARD_2, LOW); delay(1); sendPS2keyError = true;}
        } while (sendPS2keyError == true);
        digitalWrite(LED_ONBOARD_2, HIGH);
        */
        sendPS2keyrelease(JOYSTICK_MAP[JOYSTICK_MAP_ACTIVE][i]);
        //delay(20);
        #ifdef BLUEPILL_BOARD
        if (JOYSTICK_INDEX == 1) { digitalWrite(LED_ONBOARD_1, HIGH); }
        if (JOYSTICK_INDEX == 2) { digitalWrite(LED_ONBOARD_2, HIGH); }
        #endif
      }
    }
  }


    
  digitalWrite(JOYSTICK_SELECT, LOW);
  //delayMicroseconds(20);
  
  if ((digitalRead(JOYSTICK_PINS[2]) == LOW) && (digitalRead(JOYSTICK_PINS[3]) == LOW)) {
    
    for ( uint8_t i = 4; i < JOYSTICK_TOTALPINS; i++ ) {
      if (digitalRead(JOYSTICK_PINS[i]) == LOW) {
        if (JOYSTICK_PRESSCOUNT[i+2] < DB9_CYCLES_PRESS) {
          JOYSTICK_PRESSCOUNT[i+2]++;
        } else if ((JOYSTICK_STATUS[i+2] == 0) && (DB9_CYCLES_WAITCOUNT == 0)) {
          JOYSTICK_STATUS[i+2] = 1;
          DB9_CYCLES_WAITCOUNT = DB9_CYCLES_WAIT;
          /*
          do {
            sendPS2keyError = false;
            if (sendPS2keypress(JOYSTICK_MAP[JOYSTICK_MAP_ACTIVE][i+2]) != 0) {digitalWrite(LED_ONBOARD_2, LOW); delay(1); sendPS2keyError = true;}
          } while (sendPS2keyError == true);
          digitalWrite(LED_ONBOARD_2, HIGH);
          */
          sendPS2keypress(JOYSTICK_MAP[JOYSTICK_MAP_ACTIVE][i+2]);
          //delay(20);
          #ifdef BLUEPILL_BOARD
          if (JOYSTICK_INDEX == 1) { digitalWrite(LED_ONBOARD_1, LOW); }
          if (JOYSTICK_INDEX == 2) { digitalWrite(LED_ONBOARD_2, LOW); }
          #endif
          //delayMicroseconds(40);
          //return;
        }
      } else {
        if ((JOYSTICK_PRESSCOUNT[i+2] > 0) && (DB9_CYCLES_WAITCOUNT == 0)) {
          JOYSTICK_PRESSCOUNT[i+2]--;
          /*
          if ((JOYSTICK_STATUS[i+2] == 1) && (JOYSTICK_PRESSCOUNT[i+2] < 3)) {
            DB9_CYCLES_WAITCOUNT = DB9_CYCLES_WAIT;
            while (sendPS2keyrelease(JOYSTICK_MAP[JOYSTICK_MAP_ACTIVE][i+2]) != 0) {}
            //sendPS2keyrelease(JOYSTICK_MAP[JOYSTICK_MAP_ACTIVE][i+2]);
          }
          */
        } else if ((JOYSTICK_STATUS[i+2] == 1) && (DB9_CYCLES_WAITCOUNT == 0)) {
          JOYSTICK_STATUS[i+2] = 0;
          DB9_CYCLES_WAITCOUNT = DB9_CYCLES_WAIT;
          /*
          do {
            sendPS2keyError = false;
            if (sendPS2keyrelease(JOYSTICK_MAP[JOYSTICK_MAP_ACTIVE][i+2]) != 0) {digitalWrite(LED_ONBOARD_2, LOW); delay(1); sendPS2keyError = true;}
          } while (sendPS2keyError == true);
          digitalWrite(LED_ONBOARD_2, HIGH);
          */
          sendPS2keyrelease(JOYSTICK_MAP[JOYSTICK_MAP_ACTIVE][i+2]);
          //delay(20);
          #ifdef BLUEPILL_BOARD
          if (JOYSTICK_INDEX == 1) { digitalWrite(LED_ONBOARD_1, HIGH); }
          if (JOYSTICK_INDEX == 2) { digitalWrite(LED_ONBOARD_2, HIGH); }
          #endif
          //return;
        }
      }
    }
    
  }
 
  /*
  // Sega 6 buttons controller
  digitalWrite(JOYSTICK_SELECT, HIGH);
  digitalWrite(JOYSTICK_SELECT, LOW);
  digitalWrite(JOYSTICK_SELECT, HIGH);
  digitalWrite(JOYSTICK_SELECT, LOW);
  digitalWrite(JOYSTICK_SELECT, HIGH);

  // Now we can try to read the buttons sequence: up Z - down Y - left X - right M
  
  for ( uint8_t i = 0; i < 4; i++ ) {
    if (digitalRead(JOYSTICK_PINS[i]) == LOW) {
      if (JOYSTICK_PRESSCOUNT[i+8] < DB9_CYCLES_PRESS) {
        JOYSTICK_PRESSCOUNT[i+8]++;
      } else if ((JOYSTICK_STATUS[i+8] == 0) && (DB9_CYCLES_WAITCOUNT == 0)) {
        JOYSTICK_STATUS[i+8] = 1;
        DB9_CYCLES_WAITCOUNT = DB9_CYCLES_WAIT;
        sendPS2keypress(JOYSTICK_MAP[JOYSTICK_MAP_ACTIVE][i+8]);
        #ifdef BLUEPILL_BOARD
        if (JOYSTICK_INDEX == 1) { digitalWrite(LED_ONBOARD_1, LOW); }
        if (JOYSTICK_INDEX == 2) { digitalWrite(LED_ONBOARD_2, LOW); }
        #endif
      }
    } else {
      if ((JOYSTICK_PRESSCOUNT[i+8] > 0) && (DB9_CYCLES_WAITCOUNT == 0)) {
        JOYSTICK_PRESSCOUNT[i+8]--;
        if ((JOYSTICK_STATUS[i+8] == 1) && (JOYSTICK_PRESSCOUNT[i+8] < 3)) {
          DB9_CYCLES_WAITCOUNT = DB9_CYCLES_WAIT;
          sendPS2keyrelease(JOYSTICK_MAP[JOYSTICK_MAP_ACTIVE][i+8]);
        }
      } else if ((JOYSTICK_STATUS[i+8] == 1) && (DB9_CYCLES_WAITCOUNT == 0)) {
        JOYSTICK_STATUS[i+8] = 0;
        DB9_CYCLES_WAITCOUNT = DB9_CYCLES_WAIT;
        sendPS2keyrelease(JOYSTICK_MAP[JOYSTICK_MAP_ACTIVE][i+8]);
        #ifdef BLUEPILL_BOARD
        if (JOYSTICK_INDEX == 1) { digitalWrite(LED_ONBOARD_1, HIGH); }
        if (JOYSTICK_INDEX == 2) { digitalWrite(LED_ONBOARD_2, HIGH); }
        #endif
      }
    }
  }
  //delayMicroseconds(20);
  //digitalWrite(JOYSTICK_SELECT, LOW);
  */
  
  /*
  if (DB9_CYCLES_WAITCOUNT > 0) { DB9_CYCLES_WAITCOUNT--; }

  digitalWrite(DB9_1_SELECT, HIGH);
  //delayMicroseconds(20);

  for ( uint8_t i = 0; i < DB9_1_TOTALPINS; i++ ) {
    if (digitalRead(DB9_1_PINS[i]) == LOW) {
      if (DB9_1_PRESSCOUNT[i] < DB9_CYCLES_PRESS) {
        DB9_1_PRESSCOUNT[i]++;
      } else if ((DB9_1_STATUS[i] == 0) && (DB9_CYCLES_WAITCOUNT == 0)) {
        DB9_1_STATUS[i] = 1;
        DB9_CYCLES_WAITCOUNT = DB9_CYCLES_WAIT;
        while (sendPS2keypress(DB9_1_MAP[DB9_1_MAP_ACTIVE][i]) != 0) {}
        //sendPS2keypress(DB9_1_MAP[DB9_1_MAP_ACTIVE][i]);
        #ifdef BLUEPILL_BOARD
        digitalWrite(LED_ONBOARD_1, LOW);
        #endif
        //return;
      }
    } else {
      if ((DB9_1_PRESSCOUNT[i] > 0) && (DB9_CYCLES_WAITCOUNT == 0)) {
        DB9_1_PRESSCOUNT[i]--;
        if ((DB9_1_STATUS[i] == 1) && (DB9_1_PRESSCOUNT[i] < 3)) {
          DB9_CYCLES_WAITCOUNT = DB9_CYCLES_WAIT;
          while (sendPS2keyrelease(DB9_1_MAP[DB9_1_MAP_ACTIVE][i]) != 0) {}
          //sendPS2keyrelease(DB9_1_MAP[DB9_1_MAP_ACTIVE][i]);
          //return;
        }
      } else if ((DB9_1_STATUS[i] == 1) && (DB9_CYCLES_WAITCOUNT == 0)) {
        DB9_1_STATUS[i] = 0;
        DB9_CYCLES_WAITCOUNT = DB9_CYCLES_WAIT;
        while (sendPS2keyrelease(DB9_1_MAP[DB9_1_MAP_ACTIVE][i]) != 0) {}
        //sendPS2keyrelease(DB9_1_MAP[DB9_1_MAP_ACTIVE][i]);
        #ifdef BLUEPILL_BOARD
        digitalWrite(LED_ONBOARD_1, HIGH);
        #endif
        //return;
      }
    }
  }
  

    
  digitalWrite(DB9_1_SELECT, LOW);
  //delayMicroseconds(20);
  
  for ( uint8_t i = 4; i < DB9_1_TOTALPINS; i++ ) {
    if (digitalRead(DB9_1_PINS[i]) == LOW) {
      if (DB9_1_PRESSCOUNT[i+2] < DB9_CYCLES_PRESS) {
        DB9_1_PRESSCOUNT[i+2]++;
      } else if ((DB9_1_STATUS[i+2] == 0) && (DB9_CYCLES_WAITCOUNT == 0)) {
        DB9_1_STATUS[i+2] = 1;
        DB9_CYCLES_WAITCOUNT = DB9_CYCLES_WAIT;
        while (sendPS2keypress(DB9_1_MAP[DB9_1_MAP_ACTIVE][i+2]) != 0) {}
        //sendPS2keypress(DB9_1_MAP[DB9_1_MAP_ACTIVE][i+2]);
        #ifdef BLUEPILL_BOARD
        digitalWrite(LED_ONBOARD_1, LOW);
        #endif
        //return;
      }
    } else {
      if ((DB9_1_PRESSCOUNT[i+2] > 0) && (DB9_CYCLES_WAITCOUNT == 0)) {
        DB9_1_PRESSCOUNT[i+2]--;
        if ((DB9_1_STATUS[i+2] == 1) && (DB9_1_PRESSCOUNT[i+2] < 3)) {
          DB9_CYCLES_WAITCOUNT = DB9_CYCLES_WAIT;
          while (sendPS2keyrelease(DB9_1_MAP[DB9_1_MAP_ACTIVE][i+2]) != 0) {}
          //sendPS2keyrelease(DB9_1_MAP[DB9_1_MAP_ACTIVE][i+2]);
          //return;
        }
      } else if ((DB9_1_STATUS[i+2] == 1) && (DB9_CYCLES_WAITCOUNT == 0)) {
        DB9_1_STATUS[i+2] = 0;
        DB9_CYCLES_WAITCOUNT = DB9_CYCLES_WAIT;
        while (sendPS2keyrelease(DB9_1_MAP[DB9_1_MAP_ACTIVE][i+2]) != 0) {}
        //sendPS2keyrelease(DB9_1_MAP[DB9_1_MAP_ACTIVE][i+2]);
        #ifdef BLUEPILL_BOARD
        digitalWrite(LED_ONBOARD_1, HIGH);
        #endif
        //return;
      }
    }
  }
  */
  
}




/*
PS2dev mouse(PD2,PD3); // 3 data 2clock

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
