#ifndef __USER_INTERFACE_H
#define __USER_INTERFACE_H


/* few commands for LCD
lcd.noBlink();
lcd.blink();
lcd.setCursor(0, 0);
lcd.autoscroll();
lcd.noAutoscroll();
lcd.begin(numCols, numRows);
lcd.noCursor();
lcd.cursor();
lcd.scrollDisplayLeft();
lcd.scrollDisplayRight();
lcd.rightToLeft();
lcd.leftToRight();
lcd.home();
*/

#define DISPLAY_WELCOME     0
#define DISPLAY_FIO2        1
#define DISPLAY_T_V         2 
#define DISPLAY_RR          3 
#define DISPLAY_PC          4 
#define DISPLAY_I_E         5
#define DISPLAY_TRIG        6
#define DISPLAY_VMODE       7

#define DISPLAY_SET_FiO2    9
#define DISPLAY_SET_T_V     10
#define DISPLAY_SET_RR      11  
#define DISPLAY_SET_PC      12  
#define DISPLAY_SET_I_E     13  
#define DISPLAY_SET_TRIG    14
#define DISPLAY_SET_VMODE   15

#define DISPLAY_INPUT       17
#define DISPLAY_CLEAR       18
#define DISPLAY_CAUTION     90
#define DISCARD_INPUT       99


// KEY PAD MICROS
// #define KEY_0           2
// #define KEY_1           15
// #define KEY_2           14
// #define KEY_3           13
// #define KEY_4           11
// #define KEY_5           10
// #define KEY_6           9
// #define KEY_7           7
// #define KEY_8           6
// #define KEY_9           5

// #define KEY_A           12    // /
// #define KEY_B           8    // x
// #define KEY_C           4    // -
// #define KEY_D           0    // +

// #define KEY_HASH        1    // equ
// #define KEY_STAR        3     // on

#define KEY_0           11
#define KEY_1           12
#define KEY_2           8
#define KEY_3           4
#define KEY_4           13
#define KEY_5           9
#define KEY_6           5
#define KEY_7           14
#define KEY_8           10
#define KEY_9           6

#define KEY_A           0    // /
#define KEY_B           1    // x
#define KEY_C           2    // -
#define KEY_D           3    // +

#define KEY_HASH        7    // equ
#define KEY_STAR        15     // on

void LCD_setup();
void InitializeParams();
void Display();
void readSwitches();
void V_Mode_Breakdown();


#endif