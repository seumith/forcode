// constants.h

#ifndef CONSTANTS_H
#define CONSTANTS_H
#include <Servo.h>
#include <LiquidCrystal_I2C.h>

// Pin definitions
#define P_SERVO 9
#define P_CURRENT A0
#define P_RED_LED 8
#define P_GREEN_LED 7
#define P_SETTING_BUTTON 2
#define P_START_BUTTON 3

#define POT_PIN_OFFSET 0

#define LCD_H_SPACING 2
#define POT_LCD_OFFSET 1

#define START_POS 180
#define MIN_SPEED 1
#define MAX_SPEED 100

// Age groups
#define ADULT 0
#define CHILD 1
#define INFANT 2
#define PRES 0
#define RATE 1
#define VOL  2
// Sensor model
#define MODEL 0
#define VIN A1
#define QOV 0.032
const int MAX_SERVO_POS = 180;

#endif // CONSTANTS_H

