#include <Arduino.h>
#include "Adafruit_VL53L0X.h"

const int leftForward = 4;    // pin digital 4
const int leftBackward = 7;   // pin digital 7
const int rightForward = 12;  // pin digital 12
const int rightBackward = 13; // pin digital 13
const int lightsensor = A0;   // pin analog 0
const int enableLeft = 3;     // pin digital 3, needed for PWM
const int enableRight = 5;    // pin digital 5, needed for PWM

const int ir_threshold = 500;  // if > 500, detect no enemy, <= 500, detect an enemy
const int light_threshold = 7; // if light sensor value < 10, it's black, > 10 it's white

// initiate IR sensor object
Adafruit_VL53L0X lox = Adafruit_VL53L0X();

/** State structure having 3 elements: 
 * output to motor, time needed for motor to run 
 * and an array of 4 pointers that point to state structure
 */
struct State
{
  uint8_t out;                 // motor output
  uint32_t delay;               // time to delay in ms
  const struct State *next[4]; // array of 4 pointers that point to state structure
};
typedef const struct State State_t; // define a type named State_t that has State structure
#define forward &fsm[0]             //name the address of element 0 of array fsm to "forward"
#define forward2 &fsm[1]
#define forward3 &fsm[2]
#define forward4 &fsm[3]
#define rotateR &fsm[4]
#define spin_right &fsm[5]
#define forward5 &fsm[6]
#define forward6 &fsm[7]
#define forward7 &fsm[8]
#define rotateL &fsm[9]
#define spin_left &fsm[10]
#define attack &fsm[11]

enum State_enum
{
  FORWARD,  // define FORWARD = 0
  FORWARD2, // define FORWARD2 = 1
  FORWARD3, // define FORWARD3 = 2
  FORWARD4, //...
  FORWARD5,
  FORWARD6,
  FORWARD7,
  ROTATE_RIGHT,
  ROTATE_LEFT,
  SPIN_LEFT,
  SPIN_RIGHT,
  ATTACK
};

/** Finite State Machine methodology
 * lightSensor & IRsensor = 00, 01, 10, 11; 
 * lightSensor = 0 (black) = 1 (white)
 * IRsensor = 0 (detect no enemy) = 1 (detect an enemy)
 */

State_t fsm[12] = {
    {FORWARD, 0, {forward2, attack, spin_right, spin_right}},
    {FORWARD2, 0, {forward3, attack, spin_right, spin_right}},
    {FORWARD3, 0, {forward4, attack, spin_right, spin_right}},
    {FORWARD4, 0, {forward5, attack, spin_right, spin_right}},
    {ROTATE_RIGHT, 0, {spin_right, attack, spin_right, spin_right}},
    {SPIN_RIGHT, 300, {forward5, attack, spin_right, spin_right}},
    {FORWARD5, 0, {forward6, attack, spin_left, spin_left}},
    {FORWARD6, 0, {forward7, attack, spin_left, spin_left}},
    {FORWARD7, 0, {forward, attack, spin_left, spin_left}},
    {ROTATE_LEFT, 0, {spin_left, attack, spin_left, spin_left}},
    {SPIN_LEFT, 300, {forward, attack, spin_left, spin_left}},
    {ATTACK, 0, {spin_left, attack, spin_left, spin_left}}};

State_t *Spt;   // pointer to the current state
uint8_t Input;  // input for a state
uint8_t Output; // output for a state
uint32_t Delay;  // run that state in Delay ms

void setup()
{
  pinMode(leftForward, OUTPUT);
  pinMode(leftBackward, OUTPUT);
  pinMode(rightForward, OUTPUT);
  pinMode(rightBackward, OUTPUT);
  pinMode(lightsensor, INPUT);
  pinMode(enableLeft, OUTPUT);
  pinMode(enableRight, OUTPUT);
  lox.begin();    // start IRsensor
  Spt = forward;  // set first state to "forward"
  delay(5000);    // wait for 5s  
  /*Debug*/
  // Serial.begin(115200);
  /*EndOfDebug*/
}

/** Motor control explanation
 * Switch transitor ON for 100 unit of time, OFF for 155 in a period of 255, 
 * repeat this task until delay_time ms has passed
 * Make transitor ON/OFF like that can make the motor turn faster or slower
 * 0 is the slowest (not moving) and 255 is at maximum speed
 * Set voltage of pin leftForward to HIGH, leftBackward to LOW, 
 * Current flows from HIGH to LOW and make the motor turn
 * The same thing applies for right wheel
 */

void go_forward(uint32_t delay_time) // normal forward mode
{
  analogWrite(enableLeft, 200);
  analogWrite(enableRight, 200);
  digitalWrite(leftForward, HIGH);
  digitalWrite(rightForward, HIGH);
  digitalWrite(leftBackward, LOW);
  digitalWrite(rightBackward, LOW);
  delay(delay_time);
}

void fast_forward(uint32_t delay_time) // attack mode
{
  analogWrite(enableLeft, 255);
  analogWrite(enableRight, 255);
  digitalWrite(leftForward, HIGH);
  digitalWrite(rightForward, HIGH);
  digitalWrite(leftBackward, LOW);
  digitalWrite(rightBackward, LOW);
  delay(delay_time);
}
void rotate_right(uint32_t delay_time) // drift mode
{
  analogWrite(enableLeft, 200);
  analogWrite(enableRight, 200);
  digitalWrite(leftForward, HIGH);
  digitalWrite(rightForward, LOW);
  digitalWrite(leftBackward, LOW);
  digitalWrite(rightBackward, HIGH);
  delay(delay_time);
}
void rotate_left(uint32_t delay_time) // drift mode
{
  analogWrite(enableLeft, 200);
  analogWrite(enableRight, 200);
  digitalWrite(leftForward, LOW);
  digitalWrite(rightForward, HIGH);
  digitalWrite(leftBackward, HIGH);
  digitalWrite(rightBackward, LOW);
  delay(delay_time);
}

void drift_right(uint32_t delay_time) // super-drift mode
{
  analogWrite(enableLeft, 255);
  analogWrite(enableRight, 255);
  digitalWrite(leftForward, LOW);
  digitalWrite(rightForward, LOW);
  digitalWrite(leftBackward, HIGH);
  digitalWrite(rightBackward, HIGH);
  delay(500);
  digitalWrite(leftForward, HIGH);
  digitalWrite(rightForward, LOW);
  digitalWrite(leftBackward, LOW);
  digitalWrite(rightBackward, HIGH);
  delay(delay_time);
}

void drift_left(uint32_t delay_time) // super-drift mode
{
  analogWrite(enableLeft, 255);
  analogWrite(enableRight, 255);
  digitalWrite(leftForward, LOW);
  digitalWrite(rightForward, LOW);
  digitalWrite(leftBackward, HIGH);
  digitalWrite(rightBackward, HIGH);
  delay(500);
  digitalWrite(leftForward, LOW);
  digitalWrite(rightForward, HIGH);
  digitalWrite(leftBackward, HIGH);
  digitalWrite(rightBackward, LOW);
  delay(delay_time);
}

void run_motors(uint8_t condition, uint32_t time) // decision making for wheels
{
  switch (condition)
  {
  case FORWARD:
    go_forward(time);
    break;
  case FORWARD2:
    go_forward(time);
    break;
  case FORWARD3:
    go_forward(time);
    break;
  case FORWARD4:
    go_forward(time);
    break;
  case FORWARD5:
    go_forward(time);
    break;
  case FORWARD6:
    go_forward(time);
    break;
  case FORWARD7:
    go_forward(time);
    break;
  case ROTATE_RIGHT:
    rotate_right(time);
    break;
  case SPIN_RIGHT:
    drift_right(time);
    break;
  case ROTATE_LEFT:
    rotate_left(time);
    break;
  case SPIN_LEFT:
    drift_left(time);
    break;
  case ATTACK:
    fast_forward(time);
    break;
  }
}

uint8_t read_ir() // read IRsensor
{
  VL53L0X_RangingMeasurementData_t measure;
  lox.rangingTest(&measure, false);
  uint8_t ir = 0x00;
  if (measure.RangeMilliMeter < ir_threshold)
    ir = 0x01;
  /*Debug*/
  // Serial.print(analogRead(lightsensor));
  // Serial.print(';');
  // Serial.println(measure.RangeMilliMeter);
  /*EndOfDebug*/
  return ir;
}

uint8_t read_light_sensor() // read lightSensor
{
  // return 1 if detect or 0 if not detect
  uint8_t ls = 0x00;
  if (analogRead(lightsensor) > light_threshold)
    ls = 0x01;
  return ls;
}

uint8_t read_sensors() // calculate result of two sensors combine
{
  uint8_t result = read_ir() | read_light_sensor() << 1;
  return result;
}

void loop()
{
  Output = Spt->out;         // set output from FSM for motors
  Delay = Spt->delay;        // set delay for motors
  run_motors(Output, Delay); // run motor at that state
  Input = read_sensors();
  Spt = Spt->next[Input]; // go to next state
}
