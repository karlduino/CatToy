/**
 * Laser cat toy
 *
 * moves laser dot randomly
 * uses - pan/tilt bracket     sparkfun ROB-10335
 *      - 2 sub-micro servos   sparkfun ROB-09065
 *      - laser card module    sparkfun COM-00594
 *      - electret microphone   sparkfun BOB-09964
 *      - Arduino (Uno)
 *      - two LEDs with 470 Ohm resistors
 *
 * pin 5 - left LED
 * pin 6 - right led
 * pin 7 - laser
 * pin 8 - button
 * pin 9  - left/right servo
 * pin 10 - up/down servo
 * pin A0 - joystick left/right (X)
 * pin A1 - joystick up/down (Y)
 *
 */

#include <Servo.h>
#include <math.h>

Servo servos[2];

const int SERVO_PINS[2] = {9, 10}; /* side/side = 9, up/down=10 */
const int LED_PINS[2] = {6, 5};    /* left=6, right=5 */
const int BUTTON_PIN = 8;          /* button to switch between random and joystick */
const int LASER_PIN = 7;
const int JOY_PINS[2] = {A0, A1}; /* joystick, left/right and up/down */

const int CENTER[2] = {90, 75}; /* center for each servo */

const double MINPOS[2] = {40, 90};
const double MAXPOS[2] = {140, 170};

const double MINJOY[2] = {90, 10};
const double MAXJOY[2] = {1013, 1013};

const int N_STEPS = 150;       /* no. steps to fly */
const int FLY_TIME = 1000;         /* amount of time to fly */

int button_press = HIGH; /* HIGH = not pressed, because of pull-up resistor */
int prev_button_press = HIGH;
int state = 0;

double servopos[2];


void setup()
{
  double newpos[2];
  
  for(int i=0; i<2; i++)
    pinMode(LED_PINS[i], OUTPUT);

  pinMode(BUTTON_PIN, INPUT);
  digitalWrite(BUTTON_PIN, HIGH); /* set pull-up resistor */
  prev_button_press = HIGH;

  pinMode(LASER_PIN, OUTPUT);
  digitalWrite(LASER_PIN, HIGH);

  for(int i=0; i<2; i++) {
    servos[i].attach(SERVO_PINS[i]);
    servos[i].write(CENTER[i]);
    servopos[i] = CENTER[i];
  }
  flash_leds(4, 250);
  digitalWrite(LED_PINS[0], 1-state);
  digitalWrite(LED_PINS[1], state);
  
  randomSeed(analogRead(5));
  Serial.begin(9600);
}

void loop()
{
  button_press=digitalRead(BUTTON_PIN);
  if(button_press==LOW) {
    delay(50);
    if(button_press==LOW && button_press != prev_button_press) {
      Serial.println("button pressed");
      state = 1 - state; 
    }
  }
  prev_button_press = button_press;
  digitalWrite(LED_PINS[0], 1-state);
  digitalWrite(LED_PINS[1], state);

  Serial.println(state);
  if(state==1) {
   move_random();
   delay(500);
  }
  else {
    use_joystick();
  }
}

void move_random(void) {
  double newpos[2];
  for(int i=0; i<2; i++) {
    newpos[i] = random(MAXPOS[i] - MINPOS[i]) + MINPOS[i];
    Serial.print((int)newpos[i]);
    Serial.print(" ");
  }
  Serial.println();

  move_servos(servos, servopos, newpos, N_STEPS, FLY_TIME);
  
  for(int i=0; i<2; i++) servopos[i] = newpos[i];
}

void use_joystick(void) {
    double joypos[2];
    double newpos[2];
    for(int i=0; i<2; i++) {
      joypos[i] = analogRead(JOY_PINS[i]);
      Serial.print((int)joypos[i]);
      Serial.print(" ");
      double relval = (joypos[i]-MINJOY[i])/(MAXJOY[i]-MINJOY[i]); // between 0 and 1
      if(relval < 0) relval=0;
      if(relval > 1) relval=1;
      if(i==0) newpos[i] = relval*180;
      else newpos[i] = 180*(1-relval);
      Serial.print((int)newpos[i]); Serial.print("       ");
    }
    Serial.println();

  for(int i=0; i<2; i++) {
    servos[i].write(newpos[i]);
    servopos[i] = newpos[i];
  }
}



void move_servos(Servo theservo[2], double start_position[2], 
                 double end_position[2],
                int n_steps, int time_ms)
{
  int delay_per = floor((double)time_ms/(double)n_steps);
  double dist_per[2];
  for(int i=0; i<2; i++) dist_per[i] = (end_position[i] - start_position[i])/(double)n_steps;

  for(int i=1; i<n_steps; i++) {
    for(int j=0; j<2; j++) {
      theservo[j].write(start_position[j] + (double)i*dist_per[j]);
    }
    delay(delay_per);
  }
}

void flash_leds(unsigned int num, unsigned int delay_time)
{
  for(int k=0; k<num; k++) {
    if(k > 0) delay(delay_time);
    for(int i=0; i<2; i++)
      digitalWrite(LED_PINS[i], HIGH);
    delay(delay_time);
    for(int i=0; i<2; i++)
      digitalWrite(LED_PINS[i], LOW);
  }
}

