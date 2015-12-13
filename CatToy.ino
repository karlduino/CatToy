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
 */

#include <Servo.h>
#include <math.h>

Servo servos[2];

const int SERVO_PINS[2] = {9, 10}; /* side/side = 9, up/down=10 */
const int LED_PINS[2] = {6, 5};    /* left=6, right=5 */
const int BUTTON_PIN = 8;          /* button to switch left/right */
const int LASER_PIN = 7;

const int CENTER[2] = {90, 75}; /* center for each servo */
const int ANGLES[2] = {65, 75}; /* up/down angles for high/left, low/right */

const double MINPOS[2] = {40, 90};
const double MAXPOS[2] = {140, 170};

const int SPEED  = 3000; /* time to fly 180 degrees*/
const int N_STEPS = 300;       /* no. steps to fly */

int button_press = HIGH; /* HIGH = not pressed, because of pull-up resistor */
int prev_button_press = HIGH;

double servopos[2];


void setup()
{
  double newpos[2];
  
  for(int i=0; i<2; i++)
    pinMode(LED_PINS[i], OUTPUT);

  pinMode(BUTTON_PIN, INPUT);
  digitalWrite(BUTTON_PIN, HIGH); /* set pull-up resistor */

  pinMode(LASER_PIN, OUTPUT);
  digitalWrite(LASER_PIN, HIGH);

  for(int i=0; i<2; i++) {
    servos[i].attach(SERVO_PINS[i]);
    servos[i].write(CENTER[i]);
    servopos[i] = CENTER[i];
  }
  flash_leds(4, 250);
  
  randomSeed(analogRead(5));

}

void loop()
{
  delay(1000);
  move_random();

}

void move_random(void) {
  double newpos[2];
  for(int i=0; i<2; i++) newpos[i] = random(MAXPOS[i] - MINPOS[i]) + MINPOS[i];
  move_servos(servos, servopos, newpos, 200, 2000);
  for(int i=0; i<2; i++) servopos[i] = newpos[i];
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

