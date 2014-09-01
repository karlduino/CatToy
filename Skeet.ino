/**
 * Skeet practice device
 *
 * moves laser dot from left to right or back
 * ultimately want to also have voice recognition ("pull")
 * uses - pan/tilt bracket     sparkfun ROB-10335
 *      - 2 sub-micro servos   sparkfun ROB-09065
 *      - laser card module    sparkfun COM-00594
 *      - laser module mount   sparkfun COM-08674
 *      - electret microphone   sparkfun BOB-09964
 *      - Arduino (Yun)
 *      - two LEDs with 470 Ohm resistors
 *
 */

#include <Servo.h>
#include <math.h>

Servo leftservos[2];

const int SERVO_PINS[2] = {9, 10}; /* side/side = 9, up/down=10 */
const int LED_PINS[2] = {6, 5};    /* left=6, right=5 */
const int BUTTON_PIN = 8;              /* button to switch left/right */
const int LASER_PIN = 7;
const int MIC_PIN = A2;

const int CENTER[2] = {90, 135}; /* center for each servo */
const int ANGLES[2] = {45, 75}; /* up/down angles for high/left, low/right */
const double MAXPOS[2] = {-1.0, 1.0}; /* max position to left and right */

const int DELAY_BEFORE = 1000; /* delay between "pull" and fly */
const int TIME_TO_FLY  = 1500; /* time to fly */
const int DELAY_AFTER  = 3000; /* delay after fly */
const int N_STEPS = 100;       /* no. steps to fly */

/* button and skeet state */
int button_press = HIGH; /* HIGH = not pressed, because of pull-up resistor */
int prev_button_press = HIGH;
int skeet_state = 0; /* 0 = left; 1 = right */
const int MAX_SKEET_STATE = 1;

/* sound detection */
const int N_READINGS_PER = 100;
const int DELAY_PER = 2;
const int N_READINGS_NOISE = 20;
const int N_SD_NOISE = 5.0;
float noise_ave, noise_sd, noise_threshold;
float mic_reading;
bool just_pressed_button = false;

double servopos[2];


void setup()
{
  for(int i=0; i<2; i++) {
    leftservos[i].attach(SERVO_PINS[i]);
    leftservos[i].write(CENTER[i]);
    servopos[i] = CENTER[i];
  }

  for(int i=0; i<2; i++) {
    pinMode(LED_PINS[i], OUTPUT);
    digitalWrite(LED_PINS[i], LOW);
  }

  pinMode(BUTTON_PIN, INPUT);
  digitalWrite(BUTTON_PIN, HIGH); /* set pull-up resistor */

  pinMode(LASER_PIN, OUTPUT);
  digitalWrite(LASER_PIN, LOW);

  pinMode(MIC_PIN, INPUT);

  Serial.begin(9600);
  getMicBackground(MIC_PIN, &noise_ave, &noise_threshold, 
                   N_READINGS_NOISE, N_READINGS_PER, DELAY_PER);
  Serial.print("noise ave = ");
  Serial.println(noise_ave);
  Serial.print("noise threshold = ");
  Serial.println(noise_threshold);
}

void loop()
{
  /* read button */
  button_press = digitalRead(BUTTON_PIN);

  just_pressed_button = false;
  if(prev_button_press != button_press) {
    delay(50);
    just_pressed_button = true;
    if(button_press==LOW)  Serial.println("button pressed");
    if(button_press==HIGH) Serial.println("button unpressed");
  }
  if(prev_button_press == HIGH && button_press == LOW) {
    skeet_state += 1;
    if(skeet_state > MAX_SKEET_STATE) { skeet_state = 0; }
    Serial.print("skeet_state = ");
    Serial.println(skeet_state);
  }
  prev_button_press = button_press;

  /* LEDs */
  if(skeet_state == 0) {
    digitalWrite(LED_PINS[0], HIGH);
    digitalWrite(LED_PINS[1], LOW);
  }
  else if(skeet_state == 1) {
    digitalWrite(LED_PINS[0], LOW);
    digitalWrite(LED_PINS[1], HIGH);
  }
  
  /* read_mic */
  mic_reading = getMicReading(MIC_PIN, noise_ave, N_READINGS_PER, DELAY_PER);
  Serial.print(mic_reading);
  Serial.print(" (vs ");
  Serial.print(noise_threshold);
  Serial.println(")");
  
  if(mic_reading > noise_threshold && !just_pressed_button) {
    if(skeet_state==0) {
      Serial.println("Run from left");
      run_from_left();
    }
    else if(skeet_state==1) {
      Serial.println("Run from right");
      run_from_right();
    }
  }

}

void run_from_left(void) { /* run skeet from left to right */
  leftservos[1].write(ANGLES[0]);
  leftservos[0].write(calc_angle(MAXPOS[0]));

  delay(DELAY_BEFORE);

  digitalWrite(LASER_PIN, HIGH);
  move_servo(leftservos[0], MAXPOS[0], MAXPOS[1], N_STEPS, TIME_TO_FLY);
  digitalWrite(LASER_PIN, LOW);

  delay(DELAY_AFTER);
}
    

void run_from_right(void) { /* run skeet from right to left */
  leftservos[1].write(ANGLES[1]);
  leftservos[0].write(calc_angle(MAXPOS[1]));

  delay(DELAY_BEFORE);

  digitalWrite(LASER_PIN, HIGH);
  move_servo(leftservos[0], MAXPOS[1], MAXPOS[0], N_STEPS, TIME_TO_FLY);
  digitalWrite(LASER_PIN, LOW);

  delay(DELAY_AFTER);
}
    

void move_servo(Servo theservo, double start_position, double end_position, 
                int n_steps, int time_ms)
{
  int delay_per = floor((double)time_ms/(double)n_steps);
  double dist_per = (end_position - start_position)/(double)n_steps;

  for(int i=1; i<n_steps; i++) {
      theservo.write(calc_angle(start_position + (double)i*dist_per));
      delay(delay_per);
  }
}

/** calculate angle for a given position
  * position = distance from center, as multiple of distance to wall
  *    negative = left of center; positive = right of center
  **/
int calc_angle(double position)
{
  return((int)(atan(position) * 180.0/M_PI + 90.0));
}

/* get a bunch of readings from mic and return the average */
float getMicReading(int pin, float middle, int nreadings, int delay_time)
{
  float result;
  result = 0.0;
  for(int i=0; i<nreadings; i++) {
    result += (abs((float)analogRead(pin) - middle) / (float)nreadings);
    delay(delay_time);
  }
  return(result);
}

/* measure background noise */
void getMicBackground(int pin, float *middle, float *thresh, int nreadings, int nreadings_per, int delay_time_per)
{
  float value;
  double sum, sumsq;
  int ledstate = 0;
  int n4ave = nreadings/3;

  *middle = 0.0;

  for(int i=0; i<n4ave; i++) {
    /* flashing LEDs */
    for(int j=0; j<2; j++) digitalWrite(LED_PINS[j], ledstate);
    ledstate = 1 - ledstate;
      
    value = getMicReading(pin, 0.0, nreadings_per, delay_time_per);
    (*middle) += value;
  }
  (*middle) /= (float)n4ave;


  sumsq = sum = 0.0;
  for(int i=0; i<nreadings; i++) {
    for(int j=0; j<2; j++) digitalWrite(LED_PINS[j], ledstate);
    ledstate = 1 - ledstate;

    value = getMicReading(pin, *middle, nreadings_per, delay_time_per);
    sum += value;
    sumsq += (double)value*(double)value;
  }

  /* turn leds off */
  for(int j=0; j<2; j++) digitalWrite(LED_PINS[j], LOW);

  sum /= (float)nreadings;
  sumsq = sqrt( (sumsq - sum*sum*(float)nreadings)/((float)(nreadings-1)) );
  *thresh = (float)(sum + sumsq*N_SD_NOISE);
}
