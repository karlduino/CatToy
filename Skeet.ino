/**
 * Skeet practice device
 *
 * moves laser dot from left to right or back
 * ultimately want to also have voice recognition ("pull")
 * uses - pan/tilt bracket     sparkfun ROB-10335
 *      - 2 sub-micro servos   sparkfun ROB-09065
 *      - laser card module    sparkfun COM-00594
 *      - laser module mount   sparkfun COM-08674
 *      - electet microphone   sparkfun BOB-09964
 *      - Arduino (Yun)
 *
 */

#include <Servo.h>

Servo myservo;
const int servopin=10;
const double maxpos = 2;
const int n_steps = 100;
const double sec_per_sweep = 1.5;
const int delay_between = 2000;

double servopos = -maxpos;

void setup()
{
  myservo.attach(servopin);
  myservo.write(90);
  delay(delay_between);
  
  myservo.write(calc_angle(servopos));
  delay(delay_between);
}

void loop()
{
  /* sweep to right */
  move_servo(myservo, -maxpos, maxpos, n_steps, sec_per_sweep);
  delay(delay_between);

  /* sweep to left */
  move_servo(myservo, maxpos, -maxpos, n_steps, sec_per_sweep);
  delay(delay_between);
}

void move_servo(Servo theservo, double start_position, double end_position, int n_steps, double time_sec)
{
  int delay_per = floor(time_sec*1000.0/(double)n_steps);
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
