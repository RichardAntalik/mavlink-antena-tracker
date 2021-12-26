#include <Servo.h>

#define REVERSE_ROTATION true
// pulses per 360 degrees of antena rotation
#define PULSES_PER_FULL_ROTATION 7120/2
#define DEBUG false
 
 /*
Mapping of port bit to pin
 
PD2 - pin 0
PD3 - pin 1
PD1 - pin 2
PD0 - pin 3
PD4 - pin 4
*/

// Mask of which port bits are connected to which motor coil - ap = A "positive" an = A "negative" etc...
// This is driver for unipolar motor configuration.
static const char ap = B00000100;
static const char an = B00001000;
static const char bp = B00000010;
static const char bn = B00000001;

// Array with motor waveform  
static const char states[4] = {
ap|bp,
an|bp,
an|bn,
ap|bn,
};

typedef struct GeoPosition {
  float bearing; // Azimuth technically
  float elevation_angle;
} GeoPosition;

// Antena pointing to north on start 
static int curstep = PULSES_PER_FULL_ROTATION / 2;
static GeoPosition target_position = {};

static float mapf(long x, long in_min, long in_max, long out_min, long out_max) {
  return ((x - in_min) / (float)((in_max - in_min))) * (out_max - out_min) + out_min;
}

static float fmapf(float x, long in_min, long in_max, long out_min, long out_max) {
  return ((x - in_min) / (float)((in_max - in_min))) * (out_max - out_min) + out_min;
}

static int get_step_dir(void){
  float cur_bearing;
  int step_dir;
  if (REVERSE_ROTATION){
    cur_bearing = mapf(curstep, 0, PULSES_PER_FULL_ROTATION, 180, -180);
    step_dir = target_position.bearing > cur_bearing ? -1 : 1;
  }
  else {
    cur_bearing = mapf(curstep, 0, PULSES_PER_FULL_ROTATION, -180, 180);
    step_dir = target_position.bearing > cur_bearing ? 1 : -1;
  }
  if ((int)(target_position.bearing - cur_bearing) == 0){
    step_dir = 0;
  }
  else{
    if (DEBUG){
    Serial.print("current bearing ");
    Serial.print(cur_bearing);
    Serial.print(" target bearing ");
    Serial.print(target_position.bearing);
    Serial.println();
    }
  }
  return step_dir;
}

// Do one step in step_direction
static void motor_step(int step_dir){
  if(step_dir == 0){
    return;
  }
  
  curstep += step_dir;
  PORTD = states[(curstep & B00000011)];
  delayMicroseconds(1000);
  PORTD = 0;
  delayMicroseconds(1000);
}

// Angle to servo PWM mapping - adjust as needed to match reality
#define SERVO_SPEED 10 //Degrees per second.
#define SERVO_FREQ 50 //Changes in position per second.
#define SERVO_PWM_MIN 2400
#define SERVO_PWM_MAX 600
#define TRACKER_ANGLE_MIN -13
#define TRACKER_ANGLE_MAX 60

Servo myservo;
float servo_last_pos = TRACKER_ANGLE_MAX; // Set to home position - servos are moving fast, make sure initial step is not large somehow...
unsigned long servo_last_update_time;

static int servo_angle_to_pwm(float angle){
  return fmapf(angle, TRACKER_ANGLE_MIN, TRACKER_ANGLE_MAX, SERVO_PWM_MIN, SERVO_PWM_MAX);
}

// Move servo at constant velocity to servo_target_pos
static void servo_move_step(void){
  if (target_position.elevation_angle == servo_last_pos){
    return;
  }
  if (millis() <= servo_last_update_time + (1000.0f / SERVO_FREQ)){
    return;
  }

  int step_dir;
  if (servo_last_pos < target_position.elevation_angle){
    step_dir = 1;
  }else{
    step_dir = -1;
  }

  float timebase = SERVO_FREQ / 1000.0f;
  float servo_new_pos = servo_last_pos + (SERVO_SPEED * timebase * step_dir);
  servo_new_pos = roundf(1000.0f * servo_new_pos) / 1000.0f;
  int pwm = servo_angle_to_pwm(servo_new_pos);

  if (DEBUG){
    Serial.print("servo_target_pos = ");
    Serial.print(target_position.elevation_angle);
    Serial.println();
    Serial.print("Moving servo to ");
    Serial.print(servo_new_pos);
    Serial.print(" xxx = ");
    Serial.print(timebase);
    Serial.println();
  }
  
  myservo.writeMicroseconds(pwm);
  servo_last_pos = servo_new_pos;
  servo_last_update_time = millis();
}

void setup() {
  delay(2000);
  if(DEBUG){
    Serial.println("Tracker running");
  }
  
  // Motor pins
  PORTD = 0;
  pinMode(0, OUTPUT);
  pinMode(1, OUTPUT);
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  
  // Servo. For gods sake, don't use pin shared with LED, otherwise this
  // will be very interesting experience when you upload binary...
  myservo.attach(11);
  servo_last_update_time = millis();
  myservo.writeMicroseconds(servo_angle_to_pwm(servo_last_pos)); // Go to home position
  pinMode(11, OUTPUT);
}

// Get position over serial (Mavlink python script)
static void serial_read_target_position(void){
  if (Serial.available() >= (sizeof(float)*2)) {
    Serial.readBytes((char *) &target_position.bearing, sizeof(float));
    Serial.readBytes((char *) &target_position.elevation_angle, sizeof(float));
    target_position.elevation_angle = roundf(1000.0f * target_position.elevation_angle) / 1000.0f;
  }
}

void loop() {
  serial_read_target_position();
  motor_step(get_step_dir());
  servo_move_step();
}
