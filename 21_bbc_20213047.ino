#include <Servo.h>

//Arduino pin assignment
#define PIN_SERVO 10
#define PIN_IR A0

//Framework setting
#define _DIST_MIN 100
#define _DIST_MAX 410

//Servo range
#define _DUTY_MIN 1137
#define _DUTY_NEU 1426
#define _DUTY_MAX 1705

//Servo instance
Servo myservo;
int a, b;

void setup(){
//attach servo
  myservo.attach(PIN_SERVO);

//move servo to neutral position
  myservo.writeMicroseconds(_DUTY_NEU);

//initialize serial port
  Serial.begin(57600);

  a = 72;
  b = 270;
}

float ir_distance(void){ // return value unit: mm
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;
  return val;
}

void loop(){
  Serial.println(myservo.read());
  float raw_dist = ir_distance();
  float dist_cali = 100 + 300.0 / (b - a) * (raw_dist - a);
  Serial.print("min:100, max:410, dist:");
  Serial.print(raw_dist);
  Serial.print(",dist_cali:");
  Serial.println(dist_cali);
  
  if(dist_cali > 180) {
    myservo.writeMicroseconds(_DUTY_MIN);
  }
  else if(dist_cali <= 180){
    myservo.writeMicroseconds(_DUTY_MAX);
  }
  delay(20);
}#include <Servo.h>

//Arduino pin assignment
#define PIN_SERVO 10
#define PIN_IR A0

//Framework setting
#define _DIST_MIN 100
#define _DIST_MAX 410

//Servo range
#define _DUTY_MIN 1137
#define _DUTY_NEU 1426
#define _DUTY_MAX 1705

//Servo instance
Servo myservo;
int a, b;

void setup(){
//attach servo
  myservo.attach(PIN_SERVO);

//move servo to neutral position
  myservo.writeMicroseconds(_DUTY_NEU);

//initialize serial port
  Serial.begin(57600);

  a = 72;
  b = 270;
}

float ir_distance(void){ // return value unit: mm
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;
  return val;
}

void loop(){
  Serial.println(myservo.read());
  float raw_dist = ir_distance();
  float dist_cali = 100 + 300.0 / (b - a) * (raw_dist - a);
  Serial.print("min:100, max:410, dist:");
  Serial.print(raw_dist);
  Serial.print(",dist_cali:");
  Serial.println(dist_cali);
  
  if(dist_cali > 180) {
    myservo.writeMicroseconds(_DUTY_MIN);
  }
  else if(dist_cali <= 180){
    myservo.writeMicroseconds(_DUTY_MAX);
  }
  delay(20);
}
