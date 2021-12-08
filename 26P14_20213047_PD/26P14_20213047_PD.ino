#include <Servo.h>

// Configurable parameters

// Arduino pin assignment
#define PIN_LED 9  //LED in 9
#define PIN_SERVO 10 // SERVO in 10
#define PIN_IR A0  // Infrared sensor signal in A0

// Framework setting
#define _DIST_MIN 117
#define _DIST_TARGET 330  // Target distance = 25.5cm
#define _DIST_MAX 320

// Distance sensor
#define _DIST_ALPHA 0.35  // EMA filter

// global variables
const float coE[] = {0.0000204, -0.0136853, 4.0482617, -156.8686997};

// Servo range
#define _DUTY_MIN 1145  // SERVO DUTY's minimum 
#define _DUTY_NEU 1455  // SERVO DUTY's neutral 
#define _DUTY_MAX 1545  // SERVO DUTY's maximum 

// Servo speed control
#define _SERVO_ANGLE 30  // SERVO ANGLE 
#define _SERVO_SPEED 150  // SERVO SPEED 

// Event periods
#define _INTERVAL_DIST 15 
#define _INTERVAL_SERVO 20 
#define _INTERVAL_SERIAL 100 

// PID parameters
#define _Kp 0.5 // PID Propotional gain for P
#define _Ki 0.0  // PID Propotional gain for I
#define _Kd 269.0  // PID Propotional gain for D

// global variables 
float dist_min, dist_max, dist_raw, dist_ema, alpha;

// Servo instance
Servo myservo; 

// Distance sensor
float dist_target;  // location to send the ball

// Event periods
unsigned long last_sampling_time_dist, last_sampling_time_servo, last_sampling_time_serial; 
bool event_dist, event_servo, event_serial;

// Servo speed control
int duty_chg_per_interval; 
int duty_target, duty_curr;

// PID variables
float error_curr, error_prev, control, P_term, I_term, D_term;

// Filter
#define _INTERVAL_DIST 30  // DELAY_MICROS * samples_num^2 의 값이 최종 거리측정 인터벌임. 넉넉하게 30ms 잡음.
#define DELAY_MICROS  1500 // 필터에 넣을 샘플값을 측정하는 딜레이(고정값!)
float filtered_dist;       // 최종 측정된 거리값을 넣을 변수. loop()안에 filtered_dist = filtered_ir_distance(); 형태로 사용하면 됨.
float samples_num = 3;     // 스파이크 제거를 위한 부분필터에 샘플을 몇개 측정할 것인지. 3개로 충분함! 가능하면 수정하지 말 것.

void setup() {
  // initialize GPIO pins for LED and attach servo 
  pinMode(PIN_LED,OUTPUT);
  digitalWrite(PIN_LED, 1);
  myservo.attach(PIN_SERVO); 
  last_sampling_time_dist = 0;
  last_sampling_time_servo = 0;
  last_sampling_time_serial = 0;

// initialize global variables
  dist_min = _DIST_MIN;
  dist_max = _DIST_MAX;
  dist_target = _DIST_TARGET;
  alpha = _DIST_ALPHA;

  duty_target = duty_curr = 1350;

// move servo to neutral position
  myservo.writeMicroseconds(_DUTY_NEU);

// initialize serial port
  Serial.begin(57600);

// convert angle speed into duty change per interval.
  duty_chg_per_interval = (_DUTY_MAX - _DUTY_MIN) * ((float)_SERVO_SPEED / _SERVO_ANGLE) * ((float)_INTERVAL_SERVO / 1000); //[3128]
}
  
void loop() {  // Event generator

  if (millis() >= last_sampling_time_dist + _INTERVAL_DIST) {
    last_sampling_time_dist += _INTERVAL_DIST;
    event_dist = true;
  }
  if (millis() >= last_sampling_time_servo + _INTERVAL_SERVO) {
    last_sampling_time_servo += _INTERVAL_SERVO;
    event_servo = true;
  }
  if (millis() >= last_sampling_time_serial + _INTERVAL_SERIAL) {
    last_sampling_time_serial += _INTERVAL_SERIAL;
    event_serial = true;
  }

// Event handlers
  if(event_dist) {
 
    event_dist = false;
  // get a distance reading from the distance sensor
  
    dist_raw = filtered_ir_distance();
  // PID control logic
    error_curr = dist_target - dist_raw;
    P_term = _Kp * error_curr;
    I_term += error_curr * 20;
    D_term = _Kd * (error_curr - error_prev) / 20;
    control = P_term + D_term;

    //update error_prev
    error_prev = error_curr;
    
  // duty_target = f(duty_neutral, control)
    duty_target = _DUTY_NEU + control;
    
  // keep duty_target value within the range of [_DUTY_MIN, _DUTY_MAX]
      if (duty_target < _DUTY_MIN) duty_target = _DUTY_MIN; 
      if (duty_target > _DUTY_MAX) duty_target = _DUTY_MAX;

  }
  
  if(event_servo) {
    event_servo = false;
    // adjust duty_curr toward duty_target by duty_chg_per_interval
    if(duty_target > duty_curr) {
      duty_curr += duty_chg_per_interval;
      if(duty_curr > duty_target) duty_curr = duty_target;
    }  
    else {
      duty_curr -= duty_chg_per_interval;
      if(duty_curr < duty_target) duty_curr = duty_target;
    }
    // update servo position
    myservo.writeMicroseconds(duty_curr);
  }
  
  if(event_serial) {
    event_serial = false;
    
    Serial.print("dist_ir:");
    Serial.print(dist_raw);
    Serial.print(",pterm:");
    Serial.print(map(P_term,-1000,1000,510,610));
    Serial.print(",dterm:");
    Serial.print(map(D_term,-1000,1000,510,610));
    Serial.print(",duty_target:");
    Serial.print(map(duty_target,1000,2000,410,510));
    Serial.print(",duty_curr:");
    Serial.print(map(duty_curr,1000,2000,410,510));
    Serial.println(",Min:117,Low:140,dist_target:250,High:310,Max:450");
  }
}

float ir_distance(void){ // return value unit: mm 
  //distance check : Infrared sensor 
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;
  return val;
}

float under_noise_filter(void){ // 아래로 떨어지는 형태의 스파이크를 제거해주는 필터
  int currReading;
  int largestReading = 0;
  for (int i = 0; i < samples_num; i++) {

    float x = ir_distance();
    currReading = coE[0] * pow(x, 3) + coE[1] * pow(x, 2) + coE[2] * x + coE[3];
    
    if (currReading > largestReading) { largestReading = currReading; }
    // Delay a short time before taking another reading
    delayMicroseconds(DELAY_MICROS);
  }
  return largestReading;
}

float filtered_ir_distance(void){ // 아래로 떨어지는 형태의 스파이크를 제거 후, 위로 치솟는 스파이크를 제거하고 EMA필터를 적용함.
  // under_noise_filter를 통과한 값을 upper_nosie_filter에 넣어 최종 값이 나옴.
  int currReading;
  int lowestReading = 1024;
  for (int i = 0; i < samples_num; i++) {
    currReading = under_noise_filter();
    if (currReading < lowestReading) { lowestReading = currReading; }
  }
  // ema 필터 추가
  dist_ema = _DIST_ALPHA*lowestReading + (1-_DIST_ALPHA)*dist_ema;
  return dist_ema;
}
