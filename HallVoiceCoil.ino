#include <PID_v1.h>

#define ON 1
#define OFF 0

const  byte hall_data[1024] = {
  100,  100,  100,  100,  100,  100,  100,  100,  100,  100,  
  100,  100,  100,  100,  100,  100,  100,  100,  100,  100,  
  100,  100,  100,  100,  100,  100,  100,  100,  100,  100,  
  100,  100,  100,  100,  100,  100,  100,  100,  100,  100,  
  100,  100,  100,  100,  100,  100,  100,  100,  100,  100,  
  100,  100,  100,  100,  100,  100,  100,  100,  100,  100,  
  100,  100,  100,  100,  100,  100,  100,  100,  100,  100,  
  100,  100,  100,  100,  100,  100,  100,  100,  100,  100,  
  100,  100,  100,  100,  100,  100,  100,  100,  100,  100,  
  100,  100,  100,  100,  100,  100,  100,  100,  100,  100,  
  100,  100,  100,  100,  100,  100,  100,  100,  100,  100,  
  100,  100,  100,  100,  100,  100,  100,  100,  100,  100,  
  100,  100,  100,  100,  100,  100,  100,  100,  100,  100,  
  100,  100,  100,  100,  100,  100,  100,  100,  100,  100,  
  100,  100,  100,  100,  100,  100,  100,  100,  100,  100,  
  100,  100,  100,  100,  100,  100,  100,  100,  100,  100,  
  100,  100,  100,  100,  100,  100,  100,  100,  100,  100,  
  100,  100,  100,  100,  100,  100,  100,  100,  100,  100,  
  100,  100,  100,  100,  100,  100,  100,  100,  100,  100,  
  100,  100,  100,  100,  100,  100,  100,  100,  100,  100,  
  100,  100,  100,  100,  100,  100,  100,  100,  100,  100,  
  100,  100,  100,  100,  100,  100,  100,  100,  100,  100,  
  100,  100,  100,  100,  100,  100,  100,  100,  100,  100,  
  100,  100,  100,  100,  100,  100,  100,  100,  100,  100,  
  100,  100,  100,  100,  100,  100,  100,  100,  100,  100,  
  100,  100,  100,  100,  100,  100,  100,  100,  100,  100,  
  100,  100,  100,  100,  100,  100,  100,  100,  100,  100,  
  100,  100,  100,  100,  100,  100,  100,  100,  100,  100,  
  100,  100,  100,  100,  100,  100,  100,  100,  100,  100,  
  100,  100,  100,  100,  100,  100,  100,  100,  100,  100,  
  100,  100,  100,  100,  100,  100,  100,  100,  100,  100,  
  100,  100,  100,  100,  100,  100,  100,  100,  100,  100,  
  100,  100,  100,  100,  100,  100,  100,  100,  100,  100,  
  100,  100,  100,  100,  100,  100,  100,  100,  100,  100,  
  100,  100,  100,  100,  100,  100,  100,  100,  100,  100,  
  100,  100,  100,  100,  100,  100,  100,  100,  100,  100,  
  100,  100,  100,  100,  100,  100,  100,  100,  100,  100,  
  100,  100,  100,  100,  100,  100,  100,  100,  100,  100,  
  100,  100,  100,  100,  100,  100,  100,  100,  100,  100,  
  100,  100,  100,  100,  100,  100,  100,  100,  100,  100,  
  100,  100,  100,  100,  100,  100,  100,  100,  100,  100,  
  100,  100,  100,  100,  100,  100,  100,  100,  100,  100,  
  100,  100,  100,  100,  100,  100,  100,  100,  100,  100,  
  100,  100,  100,  100,  100,  100,  100,  100,  100,  100,  
  100,  100,  100,  100,  100,  100,  100,  100,  100,  100,  
  100,  100,  100,  100,  100,  100,  100,  100,  100,  100,  
  100,  100,  100,  100,  100,  100,  100,  100,  100,  100,  
  100,  100,  100,  100,  100,  100,  100,  100,  100,  100,  
  100,  100,  100,  100,  100,  100,  100,  100,  100,  100,  
  100,  100,  100,  100,  100,  100,  100,  100,  100,  100,  
  100,  100,  100,  100,  100,  100,  100,  100,  100,  100,  
  100,  100,  92,  88,  85,  82,  80,  78,  75,  73,  
  71,  69,  66,  64,  62,  60,  57,  55,  54,  52,  
  50,  49,  48,  47,  46,  45,  44,  43,  43,  42,  
  41,  41,  40,  40,  39,  39,  38,  38,  37,  37,  
  36,  36,  36,  35,  35,  35,  34,  34,  34,  34,  
  33,  33,  32,  32,  32,  32,  31,  31,  31,  31,  
  30,  30,  30,  30,  29,  29,  29,  29,  29,  28,  
  28,  28,  28,  28,  27,  27,  27,  27,  27,  27,  
  26,  26,  26,  26,  26,  26,  25,  25,  25,  25,  
  25,  25,  24,  24,  24,  24,  24,  24,  24,  23,  
  23,  23,  23,  23,  23,  23,  23,  22,  22,  22,  
  22,  22,  22,  22,  22,  21,  21,  21,  21,  21,  
  21,  21,  21,  20,  20,  20,  20,  20,  20,  20,  
  20,  20,  20,  19,  19,  19,  19,  19,  19,  19,  
  19,  19,  19,  18,  18,  18,  18,  18,  18,  18,  
  18,  18,  18,  17,  17,  17,  17,  17,  17,  17,  
  17,  17,  17,  17,  17,  16,  16,  16,  16,  16,  
  16,  16,  16,  16,  16,  16,  16,  15,  15,  15,  
  15,  15,  15,  15,  15,  15,  15,  15,  15,  15,  
  14,  14,  14,  14,  14,  14,  14,  14,  14,  14,  
  14,  14,  14,  13,  13,  13,  13,  13,  13,  13,  
  13,  13,  13,  13,  13,  13,  13,  13,  12,  12,  
  12,  12,  12,  12,  12,  12,  12,  12,  12,  12,  
  12,  12,  12,  12,  11,  11,  11,  11,  11,  11,  
  11,  11,  11,  11,  11,  11,  11,  11,  11,  11,  
  10,  10,  10,  10,  10,  10,  10,  10,  10,  10,  
  10,  10,  10,  10,  10,  10,  10,  10,  9,  9,  
  9,  9,  9,  9,  9,  9,  9,  9,  9,  9,  
  9,  9,  9,  9,  9,  9,  8,  8,  8,  8,  
  8,  8,  8,  8,  8,  8,  8,  8,  8,  8,  
  8,  8,  8,  8,  8,  8,  7,  7,  7,  7,  
  7,  7,  7,  7,  7,  7,  7,  7,  7,  7,  
  7,  7,  7,  7,  7,  7,  7,  6,  6,  6,  
  6,  6,  6,  6,  6,  6,  6,  6,  6,  6,  
  6,  6,  6,  6,  6,  6,  6,  6,  6,  5,  
  5,  5,  5,  5,  5,  5,  5,  5,  5,  5,  
  5,  5,  5,  5,  5,  5,  5,  5,  5,  5,  
  5,  5,  5,  4,  4,  4,  4,  4,  4,  4,  
  4,  4,  4,  4,  4,  4,  4,  4,  4,  4,  
  4,  4,  4,  4,  4,  4,  4,  3,  3,  3,  
  3,  3,  3,  3,  3,  3,  3,  3,  3,  3,  
  3,  3,  3,  3,  3,  3,  3,  3,  3,  3,  
  3,  3,  3,  3,  2,  2,  2,  2,  2,  2,  
  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  
  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  
  2,  1,  1,  1,  1,  1,  1,  1,  1,  1,  
  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  
  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  
  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  
  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  
  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  
  0,  0,  0,  0,  };



class Store {
public:
  uint16_t pwm;
  uint16_t hall;
  uint16_t delta_t;
  uint16_t pot;
};

// vertical voice coil
int pinI1=8;//define I1 interface
int pinI2=11;//define I2 interface 
int speedpinA=9;//enable motor A

// horizontal voice coil
int pinI3=12;//define I3 interface 
int pinI4=13;//define I4 interface 
int speedpinB=10;//enable motor B

int laser_port = 2; // output for controlling laser
int las_sw = 3; // input for enabled switch

unsigned long start_time;

// basic hardware layer
void setup_hardware(void)
{
  start_time = millis();
  Serial.begin(9600);
  // motor setup
  // vertical motor
  //pinMode(pinI1,OUTPUT);
  //pinMode(pinI2,OUTPUT);
  //pinMode(speedpinA,OUTPUT);

  // horizontal motor
  pinMode(pinI3,OUTPUT);
  pinMode(pinI4,OUTPUT);
  pinMode(speedpinB,OUTPUT);

  pinMode(laser_port, OUTPUT);
  pinMode(las_sw, INPUT);
}

void laser(int state){
  if(state != OFF){
    digitalWrite(laser_port, HIGH);
  } 
  else {
    digitalWrite(laser_port, LOW);
  }
}

int read_pot(void)
{
  int pot = analogRead(1);   // Range : 0..1023
  return pot;
}

int read_hall(void)
{
  return analogRead(0);
}

int enabled(void)
{
  return digitalRead(las_sw);
}

void pulse_right(int pwm){
  analogWrite(speedpinB,pwm);
  digitalWrite(pinI4,HIGH);//turn DC Motor B move clockwise
  digitalWrite(pinI3,LOW);
}

void pulse_left(int pwm)
{
  analogWrite(speedpinB,pwm);
  digitalWrite(pinI4,LOW);//turn DC Motor B move clockwise
  digitalWrite(pinI3,HIGH);
}

void relax(){
  analogWrite(speedpinB,0);
  digitalWrite(pinI4,LOW);//turn DC Motor B move clockwise
  digitalWrite(pinI3,LOW);
}

// PID LOOP START HERE -------------------------------------
double Setpoint, Input, Output;
double kp = .35;
double kd = .003;
double ki = 0.15;
//int positions[5] = {
//  512, 516, 529, 575, 821};
//{821, 575, 529, 516, 512};

PID myPID(&Input, &Output, &Setpoint,kp,ki,kd, DIRECT);

int scaled_pot(void)
{
  float pot = (float)read_pot();   // Range : 0..1023
  int setpoint = (int) ((pot/1024.0)*100.0);
  return setpoint;
}

int angle_hall(void)
{
  return hall_data[(read_hall())];
}

void setup_pid()
{
  myPID.SetOutputLimits(-25, 25);
  Input = angle_hall();
  Setpoint = scaled_pot();
  myPID.SetMode(AUTOMATIC);
}

void pid_loop()
{

  Input = angle_hall();
  Setpoint = scaled_pot();
  myPID.Compute();

  //*
  Serial.print("setpoint:  ");
  Serial.println(Setpoint);
  Serial.print("Input: ");
  Serial.println(Input);
  Serial.print("Output:  ");
  Serial.println(Output);
  // */

  int las = enabled();
  if(las){
    laser(ON);
  } 
  else {
    laser(OFF);
  }

  unsigned pwm = abs(Output);
  pwm = (pwm > 35)?35:pwm; // cap at 35
  pwm = (pwm < 2)?0:pwm; // dead band
  if(pwm == 0){
    relax();
  } 
  else {
    if(Output > 0){
      Serial.print("left with duty cycle: ");
      if(las){
        pulse_left(pwm);
      }
    } 
    else {
      Serial.print("right with duty cycle: ");
      if(las){
        pulse_right(pwm);
      }
    }

  }

  Serial.println(pwm);
  Serial.println("");
  if(!las){
    // shut off motor
    relax();
    delay(500);
  }
}

// END OF PID LOOP STUFF ------------------------

void setup() 
{
  setup_hardware();
  setup_pid();
}

// figure out the effect of different pot settings
void pot_loop()
{
  int pot = read_pot();
  Serial.print("Pot reading: ");
  Serial.println(pot);
  pot >>= 1;
  int setpoint = pot - 256;
  unsigned pwm = abs(setpoint);
  pwm = (pwm > 255)?255:pwm;
  if(enabled())
  {
    laser(ON);
    if(setpoint < 0){
      // backward / left
      Serial.print("left with duty cycle: ");
      pulse_left(pwm);
    } 
    else {
      // forward / right
      Serial.print("right with duty cycle: ");
      pulse_right(pwm);
    }
  } 
  else {
    Serial.print("disabled with pwm: ");
    laser(OFF);
    relax();
  }
  Serial.println(pwm);
  delay(250);
}

// flap hard left and right to get sensor readings
void flap_loop()
{
  if(enabled()){
    int raw;

    // forward / right
    Serial.println("Flapping right/forward");
    pulse_right(255);
    delay(250);
    raw = read_hall();
    Serial.print("Hall reading:  ");
    Serial.println(raw);
    relax();
    delay(1000);
    raw = read_hall();   // Range : 0..1024
    Serial.print("(relax from right) Hall reading:  ");
    Serial.println(raw);

    // backward / left
    Serial.println("Flapping left/back");
    pulse_left(255);
    delay(250);
    raw = read_hall();
    Serial.print("Hall reading:  ");
    Serial.println(raw);
    relax();
    delay(1000);
    raw = read_hall();   // Range : 0..1024
    Serial.print("(relax from left) Hall reading:  ");
    Serial.println(raw);

    raw = read_hall();   // Range : 0..1024
    Serial.print("Hall reading:  ");
    Serial.println(raw);

    Serial.println("Sleeping for 3 seconds...");
    delay(3000);
  } 
  else {
    Serial.println("Disabled.");
    delay(250);
  }
}


// try to figure step test data
// print out hall, pwm, potentiometer
unsigned long last_time;
Store step_data[512];

void store_hall(int off, int pwm)
{
  unsigned long now = millis();
  unsigned int delta = now - last_time;

  step_data[off].pwm = pwm;
  step_data[off].pot = read_pot();
  step_data[off].hall = read_hall();
  step_data[off].delta_t = delta;

  last_time = now;
}

void dump_hall(void)
{
  for(int i=0; i<512; i++){
    laser(ON);
    String out = String(step_data[i].hall) + "," + String(step_data[i].pwm) + "," + String(step_data[i].pot) + "," + String(step_data[i].delta_t);
    Serial.println(out);
    laser(OFF);
    delay(1);
  }
  last_time = millis();
}  
void report_hall(int pwm)
{
  unsigned long now = millis();

  unsigned long delta = now - last_time;
  String out = String(read_hall()) + "," + String(pwm) + "," + String(read_pot()) + "," + String(delta);
  Serial.println(out);
  last_time = now;
}

void step_loop(void)
{
  int duty = 25;
  int max_steps = 4;
  int pause = 10;
  int i, j;
  int run;
  int right=1;

  if(enabled()){

    last_time = millis();
    for(run=0; run<max_steps; run++){
      //Serial.println("Run: "+String(run)+" duty: "+String(duty)+" pause: "+String(pause)+" right:  "+String(right));
      //Serial.println("hall,pwm,pot,delta_t");

      laser(ON);
      pulse_right(255);
      delay(250);
      relax();
      laser(OFF);
      laser(ON);
      pulse_left(255);
      delay(250);
      relax();
      laser(OFF);

      delay(2000);
      report_hall(0);

      last_time = millis();
      laser(ON);
      if(right){
        pulse_right(duty);
      } 
      else {
        pulse_left(duty);
      }
      for(i=0; i < (run+1)*pause; i++){
        store_hall(i, duty);
        delay(1);
      }
      relax();
      laser(OFF);
      for(; i<512; i++){
        store_hall(i, 0);
        delay(1);
      }
      delay(10);
      dump_hall();
    } // run
    delay(50);
  } 
  else {
    report_hall(0);
    delay(1000);
  }  
}

// just print out the sensor states
void report_loop()
{
  laser(ON);
  while(1){
    report_hall(0);
    delay(50);
  }
}

void loop()
{
  pid_loop();
  //flap_loop();

  //report_loop();
  //step_loop();
  //pot_loop();
}





