#include <Wire.h>
#include <ros.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Int16.h>

// Node initialisation
ros::NodeHandle nh;

int InA1 = 5; 
int InA2 = 6;
int enabA = 7; // Enable pin

int InB1 = 8; 
int InB2 = 9;
int enabB = 10; // Enable pin
const int led = LED_BUILTIN;

int counter = 0;



// subscriber callback , when new message comes this will be called by susbscriber
void PWM_CMD_R(const std_msgs::Int16& cmd_msg_R)
{ 
  int pwm_cmdr = cmd_msg_R.data; // right wheel speed

  
  // for RIGHTT side wheels,
  if (pwm_cmdr < 0)
  {
    analogWrite(InB1,0); 
    analogWrite(InB2,abs(pwm_cmdr));
    digitalWrite(enabB,HIGH); 
  }  
  else if (pwm_cmdr > 0) 
  {
    analogWrite(InB2,0);
    analogWrite(InB1,abs(pwm_cmdr)); 
    digitalWrite(enabB,HIGH); //writing pwm to motor
  }
  else 
  {
    analogWrite(InB1,0);
    analogWrite(InB2,0);
    digitalWrite(enabB,HIGH);
  }
}


void PWM_CMD_L(const std_msgs::Int16& cmd_msg_L)
{ 
  int pwm_cmdl = cmd_msg_L.data; // left wheel speed

  // for LEFT side wheels,
  if (pwm_cmdl > 0)
  { 
    analogWrite(InA1,0); 
    analogWrite(InA2,abs(pwm_cmdl));
    digitalWrite(enabA,HIGH); 
  }  
  else if (pwm_cmdl < 0) 
  {
    analogWrite(InA2,0);
    analogWrite(InA1,abs(pwm_cmdl)); 
    digitalWrite(enabA,HIGH); 
  }
  else
  {
    analogWrite(InA1,0);
    analogWrite(InA2,0);
    digitalWrite(enabA,HIGH); 
  }
}

ros::Subscriber<std_msgs::Int16> subR("pwm_cmd_r",PWM_CMD_R);
ros::Subscriber<std_msgs::Int16> subL("pwm_cmd_l",PWM_CMD_L);



// Encoder Pin numbers and state and Encoder Reading
 
 // For the right encoder
 int enco_R_A = 35;
 int enco_R_B = 30;
 volatile int enco_R_pos = 0;
 int enco_R_ALast = LOW;
 int enco_R_n = LOW;

 // For the left encoder
 int enco_L_A = 2;
 int enco_L_B = 3;
 volatile int enco_L_pos = 0;
 int enco_L_ALast = LOW;
 int enco_L_n = LOW;

std_msgs::Int16 msg_R;
std_msgs::Int16 msg_L;
ros::Publisher enco_R("enco_R", &msg_R);
ros::Publisher enco_L("enco_L", &msg_L);

//sonar
int sonar_pin = 22;
long duration;
std_msgs::Int16 msg_sonar;
ros::Publisher distance_sensor("distance_sensor", &msg_sonar);

void setup() 
{
  Serial.begin(57600);
  enco_R_pos = 0;
  enco_L_pos = 0;
  pinMode(enabA,OUTPUT);
  digitalWrite(enabA,LOW); 
  pinMode(InA1,OUTPUT);
  analogWrite(InA1,0);
  pinMode(InA2,OUTPUT);
  analogWrite(InA2,0);

  pinMode(enabB,OUTPUT);
  digitalWrite(enabB,LOW); 
  pinMode(InB1,OUTPUT);
  analogWrite(InB1,0);
  pinMode(InB2,OUTPUT);
  analogWrite(InB2,0);

  analogWriteFrequency(InA1,20000);
  analogWriteFrequency(InA2,20000);
  analogWriteFrequency(InB1,20000);
  analogWriteFrequency(InB2,20000);

  // Define pinmode for the pins reading encoder data
  pinMode (enco_R_A,INPUT);
  pinMode (enco_R_B,INPUT);
  pinMode (enco_L_A,INPUT);
  pinMode (enco_L_B,INPUT);

  //sonar
  pinMode (sonar_pin,INPUT);
 
  pinMode(led, OUTPUT);
 
  attachInterrupt(enco_R_A, doEncoder_R_A, CHANGE);
  attachInterrupt(enco_R_B, doEncoder_R_B, CHANGE);
  attachInterrupt(enco_L_A, doEncoder_L_A, CHANGE);
  attachInterrupt(enco_L_B, doEncoder_L_B, CHANGE);

  nh.initNode();
  
  nh.subscribe(subR);
  nh.subscribe(subL);
  nh.advertise(enco_R);
  nh.advertise(enco_L);
  
  //sonar
  nh.advertise(distance_sensor);
}

void doEncoder_R_A() 
{
  // look for a low-to-high on channel A
  if (digitalRead(enco_R_A) == HIGH) 
  {

    // check channel B to see which way encoder is turning
    if (digitalRead(enco_R_B) == LOW) {
      enco_R_pos = enco_R_pos + 1;         // CW
    }
    else {
      enco_R_pos = enco_R_pos - 1;         // CCW
    }
  }

  else   // must be a high-to-low edge on channel A
  {
    // check channel B to see which way encoder is turning
    if (digitalRead(enco_R_B) == HIGH) {
      enco_R_pos = enco_R_pos;// + 1;          // CW
    }
    else {
      enco_R_pos = enco_R_pos;// - 1;          // CCW
    }
  }

  //msg_R.data = enco_R_pos;
  //msg_L.data = enco_L_pos;
  
  //enco_R.publish(&msg_R);
  //enco_L.publish(&msg_L);
  
  //Serial.println (encoder0Pos, DEC);
  // use for debugging - remember to comment out
}

void doEncoder_R_B()
{
  // look for a low-to-high on channel B
  if (digitalRead(enco_R_B) == HIGH) 
  {

    // check channel A to see which way encoder is turning
    if (digitalRead(enco_R_A) == HIGH) 
    {
      enco_R_pos = enco_R_pos;// + 1;         // CW
    }
    else 
    {
      enco_R_pos = enco_R_pos;// - 1;         // CCW
    }
  }

  // Look for a high-to-low on channel B

  else {
    // check channel B to see which way encoder is turning
    if (digitalRead(enco_R_A) == LOW) 
    {
      enco_R_pos = enco_R_pos ;//+ 1;          // CW
    }
    else 
    {
      enco_R_pos = enco_R_pos ;//- 1;          // CCW
    }
  }

  //msg_R.data = enco_R_pos;
  //msg_L.data = enco_L_pos;
  
  //enco_R.publish(&msg_R);
  //enco_L.publish(&msg_L);
}

void doEncoder_L_A() 
{
  // look for a low-to-high on channel A
  if (digitalRead(enco_L_A) == HIGH) 
  {

    // check channel B to see which way encoder is turning
    if (digitalRead(enco_L_B) == LOW) {
      enco_L_pos = enco_L_pos + 1;         // CW
    }
    else {
      enco_L_pos = enco_L_pos - 1;         // CCW
    }
  }

  else   // must be a high-to-low edge on channel A
  {
    // check channel B to see which way encoder is turning
    if (digitalRead(enco_L_B) == HIGH) {
      enco_L_pos = enco_L_pos;// + 1;          // CW
    }
    else {
      enco_L_pos = enco_L_pos;// - 1;          // CCW
    }
  }
}

void doEncoder_L_B()
{
  // look for a low-to-high on channel B
  if (digitalRead(enco_L_B) == HIGH) 
  {

    // check channel A to see which way encoder is turning
    if (digitalRead(enco_L_A) == HIGH) 
    {
      enco_L_pos = enco_L_pos;// + 1;         // CW
    }
    else 
    {
      enco_L_pos = enco_L_pos ;//- 1;         // CCW
    }
  }

  // Look for a high-to-low on channel B

  else {
    // check channel B to see which way encoder is turning
    if (digitalRead(enco_L_A) == LOW) 
    {
      enco_L_pos = enco_L_pos;// + 1;          // CW
    }
    else 
    {
      enco_L_pos = enco_L_pos ;//- 1;          // CCW
    }
  }
}

void loop()
{
  msg_R.data = 1.02487*enco_R_pos;//1.021*enco_R_pos;//1.03193754*  
  msg_L.data = enco_L_pos;
  
  enco_R.publish(&msg_R);
  enco_L.publish(&msg_L);

  duration = analogRead(sonar_pin);
  msg_sonar.data = (duration * 5) / 25.4;
  distance_sensor.publish(&msg_sonar);

  nh.spinOnce();  
  digitalWrite(led, HIGH);
  delay(10);
  
}
