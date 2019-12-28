#include "MotorControl.h"
#include "Arduino.h"


MotorControl::MotorControl(){}
MotorControl::~MotorControl(){}

void MotorControl::Init(const int m_13_spd_in, const int m_24_spd_in,const int m_13_dir_in,
                const int m_24_dir_in,const int led_channel_0_in,const int led_channel_1_in,
                const int led_freq_in, const int led_timer_precision_in){

              m_13_spd = m_13_spd_in;
              m_24_spd = m_24_spd_in;
              m_13_dir = m_13_dir_in;
              m_24_dir = m_24_dir_in;
              led_channel_0 = led_channel_0_in;
              led_channel_1 = led_channel_1_in;
              led_freq = led_freq_in;
              led_timer_precision = led_timer_precision_in;

              // Set all the motor control pins to outputs
              ledcSetup(led_channel_0, led_freq, led_timer_precision);
              ledcAttachPin(m_13_spd, led_channel_0);
              ledcWrite(led_channel_0, 0);

              ledcSetup(led_channel_1, led_freq, led_timer_precision);
              ledcAttachPin(m_24_spd, led_channel_1);
              ledcWrite(led_channel_1, 0);

              pinMode(m_13_dir,OUTPUT);
              digitalWrite(m_13_dir, 0);
              pinMode(m_24_dir,OUTPUT);
              digitalWrite(m_24_dir, 0);
}

void MotorControl::setMotorA(int speed,bool dir){

  ledcWrite(led_channel_0, speed);
  digitalWrite(m_13_dir,dir);
}

void MotorControl::setMotorB(int speed,bool dir){

  ledcWrite(led_channel_1, speed);
  digitalWrite(m_24_dir,dir);
}

void MotorControl::stopMotorA(){
  ledcWrite(led_channel_0, 0);
}

void MotorControl::stopMotorB(){
  ledcWrite(led_channel_1, 0);
}

void MotorControl::stopMotors(){
  stopMotorA();
  stopMotorB();
}

void MotorControl::processMotorCmd(float linear_vel_cmd,float angular_vel_cmd){


float motor_spd_left;
float motor_spd_right;
unsigned char left_spd,right_spd;
bool robot_dir;

if(linear_vel_cmd>=0){
  motor_spd_left = linear_vel_cmd;
  motor_spd_right = linear_vel_cmd;
  robot_dir = false;
}else{
  motor_spd_left = -1*linear_vel_cmd;
  motor_spd_right = -1*linear_vel_cmd;
  robot_dir = true;
}

if(angular_vel_cmd<0){
  motor_spd_left = motor_spd_left + angular_vel_cmd;
}else{
  motor_spd_right = motor_spd_right - angular_vel_cmd;
}

if(motor_spd_left >1){
  motor_spd_left = 1;
}else if (motor_spd_left < 0){
  motor_spd_left = 0;
}

if(motor_spd_right > 1){
  motor_spd_right = 1;
}else if (motor_spd_right < 0){
  motor_spd_right = 0;
}

left_spd = static_cast<unsigned char>(motor_spd_left*255);
right_spd = static_cast<unsigned char>(motor_spd_right*255);

/*Serial.print("\n Left Motor Command: \t");
Serial.print(right_spd);
Serial.print("\n Right Motor Command: \t");
Serial.print(left_spd);*/

setMotorA(right_spd,robot_dir);
setMotorB(left_spd,robot_dir);

}
