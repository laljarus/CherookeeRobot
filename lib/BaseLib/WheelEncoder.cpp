/*#include "WheelEncoder.h"
#include "Arduino.h"
#define pi 3.14159265359


WheelEncoder::WheelEncoder(){}
WheelEncoder::~WheelEncoder(){}
void WheelEncoder::init(const int m_1_encA_in,const int m_1_encB_in,const int m_2_encA_in,
          const int m_2_encB_in){

            m_1_encA = m_1_encA_in;
            m_1_encB = m_1_encB_in;
            m_2_encA = m_2_encA_in;
            m_2_encB = m_2_encB_in;

            //pinMode(m_1_encA,INPUT_PULLUP);
            //pinMode(m_2_encA,INPUT_PULLUP);
            pinMode(m_1_encB,INPUT);
            pinMode(m_2_encB,INPUT);

            //attachInterrupt(digitalPinToInterrupt(m_1_encA),InterruptRoutine_m1,RISING);
            //attachInterrupt(digitalPinToInterrupt(m_1_encB),InterruptRoutine_m2,RISING);

            dir1 = true;
            dir2 = true;

          }
float WheelEncoder::getPositionA(){

  float position = count1 * wheelCircumference / (pulses_per_rotation * gearRatio) ;

  return position;
}

float WheelEncoder::getPositionB(){

  float position = count2 * wheelCircumference / (pulses_per_rotation * gearRatio) ;

  return position;
}

float WheelEncoder::getSpeedA(){
  return 0.0;
}

float WheelEncoder::getSpeedB(){
  return 0.0;
}

void IRAM_ATTR InterruptRoutine_m1(){
  int val = digitalRead(m_1_encB);
  if(val){
    dir1 = true;
    count1++;
  }else{
    dir1 = false;
    count1--;
  }
}

void IRAM_ATTR InterruptRoutine_m2(){
  int val = digitalRead(m_2_encB);
  if(val){
    dir2 = true;
    count2++;
  }else{
    dir2 = false;
    count2--;
  }
}
*/
