#ifndef _MOTORCONTROL_H_
#define _MOTORCONTROL_H_
#include "PID.h"

class MotorControl {

  public:
    MotorControl();

    void Init(const int m_13_spd_in, const int m_24_spd_in,const int m_13_dir_in,const int m_24_dir_in,
              const int led_channel_0_in,const int led_channel_1_in,
              const int led_freq_in, const int led_timer_precision_in);

    void setMotorA(int speed,bool dir);
    void setMotorB(int speed,bool dir);
    void stopMotorA();
    void stopMotorB();
    void stopMotors();
    //void emergencyStop();
    void processMotorCmd(float linear_vel_cmd,float angular_vel_cmd);
    virtual ~MotorControl();

  private:
    int m_13_spd,m_24_spd,m_13_dir,m_24_dir,led_channel_0,led_channel_1,led_freq,led_timer_precision;
    PID pid_steer;

};



#endif
