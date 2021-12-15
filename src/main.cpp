#include <Arduino.h>
//#define ESP32
#include "WiFi.h"
#include "ros.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "MotorControl.h"
//#include "WheelEncoder.h"

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
/*#include "I2Cdev.h"
#include "MPU6050.h"

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif
*/

const char *ssid = "KeyHouse";
const char *password = "PM24_m19_ac68u";
// Set the rosserial socket server IP address
IPAddress server(192,168,1,37);
// Set the rosserial socket server port
const uint16_t serverPort = 11411;

ros::NodeHandle nh;
geometry_msgs::Vector3Stamped MotorSpeeds;
geometry_msgs::Vector3 SensValues_m3;
geometry_msgs::Vector3 SensValues_m4;
float linear_vel_cmd = 0;
float angular_vel_cmd = 0;
float throttle = 0;
float steering = 0;

void motor_cmd_cb(const geometry_msgs::Vector3 &msg){

  throttle = msg.x;
  steering = msg.y;
}

ros::Subscriber<geometry_msgs::Vector3> motor_cmd("motor_cmd", &motor_cmd_cb);
ros::Publisher MotSpd("MotorSpeeds", &MotorSpeeds);
ros::Publisher LeftSens("LeftSensor",&SensValues_m3);
ros::Publisher RightSens("RightSensor",&SensValues_m4);


const int m_13_dir = 13;
const int m_13_spd = 12;
const int m_24_spd = 14;
const int m_24_dir = 27;

const int m_4_encA = 36;
const int m_4_encB = 32;

const int m_3_encA = 26;
const int m_3_encB = 25;

int rpm_left =0;
int rpm_right = 0;
unsigned char left_motor_cmd = 0;
unsigned char right_motor_cmd = 0;

static bool nhChange = false;
static bool nhOldState  = false;


// use first channel of 16 channels (started from zero)
#define LEDC_CHANNEL_0     0
#define LEDC_CHANNEL_1     1

// use 13 bit precission for LEDC timer
#define LEDC_TIMER_8_BIT  8

// use 5000 Hz as a LEDC base frequency
#define LEDC_BASE_FREQ     1000

// Time in seconds for the timer
#define timeSeconds 1

unsigned long now = millis();
unsigned long lastTrigger = 0;
unsigned int count_left = 0;
unsigned int count_right= 0;
unsigned int num_tooth_whl = 20;

MotorControl DiffDriveMotors;

volatile bool dir_mot3 = false;
volatile bool dir_mot4 = false;
volatile int count_mot3 = 0;
volatile int count_mot4 = 0;

static bool m3_sensA = false;
static bool m3_sensB = false;
static bool m4_sensA = false;
static bool m4_sensB = false;
static bool interruptCheck = false;

void IRAM_ATTR InterruptRoutine_m3(){
  /*int val = digitalRead(m_3_encB);
  if(val){
    dir_mot3 = true;
    count_mot3++;
  }else{
    dir_mot3 = false;
    count_mot3--;
  }*/
  if(interruptCheck){
    interruptCheck = false;
  }else{
    interruptCheck = true;
  }
  count_mot3++;
}

void IRAM_ATTR InterruptRoutine_m4(){
  /*int val = digitalRead(m_4_encB);
  if(val){
    dir_mot4 = true;
    count_mot4--;
  }else{
    dir_mot4 = false;
    count_mot4++;
  }*/
  count_mot4++;
}

void setup(){

  Serial.begin(115200);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  // Connect the ESP8266 the the wifi AP
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.print("");
  Serial.print("WiFi connected");
  Serial.print("IP address: ");
  Serial.print(WiFi.localIP());

  // Set the connection to rosserial socket server
  nh.getHardware()->setConnection(server, serverPort);
  nh.initNode();

  // Another way to get IP
  Serial.print("IP = ");
  Serial.print(nh.getHardware()->getLocalIP());

  DiffDriveMotors.Init(m_13_spd,m_24_spd,m_13_dir,m_24_dir,LEDC_CHANNEL_0,LEDC_CHANNEL_1,
                      LEDC_BASE_FREQ,LEDC_TIMER_8_BIT);

  nh.advertise(MotSpd);
  nh.advertise(LeftSens);
  nh.advertise(RightSens);
  nh.subscribe(motor_cmd);

  MotorSpeeds.header.seq = 0;
  MotorSpeeds.header.frame_id = "Motor Speeds";

  pinMode(m_4_encA,INPUT_PULLUP);
  pinMode(m_4_encB,INPUT);

  pinMode(m_3_encA,INPUT_PULLUP);
  pinMode(m_3_encB,INPUT);

  attachInterrupt(m_3_encA,InterruptRoutine_m3,RISING);
  attachInterrupt(m_4_encA,InterruptRoutine_m4,RISING);
  dir_mot3 = true;
  dir_mot4 = true;

}

void loop(/* arguments */) {
  /* code */
  if(nh.connected()){

    //m_3_encA_val = digitalRead(m_3_encA);
    //m_3_encB_val = digitalRead(m_3_encB);

    m3_sensA = digitalRead(m_3_encA);
    m3_sensB = digitalRead(m_3_encB);
    m4_sensA = digitalRead(m_4_encA);
    m4_sensB = digitalRead(m_4_encB);

    SensValues_m3.x = float(m3_sensA);
    SensValues_m3.y = float(m3_sensB);
    SensValues_m3.z = float(dir_mot3);
    LeftSens.publish(&SensValues_m3);

    SensValues_m4.x = float(m4_sensA);
    SensValues_m4.y = float(m4_sensB);
    SensValues_m4.z = float(dir_mot4);
    RightSens.publish(&SensValues_m4);


    DiffDriveMotors.processMotorCmd(throttle,steering);

    MotorSpeeds.header.stamp = nh.now();
    MotorSpeeds.vector.x = float(count_mot3);
    MotorSpeeds.vector.y = float(count_mot4);
    //MotorSpeeds.vector.x = float(0.0);
    //MotorSpeeds.vector.y = float(0.0);
    MotorSpeeds.vector.z = float(interruptCheck);
    MotSpd.publish(&MotorSpeeds);
    Serial.print("\n Left Motor Speed: \t");
    Serial.println(count_mot3);
    Serial.print("\n Right Motor Speed: \t");
    Serial.println(count_mot4);


  }else{
    DiffDriveMotors.stopMotors();
    Serial.print("\n Left Motor Speed: \t");
    Serial.println(count_mot3);
    Serial.print("\n Right Motor Speed: \t");
    Serial.println(count_mot4);
  }


  //detachInterrupt(m_3_encA);
  //detachInterrupt(m_4_encA);
  nh.spinOnce();
  //attachInterrupt(m_3_encA,InterruptRoutine_m3,RISING);
  //attachInterrupt(m_4_encA,InterruptRoutine_m4,RISING);
}