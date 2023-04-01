#include <MadgwickAHRS.h>
#include "CurieTimerOne.h"
#include <ros.h>
#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps_V6_12.h"
#include<std_msgs/Int32.h>
#include<geometry_msgs/Twist.h>
//#include<Servo.h>


//creality melzi
#define X_STEP_PIN         8
#define X_DIR_PIN          7
#define Y_STEP_PIN         10
#define Y_DIR_PIN          11
#define XY_ENABLE_PIN       12
#define SERVO_PIN         3

#define USE_MPU6050_DMP
#define MAX_STEP_RATE 5000
#define ISR_RATE 30000.0
#define CURIE_ACC_RANGE 2
#define CURIE_GYR_RANGE 250

#define LOOP_INTERVAL_MS 10.0

#define MICROSTEPS 8.0
#define FULL_STEPS_REV 200.0
#define STEPS_REV (MICROSTEPS*FULL_STEPS_REV)
#define WHEEL_DIAMETER_M 0.080f
#define M_PER_REV (WHEEL_DIAMETER_M*3.141569)
#define M_PER_STEP  M_PER_REV/STEPS_REV
#define STEPS_PER_M STEPS_REV/M_PER_REV
#define WHEELBASE_M 0.17

// rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0
// rosrun teleop_twist_keyboard teleop_twist_keyboard.py

 #define USE_ROS
float lSpdOffset = 0;
float rSpdOffset = 0;

#define BALANCE
#undef BALANCE



MPU6050 mpu;
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
int16_t ax, ay, az;
int16_t gx, gy, gz;

//Servo servo;

volatile long stepsLeft = 0, stepsRight = 0;
volatile long lCnt, rCnt;
volatile int lRate = 0;
volatile int rRate = 0;
unsigned long microsPerCycle, microsPrevious, microsNow;
float odometry = 0;

float angleMeas = 0;
float angleSetpoint = 0;
float angleCmdPoint = 0;
float wheelSpdBalance = 0;
float odometrySetpoint = 0;
float odometryError = 0;
float oldMeas = 0;

float dErr[5], iErr[5], lErr[5];
float pidOut[5];
float Kp_b[5] = {10,  2200.0, 0, 0, 0};
float Ki_b[5] = {0.000,  0.0, 0, 0, 0};
float Kd_b[5] = {0.5, 1500.00, 0, 0, 0};

#ifdef USE_ROS
void lwheel_spd_cb( const geometry_msgs::Twist &msg)
{
  //msg vel in m/s or rad/s
  //must convert in steps/s

  float xlv = msg.linear.x * STEPS_PER_M;
  float zrv = (msg.angular.z * 0.5 * STEPS_PER_M) / (3.141569 * WHEELBASE_M) ;

angleSetpoint=msg.linear.x ;

  lSpdOffset = ((xlv + zrv));
  rSpdOffset = ((xlv - zrv));
}


ros::NodeHandle  nh;
std_msgs::Int32 left_ticks;
std_msgs::Int32 right_ticks;
ros::Publisher lticksPub("left_ticks", &left_ticks);
ros::Publisher rticksPub("right_ticks", &right_ticks);
ros::Subscriber<geometry_msgs::Twist> subTwist("cmd_vel", &lwheel_spd_cb );
#endif

float PID(int PIDindex, float setpoint, float actual, float dt, float max_out)
{
  float err = setpoint - actual;  

  //anti-windup integrator
  if(abs(PIDindex)<max_out) iErr[PIDindex] += err * dt;
  
  dErr[PIDindex] = (err - lErr[PIDindex]) / dt;
  lErr[PIDindex] = err;

  pidOut[PIDindex]= Kp_b[PIDindex] * err + Ki_b[PIDindex] * iErr[PIDindex] + Kd_b[PIDindex] * dErr[PIDindex];
  return pidOut[PIDindex];
}

float convertRawAcceleration(int aRaw, float range)
{
  float a = (aRaw * range) / 32768.0;
  return a;
}

float convertRawGyro(int gRaw, float range)
{
  float g = (gRaw * range) / 32768.0;
  return g;
}

uint16_t pwmDiv = 0, pwmCnt = 0;

void genPWM(int out_pin, int freq, float duty, long calling_freq)
{
  pwmCnt++;
  if (pwmDiv++ > calling_freq / freq)
  {
    pwmCnt = 0;
  }

  digitalWrite(SERVO_PIN, pwmCnt > duty);
}

void timedBlinkIsr()
{
  lCnt++;
  rCnt++;
  if (lCnt > abs(lRate) && lRate != 0)
  {
    //direction
    if (lRate < 0)
    {
      digitalWrite(X_DIR_PIN, HIGH);
      stepsLeft--;
    }
    else
    {
      digitalWrite(X_DIR_PIN, LOW);
      stepsLeft++;
    }
    lCnt = 0;
    //X_STEP CREALITY MELZI
    digitalWrite(X_STEP_PIN, HIGH);
    digitalWrite(X_STEP_PIN, HIGH);
    digitalWrite(X_STEP_PIN, LOW);
    digitalWrite(X_STEP_PIN, LOW);
  }

  if (rCnt > abs(rRate) && rRate != 0)
  {
    if (rRate < 0)
    {
      digitalWrite(Y_DIR_PIN, LOW);
      stepsRight--;
    }
    else
    {
      digitalWrite(Y_DIR_PIN, HIGH);
      stepsRight++;
    }
    rCnt = 0;
    //Y_STEP_PIN CREALITY MELZI
    digitalWrite(Y_STEP_PIN, HIGH);
     digitalWrite(Y_STEP_PIN, HIGH);
    digitalWrite(Y_STEP_PIN, LOW);
    digitalWrite(Y_STEP_PIN, LOW);
  }
}

//SET MOTOR SPEED IN STEPS/SEC
void setLspeed(float spd)
{
  if (spd == 0)
  {
    lRate = 0;
    return;
  }
  spd = constrain(spd, -MAX_STEP_RATE, MAX_STEP_RATE);
  lRate = (int)(ISR_RATE / spd);
}

void setRspeed(float spd)
{
  if (spd == 0)
  {
    rRate = 0;
    return;
  }
  spd = constrain(spd, -MAX_STEP_RATE, MAX_STEP_RATE);
  rRate = (int)(ISR_RATE / spd);
}

void moton()
{
  digitalWrite(XY_ENABLE_PIN, LOW);
}

void motoff()
{
  digitalWrite(XY_ENABLE_PIN, HIGH);
}

void setup() {
  pinMode(X_STEP_PIN, OUTPUT);
  pinMode(X_DIR_PIN, OUTPUT);
  pinMode(Y_STEP_PIN, OUTPUT);
  pinMode(Y_DIR_PIN, OUTPUT);
  pinMode(XY_ENABLE_PIN, OUTPUT);
  pinMode(SERVO_PIN, OUTPUT);

  //servo.attach(SERVO_PIN);
  delay(1000);

  motoff();
#ifndef USE_ROS
  Serial.begin(115200);
  Serial.println("startup");
#endif

#ifdef USE_ROS
  nh.initNode();
  nh.advertise(lticksPub);
  nh.advertise(rticksPub);
  nh.subscribe(subTwist);
#endif

  devStatus = 1;

#ifdef BALANCE
  while (devStatus != 0 )
  {
    delay(10);
    Wire.begin();
    delay(10);
    mpu.initialize();
    delay(100);

#ifndef USE_ROS
    Serial.println(mpu.testConnection() ? F("MPU6050 con ok") : F("MPU6050 connection failed"));
#endif
    devStatus = mpu.dmpInitialize();
    if (devStatus == 0)
    {
      //mpu.CalibrateAccel(6);
      //mpu.CalibrateGyro(6);
      //mpu.PrintActiveOffsets();

      mpu.setXGyroOffset(154);
      mpu.setYGyroOffset(-93);
      mpu.setZGyroOffset(49);
      mpu.setXAccelOffset(-1588);
      mpu.setYAccelOffset(2997);
      mpu.setZAccelOffset(954);

#ifdef USE_MPU6050_DMP
      mpu.setDMPEnabled(true);
      mpuIntStatus = mpu.getIntStatus();
      dmpReady = true;
      packetSize = mpu.dmpGetFIFOPacketSize();
#endif
    }
    else
    {
#ifndef USE_ROS
      // 1 = initial memory load failed
      // 2 = DMP configuration updates failed
      Serial.print(F("Initialization failed (code "));
      Serial.print(devStatus);
      Serial.println(F(")"));
#endif
    }
  }
#endif

  stepsLeft = 0;
  stepsRight = 0;

  moton();

  microsPrevious = millis();
  CurieTimerOne.start((1.0 / (ISR_RATE / 1e6)), &timedBlinkIsr);

#ifndef USE_ROS
  Serial.println("OK!");
#endif
}

void loop()
{
  microsNow = millis();

  if (microsNow - microsPrevious >= LOOP_INTERVAL_MS)
  {
    microsPrevious = millis();

#ifdef BALANCE
#ifdef USE_MPU6050_DMP
    mpu.resetFIFO();
     fifoCount = mpu.getFIFOCount();
    int cnt = 20;
    while (fifoCount < packetSize && cnt-- > 0)
    {
      //Serial.print("1");
      fifoCount = mpu.getFIFOCount();
      delayMicroseconds(100);
      
    }
    //Serial.print("2");
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    //Serial.print("3");
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    //Serial.print("4");
    mpu.dmpGetGravity(&gravity, &q);
    //Serial.print("5");
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    //Serial.print("6 ");
    angleMeas = (float)(ypr[1] * 5760);

    if (abs(oldMeas - angleMeas) > 1000)   angleMeas = oldMeas;
    oldMeas = angleMeas;
#else
    angleMeas = 0;
#endif

    angleMeas = (angleMeas)+89;

    //servo.write(map(angleMeas - 3, -90, 90, 500, 2500));

    angleCmdPoint = -PID(1, 0, odometry, 1 / (LOOP_INTERVAL_MS / 1000), 3000);
    //Serial.print("7 ");
    wheelSpdBalance = PID(0, angleSetpoint + angleCmdPoint, -angleMeas, 1 / (LOOP_INTERVAL_MS / 1000), MAX_STEP_RATE);
    setLspeed(wheelSpdBalance + lSpdOffset);
    setRspeed(wheelSpdBalance + rSpdOffset);
    //Serial.print("8 ");
    if(abs(angleMeas)<70)
    {
      stepsLeft=0;
      stepsRight=0;
    }
#else
    setLspeed(lSpdOffset);
    setRspeed(rSpdOffset);
#endif
    odometry = (stepsLeft * M_PER_STEP + stepsRight * M_PER_STEP) / 2;

#ifndef USE_ROS
    Serial.print(angleMeas); Serial.print(" ");
    Serial.print(angleCmdPoint);Serial.print(" ");
    Serial.print(wheelSpdBalance);Serial.print(" ");
    Serial.println(" ");
#endif

#ifdef USE_ROS
    left_ticks.data = stepsLeft;
    right_ticks.data = stepsRight;
    lticksPub.publish(&left_ticks);
    rticksPub.publish(&right_ticks);
    nh.spinOnce();
#endif
  }
}
