#include <ros.h>
#include<std_msgs/Int16.h>
#include<std_msgs/Float32.h>
#include<geometry_msgs/Twist.h>


#define HOVER_SERIAL_BAUD   115200      // [-] Baud rate for HoverSerial (used to communicate with the hoverboard)
#define SERIAL_BAUD         115200      // [-] Baud rate for built-in Serial (used for the Serial Monitor)
#define START_FRAME         0xABCD       // [-] Start frme definition for reliable serial communication
#define TIME_SEND           100         // [ms] Sending time interval
#define SPEED_MAX_TEST      100         // [-] Maximum speed for testing
#define SPEED_STEP          5          // [-] Speed step
// #define DEBUG_RX                        // [-] Debug received data. Prints all bytes to serial (comment-out to disable)

#include <SoftwareSerial.h>
#define HoverSerial Serial1

// Global variables
uint8_t idx = 0;                        // Index for new data pointer
uint16_t bufStartFrame;                 // Buffer Start Frame
byte *p;                                // Pointer declaration for the new received data
byte incomingByte;
byte incomingBytePrev;

typedef struct{
   uint16_t start;
   int16_t  steer;
   int16_t  speed;
   uint16_t checksum;
} SerialCommand;
SerialCommand Command;

typedef struct{
   uint16_t start;
   int16_t  cmd1;
   int16_t  cmd2;
   int16_t  speedR_meas;
   int16_t  speedL_meas;
   int16_t  batVoltage;
   int16_t  boardTemp;
   uint16_t cmdLed;
   uint16_t checksum;
} SerialFeedback;
SerialFeedback Feedback;
SerialFeedback NewFeedback;

float lSpd, rSpd;
uint16_t batteryVoltage=0;

#define WHEEL_DIAMETER_M 0.170f
#define WHEELBASE_M 0.37
#define M_PER_REV (WHEEL_DIAMETER_M*3.141569)

void lwheel_spd_cb( const geometry_msgs::Twist &msg)
{
  //msg vel in m/s and rad/s
  //must convert in RPM

  float xlv = msg.linear.x;
  float zrv = msg.angular.z; 
  
  rSpd = ((xlv + zrv * WHEELBASE_M * 0.5)) / M_PER_REV *60;
  lSpd = ((xlv - zrv * WHEELBASE_M * 0.5)) / M_PER_REV *60;
}


std_msgs::Int16 bat_msg;
std_msgs::Float32 lspeed_msg;
std_msgs::Float32 rspeed_msg;

ros::NodeHandle  nh;
ros::Subscriber<geometry_msgs::Twist> subTwist("cmd_vel", &lwheel_spd_cb );
ros::Publisher batteryPub("battery_voltage", &bat_msg);
ros::Publisher lwheelSpeedPub("left_wheel_speed", &lspeed_msg);
ros::Publisher rwheelSpeedPub("right_wheel_speed", &rspeed_msg);

// ########################## SETUP ##########################
void setup() 
{
  HoverSerial.begin(HOVER_SERIAL_BAUD);
  pinMode(LED_BUILTIN, OUTPUT);
  nh.initNode();
  nh.subscribe(subTwist);
  nh.advertise(batteryPub);
  nh.advertise(lwheelSpeedPub);
  nh.advertise(rwheelSpeedPub);
}

// ########################## SEND ##########################
void Send(int16_t uSteer, int16_t uSpeed)
{
  // Create command
  Command.start    = (uint16_t)START_FRAME;
  Command.steer    = (int16_t)uSteer;
  Command.speed    = (int16_t)uSpeed;
  Command.checksum = (uint16_t)(Command.start ^ Command.steer ^ Command.speed);

  // Write to Serial
  HoverSerial.write((uint8_t *) &Command, sizeof(Command)); 
}

// ########################## RECEIVE ##########################
void Receive()
{
    // Check for new data availability in the Serial buffer
    if (HoverSerial.available()) {
        incomingByte    = HoverSerial.read();                                   // Read the incoming byte
        bufStartFrame = ((uint16_t)(incomingByte) << 8) | incomingBytePrev;       // Construct the start frame
    }
    else {
        return;
    }

    // Copy received data
    if (bufStartFrame == START_FRAME) {                     // Initialize if new data is detected
        p       = (byte *)&NewFeedback;
        *p++    = incomingBytePrev;
        *p++    = incomingByte;
        idx     = 2;  
    } else if (idx >= 2 && idx < sizeof(SerialFeedback)) {  // Save the new received data
        *p++    = incomingByte; 
        idx++;
    } 
    
    // Check if we reached the end of the package
    if (idx == sizeof(SerialFeedback)) {
        uint16_t checksum;
        checksum = (uint16_t)(NewFeedback.start ^ NewFeedback.cmd1 ^ NewFeedback.cmd2 ^ NewFeedback.speedR_meas ^ NewFeedback.speedL_meas
                            ^ NewFeedback.batVoltage ^ NewFeedback.boardTemp ^ NewFeedback.cmdLed);

        // Check validity of the new data
        if (NewFeedback.start == START_FRAME && checksum == NewFeedback.checksum) 
        {
            // Copy the new data
            memcpy(&Feedback, &NewFeedback, sizeof(SerialFeedback));

            rspeed_msg.data=-Feedback.speedR_meas * M_PER_REV /60.0;
            lspeed_msg.data=Feedback.speedL_meas * M_PER_REV /60.0;

            lwheelSpeedPub.publish(&lspeed_msg);    
            rwheelSpeedPub.publish(&rspeed_msg); 
            
            batteryVoltage=Feedback.batVoltage;
/*
            // Print data to built-in Serial
            Serial.print("");   Serial.print(Feedback.cmd1);
            Serial.print(", ");  Serial.print(Feedback.cmd2);
            Serial.print(", ");  Serial.print(Feedback.speedR_meas);
            Serial.print(", ");  Serial.println(Feedback.speedL_meas);
            //Serial.print("");  Serial.print(Feedback.batVoltage);
            //Serial.print("");  Serial.print(Feedback.boardTemp);
            //Serial.print("");  Serial.println(Feedback.cmdLed);
            */
        } else {
          //Serial.println("Non-valid data skipped");
        }
        idx = 0;    // Reset the index (it prevents to enter in this if condition in the next cycle)
    }

    // Update previous states
    incomingBytePrev = incomingByte;
}

// ########################## LOOP ##########################

unsigned long iTimeSend = 0;
int iTest = 0;
int iStep = SPEED_STEP;
unsigned long lastMillis=0;
void loop(void)
{ 
  unsigned long timeNow = millis();

  // Check for new received data
  Receive();

  // Send commands
  if (iTimeSend > timeNow) return;
  iTimeSend = timeNow + TIME_SEND;
  
  Send(lSpd, rSpd);


  // Blink the LED
  digitalWrite(LED_BUILTIN, (timeNow%2000)<1000);

  if(millis()-lastMillis> 1000)
  {
    lastMillis=millis();
    bat_msg.data = batteryVoltage;
    batteryPub.publish(&bat_msg);    
  }

  nh.spinOnce();
}

// ########################## END ##########################
