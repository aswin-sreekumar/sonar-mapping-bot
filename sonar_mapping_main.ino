/*

SONAR MAPPING bot
Main PROGRAM

*/

// Required header files
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <SoftwareSerial.h>
#include<Servo.h>

Servo myservo;

// Defining output pins
#define ECHO 8
#define TRIGGER 4
#define LEFT_ENABLE1 5
#define LEFT_ENABLE2 7
#define RIGHT_ENABLE1 12
#define RIGHT_ENABLE2 13
#define LEFT_SPEED 11
#define RIGHT_SPEED 3

// Bluetooth module pins
SoftwareSerial BTSerial(A3,A2);

// MPU object
MPU6050 mpu;

// MPU control/status variables
bool dmpReady = false; // set true if DMP initialisation was successful
uint8_t mpuIntStatus; // holds actual interrupt status byte from MPU
uint8_t devStatus; // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize; // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount; // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high

// Orientation variables
Quaternion q; // [w, x, y, z] quaternion container
VectorFloat gravity; // [x, y, z] gravity vector
float ypr[3]; // yaw, pitch, roll values - yaw only required
double yaw;

String data="";

int x,y;
int mode=0;
int count1=0,count2=0,flag1=0,flag2=0,dir_flag=0,initial_yaw_flag=1;
float distance=0,theta=0,initial_yaw=0;
String send_data;
float distance_array[90];

int pos;
float array_dist,time_interval;
int i=0;

char rec_data;


// function to trigger interrupt when DMP data is ready
void dmpDataReady()
{
mpuInterrupt = true;
}

// function to input offsets for calibration
void mpu_offset_declare()
{
  mpu.setXGyroOffset(16);
  mpu.setYGyroOffset(-84);
  mpu.setZGyroOffset(4);
  mpu.setXAccelOffset(-1586);
  mpu.setYAccelOffset(-1480);
  mpu.setZAccelOffset(1548);
}

// function to initialise motors
void motor_initialise()
{
  pinMode(LEFT_ENABLE1,1);
  pinMode(LEFT_ENABLE2,1);
  pinMode(RIGHT_ENABLE1,1);
  pinMode(RIGHT_ENABLE2,1);
  pinMode(LEFT_SPEED,1);
  pinMode(RIGHT_SPEED,1);
  digitalWrite(LEFT_ENABLE1,0);
  digitalWrite(LEFT_ENABLE2,0);
  digitalWrite(RIGHT_ENABLE1,0);
  digitalWrite(RIGHT_ENABLE2,0);
  analogWrite(LEFT_SPEED,200);
  analogWrite(RIGHT_SPEED,200);
}

// setup
void setup()
{
   // Start serial communication
  Serial.begin(115200);
  Serial.println("INITIALISING SONAR SLAM ...");
  BTSerial.begin(9600);
  pinMode(A0,0);
  pinMode(A1,0);
  pinMode(TRIGGER,1);
  pinMode(ECHO,0);
  digitalWrite(TRIGGER,0);
  delayMicroseconds(3);
  digitalWrite(TRIGGER,0);
  while(!(BTSerial.available()));
  BTSerial.read();
  BTSerial.println("a");
  BTSerial.println("1:0:0");
  // Initialise motor pins
  motor_initialise();
  // Start I2C communication
  Wire.begin();
  // Initialise MPU
  mpu.initialize();
  // Get status of MPU
  devStatus = mpu.dmpInitialize();
  // Feed the offset values for calibration of MPU
  mpu_offset_declare();

  if (devStatus == 0) // If MPU has been initialised
  {
    mpu.setDMPEnabled(true); // Initalise DMP
    attachInterrupt(0, dmpDataReady, RISING); // Attach interrupt for MPU data reading
    mpuIntStatus = mpu.getIntStatus(); // Get status of interrupt
    dmpReady = true; // enable DMP status
    packetSize = mpu.dmpGetFIFOPacketSize(); // read data packet size
  }
  else // failing to intialise MPU
  {
  Serial.print("DMP Initialization failed");
  if(devStatus==1)
    Serial.println("Initial memory load failed");
  else if(devStatus==2)
    Serial.println("DMP configuration update failed");
  }
}

// Sending obstacle data to Python script as string
void send_obstacle_data()
{
  float avg_distance;
  for(pos=45;pos<=134;pos++)
  {
    send_data=String(distance_array[pos-45]/2)+":"+String(pos);
    BTSerial.println(send_data);
    Serial.println(send_data);
    delay(20);
  }
}

// Reading obstacle distance at differenct angles of servo
void read_dist()
{
  for (pos = 45; pos <= 134; pos += 1)
  { array_dist=0;
    myservo.write(pos);
    delay(20);
    for(i=0;i<10;i++)
    {
    digitalWrite(TRIGGER,1);
    delayMicroseconds(10);
    digitalWrite(TRIGGER,0);
    time_interval=pulseIn(ECHO,HIGH);
    array_dist+=time_interval*0.017;
    }
    distance=array_dist/10;
    //Serial.print(String(distance));
    //Serial.print(':');
    //Serial.println(String(pos));
    distance_array[pos-45]=distance;
  }
  for (pos = 134; pos >= 45; pos -= 1)
  { array_dist=0;
    myservo.write(pos);
    delay(20);
    for(i=0;i<10;i++)
    {
    digitalWrite(TRIGGER,1);
    delayMicroseconds(10);
    digitalWrite(TRIGGER,0);
    time_interval=pulseIn(ECHO,HIGH);
    array_dist+=time_interval*0.017;
    }
    distance=array_dist/10;
    //Serial.print(String(distance));
    //Serial.print(':');
    //Serial.println(String(pos));
   distance_array[pos-45]+=distance;
  }
  myservo.detach();
  delay(20);
  send_obstacle_data();
  delay(20);
  BTSerial.println("**");
  delay(20);
}

// Control of bot movement
void command_control()
{
  switch(rec_data)
  {
    case 49:
        digitalWrite(LEFT_ENABLE2,1);
        digitalWrite(LEFT_ENABLE1,0);
        digitalWrite(RIGHT_ENABLE1,1);
        digitalWrite(RIGHT_ENABLE2,0);
        mode=1;
        BTSerial.println(String(mode));
        break;

    case 51:
        digitalWrite(LEFT_ENABLE2,0);
        digitalWrite(LEFT_ENABLE1,1);
        digitalWrite(RIGHT_ENABLE1,0);
        digitalWrite(RIGHT_ENABLE2,1);
        mode=1;
        BTSerial.println(String(mode));
        break;

    case 50:
        digitalWrite(LEFT_ENABLE2,1);
        digitalWrite(LEFT_ENABLE1,0);
        digitalWrite(RIGHT_ENABLE1,0);
        digitalWrite(RIGHT_ENABLE2,1);
        mode=1;
        BTSerial.println(String(mode));
        break;

    case 52:
        digitalWrite(LEFT_ENABLE2,0);
        digitalWrite(LEFT_ENABLE1,1);
        digitalWrite(RIGHT_ENABLE1,1);
        digitalWrite(RIGHT_ENABLE2,0);
        mode=1;
        BTSerial.println(String(mode));
        break;

    case 48:
        digitalWrite(LEFT_ENABLE2,0);
        digitalWrite(LEFT_ENABLE1,0);
        digitalWrite(RIGHT_ENABLE1,0);
        digitalWrite(RIGHT_ENABLE2,0);
        mode=1;
        BTSerial.println(String(mode));
        break;

    case 98:
        myservo.attach(9);
        myservo.write(45);
        delay(100);
        mode=2;
        Serial.println(mode);
        BTSerial.println(String(mode));
        read_dist();
        break;

    default:
        digitalWrite(LEFT_ENABLE2,0);
        digitalWrite(LEFT_ENABLE1,0);
        digitalWrite(RIGHT_ENABLE1,0);
        digitalWrite(RIGHT_ENABLE2,0);
        break;

  }
  Serial.println(mode);
  if(rec_data=='1'||rec_data=='2'||rec_data=='3'||rec_data=='4'||rec_data=='0')
  {
  distance=count1*3.867;
  if(rec_data=='3'||dir_flag==1)
  {
   theta=yaw+180;
   dir_flag=1;
  }
  else
    theta=yaw;
  if(rec_data=='0')
  {
   dir_flag=0;
  }
  theta=theta-initial_yaw;
  BTSerial.println(String(distance)+":"+String(theta));
  Serial.println(String(distance)+":"+String(theta));
  }
}

// Encoder feedback for distance travelled by bot
void ir_encoder()
{
  x=analogRead(A0);
  y=analogRead(A1);
  if(x>700&&flag1==0)
  {
    count1++;
    flag1=1;
  }
  else if(x>330&&x<400)
  flag1=0;
  //Serial.print(x);
  //Serial.print(count1);
  //delay(400);
}

// Loop
void loop()
{
  if (!dmpReady) // if DMP is not ready, exit from program
    return;
  while (!mpuInterrupt && fifoCount < packetSize) // when data is not sent
  {
    if(BTSerial.available())
    {
      rec_data=BTSerial.read();
      Serial.print("REC : ");
      Serial.println(rec_data);
      command_control();
   }
  else
    if(rec_data=='1'||rec_data=='3'||rec_data=='0')
      ir_encoder();
  }
  mpuInterrupt = false; // set interrupt flag as 0
  mpuIntStatus = mpu.getIntStatus(); // get Get interrupt status
  fifoCount = mpu.getFIFOCount(); // Get FIFO buffer size
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) // FIFO overflow gas occured
  {
  mpu.resetFIFO(); //resetting MPU
  Serial.println("FIFO overflow!");
  }
  else if (mpuIntStatus & 0x02) // data is sent from MPU
  {
  while (fifoCount < packetSize) // wait till FIFO buffer is complete
    fifoCount = mpu.getFIFOCount();
  mpu.getFIFOBytes(fifoBuffer, packetSize); // get the data from MPU
  fifoCount -= packetSize; // check if multiple data packets exist
  mpu.dmpGetQuaternion(&q, fifoBuffer); // compute quaternion from FIFO raw values
  mpu.dmpGetGravity(&gravity, &q); // Calculate gravity from quaternion values
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity); // compute YPR from gravity and quaternion
  yaw = ypr[0] * 180/M_PI; // Compute roll from data obtained
  if(initial_yaw_flag==1) // initial orientation of bot reference
  {
    initial_yaw_flag=0;
    initial_yaw=yaw;
  }
  //Serial.print("YAW: "); //debugging
  //Serial.println(yaw); //debugging
  }
}
