/*

SONAR MAPPING Bot
IR_feedback and distance computation check program

Uses 0,1,4 keys for STOP, FORWARD, REVERSE
Prints the wheel rotation count and distance convered in Serial monitor

*/

// For bluetooth communication
#include<SoftwareSerial.h>

// Output pins to motor driver
#define LEFT_ENABLE1 7
#define LEFT_ENABLE2 5
#define RIGHT_ENABLE1 12
#define RIGHT_ENABLE2 13
#define LEFT_SPEED 11
#define RIGHT_SPEED 3

// Required global variables
int x,y;
int count1=0,count2=0,flag1=0,flag2=0;
float distance=0;
String send_data;
char rec_data;

// Pins of bluetooth module connections
SoftwareSerial BTSerial(A2,A3);

// Setup
void setup()
{
  // initiating data communication
  Serial.begin(9600);
  Serial.println("Initiating channel .....");
  BTSerial.begin(9600);
  // initiating IR encoder pins
  pinMode(A0,0);
  pinMode(A1,0);
  // initiating motor driver pins
  pinMode(LEFT_ENABLE1,1);
  pinMode(LEFT_ENABLE2,1);
  pinMode(RIGHT_ENABLE1,1);
  pinMode(RIGHT_ENABLE2,1);
  pinMode(LEFT_SPEED,1);
  pinMode(RIGHT_SPEED,1);
  digitalWrite(LEFT_ENABLE2,0);
  digitalWrite(LEFT_ENABLE1,0);
  digitalWrite(RIGHT_ENABLE1,0);
  digitalWrite(RIGHT_ENABLE2,0);
  // writing rotation PWM speeds
  analogWrite(LEFT_SPEED,200);
  analogWrite(RIGHT_SPEED,200);
  // Waiting for handshake character reception
  while(!(BTSerial.available()));
  BTSerial.read();
  BTSerial.println("a");
  BTSerial.println("0:0");
}

// wheel rotation control
void dir_control()
{
  switch(rec_data)
  {

    case 49: // FORWARD
        digitalWrite(LEFT_ENABLE2,1);
        digitalWrite(LEFT_ENABLE1,0);
        digitalWrite(RIGHT_ENABLE1,1);
        digitalWrite(RIGHT_ENABLE2,0);
        break;

    case 51:  // REVERSE
        digitalWrite(LEFT_ENABLE2,0);
        digitalWrite(LEFT_ENABLE1,1);
        digitalWrite(RIGHT_ENABLE1,0);
        digitalWrite(RIGHT_ENABLE2,1);
        break;

    case 48:  //STOP
        digitalWrite(LEFT_ENABLE2,0);
        digitalWrite(LEFT_ENABLE1,0);
        digitalWrite(RIGHT_ENABLE1,0);
        digitalWrite(RIGHT_ENABLE2,0);
        break;

    default:
        digitalWrite(LEFT_ENABLE2,0);
        digitalWrite(LEFT_ENABLE1,0);
        digitalWrite(RIGHT_ENABLE1,0);
        digitalWrite(RIGHT_ENABLE2,0);
        break;

  }
}

// IR encoder computation
void ir_encoder()
{
  x=analogRead(A0);
  y=analogRead(A1);
  // Threshold for IR sensor 1 spoke count
  if(x>700&&flag1==0)
  {
    count1++;
    flag1=1;
  }
  // reseting sensor after spoke count spoke count
  else if(x>330&&x<400)
  flag1=0;
  // Threshold for IR sensor 2 spoke count
  if(y>900&&flag2==0)
  {
    count2++;
    flag2=1;
  }
  // reseting sensor after spoke count spoke count
  else if(y>430&&y<500)
  flag2=0;
  //Serial.print(x);
  //Serial.print(" : ");
  //Serial.println(y);
  Serial.print(count1);
  Serial.print(" : ");
  Serial.println(count2);
  //delay(400);

}

// loop
void loop()
{
  
  if(BTSerial.available()) // checking for character received
  {
  rec_data=BTSerial.read();
  //Serial.print("REC : ");
  //Serial.println(rec_data);
  dir_control();        // wheel rotation based on character
  distance=count1*3.867;  // Spoke to circumference ratio
  //Serial.print("SEND : ");
  send_data=String(distance)+":0";
  //Serial.println(send_data);
  BTSerial.println(send_data);
  }

  else
  ir_encoder(); // Running IR_encoder feedback function continously

}
