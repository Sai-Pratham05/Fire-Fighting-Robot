


/*------ Arduino Fire Fighting Robot (with obstacle detection) ---- */
#include <SoftwareSerial.h>   //include SoftwareSerial.h library
#include <Servo.h>            //include servo.h library  

Servo myservo;
 
int pos = 0;    
boolean fire = false;

int bt_data; // variable to receive data from the serial port

int mode=1;


#define rxPinBluetooth 2
#define txPinBluetooth 3

#define GAS_SENSOR 11    //Gas sensor

#define Left 10      // left flame/line sensor
#define Right 8    // right flame/line sensor
#define Forward 9   //front flame/line sensor

#define RM1 4       // right motor pin 1
#define RM2 5       // right motor pin 2
#define LM1 6       // left motor pin 1
#define LM2 7       // left motor pin 2
#define pump 12

// Ultrasonic sensor (HC-SR04) pins - using analog pins as digital I/O
#define TRIG_PIN A0
#define ECHO_PIN A1
#define OBSTACLE_DISTANCE_CM 20  // threshold distance to consider as obstacle (in cm)

SoftwareSerial BT_Serial(rxPinBluetooth,txPinBluetooth);

void setup()
{

  BT_Serial.begin(9600);
  Serial.begin(9600);

  
  pinMode(Left, INPUT);
  pinMode(Right, INPUT);
  pinMode(Forward, INPUT);
  pinMode(GAS_SENSOR, INPUT);
  pinMode(RM1, OUTPUT);
  pinMode(RM2, OUTPUT);
  pinMode(LM1, OUTPUT);
  pinMode(LM2, OUTPUT);
  pinMode(pump, OUTPUT);

  // Ultrasonic pins
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
 
  myservo.attach(13);
  myservo.write(90); 

  digitalWrite(pump, HIGH);
  
}
 
 
void loop()
{
  
  myservo.write(90); //Sweep_Servo();  

// Serial.println(BT_Serial.available());
  if(BT_Serial.available() > 0) //if some date is sent, reads it and saves in state  
  {     
    bt_data = BT_Serial.read();  
  }

  if(bt_data == 8)  //Auto
  {
    mode=1;
  }
  else if(bt_data == 9)    //Manual 
  {
    mode=0;
  } 

  if(mode==0){     
//===============================================================================
//                          Manual (Key) Control with obstacle check
//=============================================================================== 
    if(bt_data == 1){ // forward
      int d = readUltrasonic();
      if (d > 0 && d <= OBSTACLE_DISTANCE_CM) {
        Serial.print("Obstacle detected in manual mode: ");
        Serial.print(d);
        Serial.println(" cm. Stopping.");
        Stop();
        // Optional avoidance in manual: back up slightly and turn
        obstacleAvoidance();
      } else {
        forward();
      }
    }  
    else if(bt_data == 2){ backward(); }  // if the bt_data is '2' the motor will Reverse
    else if(bt_data == 3){ turnLeft(); }  // if the bt_data is '3' the motor will turn left
    else if(bt_data == 4){ turnRight(); } // if the bt_data is '4' the motor will turn right
    else if(bt_data == 5){ Stop(); }
    else if(bt_data == 6){put_off_fire();}
    delay(10);
  }
//===============================================================================
//                          Auto Control with obstacle detection
//=============================================================================== 
  else{
    // read ultrasonic periodically
    int distance = readUltrasonic();
    if (distance > 0 && distance <= OBSTACLE_DISTANCE_CM)
    {
      Serial.print("Obstacle detected in auto mode: ");
      Serial.print(distance);
      Serial.println(" cm. Avoiding...");
      obstacleAvoidance();
      // after avoidance, continue loop and sensor-based fire detection will resume
      return; // skip rest of this cycle so we don't mix motions
    }

    if (digitalRead(Left) == 1 && digitalRead(Right)==1 && digitalRead(Forward) == 1) // no fire detected
    {
      delay(500);
      Stop();
      delay(500);
    }
    else if (digitalRead(Forward) == 0) // fire detected at front
    {
      forward();
      fire = true;
    }
      
    else if (digitalRead(Left) == 0){ turnLeft(); }  //fire at left
    else if (digitalRead(Right) == 0){ turnRight(); }  //fire at right
    delay(200);//change this value to change the distance between checks
      
    if(digitalRead(GAS_SENSOR) == 0)
    {
      Serial.println("Gas is Detected.");
      put_off_fire();
    }
    
    while (fire == true)
    {
      put_off_fire();
      Serial.println("Fire Detected.");
    }
  }
}



void put_off_fire()
{
  Stop();    
  digitalWrite(pump,LOW);
  delay(300);
 
  for (pos = 50; pos <= 130; pos += 1) 
  { 
    myservo.write(pos); 
    delay(10);  
  }
  for (pos = 130; pos >= 50; pos -= 1) { 
    myservo.write(pos); 
    delay(10);
  }
  digitalWrite(pump,HIGH);
  myservo.write(90); 
  fire=false;
}



void obstacleAvoidance()
{
  // Simple reactive obstacle avoidance:
  // 1) stop, 2) back up a little, 3) turn right (you can change to left or implement checks)
  Stop();
  delay(100);
  // back up
  backward();
  delay(400); // back up duration (ms) - tune to your robot
  // turn right
  turnRight();
  delay(450); // turn duration (ms) - tune to your robot
  Stop();
  delay(150);
}

// Ultrasonic distance measurement (HC-SR04)
int readUltrasonic()
{
  // send trigger pulse
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // measure echo pulse (timeout 30000 us = 30 ms ~ max ~5m)
  unsigned long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  if (duration == 0) {
    // no echo / out of range
    return -1;
  }
  // convert to cm (duration in microseconds)
  int distanceCm = (int)(duration / 58.2);
  return distanceCm;
}

void forward()  //forward
{ 
  digitalWrite(RM1, HIGH);
  digitalWrite(RM2, LOW);
  digitalWrite(LM1, HIGH);
  digitalWrite(LM2, LOW);
}

void backward() //backward
{ 
  digitalWrite(RM1, LOW);  //Right Motor forward Pin 
  digitalWrite(RM2, HIGH); //Right Motor backward Pin 
  digitalWrite(LM1, LOW); //Left Motor backward Pin 
  digitalWrite(LM2, HIGH);  //Left Motor forward Pin 
}

void turnRight()  //turnRight
{
  digitalWrite(RM1, LOW);
  digitalWrite(RM2, HIGH);
  digitalWrite(LM1, HIGH);
  digitalWrite(LM2, LOW);
}

void turnLeft()  //turnLeft
{
  digitalWrite(RM1, HIGH);
  digitalWrite(RM2, LOW);
  digitalWrite(LM1, LOW);
  digitalWrite(LM2, HIGH);
}

void Stop()  //stop
{
  digitalWrite(RM1, LOW); //Right Motor forword Pin 
  digitalWrite(RM2, LOW); //Right Motor backword Pin 
  digitalWrite(LM1, LOW); //Left Motor backword Pin 
  digitalWrite(LM2, LOW); //Left Motor forword Pin 
}
