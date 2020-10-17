#include <Servo.h>  //servo library
#include "SR04.h"
#include <SoftwareSerial.h>
Servo myservo;      // create servo object to control servo
 
int Echo = A4;  
int Trig = A5; 
 
#define ENA 6
#define ENB 5
#define IN1 9
#define IN2 11
#define IN3 7
#define IN4 8
  

String message = "";
double rightDistance = 0;
double leftDistance = 0;
double centerDistance = 0;

const double limit_distance = 40;
SoftwareSerial hc06(12,13);
 SR04 sr04 = SR04(Echo,Trig);
long a;


int Arrondi(float nb)
{
  int nba = int(nb);
  if(nb -nba >= 0.5) nba++;
  return nba;
}
void forward(float leftEngine = 255, float rightEngine = 255){ 
  if (leftEngine <= 100){
    leftEngine = leftEngine * 25.5;
  }
  if(rightEngine <= 100 )
  {
    rightEngine = rightEngine *25.5;
  }
  analogWrite(ENA, Arrondi(rightEngine));
  analogWrite(ENB, Arrondi(leftEngine));
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  Serial.println("Forward");
}
 
void back(int leftEngine = 255, double rightEngine = 255){ 
  if (leftEngine <= 100){
    leftEngine = leftEngine * 25.5;
  }
  if(rightEngine <= 100 )
  {
    rightEngine = rightEngine *25.5;
  }
  analogWrite(ENA, Arrondi(rightEngine));
  analogWrite(ENB, Arrondi(leftEngine));
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  Serial.println("Back");
}
 
void left(double carSpeed = 255) {
  analogWrite(ENA, carSpeed);
  analogWrite(ENB, carSpeed);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH); 
  Serial.println("Left");
}
 
void right(double carSpeed = 255) {
  analogWrite(ENA, carSpeed);
  analogWrite(ENB, carSpeed);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  Serial.println("Right");
}


void sensorsMouvement(){
  for(int i = 90; i < 180 ;i++)
  {
        myservo.write(i);
        delay(10);

  }
   for(int i = 180; i > 0 ;i--)
  {
        myservo.write(i);
        delay(10);

  }
   for(int i = 0; i < 90 ;i++)
  {
        myservo.write(i);
        delay(10);

  }
  
}
 
void stop() {
  digitalWrite(ENA, LOW);
  digitalWrite(ENB, LOW);
  Serial.println("Stop!");
} 
 
//Ultrasonic distance measurement Sub function
double getDistance() {
   a=sr04.Distance();
  return a;
}  
 
void setup() { 
  
   //Initialize Serial Monitor
  Serial.println("ENTER AT Commands:");
  //Initialize Bluetooth Serial Port
  hc06.begin(9600);
  myservo.attach(3);  // attach servo on pin 3 to servo object
  Serial.begin(9600);     
  pinMode(Echo, INPUT);    
  pinMode(Trig, OUTPUT);  
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  stop();
} 
/*
void loop() { 
    myservo.write(90);  //setservo position according to scaled value
    delay(500); 
    centerDistance = getDistance();
 
    if(centerDistance <= 25) {     
      stop();
      delay(500);                         
      myservo.write(10);          
      delay(1000);      
      rightDistance = getDistance();
      
      delay(500);
      myservo.write(90);              
      delay(1000);                                                  
      myservo.write(180);              
      delay(1000); 
      leftDistance = getDistance();
      
      delay(500);
      myservo.write(90);              
      delay(1000);
      if(rightDistance > leftDistance) {
        right();
        delay(360);
      }
      else if(rightDistance < leftDistance) {
        left();
        delay(360);
      }
      else if((rightDistance <= 20) || (leftDistance <= 20)) {
        back();
        delay(180);
      }
      else {
        forward();
      }
    }  
    else {
        forward();
    }                     
}

void autonome
  myservo.write(90);
  centerDistance = getDistance();
  stop();
  if(centerDistance){
    stop();                         
    myservo.write(0); 
    delay(1000);    
    rightDistance = getDistance();

    myservo.write(180);
    delay(1000);
    leftDistance = getDistance();

    myservo.write(90);
    delay(1000);
    centerDistance = getDistance();

    if(rightDistance > leftDistance) {
        Serial.println("Right");
        right();
        delay(360);
    } else if(rightDistance < leftDistance) {
        Serial.println("Left");
        left();
        delay(360);
    } else if((rightDistance <= limit_distance) || (leftDistance <= limit_distance)) {
        Serial.println("Back");
        back();
        delay(180);
    } else {
        Serial.println("Forward");
        forward();
    }
  } else {
        Serial.println("Forward");
        forward();
  }
} 
*/

void autonomusMode(){
  centerDistance = getDistance();
  while (sr04.Distance() > limit_distance)
  {
      forward();
     centerDistance = getDistance();   
  }
   stop();                         
    myservo.write(0); 
    delay(1000);    
    rightDistance = getDistance();

    myservo.write(180);
    delay(1000);
    leftDistance = getDistance();

    myservo.write(90);
    delay(1000);
    centerDistance = getDistance();

    if(rightDistance > leftDistance) {
        Serial.println("Right");
        right();
        delay(450);
    } else if(rightDistance < leftDistance) {
        Serial.println("Left");
        left();
        delay(450);
    } else if((rightDistance <= limit_distance) || (leftDistance <= limit_distance)) {
        Serial.println("Back");
        back();
        delay(450);
    }
}


void loop(){
    myservo.write(90);
    if(hc06.available() >0){
    int value = hc06.read();
    Serial.println(value);
    stop();
    switch (value){
      case 49:
        forward();
        break;
      case 50:
        back();
       break;
      case 51:
        right();
       break;
      case 52:
        left();
      break;
      case 53:
        sensorsMouvement();
      break;
      case 54:
        autonomusMode();
      break;
      default:
      stop();
      break;
    }
    value = 0;
      
  }
}

/*
 void loop()
 {
  delay(2000);
  forward();
  delay(2000);
  back();
  delay(2000);
  right();
  delay(2000);
  left();

 }

*/












 
  //Write from Serial Monitor to HC06
 
      
     
      
      
