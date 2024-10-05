#include <QTRSensors.h>
#include <NewPing.h>
byte g, n = 0;
float Kp = 0.1;  //0.12
#define Kd 1
int MaxSpeed = 240;
int normalSpeed = 180;
bool a = false;
bool b = false;

unsigned long previousTime = 0;  
unsigned long interval = 200;

#define NUM_SENSORS 8
#define TIMEOUT 2500
#define rightMotor1 8
#define rightMotor2 9
#define leftMotor1 6
#define leftMotor2 7
#define rightMotorPWM 10
#define leftMotorPWM 5
unsigned int sensorValues[NUM_SENSORS];
unsigned int sensors[8];
int noir = 800;
int blanc = 400;
void wait();
int led = 52;
void droite(int speedleft, int speedright);
void arr(int speedleft, int speedright);
void gauche(int speedleft, int speedright);
void avance(int speedleft, int speedright);
void Stop();
long TIMER1 ;
long TIMER3 ;


//declarations  des capteurs
QTRSensorsRC qtrrc((unsigned char[]) {
  A8, A9, A10, A11, A12, A13, A14, A15
},
NUM_SENSORS, TIMEOUT, QTR_NO_EMITTER_PIN);
/*#define TRIGGER_PIN_D 22  // Arduino pin tied to trigger pin on the ultrasonic sensor.
  #define ECHO_PIN_D 23     // Arduino pin tied to echo pin on the ultrasonic sensor.
  #define TRIGGER_PIN_G 24  // Arduino pin tied to trigger pin on the ultrasonic sensor.
  #define ECHO_PIN_G 25     // Arduino pin tied to echo pin on the ultrasonic sensor.
  #define TRIGGER_PIN_F 26  // Arduino pin tied to trigger pin on the ultrasonic sensor.
  #define ECHO_PIN_F 27     // Arduino pin tied to echo pin on the ultrasonic sensor.

  NewPing sonarD(TRIGGER_PIN_D, ECHO_PIN_D, 40);   // NewPing setup of pins and maximum distance.
  NewPing sonarG(TRIGGER_PIN_G, ECHO_PIN_G, 40);   // NewPing setup of pins and maximum distance.
  NewPing sonarF(TRIGGER_PIN_F, ECHO_PIN_F, 100);  // NewPing setup of pins and maximum distance*/

void setup() {
  Serial.begin(9600);
  pinMode(2, INPUT);
  pinMode(13, INPUT);

  pinMode(rightMotor1, OUTPUT);
  pinMode(rightMotor2, OUTPUT);
  pinMode(rightMotorPWM, OUTPUT);
  pinMode(leftMotor1, OUTPUT);
  pinMode(leftMotor2, OUTPUT);
  pinMode(leftMotorPWM, OUTPUT);
  pinMode(led, OUTPUT);

  pinMode(53, INPUT_PULLUP);
  delay(50);
//// la partie calibration --necessaire pour un qtr 
  int i;
  for (int i = 0; i < 90; i++) {
    qtrrc.calibrate();
    delay(20);


  }
  ////pas forcement obliger 
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print(qtrrc.calibratedMinimumOn[i]);
    Serial.print(' ');
  }
  for (i = 0; i < NUM_SENSORS; i++) {
    Serial.print(qtrrc.calibratedMaximumOn[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();

  while (!digitalRead(53)) {

    wait();
    delay(100);
  }

  avance(220, 220);
  delay(80);
}
int lastError = 0;
int reducteur = 130;

void loop() {
  ///////condition pour le contraire du noir et blanc ////////////////

  if (a == false)
  {

    int Cd = digitalRead(13);
    int Cg = digitalRead(2);
    int position = qtrrc.readLine(sensors);
    int error;
    error = position - 3500;
    int motorSpeed = Kp * error + Kd * (error - lastError);
    lastError = error;

    int rightMotorSpeed = normalSpeed + motorSpeed - reducteur;
    int leftMotorSpeed = normalSpeed - motorSpeed - reducteur;

    if (rightMotorSpeed > MaxSpeed) rightMotorSpeed = MaxSpeed;
    if (leftMotorSpeed > MaxSpeed) leftMotorSpeed = MaxSpeed;
    if (rightMotorSpeed < 20) rightMotorSpeed = 0;
    if (leftMotorSpeed < 20) leftMotorSpeed = 0;

    move(1, rightMotorSpeed, 1);  //
    move(0, leftMotorSpeed, 1);   //forward


    /*  int position = qtrrc.readLine(sensorValues);
      for (int i = 0; i < 8
      ; i++) {
        int position = qtrrc.readLine(sensorValues);
        Serial.print(sensorValues[i]);
        Serial.print(" ");
      }
      Serial.print(" ");
      Serial.println(position);
      delay(1000);*/

    ////////droite
    if ( sensors[1] > noir && sensors[2] > noir && n == 0)
    {

      wait();
      delay(1);
      //  arr(100,100);
      //  delay(53);
      avance(100, 100);
      delay(90);
      droite(100, 100);
      delay(220);

      arr(100, 100);
      delay(72);
      //  Stop();
      //  delay(50);
      n++;
      // return;

    }

    ////gauche
    if ( sensors[5] > noir && sensors[0] < blanc && n == 1)
    {

      wait();
      delay(1);
      //  arr(100,100);
      //  delay(60);
      avance(100, 100);
      delay(130);

      gauche(100, 90);
      delay(430);
      arr(100, 100);
      delay(39);
      //  Stop();
      //  delay(20);
      n++;
      //  return;
    }
    ////////droite

    if ( sensors[1] > noir && sensors[2] > noir && n == 2)
    {

      wait();
      delay(1);
        arr(100,100);
        delay(60);
//      avance(100, 100);
//      delay(97);
      droite(100, 100);
      delay(256);

      arr(100, 100);
      delay(51);
      //  Stop();
      //  delay(40);
      n++;
      // return;
    }

    ////gauche

    if ( sensors[7] > noir && sensors[5] > noir && sensors[0] < blanc && n == 3)
    {

      wait();
      delay(1);
      //  arr(100,100);
      //  delay(60);
      avance(100, 100);
      delay(100);

      gauche(100, 100);
      delay(420);
      //   avance(100,100);
      //  delay(110);
      arr(100, 100);
      delay(60);
      n++;
      //  return;
    }

    ////////droite

    if ( sensors[1] > noir && sensors[2] > noir && n == 4)
    {

      wait();
      delay(1);
      //  arr(100,100);
      //  delay(59);
      avance(100, 100);
      delay(130);
      droite(100, 100);
      delay(240);

      arr(100, 100);
      delay(20);
      //  Stop();
      //  delay(40);
      n++;

      reducteur = 100;
      //return;





    }

    /////////tout noir
    if ( sensors[0] > noir && sensors[1] > noir && sensors[2] > noir && sensors[3] > noir && sensors[4] > noir && sensors[5] > noir && sensors[6] > noir && sensors[7] > noir && n == 5)
    {

      wait();
      delay(5000);

      //  avance(100,100);
      //  delay(1000);

      n++;
      //return;


    }

    /////apres tout noir
    if ( sensors[5] > noir && sensors[6] > noir && sensors[0] < blanc && n == 6)
    {
      wait();
      delay(90);


      n++;
      reducteur = 100;
      //return;

    }

    /////////tout blanc
    if ( sensors[0] < blanc && sensors[1] < blanc && sensors[2] < blanc && sensors[3] < blanc && sensors[4] < blanc && sensors[5] < blanc && sensors[6] < blanc && sensors[7] < blanc && n == 7)
    {
      //
      avance(100, 100);
      delay(170);
      gauche(100, 100);
      delay(390);

      avance(100, 100);
      delay(245);

      droite (100, 100);
      delay(245);
      TIMER1 = millis();



      n++;
      return;


    }
    ////////deuxieme partie
    if ( sensors[7] > noir && sensors[6] > noir &&  sensors[0] < blanc && n == 8 && ((millis() - TIMER1) > 180))
    {
      avance(100, 100);
      delay(69);
      //  wait();
      //  delay(1000);
      gauche(150, 150);
      delay(250);
      avance(100, 100);
      delay(1);
      //  wait();
      //  delay(1000);
      TIMER3 = millis();
      n++;
      return;



    }

    //////deuxieme tout blanc
    if ( sensors[0] < blanc && sensors[4] < blanc && sensors[5] < blanc && sensors[7] < blanc && n == 9 )
    {
      avance(120, 100);
      delay(50);

      position = qtrrc.readLine(sensors);
      if ( sensors[0] < blanc && sensors [7] < blanc )
      {

        avance(140, 100);
        delay(1);
      }
      reducteur = 145;
    }
    if (((sensors[0] > noir && sensors[5] < blanc) ) && n == 9 && ((millis() - TIMER3) > 1000))
    {
      wait();
      delay(2000);
      avance(100, 100);
      delay(100);
      n++;
      return;

    }
    if (((sensors[0] > noir && sensors[4] < blanc) || ( sensors[3] < blanc && sensors[7] > noir) ) && n == 10)
    {
      wait();
      delay(2000);
      avance(100, 100);
      delay(10);
//      droite(100,100);
//      delay(100);
      wait();
      delay(2000);
      a = true;
      n++;
      reducteur=120;
      return;

    }
    return;
  }


  ///////changement
  else(a == true);
  {



    int Cd = digitalRead(13);
    int Cg = digitalRead(2);
    int position = qtrrc.readLine(sensors);
    int error;
    error = -position + 3500;
    int motorSpeed = Kp * error + Kd * (error - lastError);
    lastError = error;

    int rightMotorSpeed = normalSpeed + motorSpeed - reducteur;
    int leftMotorSpeed = normalSpeed - motorSpeed - reducteur;

    if (rightMotorSpeed > MaxSpeed) rightMotorSpeed = MaxSpeed;
    if (leftMotorSpeed > MaxSpeed) leftMotorSpeed = MaxSpeed;
    if (rightMotorSpeed < 20) rightMotorSpeed = 0;
    if (leftMotorSpeed < 20) leftMotorSpeed = 0;

    move(1, rightMotorSpeed, 1);  //
    move(0, leftMotorSpeed, 1);   //forward

    ////////tout blanc
    if ( sensors[0] < blanc && sensors[1] < blanc && sensors[2] < blanc && sensors[3] < blanc && sensors[4] < blanc && sensors[5] < blanc && sensors[6] < blanc && sensors[7] < blanc&&n==11)
    {
      wait();
      delay(5000);
      n++;
    }
  }    



}
void move(int motor, int speed, int direction) {

  boolean inPin1 = LOW;
  boolean inPin2 = HIGH;

  if (direction == 1) {
    inPin1 = LOW;
    inPin2 = HIGH;
  }
  if (direction == 0) {
    inPin1 = HIGH;
    inPin2 = LOW;
  }

  if (motor == 0) {
    digitalWrite(leftMotor1, inPin1);
    digitalWrite(leftMotor2, inPin2);
    analogWrite(leftMotorPWM, speed);
  }
  if (motor == 1) {
    digitalWrite(rightMotor1, inPin1);
    digitalWrite(rightMotor2, inPin2);
    analogWrite(rightMotorPWM, speed);
  }
}

void wait() {
  digitalWrite(leftMotor1, LOW);
  digitalWrite(leftMotor2, LOW);
  analogWrite(leftMotorPWM, 0);
  digitalWrite(rightMotor1, LOW);
  digitalWrite(rightMotor2, LOW);
  analogWrite(rightMotorPWM, 0);

}
void droite(int speedleft, int speedright) {
  digitalWrite(rightMotor1, HIGH);
  digitalWrite(rightMotor2, LOW);
  analogWrite(rightMotorPWM, speedright);
  digitalWrite(leftMotor1, LOW);
  digitalWrite(leftMotor2, HIGH);
  analogWrite(leftMotorPWM, speedleft);
}
void gauche(int speedleft, int speedright) {
  digitalWrite(rightMotor1, LOW);
  digitalWrite(rightMotor2, HIGH);
  analogWrite(rightMotorPWM, speedright);
  digitalWrite(leftMotor1, HIGH);
  digitalWrite(leftMotor2, LOW);
  analogWrite(leftMotorPWM, speedleft);
}
void avance(int speedleft, int speedright) {
  digitalWrite(rightMotor1, LOW);
  digitalWrite(rightMotor2, HIGH);
  analogWrite(rightMotorPWM, speedright);
  digitalWrite(leftMotor1, LOW);
  digitalWrite(leftMotor2, HIGH);
  analogWrite(leftMotorPWM, speedleft);
}

void arr(int speedleft, int speedright) {
  digitalWrite(rightMotor1, HIGH);
  digitalWrite(rightMotor2, LOW);
  analogWrite(rightMotorPWM, speedright);
  digitalWrite(leftMotor1, HIGH);
  digitalWrite(leftMotor2, LOW);
  analogWrite(leftMotorPWM, speedleft);
}
void Stop() {
  digitalWrite(rightMotor1, LOW);
  digitalWrite(rightMotor2, LOW);
  analogWrite(rightMotorPWM, 0);
  digitalWrite(leftMotor1, LOW);
  digitalWrite(leftMotor2, LOW);
  analogWrite(leftMotorPWM, 0);
}
