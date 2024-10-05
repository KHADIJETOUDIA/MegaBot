#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <NewPing.h>
#include <Encoder.h>
#include <SoftwareSerial.h>
#include <DHT.h>

// Définition des broches pour le capteur ultrasonique
#define TRIGGER_PIN 12
#define ECHO_PIN 13
#define MAX_DISTANCE 200

// Définition des broches pour les encodeurs
#define ENCODER1A_PIN 2
#define ENCODER1B_PIN 3
#define ENCODER2A_PIN 4
#define ENCODER2B_PIN 5

// Définition des broches pour les moteurs
#define MOTOR1_PWM_PIN 6
#define MOTOR1_IN1_PIN 7
#define MOTOR1_IN2_PIN 8
#define MOTOR2_PWM_PIN 9
#define MOTOR2_IN1_PIN 10
#define MOTOR2_IN2_PIN 11

// Adresse I2C de l'écran LCD
#define LCD_ADDRESS 0x27

// Nombre de colonnes et de lignes de l'écran LCD
#define LCD_COLUMNS 16
#define LCD_ROWS 2

// Broche pour le capteur de flamme
#define FLAME_SENSOR_PIN A0

// Configuration du module Bluetooth (HC-05 ou HC-06)
#define BT_SERIAL_RX 14 // Broche RX du module Bluetooth connectée à la broche 14 de l'Arduino
#define BT_SERIAL_TX 15 // Broche TX du module Bluetooth connectée à la broche 15 de l'Arduino

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);
Encoder encoder1(ENCODER1A_PIN, ENCODER1B_PIN);
Encoder encoder2(ENCODER2A_PIN, ENCODER2B_PIN);
LiquidCrystal_I2C lcd(LCD_ADDRESS, LCD_COLUMNS, LCD_ROWS);
SoftwareSerial BTSerial(BT_SERIAL_RX, BT_SERIAL_TX); // Création d'un objet de communication série pour le module Bluetooth
DHT dht(DHT_PIN, DHT22); // Configuration du capteur DHT22

void setup() {
  pinMode(MOTOR1_PWM_PIN, OUTPUT);
  pinMode(MOTOR1_IN1_PIN, OUTPUT);
  pinMode(MOTOR1_IN2_PIN, OUTPUT);
  pinMode(MOTOR2_PWM_PIN, OUTPUT);
  pinMode(MOTOR2_IN1_PIN, OUTPUT);
  pinMode(MOTOR2_IN2_PIN, OUTPUT);
  
  lcd.init(); // Initialisation de l'écran LCD
  lcd.backlight(); // Allumer le rétroéclairage de l'écran LCD
  lcd.setCursor(0, 0); // Déplacer le curseur à la première ligne, première colonne
  lcd.print("Alimentation OK"); // Afficher le message "Alimentation OK" sur l'écran LCD
  
  BTSerial.begin(9600); // Initialisation de la communication série avec le module Bluetooth
  dht.begin(); // Initialisation du capteur DHT22
}

void loop() {
  unsigned long currentTime = millis();

  // Le reste du code pour le mouvement du robot
  // Détection des obstacles avec le capteur ultrasonique
  int distance = sonar.ping_cm();
  if (distance > 30) {
    avancer(100); // Avancer à pleine vitesse
  } else {
    arreter();
    delay(1000);
    reculer(100); // Reculer à pleine vitesse
    delay(2000);
    arreter();
    delay(1000);
    pivoter(100); // Tourner à pleine vitesse
    delay(1000);
  }
  
  // Mesure de la température et de l'humidité avec le capteur DHT22
  float temperature = dht.readTemperature();
  float humidity = dht.readHumidity();
  
  // Envoi des données du capteur DHT22 via Bluetooth
  if (!isnan(temperature) && !isnan(humidity)) {
    BTSerial.print("Temperature: ");
    BTSerial.print(temperature);
    BTSerial.print(" °C, Humidity: ");
    BTSerial.print(humidity);
    BTSerial.println("%");
  }
  
  // Le reste du code pour la détection de flamme et l'affichage sur l'écran LCD
}

void avancer(int vitesse) {
  digitalWrite(MOTOR1_IN1_PIN, HIGH);
  digitalWrite(MOTOR1_IN2_PIN, LOW);
  digitalWrite(MOTOR2_IN1_PIN, HIGH);
  digitalWrite(MOTOR2_IN2_PIN, LOW);
  analogWrite(MOTOR1_PWM_PIN, vitesse);
  analogWrite(MOTOR2_PWM_PIN, vitesse);
}

void arreter() {
  digitalWrite(MOTOR1_PWM_PIN, LOW);
  digitalWrite(MOTOR2_PWM_PIN, LOW);
}

void reculer(int vitesse) {
  digitalWrite(MOTOR1_IN1_PIN, LOW);
  digitalWrite(MOTOR1_IN2_PIN, HIGH);
  digitalWrite(MOTOR2_IN1_PIN, LOW);
  digitalWrite(MOTOR2_IN2_PIN, HIGH);
  analogWrite(MOTOR1_PWM_PIN, vitesse);
  analogWrite(MOTOR2_PWM_PIN, vitesse);
}

void pivoter(int vitesse) {
  digitalWrite(MOTOR1_IN1_PIN, HIGH);
  digitalWrite(MOTOR1_IN2_PIN, LOW);
  digitalWrite(MOTOR2_IN1_PIN, LOW);
  digitalWrite(MOTOR2_IN2_PIN, HIGH);
  analogWrite(MOTOR1_PWM_PIN, vitesse);
  analogWrite(MOTOR2_PWM_PIN, vitesse);
}