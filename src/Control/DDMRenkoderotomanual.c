#include <Arduino.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <I2Cdev.h>
#include <Wire.h>
 

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#endif

MPU6050 mpu;

#define OUTPUT_READABLE_YAWPITCHROLL
#define INTERRUPT_PIN 20 

//#include <SoftwareSerial.h>
//SoftwareSerial bluetooth(1,0);

#define INA1 9
#define INA2 10
#define INB1 8
#define INB2 7
#define PWMA 11
#define PWMB 6

#define ENCA1 2
#define ENCA2 4
#define ENCB1 3
#define ENCB2 5

#define TRIG_PIN1 36 // Pin trigger sensor ultrasonik
#define ECHO_PIN1 34 // Pin echo sensor ultrasonik

#define JARAK_MAKSIMAL 20 

unsigned char x,rpwm,lpwm;
int Rotary_L,Rotary_R,counter_L=0,counter_R=0;
int Error_L,iError_L,dError_L,ErroSebelumnya_L,K_L,KD_L,KI_L,Target_L,U_L;
int Error_R,iError_R,dError_R,ErroSebelumnya_R,K_R,KD_R,KI_R,Target_R,U_R;

int pwm;    // mengatur kecepatan motor 0 - 255
int speed;

int pulseL;
int pulseR;

int jarakcm;
float CML;
float CMR;
float cmL;
float cmR;
float errorcmL;
float errorcmR;

// char modeSetting = 0;

int modeSetting = 0;
int modeKontrol = 0;
int modeOtomatis = 0;
char currentmode;
char SETTING;
char OTOMATIS;
char MANUAL;

float x_L,x_R;

float bacaSensorUltrasonik(int trigPin1, int echoPin1) {
  digitalWrite(trigPin1, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin1, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin1, LOW);

  long durasi = pulseIn(echoPin1, HIGH);
  float jarak_cm = durasi * 0.034 / 2;

  return jarak_cm;
}

void EnkoderA(void){    //motor kanan
  if(digitalRead(ENCA1)){
    if(digitalRead(ENCA2)) counter_L++;
    else                   counter_L--;
  }
  else{
    if(digitalRead(ENCA2)) counter_L--;
    else                   counter_L++;
  }
}
void EnkoderB(void){    //motor kiri
  if(digitalRead(ENCB1)){
    if(digitalRead(ENCB2)) counter_R--;
    else                   counter_R++;
  }
  else{
    if(digitalRead(ENCB2)) counter_R++;
    else                   counter_R--;
  }
}

void setup() {
  Serial.begin(9600);
  //Wire.begin();
  //mpu.initialize();
  pinMode(INA1, OUTPUT);
  pinMode(INA2, OUTPUT);
  pinMode(INB1, OUTPUT);
  pinMode(INB2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(ENCA1, INPUT);
  pinMode(ENCA2, INPUT);
  pinMode(ENCB1, INPUT);
  pinMode(ENCB2, INPUT);
  pinMode(TRIG_PIN1, OUTPUT);
  pinMode(ECHO_PIN1, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA1),EnkoderA,CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCB1),EnkoderB,CHANGE);
}

bool cekRintangan() {
  float jarak_cm1 = bacaSensorUltrasonik(TRIG_PIN1, ECHO_PIN1);


  if (jarak_cm1 < JARAK_MAKSIMAL) {
    Serial.println("Rintangan terdeteksi!");
    return true;
  }

  return false;
}

void jalan_maju(){            // PROGRAM JALAN MAJU
  digitalWrite(INA1, 0);
  digitalWrite(INA2, 1);
  digitalWrite(INB1, 0);
  digitalWrite(INB2, 1);
  analogWrite(PWMA, pwm);
  analogWrite(PWMB, pwm);
}

void jalan_mundur(){          // PROGRAM JALAN MUNDUR
  digitalWrite(INA1, 1);
  digitalWrite(INA2, 0);
  digitalWrite(INB1, 1);
  digitalWrite(INB2, 0);
  analogWrite(PWMA, pwm);
  analogWrite(PWMB, pwm);
}

void belok_kanan(){           // PROGRAM BELOK KANAN
  digitalWrite(INB1, 0);
  digitalWrite(INB2, 1);
  analogWrite(PWMB, pwm);
}

void belok_kiri(){            // PROGRAM BELOK KIRI
  digitalWrite(INA1, 0);
  digitalWrite(INA2, 1);
  analogWrite(PWMA, pwm);
}

void berhenti(){              // PROGRAM BERHENTI
  digitalWrite(INA1, 0);
  digitalWrite(INA2, 0);
  digitalWrite(INB1, 0);
  digitalWrite(INB2, 0);
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
}

void siku_kanan(){
  float derajat = 90 * 10.1;
  while(counter_L <= derajat){
    belok_kanan();
    if (counter_L >= derajat){
      berhenti();
      break;
    }
  }
}

void siku_kiri(){
  float derajat = 90 * 10.1;
  while(counter_R <= derajat){
    belok_kiri();
    if (counter_R >= derajat){
      berhenti();
    }
  }
}    

void write_otomatis(int jarak, char arah){ // ==================== INPUT MODE OTOMATIS ==========================
  
  if(arah == 'w'){
    jalan_maju();
    Serial.println("Robot Bergerak Maju");
    int startpulseL = pulseL;
    int startpulseR = pulseR;
    cmL = 0;
    cmR = 0;
    errorcmL = 0;
    errorcmR = 0;
    counter_L = 0;
    counter_R = 0;

    while(cmL < jarak || cmR < jarak){
      pulseL = counter_L - startpulseL;
      pulseR = counter_R - startpulseR;
      cmL = pulseL / 32.00;
      cmR = pulseR / 32.00;
      errorcmL = cmL - jarak;
      errorcmR = cmR - jarak;

      Serial.print("cmL = ");
      Serial.print(cmL);
      Serial.print("  cmR = ");
      Serial.print(cmR);
      Serial.print("  ErrL = ");
      Serial.print(errorcmL);
      Serial.print("  ErrR = ");
      Serial.println(errorcmR);

      if(cmL >= jarak && cmR >= jarak){
        berhenti();
        Serial.println("Robot Telah Mencapai Target.");
        pulseL = 0; pulseR = 0;
        counter_L = 0;
        counter_R = 0;
        break;
      }
    }
    berhenti();
  }
  else if(arah == 's'){
    jalan_mundur();
    Serial.println("Robot Bergerak Mundur");
    int startpulseL = pulseL;
    int startpulseR = pulseR;
    cmL = 0;
    cmR = 0;
    errorcmL = 0;
    errorcmR = 0;
    counter_L = 0;
    counter_R = 0;

    while(cmL > -jarak || cmR > -jarak){
      pulseL = counter_L - startpulseL;
      pulseR = counter_R - startpulseR;
      cmL = pulseL / 32.00;
      cmR = pulseR / 32.00;
      errorcmL = cmL - jarak;
      errorcmR = cmR - jarak;

      Serial.print("cmL = ");
      Serial.print(cmL);
      Serial.print("  cmR = ");
      Serial.print(cmR);
      Serial.print("  ErrL = ");
      Serial.print(errorcmL);
      Serial.print("  ErrR = ");
      Serial.println(errorcmR);

      if(cmL <= -jarak && cmR <= -jarak){
        berhenti();
        Serial.println("Robot Telah Mencapai Target.");
        pulseL = 0; pulseR = 0;
        counter_L = 0;
        counter_R = 0;
        break;
      }
    }
    berhenti();
  }
  else if(arah == 'd'){
    siku_kanan();
    jalan_maju();
    Serial.println("Robot Bergerak Belok Kanan");
    int startpulseL = pulseL;
    int startpulseR = pulseR;
    cmL = 0;
    cmR = 0;
    errorcmL = 0;
    errorcmR = 0;
    counter_L = 0;
    counter_R = 0;

    while(cmL < jarak || cmR < jarak){
      pulseL = counter_L - startpulseL;
      pulseR = counter_R - startpulseR;
      cmL = pulseL / 32.00;
      cmR = pulseR / 32.00;
      errorcmL = cmL - jarak;
      errorcmR = cmR - jarak;

      Serial.print("cmL = ");
      Serial.print(cmL);
      Serial.print("  cmR = ");
      Serial.print(cmR);
      Serial.print("  ErrL = ");
      Serial.print(errorcmL);
      Serial.print("  ErrR = ");
      Serial.println(errorcmR);

      if(cmL >= jarak && cmR >= jarak){
        berhenti();
        Serial.println("Robot Telah Mencapai Target.");
        pulseL = 0; pulseR = 0;
        counter_L = 0;
        counter_R = 0;
        break;
      }
    }
    berhenti();
  }
  else if(arah == 'a'){
    siku_kiri();
    jalan_maju();
    Serial.println("Robot Bergerak Belok Kiri");
    int startpulseL = pulseL;
    int startpulseR = pulseR;
    cmL = 0;
    cmR = 0;
    errorcmL = 0;
    errorcmR = 0;
    counter_L = 0;
    counter_R = 0;

    while(cmL < jarak || cmR < jarak){
      pulseL = counter_L - startpulseL;
      pulseR = counter_R - startpulseR;
      cmL = pulseL / 32.00;
      cmR = pulseR / 32.00;
      errorcmL = cmL - jarak;
      errorcmR = cmR - jarak;

      Serial.print("cmL = ");
      Serial.print(cmL);
      Serial.print("  cmR = ");
      Serial.print(cmR);
      Serial.print("  ErrL = ");
      Serial.print(errorcmL);
      Serial.print("  ErrR = ");
      Serial.println(errorcmR);

      if(cmL >= jarak && cmR >= jarak){
        berhenti();
        Serial.println("Robot Telah Mencapai Target.");
        pulseL = 0; pulseR = 0;
        counter_L = 0;
        counter_R = 0;
        break;
      }
    }
    berhenti();
  }
  else {
    Serial.println("Arah Tidak Valid!");
  }
}

void tampilannilai(){
  Serial.print(cmL);
  Serial.print("  ");
  Serial.println(cmR);
  delay(200);
}

void loop() {
  if (Serial.available()) {
    char received_input = Serial.read();
    if (received_input == 'j' || received_input == 'J') { 
      currentmode = SETTING;
      Serial.println("Mode SETTING");
      while(currentmode == SETTING){
        if(Serial.available() > 0){
          pwm = Serial.parseInt();
          if(pwm != 0 && pwm <= 255){
            Serial.print("PWM Di Atur = ");
            Serial.println(pwm);
            break;
          }
          else {
            Serial.println("NILAI TIDAK VALID! Inputkan Nilai PWM Kembali!");
          }
        }
      }
    }
    else if (received_input == 'k' || received_input == 'K') { 
      currentmode = MANUAL;
      Serial.println("Mode MANUAL");
      while (currentmode == MANUAL) {
        if (Serial.available()) {
          char control = Serial.read();
          if (control == 'w' || control == 'W') {
            berhenti();
            jalan_maju();

          } else if (control == 'a' || control == 'A') {
            berhenti();
            belok_kiri();

          } else if (control == 'd' || control == 'D') {
            berhenti();
            belok_kanan();

          } else if (control == 's' || control == 'S') {
            berhenti();
            jalan_mundur();

          } else if (control == 'q' || control == 'Q') {
            berhenti();

          } else if (control == 'l' || control == 'L') {
            currentmode = OTOMATIS;
            berhenti();
            break;
          } else if (control == 'j' || control == 'J'){
            currentmode = SETTING;
            berhenti();
            break;
          }
        }
      }
    }
    else if (received_input == 'l' || received_input == 'L') { 
      currentmode = OTOMATIS;
      cmL = 0;
      cmR = 0;
      errorcmL = 0;
      errorcmR = 0;
      Serial.println("Mode OTOMATIS");

      while (cekRintangan()){
        berhenti();
      }
      while(currentmode == OTOMATIS){
        if(Serial.available() >= 3){
          int received_jarak = Serial.parseInt();
          char received_arah = Serial.read();
          write_otomatis(received_jarak, received_arah);
          delay(500);
          
        }
      }
      }
    }
  
}