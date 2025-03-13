#include <Wire.h>
#include <MPU6050_tockn.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <BH1750.h>
#include <driver/ledc.h>

#define TCAADDR 0x70

MPU6050 mpu6050(Wire);
BH1750 lightMeter;

unsigned long task1_millis;
unsigned long acquire_millis;
unsigned long change_i2cport_millis;

//PARALLAX DISTANCE(CM)
unsigned int cm[4]; //0 FRONT | 1 RIGHT | 2 LEFT | 3 BALL 
//LIGHSENSOR VALUES 
unsigned int light[16]; 

//MOTOR DIRECTION
int b = 5;
int a = 17;
int c = 16;
int d = 15;
//MOTOR PWM
int enc = 2;
int ena = 19;
int enb = 18;
int end = 4;

//MUX SELECTORS
int s0 = 26;
int s1 = 27;
int s2 = 14;
int s3 = 12;

//MUX SIGNAL
int muxParallax = 13;
int muxIR = 25;
int muxLight = 23;

//ARRAYS FOR INFRARED SENSORS
int segnali[16];
int medi[16];

//CONFIGURATION OF PWM FOR ESP32
const int freq = 5000;  // Frequenza PWM in Hz
const int resolution = 8;  // Risoluzione a 8 bit (0-255)

float lux[8];

int minimo, sensore, valore,segnale,duty,on,off,periodo,v_rotazione, v_dritto, lightValue = 2000, count = 0;
double duration, distancecm;
float angolo, angleZ;
bool cattura = false;

volatile unsigned long riseTime = 0;
volatile unsigned long pulseWidth = 0;

void IRAM_ATTR onFallingEdge();
void IRAM_ATTR onRisingEdge();

void tcaselect(uint8_t i) {
  if (i > 7) return;
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i); // Bit shift per selezionare il canale
  Wire.endTransmission();
}

void readLightSensors(void * parameter) {
  while (1) {
    if (millis() - change_i2cport_millis > 15) {
      change_i2cport_millis = millis();

      if(count > 7)
        count = 0;

      tcaselect(count);

      lux[count] = lightMeter.readLightLevel();
      //Serial.print("Light: ");
      //Serial.print(lux);
      //Serial.println(" lx");
      if(count == 0){
        mpu6050.update();
        angleZ = mpu6050.getAngleZ();
        //Serial.println(angleZ);
      }

      vTaskDelay(pdMS_TO_TICKS(50));
    }
  }
}


// Function to trigger the Parallax sensor
void triggerParallax() {
  pinMode(muxParallax, OUTPUT);
  digitalWrite(muxParallax, LOW);
  delayMicroseconds(2);
  digitalWrite(muxParallax, HIGH);
  delayMicroseconds(10);
  digitalWrite(muxParallax, LOW);
  pinMode(muxParallax, INPUT);
}

// Interrupt service routine for the rising edge
void IRAM_ATTR onRisingEdge() {
  riseTime = micros(); // Record the timestamp of the rising edge
  attachInterrupt(digitalPinToInterrupt(muxParallax), onFallingEdge, FALLING); // Switch to falling edge
}

// Interrupt service routine for the falling edge
void IRAM_ATTR onFallingEdge() {
  unsigned long fallTime = micros(); // Record the timestamp of the falling edge
  pulseWidth = fallTime - riseTime; // Calculate the pulse width
  attachInterrupt(digitalPinToInterrupt(muxParallax), onRisingEdge, RISING); // Switch back to rising edge
}

void movimento(){
  
  if (angolo == 0 || angolo == 337.5 || angolo == 22.5) {
    movimentoDritto(230, 230, 0);
  }

  if (angolo == 292.5 || angolo == 315) {
    movimentoDrittoDestra(120, 120, 220);
  }

  if (angolo == 270 || angolo == 157.5) {
    movimento45Destra(0, 230, 230);
  }

  if (angolo == 202.5 || angolo == 90) {
      movimento45Sinistra(230, 0, 230);
  }

  if (angolo == 247.5 || angolo == 225 || angolo == 180 || angolo == 112.5 || angolo == 135) {
    movimentoDietro(230, 230, 0);
  }

  if (angolo == 45 || angolo == 67.5) {
    movimentoDrittoSinistra(120, 120, 220);
  }

}

void movimento_gol(){
    if(angleZ >= -3 && angleZ <= 3){
        if(cm[2] > 50){
          movimento45Sinistra(230, 0, 230);
        }else if(cm[1] > 50){   
          movimento45Destra(0, 230, 230);
        }
        else{
          movimentoDritto(230, 230, 0);
        } 
    }
}

void readParallax(unsigned int cm[], int i){
  if(i < 4){
    digitalWrite(muxParallax, HIGH);
    delayMicroseconds(2);
    digitalWrite(muxParallax, LOW);

    pinMode(muxParallax, INPUT);   

    unsigned long startTime = millis();
    while (digitalRead(muxParallax) == HIGH) {
        if (millis() - startTime > 50) break;  // Timeout di 50ms
    }

    duration = pulseIn(muxParallax, HIGH); 

    distancecm = duration*0.034/2;

    pinMode(muxParallax, OUTPUT);       
    digitalWrite(muxParallax, LOW);

    cm[i] = distancecm;
  }
}

void readLight(unsigned int light[], int i){
  int lightValue = analogRead(muxLight);
  light[i] = lightValue;
}

void setMuxChannel(int channel) {
  digitalWrite(s0, channel & 1);
  digitalWrite(s1, (channel >> 1) & 1);
  digitalWrite(s2, (channel >> 2) & 1);
  digitalWrite(s3, (channel >> 3) & 1);
}

void movimentoDritto(int pwmA, int pwmB, int pwmC) {
    Serial.println("movimentoDritto");
    ledcWrite(ena, pwmA);
    ledcWrite(enb, pwmB);
    ledcWrite(enc, pwmC);
   
    digitalWrite(d, HIGH);
    digitalWrite(b, LOW);
    digitalWrite(c, LOW);
}

void movimentoDrittoDestra(int pwmA, int pwmB, int pwmC) {
    Serial.println("movimentoDrittoDestra");
    ledcWrite(ena, pwmA);
    ledcWrite(enb, pwmB);
    ledcWrite(enc, pwmC);
   
    digitalWrite(a, HIGH);
    digitalWrite(b, HIGH);
    digitalWrite(c, LOW);
}

void movimento45Destra(int pwmA, int pwmB, int pwmC) {
    Serial.println("movimento45Destra");
    ledcWrite(ena, pwmA);
    ledcWrite(enb, pwmB);
    ledcWrite(enc, pwmC);
   
    digitalWrite(a, LOW);
    digitalWrite(b, HIGH);
    digitalWrite(c, LOW);
}

void movimento45Sinistra(int pwmA, int pwmB, int pwmC) {
    Serial.println("movimento45Sinistra");
    ledcWrite(ena, pwmA);
    ledcWrite(enb, pwmB);
    ledcWrite(enc, pwmC);
   
    digitalWrite(a, LOW);
    digitalWrite(b, LOW);
    digitalWrite(c, HIGH);
}

void movimentoDietro(int pwmA, int pwmB, int pwmC) {
    Serial.println("movimentoDietro");
    ledcWrite(ena, pwmA);
    ledcWrite(enb, pwmB);
    ledcWrite(enc, pwmC);
   
    digitalWrite(a, LOW);
    digitalWrite(b, HIGH);
    digitalWrite(c, LOW);
}

void movimentoDrittoSinistra(int pwmA, int pwmB, int pwmC) {
    Serial.println("movimentoDrittoSinistra");
    ledcWrite(ena, pwmA);
    ledcWrite(enb, pwmB);
    ledcWrite(enc, pwmC);
   
    digitalWrite(a, LOW);
    digitalWrite(b, LOW);
    digitalWrite(c, HIGH);
}


void reazioneLinea(int n){
  switch (n) {
    case 0:
      gestioneDribbler(true);
      movimentoDietro(230, 230, 0); // Dietro
      break;
    case 1:
      gestioneDribbler(true);
      movimentoDietro(230, 230, 0); // Dietro
      break;
    case 2:
      movimento45Destra(0, 230, 230); // 45° Destra
      break;
    case 3:
      movimentoDrittoDestra(120, 120, 220); // Dritto Destra
      break;
    case 4:
      movimentoDrittoDestra(120, 120, 220); // Dritto Destra
      break;
    case 5:
      movimentoDrittoDestra(120, 120, 220); // Dritto Destra
      break;
    case 6:
      movimentoDrittoDestra(120, 120, 220); // Dritto Destra
      break;
    case 7:
      movimentoDritto(230, 230, 0); // Dritto
      break;
    case 8:
      movimentoDritto(230, 230, 0); // Dritto
      break;
    case 9:
      movimentoDritto(230, 230, 0); // Dritto
      break;
    case 10:
      movimentoDrittoSinistra(120, 120, 220); // Dritto Sinistra
      break;
    case 11:
      movimentoDrittoSinistra(120, 120, 220); // Dritto Sinistra
      break;
    case 12:
      movimentoDrittoSinistra(120, 120, 220); // Dritto Sinistra
      break;
    case 13:
      movimentoDrittoSinistra(120, 120, 220); // Dritto Sinistra
      break;
    case 14:
      movimento45Sinistra(230, 0, 230); // 45° Sinistra
      break;
    case 15:
      gestioneDribbler(true);
      movimentoDietro(230, 230, 0); // Dietro
      break;
    default:
      break;
  }
}

void correzioneRotazione() {
    float errore = angleZ - 0;  // Differenza tra angolo attuale e 0°
    float Kp = 2.0;  // Costante proporzionale che può essere regolato

    int correzione = Kp * errore;  

    // Limita la correzione per evitare oscillazioni e movimenti bruschi
    correzione = constrain(correzione, -100, 100);

    if (abs(errore) > 3) { 
        analogWrite(ena, abs(correzione));
        analogWrite(enb, abs(correzione));
        analogWrite(enc, abs(correzione));

        if (errore > 0) { 
            digitalWrite(a, HIGH);
            digitalWrite(b, HIGH);
            digitalWrite(c, HIGH);
        } else {
            digitalWrite(a, LOW);
            digitalWrite(b, LOW);
            digitalWrite(c, LOW);
        }
    }
}

void gestioneDribbler(bool linea){
  ledcWrite(pwmChannelD, 255);

  if(linea){
    if(cm[0] < 40){
      digitalWrite(d, LOW);
    }
    else{
      digitalWrite(d, HIGH);
    }
  }
  else{
    digitalWrite(d, HIGH);
  }
}


void setup() {
  Serial.begin(115200);

  pinMode(c,OUTPUT);
  pinMode(a,OUTPUT);
  pinMode(b,OUTPUT);
  pinMode(d,OUTPUT);

  ledcAttachChannel(ena, freq, resolution, 0);
  ledcAttachChannel(enb, freq, resolution, 1);
  ledcAttachChannel(enc, freq, resolution, 2);
  ledcAttachChannel(end, freq, resolution, 3);

  pinMode(s0, OUTPUT);
  pinMode(s1, OUTPUT);
  pinMode(s2, OUTPUT);
  pinMode(s3, OUTPUT);

  digitalWrite(a,LOW);
  digitalWrite(b,LOW);
  digitalWrite(c,LOW);
  digitalWrite(d,LOW);

  digitalWrite(s0,LOW);
  digitalWrite(s1,LOW);
  digitalWrite(s2,LOW);
  digitalWrite(s3,LOW);

  attachInterrupt(digitalPinToInterrupt(muxParallax), onRisingEdge, RISING);

  Wire.begin();

  tcaselect(0);
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);

  for(int i = 0; i < 8; i++){
    tcaselect(i);
    lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE, 0x23, &Wire);  
  }

  
  xTaskCreate(
    readLightSensors,    // Funzione del task
    "ReadLightSensors",  // Nome del task
    5000,               // Dimensione dello stack
    NULL,                // Parametri del task
    0,                   // Priorità del task
    NULL                 // Handle del task
  );
  
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(10));

  for(int channel = 0; channel < 16; channel++){
        setMuxChannel(channel);
        
        if (channel < 4) {
          triggerParallax();
          delay(10); 
          cm[channel] = pulseWidth * 0.034 / 2;
          //Serial.print("Distance (cm) for channel ");
          //Serial.print(channel);
          //Serial.print(": ");
          //Serial.println(cm[channel]);
          
        }

        segnale = analogRead(muxIR);
        
        duty = (segnale/1023.0)*100;
        segnali[channel] = duty; 
        medi[channel] = medi[channel]*0.9 + segnale*0.1;   
        //Serial.println(medi[channel]);  
    }

    //Serial.println("////////////////////////////////");

  minimo = medi[0];
  sensore = 0;

  for(int n = 1; n < 16; n++){
    if(medi[n] < minimo){
        minimo = medi[n];
        sensore = n;   
    }

    if(light[n] > lightValue){
        //reazioneLinea(n);
    }
  }

  angolo = sensore * 22.5; 
  //Serial.print("Angolo palla: ");
  //Serial.print(sensore);
  //Serial.print("--");
  //Serial.println(angolo);

  if(cm[3]){
    //gestioneDribbler(false);
  }

  if(cm[0] >= 1 && cm[0] <= 7 && cm[1] > 3){
    cattura = true;
  }else{
    cattura = false;
  }

  //correzioneRotazione();

  /*
  if(cattura){
    movimento_gol();
  }else{
  
  }
  */ 
  
  movimento();
  
}
