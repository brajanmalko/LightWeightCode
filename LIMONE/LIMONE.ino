#include <Wire.h>
#include <MPU6050_tockn.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <BH1750.h>
#include <driver/ledc.h>
#include <esp_task_wdt.h>
#include <math.h>

#define TCAADDR 0x70

MPU6050 mpu6050(Wire);
BH1750 lightMeter[8];

unsigned long acquire_millis;

const unsigned long SENSOR_INTERVAL = 0.5;

static unsigned long lastParallaxTrigger = 0;
static int parallaxPhase = 0;
static int currentParallaxChannel = 0;
static unsigned long triggerStartTime = 0;

//PARALLAX DISTANCE(CM)
unsigned int cm[4];  //0 FRONT | 1 RIGHT | 2 LEFT | 3 BALL
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

//ARRAY FOR LUX SENSORS
float lux[8];

//CONFIGURATION OF PWM FOR ESP32
const int freq = 2000;     // Frequenza PWM in Hz
const int resolution = 8;  // Risoluzione a 8 bit (0-255)

//GENERAL VARIABLES
int minimo, sensore, valore, segnale, duty, on, off, periodo, v_rotazione, v_dritto, count = 0, contCicli = 0, maxLux, vFront = 130, mCampo = 50, calLux = 11;
double duration, distancecm;
float angolo, angleZ;
bool cattura = false;

//VARIABLES USED BY TASKS
volatile bool inverti = false;
volatile int nLux = 0;

//VARIABLES FOR PARALLAX READING
volatile unsigned long riseTime = 0;
volatile unsigned long pulseWidth = 0;

void IRAM_ATTR onFallingEdge();
void IRAM_ATTR onRisingEdge();

void tcaselect(uint8_t i) {
  if (i > 7) return;
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);  // Bit shift per selezionare il canale
  Wire.endTransmission();
}

void coreTask(void *pvParameters) {
  esp_task_wdt_add(NULL);

  while (1) {
    uint32_t now = millis(); 
    static uint32_t lastRead = 0;

    tcaselect(0);
    mpu6050.update();
    angleZ = mpu6050.getAngleZ();
    Serial.println(angleZ);

    if (now - lastRead >= SENSOR_INTERVAL) {
      lastRead = now;

      if(count == 8){
        count = 0;
      }

      tcaselect(count);
      // Lettura semplificata luce da ogni sensore
      lux[count] = lightMeter[count].readLightLevel();
      //Serial.println(lux[7]);
      if (lux[count] > maxLux) {
        maxLux = lux[count];
        if (lux[count] > calLux) {
          nLux = count;
          if(count == 4 || count == 5 || count == 6){
            inverti = true;
          }
          
          reazioneLinea(nLux);
          //maxLux = 0;
        }
      }
      //Serial.print("Luce sensore ");
      //Serial.print(count);
      //Serial.print(": ");
      //Serial.println(lux);
      count++;
    }

    vTaskDelay(1);
    esp_task_wdt_reset();
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
  riseTime = micros();                                                          // Record the timestamp of the rising edge
  attachInterrupt(digitalPinToInterrupt(muxParallax), onFallingEdge, FALLING);  // Switch to falling edge
}

// Interrupt service routine for the falling edge
void IRAM_ATTR onFallingEdge() {
  unsigned long fallTime = micros();                                          // Record the timestamp of the falling edge
  pulseWidth = fallTime - riseTime;                                           // Calculate the pulse width
  attachInterrupt(digitalPinToInterrupt(muxParallax), onRisingEdge, RISING);  // Switch back to rising edge
}

void movimento() {

  if (angolo == 0) {
    //fermo(0,0,0);
    movimentoDritto(vFront, vFront, 0);
  }

  if (angolo == 337.5) {
    //fermo(0,0,0);
    movimentoDrittoDestra(80, 80, 140);
  }

  if (angolo == 292.5 || angolo == 270 || angolo == 157.5 || angolo == 315) {
    //fermo(0,0,0);
    movimento45Destra(0, 120, 120);
  }

  if (angolo == 90 || angolo == 202.5 || angolo == 67.5 || angolo == 45) {
    //fermo(0,0,0);
    movimento45Sinistra(120, 0, 120);
  }

  if (angolo == 247.5 || angolo == 225 || angolo == 180 || angolo == 112.5 || angolo == 135) {
    //fermo(0,0,0);
    movimentoDietro(120, 120, 0);
  }

  if (angolo == 22.5) {
    //fermo(0,0,0);
    movimentoDrittoSinistra(80, 80, 140);
  }
}

void movimento_gol() {
  if (angleZ >= -15 && angleZ <= 15) {
    /*
        if(cm[2] > mCampo){
          movimento45Sinistra(0, 230, 230);
        }else if(cm[1] > mCampo){   
          movimento45Destra(230, 0, 230);
        }
        else{
          movimentoDritto(200, 200, 0);
        }
        */
    movimentoDritto(200, 200, 0);
  }
}

void readParallax(unsigned int cm[], int i) {
  if (i < 4) {
    digitalWrite(muxParallax, HIGH);
    delayMicroseconds(2);
    digitalWrite(muxParallax, LOW);

    pinMode(muxParallax, INPUT);

    unsigned long startTime = millis();
    while (digitalRead(muxParallax) == HIGH) {
      if (millis() - startTime > 50) break;  // Timeout di 50ms
    }

    duration = pulseIn(muxParallax, HIGH);

    distancecm = duration * 0.034 / 2;

    pinMode(muxParallax, OUTPUT);
    digitalWrite(muxParallax, LOW);

    cm[i] = distancecm;
  }
}

void readLight(unsigned int light[], int i) {
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
  //Serial.println("movimentoDritto");
  ledcWrite(ena, pwmA);
  ledcWrite(enb, pwmB);
  ledcWrite(enc, pwmC);

  digitalWrite(a, HIGH);
  digitalWrite(b, LOW);
  digitalWrite(c, LOW);
}

void movimentoDrittoDestra(int pwmA, int pwmB, int pwmC) {
  //Serial.println("movimentoDrittoDestra");
  ledcWrite(ena, pwmA);
  ledcWrite(enb, pwmB);
  ledcWrite(enc, pwmC);

  digitalWrite(a, HIGH);
  digitalWrite(b, HIGH);
  digitalWrite(c, LOW);
}

void movimento45Destra(int pwmA, int pwmB, int pwmC) {
  //Serial.println("movimento45Destra");
  ledcWrite(ena, pwmA);
  ledcWrite(enb, pwmB);
  ledcWrite(enc, pwmC);

  digitalWrite(a, LOW);
  digitalWrite(b, HIGH);
  digitalWrite(c, LOW);
}

void movimento45Sinistra(int pwmA, int pwmB, int pwmC) {
  //Serial.println("movimento45Sinistra");
  ledcWrite(ena, pwmA);
  ledcWrite(enb, pwmB);
  ledcWrite(enc, pwmC);

  digitalWrite(a, LOW);
  digitalWrite(b, LOW);
  digitalWrite(c, HIGH);
}

void movimentoDietro(int pwmA, int pwmB, int pwmC) {
  //Serial.println("movimentoDietro");
  ledcWrite(ena, pwmA);
  ledcWrite(enb, pwmB);
  ledcWrite(enc, pwmC);

  digitalWrite(a, LOW);
  digitalWrite(b, HIGH);
  digitalWrite(c, LOW);
}

void movimentoDrittoSinistra(int pwmA, int pwmB, int pwmC) {
  //Serial.println("movimentoDrittoSinistra");
  ledcWrite(ena, pwmA);
  ledcWrite(enb, pwmB);
  ledcWrite(enc, pwmC);

  digitalWrite(a, LOW);
  digitalWrite(b, LOW);
  digitalWrite(c, HIGH);
}

void movimento45DGol(int pwmA, int pwmB, int pwmC) {
  ledcWrite(ena, pwmA);
  ledcWrite(enb, pwmB);
  ledcWrite(enc, pwmC);

  digitalWrite(a, HIGH);
  digitalWrite(b, HIGH);
  digitalWrite(c, LOW);
}

void movimento45SGol(int pwmA, int pwmB, int pwmC) {
  ledcWrite(ena, pwmA);
  ledcWrite(enb, pwmB);
  ledcWrite(enc, pwmC);

  digitalWrite(a, LOW);
  digitalWrite(b, LOW);
  digitalWrite(c, HIGH);
}

void reazioneLinea(int n) {
  switch (n) {
    case 0:
      movimentoDritto(200, 200, 0);  // Dietro
      delay(40);
      break;
    case 1:
      movimentoDritto(200, 200, 0);  // Dietro
      delay(40);
      break;
    case 2:
      movimentoDrittoDestra(100, 100, 160);  // Dritto Sinistra      
      delay(40);
      break;
    case 3:
      movimentoDietro(200, 200, 0);  // Dietro
      delay(40);
      break;
    case 4:
      movimentoDietro(200, 200, 0);  // Dietro
      delay(40);
      break;
    case 5:
      movimentoDietro(200, 200, 0);  // Dietro
      delay(40);
      break;
    case 6:
      movimentoDrittoSinistra(100, 100, 160);  // Dritto Destra
      delay(40);
      break;
    case 7:
      movimentoDritto(200, 200, 0);  // Dritto
      delay(40);
      break;
    default:
      break;
  }
}

void correzioneRotazioneC() {
  int angoloC = fmod(angleZ, 360.0);

  if (angleZ < -15) {
    ledcWrite(ena, 0);
    ledcWrite(enb, 0);
    ledcWrite(enc, 160);

    digitalWrite(a, LOW);
    digitalWrite(b, HIGH);
    digitalWrite(c, LOW);
  } else if (angleZ > 15) {
    ledcWrite(ena, 0);
    ledcWrite(enb, 0);
    ledcWrite(enc, 160);

    digitalWrite(a, LOW);
    digitalWrite(b, HIGH);
    digitalWrite(c, HIGH);
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(c, OUTPUT);
  pinMode(a, OUTPUT);
  pinMode(b, OUTPUT);
  pinMode(d, OUTPUT);

  ledcAttachChannel(ena, freq, resolution, 0);
  ledcAttachChannel(enb, freq, resolution, 1);
  ledcAttachChannel(enc, freq, resolution, 2);
  ledcAttachChannel(end, freq, resolution, 3);

  pinMode(s0, OUTPUT);
  pinMode(s1, OUTPUT);
  pinMode(s2, OUTPUT);
  pinMode(s3, OUTPUT);

  digitalWrite(a, LOW);
  digitalWrite(b, LOW);
  digitalWrite(c, LOW);
  digitalWrite(d, LOW);

  digitalWrite(s0, LOW);
  digitalWrite(s1, LOW);
  digitalWrite(s2, LOW);
  digitalWrite(s3, LOW);

  attachInterrupt(digitalPinToInterrupt(muxParallax), onRisingEdge, RISING);

  

  //configurazione del watchdog
  const esp_task_wdt_config_t wdt_config = {
    .timeout_ms = 5000,          // Timeout di 5 secondi
    .idle_core_mask = (1 << 0),  // Core 0
    .trigger_panic = true,       // Causa panic se non viene resettato
  };
  esp_task_wdt_init(&wdt_config);  // Passa il puntatore alla config


  Wire.begin();

  scan(0);
  lightMeter[0].begin(BH1750::CONTINUOUS_HIGH_RES_MODE, 0x23, &Wire);
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
  scan(1);
  lightMeter[1].begin(BH1750::CONTINUOUS_HIGH_RES_MODE, 0x23, &Wire);
  scan(2);
  lightMeter[2].begin(BH1750::CONTINUOUS_HIGH_RES_MODE, 0x23, &Wire);
  scan(3);
  lightMeter[3].begin(BH1750::CONTINUOUS_HIGH_RES_MODE, 0x23, &Wire);
  scan(4);
  lightMeter[4].begin(BH1750::CONTINUOUS_HIGH_RES_MODE, 0x23, &Wire);
  scan(5);
  lightMeter[5].begin(BH1750::CONTINUOUS_HIGH_RES_MODE, 0x23, &Wire);
  scan(6);
  lightMeter[6].begin(BH1750::CONTINUOUS_HIGH_RES_MODE, 0x23, &Wire);
  scan(7);
  lightMeter[7].begin(BH1750::CONTINUOUS_HIGH_RES_MODE, 0x23, &Wire);

  // Task principale con più stack
  xTaskCreatePinnedToCore(
    coreTask,
    "MainTask",
    8192,
    NULL,
    1,
    NULL,
    0);
}

void loop() {
  //vTaskDelay(pdMS_TO_TICKS(10));

  for (int channel = 0; channel < 16; channel++) {
    setMuxChannel(channel);

    if (contCicli == 50) {
      if (channel < 4) {
        if (cm[3] < 4) {
          triggerParallax();
          delay(10);
          cm[channel] = pulseWidth * 0.034 / 2;
          //Serial.println(cm[3]);
        } else {
          if (channel == 3) {
            triggerParallax();
            delay(10);
            cm[channel] = pulseWidth * 0.034 / 2;
            //Serial.println(cm[3]);
          }
        }

        //Serial.print("Distance (cm) for channel ");
        //Serial.print(channel);
        //Serial.print(": ");
        //Serial.println(cm[channel]);
      }
    }


    segnale = analogRead(muxIR);

    duty = (segnale / 1023.0) * 100;
    segnali[channel] = duty;
    medi[channel] = medi[channel] * 0.9 + segnale * 0.1;
    //Serial.println(medi[channel]);
  }

  //Serial.println("////////////////////////////////");

  minimo = medi[0];
  sensore = 0;

  for (int n = 1; n < 16; n++) {
    if (medi[n] < minimo) {
      minimo = medi[n];
      sensore = n;
    }
  }

  angolo = sensore * 22.5;
  //Serial.print("Angolo palla: ");
  //Serial.print(sensore);
  //Serial.print("--");
  //Serial.println(angolo);

  if (maxLux > calLux) {
    Serial.println("cazzo sto uscendo");
    reazioneLinea(nLux);
    maxLux = 0;
  } else {
    if (cm[3] < 4) {
      cattura = true;
      vFront = 170;
      Serial.println("catturata");
    } else {
      cattura = false;
      vFront = 130;
    }

    if (cattura) {
      //Serial.println(angoloCorretto);
      if (angleZ < -15 || angleZ > 15) {
        correzioneRotazioneC();
        Serial.println("correggo rotazione");
      } else {
        movimento_gol();
        Serial.println("faccio gol");
      }
    } else {
      movimento();
    }

    if (contCicli == 50) {
      contCicli = 0;
    } else {
      contCicli++;
    }
  }

  if (!inverti) {
    ledcWrite(end, 255);
    digitalWrite(d, HIGH);
  } else {
    Serial.println("tiroooo");
    ledcWrite(end, 255);
    digitalWrite(d, LOW);
    delay(300);

    inverti = false;
  }
}

void scan(uint8_t port) {
  tcaselect(port);
  byte error, address;
  int nDevices;

  Serial.println("Scanning...");

  nDevices = 0;
  for (address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.println(address, HEX);

      nDevices++;
    } else if (error == 4) {
      Serial.print("Unknown error at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.println(address, HEX);
    }
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found");
  else
    Serial.println("done");

  for (address = 0x68; address <= 0x69; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0) {
      Serial.print("Trovato dispositivo MPU a 0x");
      Serial.println(address, HEX);
    } else if (error = 4) {
      Serial.println("Non trovato mpu");
    }
  }

  delay(50);  // wait 1 second for next scan
}
