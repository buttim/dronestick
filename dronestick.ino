#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <SPI.h>
#include <Servo.h>
#include <QuickPID.h>
extern "C" {
  #include <hardware/flash.h>
};
#include "LT8920.h"

#define DEBUG

const uint8_t servoPitch_pin=5, servoRoll_pin=6;

float pitch,outputPitch,setpointPitch=0.0,
   roll,outputRoll,setpointRoll=0.0;
Servo servoPitch, servoRoll;
Adafruit_MPU6050 mpu;
QuickPID pidPitch(&pitch,&outputPitch,&setpointPitch,0.5,0.2,0.1,QuickPID::Action::direct),
  pidRoll(&roll,&outputRoll,&setpointRoll,0.5,0.2,0.1,QuickPID::Action::direct);
TwoWire i2c(sda_pin,scl_pin);
MbedSPI spi(radio_miso,radio_mosi,radio_sck);
uint32_t syncword=0;//0xF3AA180C;
uint8_t channels[] = {0,0,0,0,0,0,0,0};//{34, 74, 4, 44, 29, 69, 9, 49};
unsigned long tLast=0;
int nConsecutiveTimeout;

typedef struct {
  uint32_t syncword;
  uint8_t channels[8];
} Config;

void saveConfig() {
  Config cfg;
  cfg.syncword=syncword;
  memcpy(cfg.channels,channels,sizeof channels);
  noInterrupts();
  flash_range_erase((PICO_FLASH_SIZE_BYTES - FLASH_SECTOR_SIZE), FLASH_SECTOR_SIZE);
  flash_range_program((PICO_FLASH_SIZE_BYTES - FLASH_SECTOR_SIZE), (uint8_t *)&cfg, FLASH_PAGE_SIZE);
  interrupts();
}

void getConfig() {
  Config *pCfg=(Config *)(XIP_BASE+PICO_FLASH_SIZE_BYTES - FLASH_SECTOR_SIZE);
  syncword=pCfg->syncword;
  memcpy(channels,pCfg->channels,sizeof channels);
}

void setup(void) {
  Serial.begin(115200);
  while (!Serial)
    ;
  Serial.println("VIA");

  pinMode(LED_BUILTIN,OUTPUT);

  pinMode(radio_reset,OUTPUT);
  pinMode(radio_cs,OUTPUT);
  digitalWrite(radio_cs,HIGH);
  digitalWrite(radio_reset,LOW);
  delay(100);
  digitalWrite(radio_reset,HIGH);
  delay(100);

  spi.begin();
  spi.beginTransaction(SPISettings(4000000,MSBFIRST,SPI_MODE1));

  getConfig();

#ifdef DEBUG
  if (syncword==0)
    Serial.println("Pairing...");
#endif

  LT8920Begin(syncword);
  LT8920StartListening(syncword==0?33:channels[0]);
  
  servoPitch.attach(servoPitch_pin);
  servoPitch.write(0);
  servoRoll.attach(servoRoll_pin);
  servoRoll.write(0);

  i2c.begin();
  mpu.begin(MPU6050_I2CADDR_DEFAULT,&i2c);
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  pidPitch.SetOutputLimits(0,180);
  pidPitch.SetMode(QuickPID::Control::automatic);

  pidRoll.SetOutputLimits(0,180);
  pidRoll.SetMode(QuickPID::Control::automatic);

  delay(100);
}

void hop(int n=1) {
  static int nChannel=0;

  LT8920StopListening();
  nChannel += n;
  nChannel %= sizeof channels / sizeof(*channels);
  LT8920StartListening(channels[nChannel]);
}

void buf2pwm(uint8_t buf[],uint16_t pwm[]) {
  //TODO: mettere a posto
  pwm[0] = ((buf[2] & 0x07) << 8) | buf[1];
  pwm[1] = (((buf[3]<<4) | (buf[2]>>4))&0x07FF)*2;
  pwm[2] = ((buf[5] & 0x1) << 8) | buf[4];
  pwm[3] = (buf[6]&0x0F) | ((buf[5]&0xFE)<< 4);
  pwm[4] = (buf[7] << 4) | (buf[6] >> 4);
  pwm[5] = buf[8]<<4 | ((buf[0]&0xF0)<<4);
}

bool rx(uint16_t pwm[],bool pairing=false) {
  static bool connected=false;
  uint8_t  buf[32];
  uint16_t status=LT8920ReadRegister(R_STATUS);

  if (!pairing) {
    if (millis()-tLast>42) {
      tLast=millis();
      hop();
      if (nConsecutiveTimeout == 5) {
  #ifdef DEBUG
        Serial.println("Perso collegamento");
  #endif
        nConsecutiveTimeout++;
        connected=false;
      }
      else if (nConsecutiveTimeout<5)
        nConsecutiveTimeout++;
      return connected;
    }
  }

  if ((status&_BV(STATUS_PKT_FLAG_BIT))!=0)
    return connected;
  if (0!=(status & _BV(STATUS_CRC_BIT))) {
    hop();
    return connected;
  }

  int n = LT8920Read(buf, sizeof buf);
  if (pairing) {
    if (n==20) {
      tLast=millis();
      syncword=0xF3000000|buf[1]<<16|buf[2]<<8|buf[3];
      for (int i=0;i<8;i++)
        channels[i]=buf[i+4];
      saveConfig();
      LT8920Begin(syncword);
      LT8920StartListening(channels[0]);
#ifdef DEBUG
      Serial.println("Paired!");
#endif
    }
  }
  else if (n==9) {
    connected=true;
    tLast=millis();
    nConsecutiveTimeout=0;
    buf2pwm(buf,pwm);
#ifdef DEBUG
    for (int i=0;i<9;i++) {
      if (buf[i]<16)
        Serial.print("0");
      Serial.print(buf[i],HEX);
      Serial.print(" ");
    }
    Serial.println();
#endif
  }
  hop();
  return connected;
}

void failSafe() {
  //TODO:
}

void loop() {
  uint16_t pwm[6];

  bool connected=rx(pwm,syncword==0);
  digitalWrite(LED_BUILTIN,millis()%(connected?250:1000)<50?HIGH:LOW);

  if (connected) {
    //TODO:
  }
  else
    failSafe();

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  pitch=map(a.acceleration.y,  -10, 10, 180, 0);;
  if(pidPitch.Compute()) {
    // Serial.print("pitch: ");
    // Serial.print(pitch);
    // Serial.print("\toutputPitch: ");
    // Serial.println(outputPitch);
  }
  servoPitch.write(outputPitch);
  
  roll = map(a.acceleration.x,  -10, 10, 180, 0);
  if(pidPitch.Compute()) {
    // Serial.print("roll: ");
    // Serial.print(roll);
    // Serial.print("\toutputRoll: ");
    // Serial.println(outputRoll);
  }
  servoRoll.write(outputRoll);
} 