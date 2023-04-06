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

const uint8_t servo_pin=5;

float input,output,setpoint=90.0;
Servo servo;
Adafruit_MPU6050 mpu;
QuickPID pid(&input,&output,&setpoint,0.5,0.2,0.1,QuickPID::Action::direct);
TwoWire i2c(sda_pin,scl_pin);
MbedSPI spi(radio_miso,radio_mosi,radio_sck);
uint32_t syncword=0;//0xF3AA180C;
uint8_t channels[] = {0,0,0,0,0,0,0,0};//{34, 74, 4, 44, 29, 69, 9, 49};
unsigned long tLast=0;
int nConsecutiveTimeout;

void getConfig() {
  /*EEPROM.begin(16);
  syncword=0xF3000000|EEProm.read(0)<<16|EEprom.read(1)<<8|EEprom.read(2);
  for (int i=0;i<8;i++)
    channels[i]=EEPROM.read(i+3);
  EEPROM.end()*/
}

void saveConfig() {
  /*EEPROM.begin(16);
  EEPROM.write(0,syncword&0xFF);
  EEPROM.write(1,(syncword>>8)&0xFF);
  EEPROM.write(2,(syncword>>16)&0xFF);
  for (int i=0;i<8;i++)
    EEPROM.write(i+2,channels[i]);
  EEPROM.commit();
  EEPROM.end();*/
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
  
  servo.attach(servo_pin);
  servo.write(0);

  i2c.begin();
  mpu.begin(MPU6050_I2CADDR_DEFAULT,&i2c);
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  pid.SetOutputLimits(0,180);
  pid.SetMode(QuickPID::Control::automatic);

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
//TODO: pairing

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

  digitalWrite(LED_BUILTIN,millis()%1000<50?HIGH:LOW);

  bool connected=rx(pwm,syncword==0);

  if (connected) {
    //TODO:
  }
  else
    failSafe();

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  int value = a.acceleration.y;
  value = map(value,  -10, 10, 180, 0);
  input=value;
  if(pid.Compute()) {
    // Serial.print("value: ");
    // Serial.print(value);
    // Serial.print("\toutput: ");
    // Serial.println(output);
  }
  servo.write(output);
} 
