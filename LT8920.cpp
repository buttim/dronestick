#include "LT8920.h"

uint16_t LT8920ReadRegister(uint8_t reg) {
  uint8_t h, l;

  digitalWrite(radio_cs,LOW);
  delayMicroseconds(10);////////////////////

  spi.transfer(REGISTER_READ | reg);
  h = spi.transfer(0);
  l = spi.transfer(0);

  digitalWrite(radio_cs,HIGH);
  return (h << 8) | l;
}

uint8_t LT8920WriteRegister2(uint8_t reg, uint8_t high, uint8_t low) {
  uint8_t result;

  digitalWrite(radio_cs,LOW);
  result = spi.transfer(REGISTER_WRITE | reg);
  spi.transfer(high);
  spi.transfer(low);

  digitalWrite(radio_cs,HIGH);
  return result;
}

uint8_t LT8920WriteRegister(uint8_t reg, uint16_t val) {
  return LT8920WriteRegister2(reg, val >> 8, val);
}

void LT8920Begin(uint32_t syncword) {
  LT8920WriteRegister(0, 0x6FE0);
  LT8920WriteRegister(2, 0x6617);
  LT8920WriteRegister(4, 0x9CC9);
  LT8920WriteRegister(5, 0x6637);
  LT8920WriteRegister(7, 0x0000);
  LT8920WriteRegister(8, 0x6C90);
  LT8920WriteRegister(9, 0x1840);
  LT8920WriteRegister(11, 0x0008);
  LT8920WriteRegister(13, 0x48BD);
  LT8920WriteRegister(22, 0x00FF);
  LT8920WriteRegister(23, 0x8005);
  LT8920WriteRegister(24, 0x0067);
  LT8920WriteRegister(26, 0x19E0);
  LT8920WriteRegister(27, 0x1300);
  LT8920WriteRegister(32, 0x6800);
  LT8920WriteRegister(33, 0x3FC7);
  LT8920WriteRegister(34, 0x2000);
  LT8920WriteRegister(35, 0x0300);
  LT8920WriteRegister(40, 0x4401);
  LT8920WriteRegister(41, 0xB400);
  LT8920WriteRegister(42, 0xFDB0);
  LT8920WriteRegister(44, 0x0800);
  LT8920WriteRegister(45, 0x0552);
  if (syncword==0) {  //pairing
    LT8920WriteRegister(39, 0x1234);
    LT8920WriteRegister(36, 0x5678);
  } else {
    LT8920WriteRegister(39, syncword>>16);
    LT8920WriteRegister(36, syncword&0xFFFF);
  }
  LT8920WriteRegister(52, 0x8080);
}

/*void LT8920SetCurrentControl(uint8_t power, uint8_t gain) {
  LT8920WriteRegister(R_CURRENT,
                      ((power << CURRENT_POWER_SHIFT) & CURRENT_POWER_MASK) |
                          ((gain << CURRENT_GAIN_SHIFT) & CURRENT_GAIN_MASK));
}*/

void LT8920StopListening() {
  LT8920WriteRegister(R_CHANNEL, 0); // turn off rx/tx
}

void LT8920StartListening(int channel) {
  LT8920WriteRegister(R_FIFO_CONTROL, 0x8080); // flush rx
  LT8920WriteRegister(R_CHANNEL, (channel & CHANNEL_MASK) |
                                     _BV(CHANNEL_RX_BIT)); // enable RX
}

int LT8920Read(uint8_t *buffer, size_t maxBuffer) {
  uint8_t pos = 0, packetSize; 
  
  digitalWrite(radio_cs,LOW);
  spi.transfer(REGISTER_READ|R_FIFO);
  packetSize=spi.transfer(0);
  
  if (packetSize > maxBuffer)
    packetSize=-2;
  else
    while (pos < packetSize)
      buffer[pos++] = spi.transfer(0);
  digitalWrite(radio_cs,HIGH);
  return packetSize;
}

#if 0
void LT8920SetSyncWord(uint32_t syncWordLow, uint32_t syncWordHigh) {
  LT8920WriteRegister(R_SYNCWORD1, syncWordLow);
  LT8920WriteRegister(R_SYNCWORD2, syncWordLow >> 16);
  LT8920WriteRegister(R_SYNCWORD3, syncWordHigh);
  LT8920WriteRegister(R_SYNCWORD4, syncWordHigh >> 16);
}

void LT8920SetSyncWordLength(uint8_t option) {
  option &= 0x03;

  LT8920WriteRegister(32, (LT8920ReadRegister(32) & 0x0300) | (option << 11));
}

bool LT8920SendPacket(int channel, uint8_t *val, size_t packetSize) {
  uint8_t pos;
  if (packetSize < 1 || packetSize > 255)
    return false;

  // LT8920WriteRegister(R_CHANNEL, 0x0000);
  LT8920WriteRegister(R_FIFO_CONTROL, 0); // 0x8000);  //flush tx

  ////////////////////////////////////////////////////////
  LT8920WriteRegister(R_CHANNEL, (channel & CHANNEL_MASK) |
                                     (1 << CHANNEL_TX_BIT)); // enable TX
  ////////////////////////////////////////////////////////

  // packets are sent in 16bit words, and the first word will be the packet
  // size. start spitting out words until we are done.

  pos = 0;
  LT8920WriteRegister2(R_FIFO, packetSize, val[pos++]);
  while (pos < packetSize) {
    uint8_t msb = val[pos++];
    uint8_t lsb = val[pos++];

    LT8920WriteRegister2(R_FIFO, msb, lsb);
  }

  // LT8920WriteRegister(R_CHANNEL,  (_channel & CHANNEL_MASK) |
  // (1<<CHANNEL_TX_BIT));   //enable TX

  // Wait until the packet is sent.
  /*while (digitalRead(_pin_pktflag) == 0)
  {
      //do nothing.
  }*/

  return true;
}
#endif