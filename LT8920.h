#ifndef __RADIO_H
#define __RADIO_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <SPI.h>

const uint8_t radio_miso=0;
const uint8_t radio_cs=1;
const uint8_t radio_sck=2;
const uint8_t radio_mosi=3;
const uint8_t radio_reset=4;
const uint8_t sda_pin = 20;
const uint8_t scl_pin = 21;

extern MbedSPI spi;

#define _BV(n) (1 << (n))


#define REGISTER_READ       		0x80
#define REGISTER_WRITE      		0
    
#define R_CHANNEL           		7
#define CHANNEL_RX_BIT      		7
#define CHANNEL_TX_BIT      		8
#define CHANNEL_MASK        		0x7F
#define DEFAULT_CHANNEL     		0x31

#define R_CURRENT           		9
#define CURRENT_POWER_SHIFT 		12
#define CURRENT_POWER_MASK  		0xF000
#define CURRENT_GAIN_SHIFT  		7
#define CURRENT_GAIN_MASK   		0x0F00

#define R_DATARATE          		44
#define DATARATE_MASK       		0xFF00
#define DATARATE_1MBPS      		0x0101
#define DATARATE_250KBPS    		0x0401
#define DATARATE_125KBPS    		0x0801
#define DATARATE_62KBPS     		0x1001

#define R_SYNCWORD1         		36
#define R_SYNCWORD2         		37
#define R_SYNCWORD3         		38
#define R_SYNCWORD4         		39

#define R_PACKETCONFIG      		41
#define PACKETCONFIG_CRC_ON             0x8000
#define PACKETCONFIG_SCRAMBLE_ON        0x4000
#define PACKETCONFIG_PACK_LEN_ENABLE    0x2000
#define PACKETCONFIG_FW_TERM_TX         0x1000
#define PACKETCONFIG_AUTO_ACK           0x0800
#define PACKETCONFIG_PKT_FIFO_POLARITY  0x0400

#define R_STATUS            		48
#define STATUS_CRC_BIT			15
#define STATUS_FEC23_ERROR_BIT		14
#define STATUS_SYNCWORD_RECV_BIT      	7
#define STATUS_PKT_FLAG_BIT      	6
#define STATUS_FIFO_FLAG_BIT      	5

#define R_FIFO              		50
#define R_FIFO_CONTROL      		52

uint16_t LT8920ReadRegister(uint8_t n);
uint8_t LT8920WriteRegister(uint8_t reg,uint16_t val);
void LT8920StartListening(int channel);
void LT8920StopListening();
int LT8920Read(uint8_t *buffer, size_t maxBuffer);
void LT8920SetSyncWord(uint32_t syncWordLow,uint32_t syncWordHigh);
void LT8920SetSyncWordLength(uint8_t option);
void LT8920Begin(uint32_t syncword);
void LT8920SetCurrentControl(uint8_t power, uint8_t gain);
bool LT8920SendPacket(int channel,uint8_t *val, size_t packetSize);
#endif