#ifndef config_h
#define config_h

#include <Ethernet.h>
#include <EthernetUdp.h>

#define DEBUG

//#define WITHOUT_RB

#define OTA_ESP8266
#define OTA_HANDLE_TIMER  5000 

#define VS1053_MODULE
//#define ADAFRUIT_VS1053

//#define I2S_DAC_MODULE

#define UDP_PORT 3483

static IPAddress LMS_addr(0,0,0,0);

#endif
