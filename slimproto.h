#ifndef slimproto_h
#define slimproto_h

#include "config.h"
#include <Arduino.h>
#include "stRingBuffer.h"
#include <math.h>

//#define ADAFRUIT_VS1053

#ifdef ESP32
  #define RINGBFSIZ 100000
#else
  #ifdef DEBUG
    #define RINGBFSIZ 20000
  #else  
    #define RINGBFSIZ 24000
  #endif
#endif
    
#ifdef ESP32
  #include <WiFi.h>
#else
  #include <ESP8266WiFi.h>	
#endif

#ifdef ADAFRUIT_VS1053
  #include <Adafruit_VS1053.h>
#else
  #include <VS1053.h>
#endif


struct __attribute__((packed)) StrmStructDef
{
  uint8_t command;
  uint8_t autostart;
  uint8_t formatbyte;
  uint8_t pcmsamplesize;
  uint8_t pcmsamplerate;
  uint8_t pcmchannels;
  uint8_t pcmendian;
  uint8_t threshold;
  uint8_t spdif_enable;
  uint8_t trans_period;
  uint8_t trans_type;
  uint8_t flags;
  uint8_t output_threshold;
  uint8_t RESERVED;
  uint32_t replay_gain;
  //uint8_t server_port[2];
  uint16_t server_port;
  uint8_t server_ip[4];
};

struct __attribute__((packed)) audg_packet {
  char  opcode[4];
  uint32_t old_gainL;     // unused
  uint32_t old_gainR;     // unused
  uint8_t  adjust;
  uint8_t  preamp;        // unused
  uint32_t gainL;
  uint32_t gainR;
  // squence ids - unused
};


class responseBase
{
public : 
  responseBase(WiFiClient * pClient);
  ~responseBase();
  virtual void sendResponse() = 0;

protected :

struct stResponse{
                uint8_t command[4];
                uint32_t sizeResponse  = 1;
                };

  WiFiClient * vcClient;
  stResponse  vcResponse;
};


class reponseHelo : public responseBase {
  
public : 
  reponseHelo(WiFiClient * pClient);
void sendResponse();

private :

//                               H    E    L    0     {   SIZE          } {ID} {REV}  {  MAC ADDRESS              } {                   UUID                                                      }{   WLAN CHANNEL} { BYTE RECEIVED ???                   }{LANGUAGE}
// const unsigned char helo[] = {0x48,0x45,0x4c,0x4f ,0x00,0x00,0x00,0x24,0x08, 0x0b ,0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x00       ,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x45,0x4e};

/*
struct HELO_packet {
  char  opcode[4] = {0x48,0x45,0x4c,0x4f};
  u32_t length;
  u8_t  deviceid  = 0x08;
  u8_t  revision  = 0x0b;
  u8_t  mac[6] = {0x00,0x00,0x00,0x00,0x00,0x01};
  u8_t  uuid[16] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01};
  u16_t wlan_channellist  = 0;
  u32_t bytes_received_H, bytes_received_L = 0;
  char  lang[2] = {0x45,0x4e};
  //  u8_t capabilities[];
};

*/

struct __attribute__((packed)) stResponse
{
  char command[4] = {0x48,0x45,0x4c,0x4f};   // HELO
  uint32_t sizeResponse ;
  uint8_t diviceID = 12 ; // 0x0C;   // SqueezeSlave Device = 8
  uint8_t firmwareRevision = 0;  //0x4d; // 0x0b
  uint8_t macAddr[6] = {0x00,0x00,0x00,0x00,0x00,0x01};
  uint8_t uuid[16] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01};
  uint8_t wlanChannel[2] = {0x00,0x00};
  uint8_t receivedData[8] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
  char language[2] = {0x45,0x4e};

  #ifdef ESP32
    char capabilites[95] = "Model=squeezeesp,ModelName=SqueezeEsp32,Firmware=77,ogg,flc,pcm,mp3,SampleRate=44100,HasPreAmp";
  #else
    char capabilites[97] = "Model=squeezeesp,ModelName=SqueezeEsp8266,Firmware=77,ogg,flc,pcm,mp3,SampleRate=44100,HasPreAmp";
  #endif

};

  stResponse  vcResponse;
};


class reponseSTAT : public responseBase {
  
public : 
  reponseSTAT(WiFiClient * pClient);
void sendResponse();

private :

  
struct __attribute__((packed)) STAT_packet {
  char  opcode[4] = {0x53,0x54,0x41,0x54};   // STAT
  uint32_t sizeResponse;
  //u32_t event;
  char  event[4];
  uint8_t  num_crlf;
  uint8_t  mas_initialized;
  uint8_t  mas_mode;
  uint32_t stream_buffer_size;
  uint32_t stream_buffer_fullness;
  uint32_t bytes_received_H;
  uint32_t bytes_received_L;
  uint16_t signal_strength;
  uint32_t jiffies;
  uint32_t output_buffer_size;
  uint32_t output_buffer_fullness;
  uint32_t elapsed_seconds;
  uint16_t voltage;
  uint32_t elapsed_milliseconds;
  uint32_t server_timestamp;
  uint16_t error_code;
};

public : 

STAT_packet vcResponse;

};

typedef struct StrmStructDef StrmStruct;
typedef struct audg_packet AudgStruct;

  
class slimproto
{
public:
      #ifdef ADAFRUIT_VS1053
        slimproto(String pAdrLMS, WiFiClient * pClient, Adafruit_VS1053 * pPlayer);
      #else
        slimproto(String pAdrLMS,WiFiClient * pClient, VS1053 * pPlayer);
      #endif
			
      slimproto(WiFiClient * pClient);
      ~slimproto();
			
			/**
			* Read message from socket and push them in a local buffer
			**/
			uint16_t HandleMessages();
     
      uint16_t HandleAudio();

      enum player_status {
          StopStatus,
          PlayStatus,
          PauseStatus
        };

      player_status vcPlayerStat = StopStatus ;    /* 0 = stop , 1 = play , 2 = pause */

private:

      stRingBuffer * vcRingBuffer;

      String vcAdrLMS;
      uint16_t vcPortLMS;

			uint16_t _pin;
			String vcBufferInput;

      uint16_t vcCommandSize;

      uint8_t * ringbuf ;                                 // Ringbuffer for VS1053

      uint32_t StartTimeCurrentSong = 0;
      uint32_t EndTimeCurrentSong = 0;
      uint32_t ByteReceivedCurrentSong = 0;

      uint32_t TimeCounter = 0;
      uint32_t LastStatMsg = 0;
			
			void HandleCommand(uint8_t pCommand [], uint16_t pSize);
			void HandleStrmQCmd(uint8_t pCommand [], uint16_t pSize);
			void HandleStrmTCmd(uint8_t pCommand [], uint16_t pSize);
			void HandleStrmSCmd(uint8_t pCommand [], uint16_t pSize);
			void HandleStrmPCmd(uint8_t pCommand [], uint16_t pSize);
			void HandleStrmUCmd(uint8_t pCommand [], uint16_t pSize);

      void sendSTAT(const char *event, uint32_t server_timestamp);

      void HandleAudgCmd(uint8_t pCommand [], uint16_t pSize);
      
			void ExtractCommand(uint8_t * pBuf, uint16_t pSize);
			void ByteArrayCpy(uint8_t * pDst, uint8_t * pSrv, uint16_t pSize);

      #ifdef DEBUG
        void PrintByteArray(uint8_t * psrc, uint16_t pSize);
        void PrintByteArray(String psrc, uint16_t pSize);
        void PrintHex8(uint8_t *data, uint8_t length);
      #endif
            
      void packN(uint32_t *dest, uint32_t val);
      void packn(uint16_t *dest, uint16_t val);
      uint32_t unpackN(uint32_t *src);
      uint16_t unpackn(uint16_t *src);
      
		  WiFiClient * vcClient;            // Client to handle control messages
      WiFiClient vcStreamClient;      // Client to handle audio stream
 
      #ifdef ADAFRUIT_VS1053
        Adafruit_VS1053 * vcplayer;
      #else
        VS1053 * vcplayer;
      #endif
   
};
    
#endif
