//*******************************************************************************************
// SqueezeESP32
// fork from https://github.com/bgiraut/SqueezeEsp32
//*******************************************************************************************
/*
  Wiring:
  --------------------------------
  | VS1053  | ESP8266 |  ESP32   |
  --------------------------------
  |   SCK   |   D5    |   18     |
  |   MISO  |   D6    |   19     |
  |   MOSI  |   D7    |   23     |
  |   XRST  |   RST   |   EN     |
  |   XCS   |   D1    |   32     |
  |   XDCS  |   D0    |   33     |
  |   DREQ  |   D3    |   35     |
  |   5V    |   VIN   |   VCC    |
  |   GND   |   GND   |   GND    |
  --------------------------------
*/

#include <Arduino.h>
#include "config.h"
#include "slimproto.h"
#include <DNSServer.h>            //Local DNS Server used for redirecting all requests to the configuration portal
#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager WiFi Configuration Magic

#include <WiFiUdp.h>
#include <ArduinoOTA.h>

#ifdef ESP32
  #include <WiFi.h>
  #include <WebServer.h> 
#else
  #include <ESP8266WiFi.h>
  #include <ESP8266WebServer.h>     //Local WebServer used to serve the configuration portal
#endif


#ifdef I2S_DAC_MODULE 
  #include "AudioFileSourceICYStream.h"
  #include "AudioFileSourceBuffer.h"
  #include "AudioGeneratorMP3.h"
  #include "AudioOutputI2SNoDAC.h"
#endif

int16_t     viCnxAttempt = -1; 
slimproto   *vislimCli = 0;
WiFiClient  client;
WiFiUDP     udp;
uint32_t    lastRetry;

#ifdef VS1053_MODULE
  #include "flac_plugin.h"

  #undef I2S_DAC_MODULE
    #ifdef ESP32
      #define VS1053_CS     32   // D1 // 5
      #define VS1053_DCS    33  // D0 // 16
      #define VS1053_DREQ   35   // D3 // 4
    #else  // ESP8266
      #define VS1053_CS     D1 // 5
      #define VS1053_DCS    D0 // 16
      #define VS1053_DREQ   D3 // 4
    #endif
  
    #ifdef ADAFRUIT_VS1053
      #include <Adafruit_VS1053.h>
      Adafruit_VS1053 viplayer(-1, VS1053_CS, VS1053_DCS, VS1053_DREQ);
    #else
      #include <VS1053.h>
      VS1053 viplayer(VS1053_CS, VS1053_DCS, VS1053_DREQ);
    #endif
#endif

#ifdef VS1053_MODULE
  void LoadPlugin(const uint16_t* plugin, uint16_t plugin_size) {
    uint16_t i = 0;
    while (i<plugin_size) {
      uint16_t addr, n, val;
      addr = plugin[i++];
      n = plugin[i++];
      if (n & 0x8000U) { /* RLE run, replicate n samples */
        n &= 0x7FFF;
        val = plugin[i++];
        while (n--) {
          #ifdef ADAFRUIT_VS1053
            viplayer.sciWrite(addr, val);
          #else
            viplayer.write_register(addr, val);
          #endif
        }
      } else { /* Copy run, copy n samples */
        while (n--) {
          val = plugin[i++];
          #ifdef ADAFRUIT_VS1053
            viplayer.sciWrite(addr, val);
          #else
            viplayer.write_register(addr, val);
          #endif
        }
      }
    }
  }
#endif //VS1053_MODULE

//**********************************************************************
//      SETUP
//**********************************************************************
void setup() {
 
  viCnxAttempt = 0;
  
  #ifdef DEBUG
    Serial.begin(115200);
  #endif
  delay(1000); // Wait for VS1053 to power up
  
  SPI.begin();
  #ifdef VS1053_MODULE
    viplayer.begin();
    
    #ifdef DEBUG
      Serial.println();
      Serial.println("+++ Start SqueezeESP +++");
      Serial.println("Load FLAC Plugin...");
    #endif
      
    #ifndef ADAFRUIT_VS1053
      viplayer.switchToMp3Mode();
      LoadPlugin(plugin,PLUGIN_SIZE);
      viplayer.setVolume(80);
    #else
      viplayer.begin();
      //viplayer.applyPatch(plugin,PLUGIN_SIZE);
      LoadPlugin(plugin,PLUGIN_SIZE);
      viplayer.setVolume(20,20);
    #endif
  #endif

  #ifdef DEBUG
    Serial.println("Connecting to WiFi...");
  #endif
  
  WiFiManager wifiManager;

  //reset saved settings
  //wifiManager.resetSettings();
  
  wifiManager.autoConnect("SqueezeESP");

  #ifdef OTA_ESP
    ArduinoOTA.setHostname("SqueezeESP8266");
    ArduinoOTA.onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH) {
        type = "sketch";
      } else { // U_SPIFFS
        type = "filesystem";
      }

      #ifdef DEBUG
        // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
        Serial.println("Start updating " + type);
      #endif  
    });
    ArduinoOTA.onEnd([]() {
      #ifdef DEBUG
        Serial.println("\nEnd");
      #endif  
    });
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
      #ifdef DEBUG
        Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
      #endif  
    });
    ArduinoOTA.onError([](ota_error_t error) {
      #ifdef DEBUG
        Serial.printf("Error[%u]: ", error);
      #endif  
      if (error == OTA_AUTH_ERROR) {
        #ifdef DEBUG
          Serial.println("Auth Failed");
        #endif  
      } else if (error == OTA_BEGIN_ERROR) {
        #ifdef DEBUG
          Serial.println("Begin Failed");
        #endif  
      } else if (error == OTA_CONNECT_ERROR) {
        #ifdef DEBUG
          Serial.println("Connect Failed");
        #endif
      } else if (error == OTA_RECEIVE_ERROR) {
        #ifdef DEBUG
          Serial.println("Receive Failed");
        #endif
      } else if (error == OTA_END_ERROR) {
        #ifdef DEBUG
          Serial.println("End Failed");
        #endif
      }
    });
    ArduinoOTA.begin();
  #endif

  udp.begin(UDP_PORT);

  connectToLMS();
}

//**********************************************************************
//      LOOP
//**********************************************************************

void loop() {

  if (client.connected()) {

    #ifdef OTA_ESP
      if (vislimCli->vcPlayerStat != vislimCli->PlayStatus && (millis() - lastRetry) > OTA_HANDLE_TIMER) {
        lastRetry = millis();
        #ifdef DEBUG
          Serial.println("Handle OTA"); 
        #endif   
        ArduinoOTA.handle();
      }  
    #endif

    if (!vislimCli->HandleMessages()) {
      #ifdef DEBUG
        Serial.println("Client Disconnected !"); 
      #endif   
      connectToLMS();
    }
    vislimCli->HandleAudio();
  }
  //yield();
}

//*********************************************************************************************

//*********************************************************************************************

void connectToLMS() {
  if (viCnxAttempt == -1) {
    LMS_addr = IPAddress(0,0,0,0);
  }

  // If no LMS specified in config
  if (LMS_addr[0] == 0) {
    discoverLMS();
  }

  if (LMS_addr[0] != 0) { //if LMS IP address was found or specified in config
    #ifdef DEBUG
      Serial.print("Connecting to LMS server ");Serial.print(LMS_addr);Serial.println(" ..."); 
    #endif

    viCnxAttempt++;

    if (!client.connect(LMS_addr, 3483)) {
      #ifdef DEBUG
        Serial.println("Connection failed, pause and retry...");
      #endif

      viCnxAttempt++;
      
      if (viCnxAttempt > 30)
        viCnxAttempt = -1; // Will erase LMS addr in the next attempt

        delay(2000);
        return;
      }

      viCnxAttempt = 0;

      if (vislimCli) delete vislimCli,vislimCli = 0;

      #ifdef VS1053_MODULE
        vislimCli = new slimproto(LMS_addr.toString(), & client, &viplayer);
      #else
        vislimCli = new slimproto(LMS_addr.toString(), client);
      #endif

      #ifdef DEBUG
        Serial.println("Connection Ok, send HELO to LMS");
      #endif

      reponseHelo *  HeloRsp = new reponseHelo(&client);
      HeloRsp->sendResponse();
    } else {
      #ifdef DEBUG
        Serial.println("No LMS server found, try again in 5 seconds"); 
      #endif
      delay(5000);
      connectToLMS();
    }
}

void discoverLMS() {

  #ifdef DEBUG
    Serial.println("Discover LMS server..."); 
  #endif
  for(int nbSend = 0; nbSend < 10; nbSend++) {
    udp.flush();
    udp.beginPacket("255.255.255.255",UDP_PORT);
    udp.printf("e");      //Send udp packet for autodiscovery
    udp.endPacket();
    
    delay(2000);
    
    if(udp.parsePacket() > 0) {
      char udp_packet; 
      udp_packet = udp.read();

      if(udp_packet == 'E') {
        LMS_addr = udp.remoteIP();
        #ifdef DEBUG
          Serial.print("LMS server found !"); 
          Serial.println(LMS_addr); 
        #endif
        udp.stop(); // initialy comment but crash with ESP32 !!!
          
        break; // LMS found we can go to the next step
      }
      
    } else {
      delay(2000);
    }  
  }

}
