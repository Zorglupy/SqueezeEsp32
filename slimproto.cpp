#include "slimproto.h"

//#define ADAFRUIT_VS1053

#define DEBUG

// Default volume
#define VOLUME  80


responseBase::responseBase(WiFiClient * pClient)
{
  vcClient = pClient;
}

responseBase::~responseBase()
{

}

reponseHelo::reponseHelo(WiFiClient * pClient) : responseBase(pClient)
{
}

void reponseHelo::sendResponse()
{
  // Type of response -> HELO
  WiFi.macAddress(vcResponse.macAddr);    // Get WiFi MAC

  vcResponse.sizeResponse = __builtin_bswap32(sizeof(vcResponse) - 8); // N'inclus pas la commande ni la taille.
  vcClient->write((char*) &vcResponse, sizeof(vcResponse));
#ifdef DEBUG
  Serial.print("Send HELO to Server : ");
#endif

}

reponseSTAT::reponseSTAT(WiFiClient * pClient) : responseBase(pClient)
{
  // Clear the reponse struct
  memset((void *) &vcResponse, '\0', sizeof(vcResponse));
}

void reponseSTAT::sendResponse()
{
  // Type of response -> STAT
  memcpy((void *) vcResponse.opcode, "STAT", 4);

  vcResponse.sizeResponse = __builtin_bswap32(sizeof(vcResponse) - 8); // N'inclus pas la commande ni la taille. 61-8 = 53
  vcClient->write((char*) &vcResponse, sizeof(vcResponse));
#ifdef DEBUG
  //Serial.print("Send STAT (");Serial.print(vcClient->write((char*) &vcResponse, sizeof(vcResponse)));Serial.println(") to Server...");
  Serial.print("Send STAT to Server : ");
#endif

}

/************************************************************
  Constructor  SLIM PROTO
*************************************************************/

#ifdef ADAFRUIT_VS1053
slimproto::slimproto(String pAdrLMS, WiFiClient * pClient, Adafruit_VS1053 * pPlayer)
#else
slimproto::slimproto(String pAdrLMS, WiFiClient * pClient, VS1053 * pPlayer)
#endif
{
  vcCommandSize = 0;

  vcPlayerStat = StopStatus;    /* 0 = stop , 1 = play , 2 = pause */

  uint32_t viFreeMem = system_get_free_heap_size();
  Serial.print("Free Memory for RingBuffer : "), Serial.print(viFreeMem), Serial.println(" bytes");

  // Initialize the ringbuffer for audio
  //vcRingBuffer = new stRingBuffer(RINGBFSIZ > viFreeMem ? viFreeMem-5000 : RINGBFSIZ); // if not enought memory for the ringbuffer use free memory minus 5ko
  vcRingBuffer = new stRingBuffer(RINGBFSIZ); // if not enought memory for the ringbuffer use free memory minus 5ko
  vcAdrLMS = pAdrLMS;

  vcClient = pClient;
  vcplayer = pPlayer;

  LastStatMsg = millis();
  TimeCounter = millis();

  EndTimeCurrentSong = StartTimeCurrentSong = 0;

}

slimproto::slimproto(WiFiClient * pClient) {
  vcCommandSize = 0;

  vcPlayerStat = StopStatus;    /* 0 = stop , 1 = play , 2 = pause */

  vcClient = pClient;

  TimeCounter = millis();
  EndTimeCurrentSong = StartTimeCurrentSong = 0;
}


slimproto::~slimproto()
{
  if (vcRingBuffer) delete vcRingBuffer, vcRingBuffer = 0;
}

//********************************************************
//********************************************************
uint16_t slimproto::HandleMessages()
//********************************************************
//********************************************************
{
  uint8_t viBuffer;
  uint16_t viSizeRead;

  if (vcClient->connected()) {
    if (vcCommandSize == 0 && vcClient->available() >= 2) { // Message Received from Server ?
      uint8_t  viExtractSize[2];

      vcClient->read(viExtractSize, 2);

      // Convert string size into integer
      vcCommandSize = (viExtractSize[0] << 8) | viExtractSize[1];

#ifdef DEBUG

      //if(vcCommandSize != 172)
      //  Serial.print("Expected command size : "), Serial.println(vcCommandSize);
#endif

    }

    if (vcCommandSize > 250) {  // BAD MESSAGE ?
      uint16_t availableSize = vcClient->available();

      Serial.println("Expected command size to big ??!!??");
      Serial.print("Available size : "), Serial.println(availableSize);
      uint8_t  viExtractCommand[availableSize];
      viSizeRead =  vcClient->read(viExtractCommand, availableSize);

#ifdef DEBUG
      PrintByteArray(viExtractCommand, viSizeRead );
#endif

      vcCommandSize = 0;
    }

    if (vcCommandSize && vcClient->available() >= vcCommandSize) {  // GOOD MESSAGE

#ifdef DEBUG
      Serial.print("Message received : ");
#endif

      uint8_t  viExtractCommand[vcCommandSize];

      viSizeRead =  vcClient->read(viExtractCommand, vcCommandSize);

      if (viSizeRead != vcCommandSize) {
        Serial.println("Not enough data as expected !!!");
      }

      if (vcCommandSize != 172)
#ifdef DEBUG
        PrintByteArray(viExtractCommand, viSizeRead);
#endif

      HandleCommand(viExtractCommand, viSizeRead);  // Execute la commande recue
      vcCommandSize = 0;
    }
  }

  // Send Stat Message if last one is more than 60 seconds
  if (millis() - LastStatMsg >= ( 60 * 1000)) {
    Serial.println("No Stat request from 60 seconds, Is there any probem ?");

    // *$***
    //reponseSTAT viResponse(vcClient);
    //viResponse.vcResponse.elapsed_seconds = 0;
    //viResponse.sendResponse();
    //LastStatMsg = millis();
    return false;
  }

  return true;
}

//********************************************************
//********************************************************
uint16_t slimproto::HandleAudio()
//********************************************************
//********************************************************
{
  // Lire des données du stream et les envoyer dans le vs1053
  uint32_t viRead;
  __attribute__((aligned(4))) uint8_t buf[32] ; // Buffer for chunk

#ifndef WITHOUT_RB
  if (vcStreamClient.connected()) {   // Fill in Ring Buffer

    if (vcPlayerStat == StopStatus) { // Status STOP

      vcRingBuffer->clearBuffer();
      vcStreamClient.stop();

      reponseSTAT viResponseF(vcClient);
      memcpy((void *) viResponseF.vcResponse.event, "STMf", 4);   // Connection Flushed

      packN(&viResponseF.vcResponse.bytes_received_H, (uint32_t)ByteReceivedCurrentSong >> 32);
      packN(&viResponseF.vcResponse.bytes_received_L, (uint32_t)ByteReceivedCurrentSong & 0xffffffff);

      //viResponse.vcResponse.elapsed_seconds =  (EndTimeCurrentSong - StartTimeCurrentSong)/ 1000;
      packN(&viResponseF.vcResponse.elapsed_seconds, (EndTimeCurrentSong - StartTimeCurrentSong) / 1000);
      packN(&viResponseF.vcResponse.elapsed_milliseconds, (EndTimeCurrentSong - StartTimeCurrentSong));

      packN(&viResponseF.vcResponse.stream_buffer_fullness, vcRingBuffer->dataSize());  // ??
      packN(&viResponseF.vcResponse.stream_buffer_size, vcRingBuffer->getBufferSize()); // ???

      packN(&viResponseF.vcResponse.output_buffer_fullness, vcRingBuffer->getBufferSize());  // ??
      packN(&viResponseF.vcResponse.output_buffer_size, vcRingBuffer->getBufferSize()); // ???

      packN(&viResponseF.vcResponse.jiffies, millis()); // ??

      viResponseF.sendResponse();
      return true;
    }

/*
    if (vcPlayerStat == PauseStatus) { // Status PAUSE  // TEST
      vcplayer->stopSong();
      Serial.println("PAUSE Stream");
      return true;
    }
*/
    viRead =  vcStreamClient.available() ;

    while (vcRingBuffer->isFreeSpace() && viRead-- ) {
      vcRingBuffer->putData(vcStreamClient.read()) ;                // Yes, store one byte in ringbuffer
      yield();
    }

#ifdef DEBUG
    if (millis() - TimeCounter >= ( 5 * 1000)) {  // Every 5 seconds display RingBuffer Status
      TimeCounter = millis();
      Serial.print("Audio : RingBuffer Size : "); Serial.print(vcRingBuffer->dataSize());
      Serial.print(" / "); Serial.println(vcRingBuffer->getBufferSize());
    }
#endif

  } else { //stream disconnected

    if (vcPlayerStat == PlayStatus) { // Status PLAY
      vcPlayerStat = PauseStatus;    /* 0 = stop , 1 = play , 2 = pause */

#ifdef DEBUG
      Serial.println("Audio : Stream disconnected");
#endif

      EndTimeCurrentSong = millis();

      // Stream buffer empty
      reponseSTAT viResponse(vcClient);
      memcpy((void *) viResponse.vcResponse.event, "STMd", 4);    // Decoder Ready

      packN(&viResponse.vcResponse.bytes_received_H, (uint32_t)ByteReceivedCurrentSong >> 32);
      packN(&viResponse.vcResponse.bytes_received_L, (uint32_t)ByteReceivedCurrentSong & 0xffffffff);

      //viResponse.vcResponse.elapsed_seconds =  (EndTimeCurrentSong - StartTimeCurrentSong)/ 1000;
      packN(&viResponse.vcResponse.elapsed_seconds, (EndTimeCurrentSong - StartTimeCurrentSong) / 1000);
      packN(&viResponse.vcResponse.elapsed_milliseconds, (EndTimeCurrentSong - StartTimeCurrentSong));

      packN(&viResponse.vcResponse.stream_buffer_fullness, vcRingBuffer->dataSize());  // ??
      packN(&viResponse.vcResponse.stream_buffer_size, vcRingBuffer->getBufferSize()); // ???

      packN(&viResponse.vcResponse.output_buffer_fullness, vcRingBuffer->getBufferSize());  // ??
      packN(&viResponse.vcResponse.output_buffer_size, vcRingBuffer->getBufferSize()); // ???

      packN(&viResponse.vcResponse.jiffies, millis()); // ??

      viResponse.sendResponse();
    }

    if (vcPlayerStat == PauseStatus && millis() - TimeCounter >= ( 1 * 1000)) {  // Every 5 seconds display RingBuffer Status
      TimeCounter = millis();
      Serial.print("Audio : RingBuffer Size : "); Serial.print(vcRingBuffer->dataSize());
      Serial.print(" / "); Serial.println(vcRingBuffer->getBufferSize());
    }

    if (vcPlayerStat == PauseStatus && vcRingBuffer->dataSize() == 0) { // Status PLAY

      //vcPlayerStat = StopStatus;     // pour test

#ifdef DEBUG
      Serial.println("Audio : Paused and Buffer empty");
#endif

      EndTimeCurrentSong = millis();

      reponseSTAT viResponseU(vcClient);
      memcpy((void *) viResponseU.vcResponse.event, "STMu", 4);   // Underrun

      packN(&viResponseU.vcResponse.bytes_received_H, (uint32_t)ByteReceivedCurrentSong >> 32);
      packN(&viResponseU.vcResponse.bytes_received_L, (uint32_t)ByteReceivedCurrentSong & 0xffffffff);

      packN(&viResponseU.vcResponse.elapsed_seconds, (EndTimeCurrentSong - StartTimeCurrentSong) / 1000);
      packN(&viResponseU.vcResponse.elapsed_milliseconds, (EndTimeCurrentSong - StartTimeCurrentSong));

      //packN(&viResponseU.vcResponse.elapsed_seconds, 0);

      packN(&viResponseU.vcResponse.stream_buffer_fullness, 0);  // ??
      packN(&viResponseU.vcResponse.stream_buffer_size, vcRingBuffer->getBufferSize()); // ???

      packN(&viResponseU.vcResponse.output_buffer_fullness, 0);  // ??
      packN(&viResponseU.vcResponse.output_buffer_size, vcRingBuffer->getBufferSize()); // ???

      packN(&viResponseU.vcResponse.jiffies, millis()); // ??

      viResponseU.sendResponse();
    }

#ifdef DEBUG
    if (millis() - TimeCounter >= ( 5 * 1000)) { // Every 5 seconds check End of Stream
      TimeCounter = millis();
      Serial.println("Audio : Stream not connected");
    }
#endif
  }

#endif
 
  // Send data to the vs1053 if available
#ifdef ADAFRUIT_VS1053
  while (vcplayer->readyForData() && vcRingBuffer->dataSize() > 0) // Try to keep ADAFRUIT VS1053 filled
#else
#ifndef WITHOUT_RB
  while (vcplayer->data_request() && vcRingBuffer->dataSize() > 0) // Try to keep VS1053 filled
#endif
#endif

#ifndef WITHOUT_RB
  {
    uint16_t viByteRetrieve = 0;

    while (vcRingBuffer->dataSize() && viByteRetrieve < 32 ) { // Get Max 32 byte of data from the RingBuffer
      buf[viByteRetrieve++] =  vcRingBuffer->getData();
    }

    ByteReceivedCurrentSong += viByteRetrieve;

#ifdef ADAFRUIT_VS1053
    vcplayer->playData (buf, viByteRetrieve) ;
#else
    vcplayer->playChunk (buf, viByteRetrieve) ;
#endif

    yield() ;
  }
#endif

#ifdef WITHOUT_RB
  if (vcStreamClient.available() > 0) {
    uint8_t bytesRead = vcStreamClient.read(buf, 32);
    vcplayer->playChunk (buf, bytesRead);
    yield();
  }
#endif

}

//**************************************************************
void slimproto::HandleCommand(uint8_t pCommand[], uint16_t pSize)
//**************************************************************
{
  byte viCommand[5] = {0};

  ByteArrayCpy(viCommand, pCommand, 4);

  //Serial.print("Handle command : ");Serial.println((char *) viCommand);

  if (strcmp((const char *)viCommand, "strm") == 0) {
    //Serial.println("strm command");
    char viSubCmd;

    viSubCmd = (char) pCommand[4];

    //Serial.print("Sub command : ");Serial.println(viSubCmd);
    switch (viSubCmd) {
      case 'q':
        Serial.println("Sub command q (STOP)");
        HandleStrmQCmd(pCommand, pSize );
        break;
      case 't':
        Serial.println("Sub command t (STATUS)");
        HandleStrmTCmd(pCommand, pSize );
        break;
      case 'p':
        Serial.println("Sub command p (PAUSE)");
        HandleStrmPCmd(pCommand, pSize );
        break;
      case 'u':
        Serial.println("Sub command u (UNPAUSE)");
        HandleStrmUCmd(pCommand, pSize );
        break;
      case 's':
        Serial.println("Sub command s (START)");
        HandleStrmSCmd(pCommand, pSize );
        break;
      case 'a':
        Serial.println("Sub command a (SKIP-AHEAD)");
        break;
      case 'f':
        Serial.println("Sub command f (FLUSH)");
        break;
      default:
        Serial.println("default SubCommand");
    }
  }


  else if (strcmp((const char *)viCommand, "vfdc") == 0) {  // Message display
    Serial.println("vfdc command");
  }

  else if (strcmp((const char *)viCommand, "aude") == 0) {  // Message display
    Serial.println("aude command");
  }
  else if (strcmp((const char *)viCommand, "audg") == 0) {  // Modification volume
    Serial.println("audg command");
    HandleAudgCmd(pCommand, pSize);
  }
  else {
    Serial.print("Other command : [");
    Serial.print((char *)viCommand);
    Serial.println("]");
    Serial.print("Size : ");
    Serial.println(pSize);
  }
}

/***********************************************
   Stop : command Q
 ***********************************************/
void slimproto::HandleStrmQCmd(uint8_t pCommand [], uint16_t pSize)
{
#ifndef ADAFRUIT_VS1053
  vcplayer->stopSong();
#endif

  //  reponseSTAT * viResponse = 0;
  reponseSTAT viResponse(vcClient);

  vcPlayerStat = StopStatus;
  EndTimeCurrentSong = millis();

  //viResponse = new reponseSTAT(vcClient);

  memcpy((void *) viResponse.vcResponse.event, "STMf", 4);

  packN(&viResponse.vcResponse.stream_buffer_fullness, vcRingBuffer->dataSize());
  packN(&viResponse.vcResponse.stream_buffer_size, vcRingBuffer->getBufferSize());

  packN(&viResponse.vcResponse.output_buffer_fullness, vcRingBuffer->dataSize());
  packN(&viResponse.vcResponse.output_buffer_size, vcRingBuffer->getBufferSize());

  //packN(&viResponse.vcResponse.elapsed_seconds, (EndTimeCurrentSong - StartTimeCurrentSong) / 1000);
  //packN(&viResponse.vcResponse.elapsed_milliseconds, (EndTimeCurrentSong - StartTimeCurrentSong));

  viResponse.vcResponse.elapsed_seconds =  0;    // A voir

  viResponse.sendResponse();                     // Send stop confirm
}

/************************************************************
   Status : command T
 ************************************************************ */
void slimproto::HandleStrmTCmd(uint8_t pCommand [], uint16_t pSize)
{
  StrmStruct strmInfo;
  memcpy(&strmInfo, pCommand + 4, sizeof(strmInfo));

  reponseSTAT viResponse(vcClient);
  memcpy((void *) viResponse.vcResponse.event, "STMt", 4);
  //memcpy((void *) viResponse.vcResponse.event, "strm", 4);  // ?????

  //packN(&pkt.stream_buffer_fullness, status.stream_full);
  //packN(&pkt.stream_buffer_size, status.stream_size);

  packN(&viResponse.vcResponse.stream_buffer_fullness, vcRingBuffer->dataSize());
  packN(&viResponse.vcResponse.stream_buffer_size, vcRingBuffer->getBufferSize());

  packN(&viResponse.vcResponse.output_buffer_fullness, vcRingBuffer->dataSize());
  packN(&viResponse.vcResponse.output_buffer_size, vcRingBuffer->getBufferSize());

  //viResponse.vcResponse.bytes_received_L = ByteReceivedCurrentSong;

  //packN(&pkt.bytes_received_H, (u64_t)status.stream_bytes >> 32);
  //packN(&pkt.bytes_received_L, (u64_t)status.stream_bytes & 0xffffffff);

  packN(&viResponse.vcResponse.bytes_received_H, (uint32_t)ByteReceivedCurrentSong >> 32);
  packN(&viResponse.vcResponse.bytes_received_L, (uint32_t)ByteReceivedCurrentSong & 0xffffffff);

  viResponse.vcResponse.server_timestamp = strmInfo.replay_gain;

  // If current stat is 'stop' send 0 as elapsed time
  if (vcPlayerStat == StopStatus) {
    viResponse.vcResponse.elapsed_seconds = 0;

    //packN(&viResponse.vcResponse.elapsed_seconds, (EndTimeCurrentSong - StartTimeCurrentSong) / 1000);
    //packN(&viResponse.vcResponse.elapsed_milliseconds, (EndTimeCurrentSong - StartTimeCurrentSong));

    // else elapsed time of the current song
  } else {
    uint32_t elapsedMSeconds = millis() - StartTimeCurrentSong;
    uint32_t elapsedSeconds = elapsedMSeconds / 1000;
    //packN(&viResponse.vcResponse.elapsed_seconds, (millis() - StartTimeCurrentSong) / 1000);
    packN(&viResponse.vcResponse.elapsed_seconds, elapsedSeconds);
    //packN(&viResponse.vcResponse.elapsed_milliseconds, (millis() - StartTimeCurrentSong));
    packN(&viResponse.vcResponse.elapsed_milliseconds, elapsedMSeconds);

#ifdef DEBUG
    Serial.print("Elapsed Seconds : "); Serial.print(elapsedSeconds); Serial.print(" / ");
    Serial.print("Elapsed Mille Seconds : "); Serial.println(elapsedMSeconds);
#endif

  }

  float RssI = WiFi.RSSI();
  RssI = isnan(RssI) ? -100.0 : RssI;
  RssI = min(max(2 * (RssI + 100.0), 0.0), 100.0);
#ifdef DEBUG
  Serial.print("Wifi Signal : "); Serial.print((uint16_t)RssI); Serial.println("%");
#endif
  packn(&viResponse.vcResponse.signal_strength, RssI);

  packN(&viResponse.vcResponse.jiffies, millis());

  viResponse.sendResponse();  // Send Packet STAT to server

  LastStatMsg = millis();

#ifdef DEBUG
  PrintByteArray((uint8_t *)&viResponse.vcResponse, sizeof(viResponse.vcResponse));
#endif
}


/**********************************************
  Start : command S
***********************************************/

void slimproto::HandleStrmSCmd(uint8_t pCommand [], uint16_t pSize)
{
  char viUrl[pSize + 100];
  uint8_t viTmpUrl[pSize + 100];

  memset(viUrl, 0, sizeof(viUrl));
  memset(viTmpUrl, 0, sizeof(viTmpUrl));

  StrmStruct strmInfo;
  memcpy(&strmInfo, pCommand + 4, sizeof(strmInfo));

#ifdef DEBUG
  if (strmInfo.formatbyte == 'm') {
    Serial.println("Format MP3");
  } else if (strmInfo.formatbyte == 'f') {
    Serial.println("Format FLAC");
  } else if (strmInfo.formatbyte == '0') {
    Serial.println("Format OGG");
  }
#endif

  packn(&vcPortLMS, strmInfo.server_port);

  ByteArrayCpy(viTmpUrl, pCommand + sizeof(strmInfo) + 4 , pSize - sizeof(strmInfo) - 4);

  if (strmInfo.server_ip[0] == 0) {
    vcAdrLMS.toCharArray(viUrl, pSize);
  } else
    sprintf(viUrl, "%d.%d.%d.%d", strmInfo.server_ip[0], strmInfo.server_ip[1], strmInfo.server_ip[2], strmInfo.server_ip[3]);

  Serial.print("Server IP:Port : "); Serial.print(viUrl); Serial.print(":"); Serial.println(vcPortLMS);

  //file = new AudioFileSourceICYStream((char *) viUrl);
  //file = new AudioFileSourceICYStream("http://192.168.0.11:9000/stream.mp3");

  vcRingBuffer->clearBuffer();      // Flush Ring Buffer

#ifndef ADAFRUIT_VS1053
  vcplayer->startSong();
#else
  //vcplayer-> begin();
  //vcplayer->softReset();
#endif

  vcPlayerStat  = PlayStatus;   // on flag l'état à 'lecture'

  StartTimeCurrentSong = millis();   // Starting time

  ByteReceivedCurrentSong = 0;

  Serial.println("Connecting to stream... ");

  if (!vcStreamClient.connect(viUrl, vcPortLMS)) {
    Serial.println("Connection failed");
    return;
  }

  Serial.println("Connexion ok");

  Serial.print("Ask for : "), Serial.println(String((char*) viTmpUrl));

  vcStreamClient.print(String("") + String((char*) viTmpUrl) + "\r\n" +
                       "Host: " + viUrl + "\r\n" +
                       "Connection: close\r\n\r\n");
  // Open Stream

  reponseSTAT viResponseSTMc(vcClient);
  memcpy((void *) viResponseSTMc.vcResponse.event, "STMc", 4);  // Connect
  viResponseSTMc.sendResponse();

  reponseSTAT viResponseSTMe(vcClient);
  memcpy((void *) viResponseSTMe.vcResponse.event, "STMe", 4);  //Stream Connection Established
  viResponseSTMe.sendResponse();

  // Send HTTP headers received from stream connection
  reponseSTAT viResponseSTMh(vcClient);
  memcpy((void *) viResponseSTMh.vcResponse.event, "STMh", 4);  // HTTP headers received
  viResponseSTMh.sendResponse();

  // Send Track Started
  reponseSTAT viResponseSTMs(vcClient);
  memcpy((void *) viResponseSTMs.vcResponse.event, "STMs", 4);  // Track Started
  viResponseSTMs.sendResponse();
}

/*********************************************************
  Pause : command P
**********************************************************/
void slimproto::HandleStrmPCmd(uint8_t pCommand [], uint16_t pSize)
{
  // Send Pause confirm
  reponseSTAT viResponseSTMp(vcClient);

  StrmStruct strmInfo;
  memcpy(&strmInfo, pCommand + 4, sizeof(strmInfo));

  // Inutile...
  uint32_t interval = unpackN(&strmInfo.replay_gain);
  
  #ifdef DEBUG
    Serial.print("Replay_Gain : "); Serial.println(interval);
  #endif

  memcpy((void *) viResponseSTMp.vcResponse.event, "STMp", 4);
  viResponseSTMp.sendResponse();

  vcPlayerStat = PauseStatus;
}

/*********************************************************
  Unpause : command R
**********************************************************/

void slimproto::HandleStrmUCmd(uint8_t pCommand [], uint16_t pSize)
{
  reponseSTAT viResponseSTMp(vcClient);
  //vcplayer->pausePlaying(false); // Adafruit VS1053
  // Send UnPause confirm

  vcPlayerStat  = PlayStatus;   // on flag l'état à 'lecture'

  StartTimeCurrentSong = millis();   // Starting time

  memcpy((void *) viResponseSTMp.vcResponse.event, "STMr", 4);
  viResponseSTMp.sendResponse();

}

/**********************************************************
   Handle volume control
 **********************************************************/
void slimproto::HandleAudgCmd(uint8_t pCommand [], uint16_t pSize)
{
  audg_packet viVolDef;

  memcpy(&viVolDef, pCommand, sizeof(audg_packet));

  uint32_t viVol = unpackN((uint32_t *) (pCommand + 14)) ;

  Serial.print("Volume : "); Serial.println(viVol);

  uint32_t  newvolume = ((100 * log10((viVol * 100) / 65)) / 5);

  Serial.print("New volume  : "); Serial.println(newvolume);

#ifdef ADAFRUIT_VS1053
  vcplayer->setVolume((viVol  * 100) / 65536, (viVol  * 100) / 65536);
#else
  vcplayer->setVolume(newvolume);
#endif
}


//***********************************************************************************

//***********************************************************************************

void slimproto::ExtractCommand(uint8_t * pBuf, uint16_t pSize) {
  for (int i = 0; i < pSize; i++)
  {
    pBuf[i] = vcBufferInput[i];
  }

  vcBufferInput.remove(0, pSize);
}

void slimproto::ByteArrayCpy(uint8_t * pDst, uint8_t * pSrv, uint16_t pSize) {
  for (int i = 0; i < pSize; i++) {
    pDst[i] = pSrv[i];
  }
}

void slimproto::packN(uint32_t *dest, uint32_t val) {
  uint8_t *ptr = (uint8_t *)dest;
  *(ptr)   = (val >> 24) & 0xFF; *(ptr + 1) = (val >> 16) & 0xFF; *(ptr + 2) = (val >> 8) & 0xFF; *(ptr + 3) = val & 0xFF;
}

void slimproto::packn(uint16_t *dest, uint16_t val) {
  uint8_t *ptr = (uint8_t *)dest;
  *(ptr) = (val >> 8) & 0xFF; *(ptr + 1) = val & 0xFF;
}

uint32_t slimproto::unpackN(uint32_t *src) {
  uint8_t *ptr = (uint8_t *)src;
  return *(ptr) << 24 | *(ptr + 1) << 16 | *(ptr + 2) << 8 | *(ptr + 3);
}

uint16_t slimproto::unpackn(uint16_t *src) {
  uint8_t *ptr = (uint8_t *)src;
  return *(ptr) << 8 | *(ptr + 1);
}

void slimproto::PrintByteArray(uint8_t * psrc, uint16_t pSize) {
  Serial.print("Array(");
  Serial.print(pSize);
  Serial.print(") : ");

  char tmp[16];
  for (int i = 0; i < pSize; i++) {
    sprintf(tmp, "%.2X", psrc[i]);
    Serial.print(tmp); Serial.print(" ");
  }

  Serial.println("");
}

void slimproto::PrintByteArray(String psrc, uint16_t pSize) {
  Serial.print("Array : ");
  for (int i = 0; i < pSize; i++) {
    Serial.print(psrc[i], HEX);
  }
  Serial.println("");
}

void slimproto::PrintHex8(uint8_t *data, uint8_t length) { // prints 8-bit data in hex with leading zeroes
  Serial.print("Array(");
  Serial.print(length);
  Serial.print(") : ");

  char tmp[16];
  for (int i = 0; i < length; i++) {
    sprintf(tmp, "%.2X", data[i]);
    Serial.print(tmp); Serial.print(" ");
  }
  Serial.println("");
}
