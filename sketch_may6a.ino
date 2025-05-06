  // #include <esp_now.h>
  // #include <WiFi.h>
  // #include "ICM_20948.h"
  // #include <SPI.h>

  // // Hardware Configuration
  // #define RXD2 16
  // #define TXD2 17
  // #define STM_BAUD 921600
  // #define CS_PIN 5
  // #define HSPI_MISO 19
  // #define HSPI_MOSI 23
  // #define HSPI_SCLK 18

  // HardwareSerial stmSerial(2);
  // ICM_20948_SPI myICM;

  // // Network Configuration
  // uint8_t broadcastAddress[] = {0x64, 0xb7, 0x08, 0xc8, 0xb5, 0x1c};

  // // Data Structure (Maintains original flex layout + IMU)
  // typedef struct sensor_data {
  //   // Original flex fields
  //   uint32_t timestamp;
  //   int16_t flex0;
  //   int16_t flex1;
  //   int16_t flex2;
  //   uint8_t dataReady;
    
  //   // New IMU fields
  //   float accX, accY, accZ;
  //   float gyrX, gyrY, gyrZ;
  // } sensor_data;

  // sensor_data sensorReadings;
  // esp_now_peer_info_t peerInfo;

  // // Flex Sensor Variables
  // char serialBuffer[128];
  // int bufferIndex = 0;
  // bool flex0Updated = false;
  // bool flex1Updated = false;
  // bool flex2Updated = false;

  // // IMU Variables
  // unsigned long lastImuSend = 0;
  // #define IMU_INTERVAL 50  // Send IMU every 50ms

  // void OnDataSent(const uint8_t *mac, esp_now_send_status_t status) {
  //   Serial.print("\nSend Status: ");
  //   Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Failed");
    
  //   // Only reset flex data if send succeeded
  //   if (status == ESP_NOW_SEND_SUCCESS && (sensorReadings.dataReady & 0x01)) {
  //     flex0Updated = flex1Updated = flex2Updated = false;
  //     sensorReadings.dataReady &= ~0x01;  // Clear flex ready flag
  //   }
  // }

  // void setup() {
  //   Serial.begin(921600);
  //   stmSerial.begin(STM_BAUD, SERIAL_8N1, RXD2, TXD2);

  //   // Initialize ESP-NOW (original configuration)
  //   WiFi.mode(WIFI_STA);
  //   if (esp_now_init() != ESP_OK) return;
  //   esp_now_register_send_cb(OnDataSent);
  //   memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  //   esp_now_add_peer(&peerInfo);

  //   // Initialize IMU
  //   SPI.begin(HSPI_SCLK, HSPI_MISO, HSPI_MOSI, CS_PIN);
  //   while(myICM.begin(CS_PIN, SPI) != ICM_20948_Stat_Ok){
  //     Serial.println("IMU initializing...");
  //     delay(500);
  //   }

  //   memset(&sensorReadings, 0, sizeof(sensorReadings));
  //   Serial.println("System Ready");
  // }

  // bool parseFlexData(const char* line) {
  //   // Original flex parsing logic
  //   int flexNum, flexValue;
  //   if (sscanf(line, "F%d:%d", &flexNum, &flexValue) == 2) {
  //     switch (flexNum) {
  //       case 0: sensorReadings.flex0 = flexValue; flex0Updated = true; break;
  //       case 1: sensorReadings.flex1 = flexValue; flex1Updated = true; break;
  //       case 2: sensorReadings.flex2 = flexValue; flex2Updated = true; break;
  //       default: return false;
  //     }
      
  //     if (flex0Updated && flex1Updated && flex2Updated) {
  //       sensorReadings.dataReady |= 0x01;  // Set flex ready flag
  //       return true;
  //     }
  //   }
  //   return false;
  // }

  // void sendData() {
  //   sensorReadings.timestamp = micros();
  //   esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&sensorReadings, sizeof(sensorReadings));
    
  //   if (result == ESP_OK) {
  //     Serial.println("\nSent Data:");
  //     if (sensorReadings.dataReady & 0x01) {
  //       Serial.printf("Flex: %d, %d, %d\n", sensorReadings.flex0, sensorReadings.flex1, sensorReadings.flex2);
  //     }
  //     if (sensorReadings.dataReady & 0x02) {
  //       Serial.printf("Accel: %.2f, %.2f, %.2f mg\n", sensorReadings.accX, sensorReadings.accY, sensorReadings.accZ);
  //     }
  //     Serial.printf("Timestamp: %u\n", sensorReadings.timestamp);
  //     Serial.println("-------------------------------");
  //   }
  // }

  // void loop() {
  //   // Original flex processing
  //   while (stmSerial.available() > 0) {
  //     char c = stmSerial.read();
  //     Serial.write(c);
      
  //     if (bufferIndex < sizeof(serialBuffer) - 1) {
  //       serialBuffer[bufferIndex++] = c;
  //     }
      
  //     if (c == '\n' || bufferIndex >= sizeof(serialBuffer) - 1) {
  //       serialBuffer[bufferIndex] = '\0';
  //       if (parseFlexData(serialBuffer)) {
  //         sendData();
  //       }
  //       bufferIndex = 0;
  //     }
  //   }

  //   // IMU processing (non-blocking)
  //   if (myICM.dataReady()) {
  //     myICM.getAGMT();
  //     sensorReadings.accX = myICM.accX();
  //     sensorReadings.accY = myICM.accY();
  //     sensorReadings.accZ = myICM.accZ();
  //     sensorReadings.dataReady |= 0x02;  // Set IMU ready flag
  //   }

  //   // Send IMU data at fixed interval if ready
  //   if (millis() - lastImuSend >= IMU_INTERVAL) {
  //     if (sensorReadings.dataReady & 0x02) {
  //       sendData();
  //       sensorReadings.dataReady &= ~0x02;  // Clear IMU flag
  //       lastImuSend = millis();
  //     }
  //   }

  //   delay(10);
  // }



#include <esp_now.h>
#include <WiFi.h>
#include "ICM_20948.h"
#include <SPI.h>

#define RXD2 16
#define TXD2 17
#define STM_BAUD 921600
#define CS_PIN 5
#define HSPI_MISO 19
#define HSPI_MOSI 23
#define HSPI_SCLK 18

HardwareSerial stmSerial(2);
ICM_20948_SPI myICM;
static const uint8_t broadcastAddr[6] = {0x64,0xB7,0x08,0xC8,0xB5,0x1C};

/* ---------- data packet (packed avoids hidden padding) ---------- */
typedef struct __attribute__((packed)) {
  uint32_t timestamp;
  int16_t  flex0, flex1, flex2;
  uint8_t  dataReady;             // bit 0 = flex, bit 1 = IMU
  float    accX, accY, accZ;
  float    gyrX, gyrY, gyrZ;
} sensor_data_t;

static volatile sensor_data_t pkt;                // shared with ISR
static volatile bool          sendFlag   = false; // let main loop print
static volatile esp_now_send_status_t lastStatus;

/* ---------- ISR‑safe send callback ---------- */
void IRAM_ATTR onDataSent(const uint8_t *, esp_now_send_status_t status)
{
  lastStatus = status;
  sendFlag   = true;               // defer printing to main loop
  if (status == ESP_NOW_SEND_SUCCESS) pkt.dataReady = 0;
}

SET_LOOP_TASK_STACK_SIZE(50*1024); 

void setup()
{
  Serial.begin(115200);            // slower = safer while debugging
  stmSerial.begin(STM_BAUD, SERIAL_8N1, RXD2, TXD2);

  /* ---- Wi‑Fi / ESP‑NOW ---- */
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) { Serial.println("ESP‑NOW init failed"); esp_restart(); }

  esp_now_register_send_cb(onDataSent);

  esp_now_peer_info_t peerInfo;
  memset(&peerInfo, 0, sizeof(peerInfo));          // **DON’T SKIP THIS** :contentReference[oaicite:0]{index=0}
  memcpy(peerInfo.peer_addr, broadcastAddr, 6);
  peerInfo.channel = 0;                            // same Wi‑Fi channel
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) { Serial.println("Add‑peer failed"); esp_restart(); }

  /* ---- IMU ---- */
  SPI.begin(HSPI_SCLK, HSPI_MISO, HSPI_MOSI, CS_PIN);
  while (myICM.begin(CS_PIN, SPI) != ICM_20948_Stat_Ok) {
    Serial.println("Waiting for IMU…");
    delay(500);
  }
  memset((void *)&pkt, 0, sizeof(pkt));
  Serial.println("System ready");
}

/* ---------- helpers ---------- */
void sendPacket()
{
  pkt.timestamp = micros();
  esp_now_send(broadcastAddr, (uint8_t *)&pkt, sizeof(pkt));
}

/* ---------- main loop ---------- */
char lineBuf[128]; uint8_t idx = 0;
unsigned long lastImu = 0;

void loop()
{
  /* print send result (outside ISR) */
  if (sendFlag) {
    sendFlag = false;
    Serial.print("Send: ");
    Serial.println(lastStatus == ESP_NOW_SEND_SUCCESS ? "OK" : "FAIL");
  }

  /* ----- flex UART parsing ----- */
  while (stmSerial.available()) {
    char c = stmSerial.read();
    if (c != '\n' && idx < sizeof(lineBuf)-1) { lineBuf[idx++] = c; continue; }

    lineBuf[idx] = '\0';  idx = 0;   // terminate + reset
    int n,val;
    if (sscanf(lineBuf, "F%d:%d", &n, &val) == 2 && n>=0 && n<=2) {
      if (n==0) pkt.flex0 = val; else if (n==1) pkt.flex1 = val; else pkt.flex2 = val;
      pkt.dataReady |= 0x01;
      sendPacket();
    }
  }

  /* ----- IMU every 50 ms ----- */
  if (myICM.dataReady()) {
    myICM.getAGMT();
    pkt.accX = myICM.accX();  pkt.accY = myICM.accY();  pkt.accZ = myICM.accZ();
    pkt.gyrX = myICM.gyrX();  pkt.gyrY = myICM.gyrY();  pkt.gyrZ = myICM.gyrZ();
    pkt.dataReady |= 0x02;
  }
  if (millis() - lastImu >= 50 && (pkt.dataReady & 0x02)) {
    sendPacket();
    lastImu = millis();
  }
  delay(10);
}
