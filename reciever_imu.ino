#include <esp_now.h>
#include <WiFi.h>
#include <string.h>     // for memcpy()

/* ---------- same packed struct as the sender ---------- */
typedef struct __attribute__((packed)) {
  uint32_t timestamp;          // µs
  int16_t  flex0, flex1, flex2;
  uint8_t  dataReady;          // bit0 = flex, bit1 = IMU
  float    accX, accY, accZ;   // mg
  float    gyrX, gyrY, gyrZ;   // dps
} sensor_data_t;

/* ---------- shared between ISR and loop() ---------- */
static volatile sensor_data_t latestPkt;
static volatile bool          newPkt = false;

/* ---------- ESPNOW receive callback (new API) ---------- */
void IRAM_ATTR onDataRecv(const esp_now_recv_info_t *info,
                          const uint8_t             *data,
                          int                        len)
{
  if (len == sizeof(sensor_data_t)) {
    memcpy((void*)&latestPkt, data, sizeof(sensor_data_t));
    newPkt = true;
  }
}

void setup()
{
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);                   // ESPNOW needs STA or AP

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP‑NOW init failed ‑ restarting");
    esp_restart();
  }
  esp_now_register_recv_cb(onDataRecv);

  /* (optional) add the sender as a peer if you want RSSI info or encryption
  esp_now_peer_info_t peer = {};
  uint8_t senderMac[6] = {0x64,0xB7,0x08,0xC8,0xB5,0x1C};  // <-- your sender's MAC
  memcpy(peer.peer_addr, senderMac, 6);
  peer.ifidx   = WIFI_IF_STA;
  peer.channel = 0;
  peer.encrypt = false;
  esp_now_add_peer(&peer);
  */

  Serial.println("Receiver ready – waiting for packets…");
}

void loop()
{
  if (newPkt) {
    sensor_data_t pkt;
    noInterrupts();                         // atomic copy from ISR buffer
    memcpy(&pkt, (const void*)&latestPkt, sizeof(pkt));
    newPkt = false;
    interrupts();

    /* ---- print in requested f‑string style ---- */
    Serial.printf(
      "Flex Sensors: %d, %d, %d\n"
      "Acceleration (mg): X=%.2f, Y=%.2f, Z=%.2f\n"
      "Gyro (DPS):       X=%.2f, Y=%.2f, Z=%.2f\n"
      "Timestamp: %lu µs\n"
      "---------------------------------------\n",
      pkt.flex0, pkt.flex1, pkt.flex2,
      pkt.accX,  pkt.accY,  pkt.accZ,
      pkt.gyrX,  pkt.gyrY,  pkt.gyrZ,
      (unsigned long)pkt.timestamp
    );
  }

  // keep loop fast so Wi‑Fi task isn’t starved
  delay(1);
}
