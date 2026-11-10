/*
 * foguete_tx.ino — Transmissor de telemetria do foguete
 * ESP32-C3 SuperMini + RFM95W (SX1276) + GPS u-blox (M8/M10) + BMP280
 *
 * Envia a 5 Hz (configurável): posição GPS, velocidade NED, altitude
 * barométrica relativa ao boot, satélites e tensão da bateria.
 *
 * Bibliotecas (Arduino Library Manager):
 *   - RadioLib                       (jgromes)
 *   - SparkFun u-blox GNSS v3        (SparkFun_u-blox_GNSS_v3)
 *   - Adafruit BMP280 Library        (+ Adafruit Unified Sensor)
 *
 * Placa: "ESP32C3 Dev Module", USB CDC On Boot: Enabled.
 * Confira o pinout da sua placa antes de ligar!
 */

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <WiFi.h>
#include <RadioLib.h>
#include <SparkFun_u-blox_GNSS_v3.h>
#include <Adafruit_BMP280.h>

// ---------------- Configuração ----------------
#define LORA_FREQ_MHZ   917.0     // 915–928 MHz no Brasil (Anatel, radiação restrita)
#define LORA_BW_KHZ     250.0
#define LORA_SF         7
#define LORA_CR         5         // 4/5
#define LORA_POWER_DBM  14        // 14 dBm já dá >25 dB de margem a 3 km
#define LORA_PREAMBLE   8
#define TX_HZ           5         // 5 (padrão) ou 10
#define GPS_NAV_HZ      5         // taxa de navegação do GPS

// Usar SX1262 (ex.: Heltec Wireless Tracker) em vez de SX1276/RFM95W?
#define RADIO_SX1262    0

// ---------------- Pinos (ESP32-C3 SuperMini) ----------------
#define PIN_SCK    4
#define PIN_MISO   5
#define PIN_MOSI   6
#define PIN_NSS    7
#define PIN_DIO0   3     // DIO0 no SX1276 / DIO1 no SX1262
#define PIN_RST    10
#define PIN_BUSY   RADIOLIB_NC  // só SX1262: ligar ao pino BUSY
#define PIN_GPS_RX 20    // recebe do TX do GPS
#define PIN_GPS_TX 21    // vai para o RX do GPS
#define PIN_SDA    0
#define PIN_SCL    2     // SCL tem pull-up: mantém o pino de boot em nível alto
#define PIN_VBAT   1     // divisor 100k/100k a partir do + da LiPo

// ---------------- Pacote (deve ser idêntico no receptor) ----------------
struct __attribute__((packed)) Packet {
  uint8_t  magic;     // 0xA5
  uint8_t  flags;     // bit0 = GPS com fix 3D, bit1 = barômetro OK
  uint16_t seq;
  int32_t  lat;       // graus * 1e7
  int32_t  lon;       // graus * 1e7
  int32_t  altMsl;    // cm acima do nível do mar (GPS)
  int16_t  baroRel;   // dm em relação ao boot (barômetro)
  int16_t  vN;        // cm/s
  int16_t  vE;        // cm/s
  int16_t  vD;        // cm/s (positivo = descendo)
  uint8_t  sats;
  uint8_t  vbat;      // unidades de 20 mV
};
static_assert(sizeof(Packet) == 26, "tamanho do pacote mudou");

#if RADIO_SX1262
SX1262 radio = new Module(PIN_NSS, PIN_DIO0, PIN_RST, PIN_BUSY);
#else
SX1276 radio = new Module(PIN_NSS, PIN_DIO0, PIN_RST, RADIOLIB_NC);
#endif

SFE_UBLOX_GNSS_SERIAL gnss;
Adafruit_BMP280 bmp(&Wire);

bool  baroOk = false;
float baroP0 = 101325.0f;   // pressão de referência (média no boot)
Packet pkt = {};
uint16_t seq = 0;

// Últimos dados do GPS
struct {
  bool fix = false;
  int32_t lat = 0, lon = 0, altMsl = 0;
  int32_t vN = 0, vE = 0, vD = 0;   // mm/s
  uint8_t sats = 0;
} gps;

static int16_t clamp16(int32_t v) {
  if (v > 32767) return 32767;
  if (v < -32768) return -32768;
  return (int16_t)v;
}

bool beginGnssAt(uint32_t baud) {
  Serial1.end();
  Serial1.begin(baud, SERIAL_8N1, PIN_GPS_RX, PIN_GPS_TX);
  delay(100);
  return gnss.begin(Serial1);
}

void setupGnss() {
  Serial1.setRxBufferSize(1024);
  bool ok = beginGnssAt(38400);           // padrão do u-blox M10
  if (!ok) {
    ok = beginGnssAt(9600);               // padrão do u-blox M8/NEO
    if (ok) {                             // sobe para 38400 para aguentar 5–10 Hz
      gnss.setSerialRate(38400);
      delay(100);
      ok = beginGnssAt(38400);
    }
  }
  if (!ok) {
    Serial.println("ERRO: GPS nao encontrado");
    return;
  }
  gnss.setUART1Output(COM_TYPE_UBX);                 // só UBX, sem NMEA
  gnss.setNavigationFrequency(GPS_NAV_HZ);
  if (!gnss.setDynamicModel(DYN_MODEL_AIRBORNE4g))   // essencial em foguete
    Serial.println("Aviso: falha ao definir modelo dinamico");
  gnss.setAutoPVT(true);
  Serial.println("GPS OK");
}

void setupBaro() {
  Wire.begin(PIN_SDA, PIN_SCL);
  baroOk = bmp.begin(0x76) || bmp.begin(0x77);
  if (!baroOk) { Serial.println("ERRO: BMP280 nao encontrado"); return; }
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                  Adafruit_BMP280::SAMPLING_X2,    // temperatura
                  Adafruit_BMP280::SAMPLING_X8,    // pressão
                  Adafruit_BMP280::FILTER_X2,      // filtro leve (pouco atraso)
                  Adafruit_BMP280::STANDBY_MS_1);
  delay(100);
  double sum = 0;
  for (int i = 0; i < 50; i++) { sum += bmp.readPressure(); delay(20); }
  baroP0 = sum / 50.0;
  Serial.printf("Barometro OK, P0 = %.1f Pa\n", baroP0);
}

void setupRadio() {
  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_NSS);
#if RADIO_SX1262
  // Ajuste a tensão do TCXO para a sua placa (Heltec: 1.8 V; sem TCXO: 0)
  int st = radio.begin(LORA_FREQ_MHZ, LORA_BW_KHZ, LORA_SF, LORA_CR,
                       RADIOLIB_SX126X_SYNC_WORD_PRIVATE, LORA_POWER_DBM,
                       LORA_PREAMBLE, 1.8, false);
#else
  int st = radio.begin(LORA_FREQ_MHZ, LORA_BW_KHZ, LORA_SF, LORA_CR,
                       RADIOLIB_SX127X_SYNC_WORD, LORA_POWER_DBM,
                       LORA_PREAMBLE, 0);
#endif
  if (st != RADIOLIB_ERR_NONE) {
    Serial.printf("ERRO: radio.begin() = %d\n", st);
    while (true) delay(1000);
  }
  radio.setCRC(true);
  Serial.println("Radio OK");
}

void setup() {
  Serial.begin(115200);
  delay(500);
  WiFi.mode(WIFI_OFF);   // economiza energia e evita interferência
  btStop();
  analogReadResolution(12);

  setupRadio();
  setupBaro();
  setupGnss();
}

void readGnss() {
  if (gnss.getPVT()) {
    gps.fix    = (gnss.getFixType() >= 3) && gnss.getGnssFixOk();
    gps.lat    = gnss.getLatitude();
    gps.lon    = gnss.getLongitude();
    gps.altMsl = gnss.getAltitudeMSL();     // mm
    gps.vN     = gnss.getNedNorthVel();     // mm/s
    gps.vE     = gnss.getNedEastVel();
    gps.vD     = gnss.getNedDownVel();
    gps.sats   = gnss.getSIV();
  }
}

void loop() {
  gnss.checkUblox();   // processa bytes da UART o tempo todo
  readGnss();

  static uint32_t lastTx = 0;
  const uint32_t period = 1000 / TX_HZ;
  if (millis() - lastTx < period) return;
  lastTx = millis();

  float baroRelM = 0;
  if (baroOk) {
    float p = bmp.readPressure();
    baroRelM = 44330.0f * (1.0f - powf(p / baroP0, 0.190295f));
  }

  pkt.magic   = 0xA5;
  pkt.flags   = (gps.fix ? 0x01 : 0) | (baroOk ? 0x02 : 0);
  pkt.seq     = seq++;
  pkt.lat     = gps.lat;
  pkt.lon     = gps.lon;
  pkt.altMsl  = gps.altMsl / 10;                 // mm -> cm
  pkt.baroRel = clamp16(lroundf(baroRelM * 10)); // m -> dm
  pkt.vN      = clamp16(gps.vN / 10);            // mm/s -> cm/s
  pkt.vE      = clamp16(gps.vE / 10);
  pkt.vD      = clamp16(gps.vD / 10);
  pkt.sats    = gps.sats;
  uint32_t mv = analogReadMilliVolts(PIN_VBAT) * 2;   // divisor 1:2
  pkt.vbat    = (uint8_t)min<uint32_t>(255, mv / 20);

  int st = radio.transmit((uint8_t*)&pkt, sizeof(pkt));   // ~33 ms em SF7/BW250
  if (st != RADIOLIB_ERR_NONE) Serial.printf("TX erro %d\n", st);

  Serial.printf("seq=%u fix=%d sats=%u lat=%.7f lon=%.7f alt=%.1fm baro=%.1fm vD=%.1fm/s vbat=%.2fV\n",
                pkt.seq, gps.fix, gps.sats, gps.lat * 1e-7, gps.lon * 1e-7,
                gps.altMsl / 1000.0, baroRelM, gps.vD / 1000.0, mv / 1000.0);
}
