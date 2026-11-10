/*
 * tracker_rx.ino — Estação de solo / antena tracker
 * ESP32 DevKit V1 (WROOM-32) + RFM95W (SX1276) + 2 servos 180° (pan/tilt)
 *
 * Recebe a telemetria do foguete, calcula azimute/elevação a partir da
 * posição da base, compensa a latência com a velocidade GPS e aponta a
 * antena. Usa o truque do "flip" para cobrir o hemisfério com servos de 180°.
 *
 * Calibração em campo (sem GPS na base):
 *   1. BASE (GPIO27, segurar 2 s): com o transmissor do foguete EM CIMA do
 *      tracker e com fix, grava a posição da base (média de 20 pacotes).
 *   2. Leve o foguete para a rampa. Com o pan no centro e o tilt em 0°,
 *      gire o tripé até a antena apontar para a rampa.
 *   3. ALINHAR (BOOT/GPIO0, segurar 2 s): grava o azimute da rampa como
 *      "frente" do pan, a altitude GPS da rampa e a referência do barômetro.
 *   Não mova o tripé depois disso. Os valores ficam salvos na flash.
 *
 * Bibliotecas: RadioLib, ESP32Servo, (opcional) Adafruit SSD1306 + GFX.
 * Saída serial CSV a 115200 baud para registro no notebook.
 */

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <RadioLib.h>
#include <ESP32Servo.h>
#include <Preferences.h>

// ---------------- Configuração ----------------
#define LORA_FREQ_MHZ   917.0
#define LORA_BW_KHZ     250.0
#define LORA_SF         7
#define LORA_CR         5
#define LORA_PREAMBLE   8
#define RADIO_SX1262    0        // 1 para Heltec WiFi LoRa 32 V3 etc.

#define SIMULATE        0        // 1 = testa os servos com um voo simulado, sem rádio
#define USE_OLED        1        // 0 se não tiver display

#define LEAD_S          0.15f    // compensação de atraso (rádio + servo), s
#define MAX_PREDICT_S   1.5f     // não extrapola além disso sem pacote novo
#define LINK_LOST_MS    2000
#define SLEW_DEG_S      200.0f   // velocidade máxima comandada aos servos
#define PAN_DIR         1        // -1 se o pan girar ao contrário
#define TILT_DIR        1        // -1 se o tilt girar ao contrário
#define PAN_TRIM_US     0        // ajuste fino do centro do pan
#define TILT_TRIM_US    0        // ajuste fino do horizonte do tilt
#define SERVO_MIN_US    500
#define SERVO_MAX_US    2500

// ---------------- Pinos ----------------
#define PIN_SCK   18
#define PIN_MISO  19
#define PIN_MOSI  23
#define PIN_NSS   5
#define PIN_RST   14
#define PIN_DIO0  26
#define PIN_BUSY  RADIOLIB_NC
#define PIN_PAN   32
#define PIN_TILT  33
#define PIN_BTN_ALIGN 0      // botão BOOT da placa
#define PIN_BTN_BASE  27
#define PIN_BUZZER    25

// ---------------- Pacote (idêntico ao transmissor) ----------------
struct __attribute__((packed)) Packet {
  uint8_t  magic;
  uint8_t  flags;
  uint16_t seq;
  int32_t  lat, lon;
  int32_t  altMsl;    // cm
  int16_t  baroRel;   // dm
  int16_t  vN, vE, vD;// cm/s
  uint8_t  sats;
  uint8_t  vbat;      // 20 mV
};
static_assert(sizeof(Packet) == 26, "tamanho do pacote mudou");

#if RADIO_SX1262
SX1262 radio = new Module(PIN_NSS, PIN_DIO0, PIN_RST, PIN_BUSY);
#else
SX1276 radio = new Module(PIN_NSS, PIN_DIO0, PIN_RST, RADIOLIB_NC);
#endif

#if USE_OLED
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
Adafruit_SSD1306 oled(128, 64, &Wire, -1);
bool oledOk = false;
#endif

Servo servoPan, servoTilt;
Preferences prefs;

// ---------------- Estado ----------------
struct Calib {
  bool   homeSet = false;
  double homeLat = 0, homeLon = 0;   // graus
  double homeAlt = 0;                // m
  bool   aligned = false;
  float  azFront = 0;                // azimute (graus) para onde o pan central aponta
  double padAlt = 0;                 // m (GPS, na rampa)
  float  baroAtPad = 0;              // m (barômetro do foguete na rampa)
  float  padDist = 500;              // m (distância horizontal base–rampa)
} cal;

volatile bool rxFlag = false;
Packet last = {};
bool   haveLast = false;
uint32_t lastRxMs = 0;
float  lastRssi = 0, lastSnr = 0;

// última posição horizontal válida (ENU, m), usada se o GPS cair
double lastN = 0, lastE = 0;
bool   haveLastHoriz = false;

float cmdPan = 90, cmdTilt = 0;     // comando atual (graus de servo)
bool  flipped = false;
float az = 0, el = 0, distM = 0, upM = 0;

// ---------------- Utilidades ----------------
static const double DEG2RAD = PI / 180.0;
static const double R_EARTH = 6371000.0;

void IRAM_ATTR onRx() { rxFlag = true; }

float wrap180(float a) { while (a > 180) a -= 360; while (a < -180) a += 360; return a; }
float wrap360(float a) { while (a >= 360) a -= 360; while (a < 0) a += 360; return a; }

void beep(int ms) { digitalWrite(PIN_BUZZER, HIGH); delay(ms); digitalWrite(PIN_BUZZER, LOW); }

// posição geográfica -> ENU local em relação à base (terra plana: ótimo para < 10 km)
void geoToNE(double latDeg, double lonDeg, double &n, double &e) {
  n = (latDeg - cal.homeLat) * DEG2RAD * R_EARTH;
  e = (lonDeg - cal.homeLon) * DEG2RAD * R_EARTH * cos(cal.homeLat * DEG2RAD);
}

void saveCal() {
  prefs.begin("tracker", false);
  prefs.putBool("homeSet", cal.homeSet);
  prefs.putDouble("hLat", cal.homeLat);
  prefs.putDouble("hLon", cal.homeLon);
  prefs.putDouble("hAlt", cal.homeAlt);
  prefs.putBool("aligned", cal.aligned);
  prefs.putFloat("azFront", cal.azFront);
  prefs.putDouble("padAlt", cal.padAlt);
  prefs.putFloat("baroPad", cal.baroAtPad);
  prefs.putFloat("padDist", cal.padDist);
  prefs.end();
}

void loadCal() {
  prefs.begin("tracker", true);
  cal.homeSet   = prefs.getBool("homeSet", false);
  cal.homeLat   = prefs.getDouble("hLat", 0);
  cal.homeLon   = prefs.getDouble("hLon", 0);
  cal.homeAlt   = prefs.getDouble("hAlt", 0);
  cal.aligned   = prefs.getBool("aligned", false);
  cal.azFront   = prefs.getFloat("azFront", 0);
  cal.padAlt    = prefs.getDouble("padAlt", 0);
  cal.baroAtPad = prefs.getFloat("baroPad", 0);
  cal.padDist   = prefs.getFloat("padDist", 500);
  prefs.end();
}

void writeServos(float panDeg, float tiltDeg) {
  panDeg  = constrain(panDeg, 0, 180);
  tiltDeg = constrain(tiltDeg, 0, 180);
  float p = (PAN_DIR  > 0) ? panDeg  : 180 - panDeg;
  float t = (TILT_DIR > 0) ? tiltDeg : 180 - tiltDeg;
  int usP = SERVO_MIN_US + (int)(p / 180.0f * (SERVO_MAX_US - SERVO_MIN_US)) + PAN_TRIM_US;
  int usT = SERVO_MIN_US + (int)(t / 180.0f * (SERVO_MAX_US - SERVO_MIN_US)) + TILT_TRIM_US;
  servoPan.writeMicroseconds(usP);
  servoTilt.writeMicroseconds(usT);
}

// ---------------- Recepção ----------------
bool pollRadio() {
  if (!rxFlag) return false;
  rxFlag = false;
  Packet p;
  size_t len = radio.getPacketLength();
  int st = radio.readData((uint8_t*)&p, sizeof(p));
  radio.startReceive();
  if (st != RADIOLIB_ERR_NONE || len != sizeof(Packet) || p.magic != 0xA5) return false;
  last = p;
  haveLast = true;
  lastRxMs = millis();
  lastRssi = radio.getRSSI();
  lastSnr  = radio.getSNR();
  return true;
}

// ---------------- Calibração por botão ----------------
// segura o botão 2 s para confirmar (evita toques acidentais)
bool longPress(int pin) {
  if (digitalRead(pin) != LOW) return false;
  uint32_t t0 = millis();
  while (digitalRead(pin) == LOW) {
    if (millis() - t0 > 2000) { beep(80); return true; }
    delay(10);
  }
  return false;
}

void doSetBase() {
  // média de 20 pacotes com fix (~4 s a 5 Hz)
  double sLat = 0, sLon = 0, sAlt = 0; int n = 0;
  uint32_t t0 = millis();
  while (n < 20 && millis() - t0 < 15000) {
    if (pollRadio() && (last.flags & 0x01)) {
      sLat += last.lat * 1e-7; sLon += last.lon * 1e-7; sAlt += last.altMsl / 100.0; n++;
    }
    delay(5);
  }
  if (n < 10) { beep(600); Serial.println("# BASE falhou: sem fix GPS suficiente"); return; }
  cal.homeLat = sLat / n; cal.homeLon = sLon / n; cal.homeAlt = sAlt / n;
  cal.homeSet = true; cal.aligned = false;
  saveCal();
  beep(80); delay(80); beep(80);
  Serial.printf("# BASE gravada: %.7f, %.7f, %.1f m (n=%d)\n", cal.homeLat, cal.homeLon, cal.homeAlt, n);
}

void doAlign() {
  if (!cal.homeSet) { beep(600); Serial.println("# Grave a BASE antes de ALINHAR"); return; }
  double sN = 0, sE = 0, sAlt = 0, sBaro = 0; int n = 0;
  uint32_t t0 = millis();
  while (n < 10 && millis() - t0 < 10000) {
    if (pollRadio() && (last.flags & 0x01)) {
      double pn, pe; geoToNE(last.lat * 1e-7, last.lon * 1e-7, pn, pe);
      sN += pn; sE += pe; sAlt += last.altMsl / 100.0; sBaro += last.baroRel / 10.0; n++;
    }
    delay(5);
  }
  if (n < 5) { beep(600); Serial.println("# ALINHAR falhou: sem fix GPS"); return; }
  double pn = sN / n, pe = sE / n;
  if (hypot(pn, pe) < 30) { beep(600); Serial.println("# Rampa a menos de 30 m da base: afaste o tracker"); return; }
  cal.azFront   = wrap360(atan2(pe, pn) / DEG2RAD);
  cal.padAlt    = sAlt / n;
  cal.baroAtPad = sBaro / n;
  cal.padDist   = hypot(pn, pe);
  cal.aligned   = true;
  haveLastHoriz = false;
  saveCal();
  beep(80); delay(80); beep(80); delay(80); beep(80);
  Serial.printf("# ALINHADO: rampa a %.0f m, azimute %.1f graus\n", hypot(pn, pe), cal.azFront);
}

// ---------------- Simulação de voo (SIMULATE = 1) ----------------
// Rampa 500 m ao norte da base, apogeu ~1100 m, vento de 5 m/s para leste.
void simulateENU(float t, double &n, double &e, double &u, double &vn, double &ve, double &vu) {
  const float tLaunch = 5, tBurn = 3, aBoost = 60, aCoast = 19.6f;
  n = 500; e = 0; u = 0; vn = ve = vu = 0;
  if (t < tLaunch) return;
  float s = t - tLaunch;
  float vBo = aBoost * tBurn, hBo = 0.5f * aBoost * tBurn * tBurn;   // 180 m/s, 270 m
  float tCoast = vBo / aCoast, hApo = hBo + vBo * vBo / (2 * aCoast); // ~1100 m
  if (s < tBurn) { u = 0.5 * aBoost * s * s; vu = aBoost * s; return; }
  if (s < tBurn + tCoast) { float c = s - tBurn; u = hBo + vBo * c - 0.5 * aCoast * c * c; vu = vBo - aCoast * c; return; }
  float d = s - tBurn - tCoast;                    // descida: 20 m/s até 300 m, depois 6 m/s
  float tDrogue = (hApo - 300) / 20;
  ve = 5; e = 5 * d;
  if (d < tDrogue) { u = hApo - 20 * d; vu = -20; }
  else { u = max(0.0f, 300 - 6 * (d - tDrogue)); vu = (u > 0) ? -6 : 0; if (u == 0) ve = 0; }
}

// ---------------- Setup ----------------
void setup() {
  Serial.begin(115200);
  pinMode(PIN_BTN_ALIGN, INPUT_PULLUP);
  pinMode(PIN_BTN_BASE, INPUT_PULLUP);
  pinMode(PIN_BUZZER, OUTPUT);

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  servoPan.setPeriodHertz(50);
  servoTilt.setPeriodHertz(50);
  servoPan.attach(PIN_PAN, SERVO_MIN_US, SERVO_MAX_US);
  servoTilt.attach(PIN_TILT, SERVO_MIN_US, SERVO_MAX_US);
  writeServos(90, 0);   // pan no centro, tilt no horizonte: posição de alinhamento

  loadCal();

#if USE_OLED
  Wire.begin(21, 22);
  oledOk = oled.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  if (oledOk) { oled.clearDisplay(); oled.setTextColor(SSD1306_WHITE); oled.display(); }
#endif

#if SIMULATE
  cal.homeSet = true; cal.homeLat = 0; cal.homeLon = 0; cal.homeAlt = 0;
  cal.aligned = true; cal.azFront = 0; cal.padAlt = 0; cal.baroAtPad = 0;
  Serial.println("# MODO SIMULACAO");
#else
  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_NSS);
#if RADIO_SX1262
  int st = radio.begin(LORA_FREQ_MHZ, LORA_BW_KHZ, LORA_SF, LORA_CR,
                       RADIOLIB_SX126X_SYNC_WORD_PRIVATE, 10, LORA_PREAMBLE, 1.8, false);
  radio.setPacketReceivedAction(onRx);
#else
  int st = radio.begin(LORA_FREQ_MHZ, LORA_BW_KHZ, LORA_SF, LORA_CR,
                       RADIOLIB_SX127X_SYNC_WORD, 10, LORA_PREAMBLE, 0);
  radio.setPacketReceivedAction(onRx);
#endif
  if (st != RADIOLIB_ERR_NONE) { Serial.printf("ERRO radio %d\n", st); while (true) { beep(200); delay(800); } }
  radio.setCRC(true);
  radio.startReceive();
#endif

  Serial.printf("# base=%d (%.7f, %.7f) alinhado=%d azFront=%.1f\n",
                cal.homeSet, cal.homeLat, cal.homeLon, cal.aligned, cal.azFront);
  Serial.println("ms,seq,fix,sats,lat,lon,altMsl_m,baroRel_m,rssi,snr,vbat,az,el,dist_m,up_m,pan,tilt");
  beep(150);
}

// ---------------- Cálculo do alvo ----------------
bool computeTarget() {
  double n, e, u, vn = 0, ve = 0, vu = 0;
#if SIMULATE
  simulateENU(millis() / 1000.0f, n, e, u, vn, ve, vu);
  float dt = LEAD_S;
#else
  if (!haveLast || !cal.homeSet || !cal.aligned) return false;
  float dt = (millis() - lastRxMs) / 1000.0f + LEAD_S;
  if (dt > MAX_PREDICT_S) dt = MAX_PREDICT_S;

  bool fix = last.flags & 0x01, baro = last.flags & 0x02;
  if (fix) {
    geoToNE(last.lat * 1e-7, last.lon * 1e-7, n, e);
    u  = last.altMsl / 100.0 - cal.homeAlt;
    vn = last.vN / 100.0; ve = last.vE / 100.0; vu = -last.vD / 100.0;
    lastN = n; lastE = e; haveLastHoriz = true;
  } else if (baro) {
    // GPS caiu (ex.: na queima): mantém a última posição horizontal
    // (ou a rampa, se ainda não houve fix) e usa a altitude do barômetro
    if (haveLastHoriz) { n = lastN; e = lastE; }
    else { n = cal.padDist * cos(cal.azFront * DEG2RAD); e = cal.padDist * sin(cal.azFront * DEG2RAD); }
    u = (last.baroRel / 10.0 - cal.baroAtPad) + (cal.padAlt - cal.homeAlt);
    dt = 0;   // sem velocidade confiável: não extrapola
  } else {
    return false;
  }
#endif
  n += vn * dt; e += ve * dt; u += vu * dt;
  double h = hypot(n, e);
  distM = sqrt(h * h + u * u);
  upM = u;
  if (h < 3 && fabs(u) < 3) return false;   // alvo em cima da base: mantém posição
  az = wrap360(atan2(e, n) / DEG2RAD);
  el = constrain(atan2(u, h) / DEG2RAD, 0, 90);
  return true;
}

// ---------------- Apontamento com flip ----------------
void updateServos(float dtS) {
  float rel = wrap180(az - cal.azFront);
  // histerese de 5° para não ficar alternando perto de ±90°
  if (!flipped && fabs(rel) > 95) flipped = true;
  if (flipped && fabs(rel) < 85) flipped = false;

  float panT, tiltT;
  if (!flipped) { panT = 90 + rel; tiltT = el; }
  else { float r2 = (rel > 0) ? rel - 180 : rel + 180; panT = 90 + r2; tiltT = 180 - el; }

  float maxStep = SLEW_DEG_S * dtS;
  cmdPan  += constrain(panT  - cmdPan,  -maxStep, maxStep);
  cmdTilt += constrain(tiltT - cmdTilt, -maxStep, maxStep);
  writeServos(cmdPan, cmdTilt);
}

void drawOled() {
#if USE_OLED
  if (!oledOk) return;
  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setCursor(0, 0);
  if (!cal.homeSet) oled.println("BASE: nao gravada");
  else if (!cal.aligned) oled.println("ALINHAR pendente");
  else oled.printf("Az %5.1f  El %4.1f\n", az, el);
  oled.printf("Dist %5.0fm Alt %5.0fm\n", distM, upM);
  uint32_t age = millis() - lastRxMs;
  oled.printf("RSSI %4.0f SNR %4.1f\n", lastRssi, lastSnr);
  oled.printf("Sats %2u  Fix %d  %s\n", last.sats, last.flags & 1, flipped ? "FLIP" : "");
  oled.printf("Bat %.2fV  Pac %lus\n", last.vbat * 0.02f, (unsigned long)(age / 1000));
  if (haveLast && (last.flags & 1))
    oled.printf("%.6f\n%.6f\n", last.lat * 1e-7, last.lon * 1e-7);
  oled.display();
#endif
}

// ---------------- Loop ----------------
void loop() {
#if !SIMULATE
  if (longPress(PIN_BTN_BASE))  doSetBase();
  if (longPress(PIN_BTN_ALIGN)) doAlign();

  if (pollRadio()) {
    computeTarget();
    Serial.printf("%lu,%u,%d,%u,%.7f,%.7f,%.1f,%.1f,%.0f,%.1f,%.2f,%.1f,%.1f,%.0f,%.0f,%.1f,%.1f\n",
                  (unsigned long)lastRxMs, last.seq, last.flags & 1, last.sats,
                  last.lat * 1e-7, last.lon * 1e-7, last.altMsl / 100.0, last.baroRel / 10.0,
                  lastRssi, lastSnr, last.vbat * 0.02f, az, el, distM, upM, cmdPan, cmdTilt);
  }

  // aviso sonoro de link perdido
  static uint32_t lastWarn = 0;
  if (haveLast && millis() - lastRxMs > LINK_LOST_MS && millis() - lastWarn > 2000) {
    lastWarn = millis();
    digitalWrite(PIN_BUZZER, HIGH); delay(30); digitalWrite(PIN_BUZZER, LOW);
  }
#endif

  // laço de controle a 50 Hz
  static uint32_t lastCtl = 0;
  if (millis() - lastCtl >= 20) {
    float dtS = (millis() - lastCtl) / 1000.0f;
    lastCtl = millis();
    if (cal.aligned && computeTarget()) updateServos(dtS);
  }

  static uint32_t lastOled = 0;
  if (millis() - lastOled >= 250) { lastOled = millis(); drawOled(); }
}
