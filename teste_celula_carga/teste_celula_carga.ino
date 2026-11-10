/*
 * teste_celula_carga.ino — Teste da célula de carga do teste estático (HX711 + ESP32)
 *
 * Baseado na placa_estatico (KiCad) e no testeestatico.ino antigo, mas só com a balança:
 * sem BMP280, sem SD, sem FreeRTOS. Serve para conferir ligação, tara, calibração e ruído.
 *
 * Ligações (iguais às da placa_estatico):
 *   HX711 DT (DOUT) -> GPIO21      HX711 SCK -> GPIO22
 *   HX711 VCC       -> 3V3 (no teste avulso; ver observação)   HX711 GND -> GND
 *   Célula: E+ vermelho, E- preto, A- branco, A+ verde (cores mais comuns; confira a sua)
 *   LED "balança" da placa -> GPIO26 (pisca a cada leitura; no ESP avulso não faz nada)
 *
 * Observação: na placa_estatico o HX711 é alimentado com 5 V. No teste avulso, prefira 3V3:
 * o HX711 funciona de 2,6 a 5,5 V e, assim, o pino DT não manda 5 V para o ESP32.
 *
 * Biblioteca: HX711 (Rob Tillaart) — já instalada em ~/Documents/Arduino/libraries.
 * Placa: "ESP32 Dev Module". Monitor Serial: 115200 baud (ou Serial Plotter no modo leitura).
 */
#include <Arduino.h>
#include <HX711.h>
#include <Preferences.h>

const uint8_t PIN_DT  = 21;
const uint8_t PIN_SCK = 22;
const uint8_t PIN_LED = 26;
const float   G_LOCAL = 9.789f;          // gravidade usada no código antigo (m/s²)
const float   ESCALA_PADRAO = -13.723f;  // contagens por grama = 1/(-0,07287) do testeestatico.ino

HX711 balanca;
Preferences prefs;
bool lendo = false;
uint32_t t0Leitura = 0;

float media(float *v, int n) { float s = 0; for (int i = 0; i < n; i++) s += v[i]; return s / n; }
float desvio(float *v, int n) { float m = media(v, n), s = 0; for (int i = 0; i < n; i++) s += (v[i] - m) * (v[i] - m); return sqrt(s / n); }

String lerLinha(uint32_t timeoutMs = 30000) {
  String s; uint32_t t = millis();
  while (millis() - t < timeoutMs) {
    while (Serial.available()) {
      char c = Serial.read();
      if (c == '\n' || c == '\r') { if (s.length()) { s.trim(); return s; } }
      else s += c;
    }
    delay(5);
  }
  s.trim(); return s;
}

void menu() {
  Serial.println(F("------------------------------------------------"));
  Serial.println(F("TESTE DA CÉLULA DE CARGA (HX711 + ESP32)"));
  Serial.println(F(" l = liga/desliga leitura contínua (10 Hz)   g = 1 leitura agora"));
  Serial.println(F(" t = tara (zerar, sem peso)                  c = calibrar com peso conhecido"));
  Serial.println(F(" r = valores crus + ruído                    i = informações / calibração atual"));
  Serial.println(F(" z = voltar à calibração padrão"));
  Serial.println(F("------------------------------------------------"));
}

void info() {
  Serial.printf("Escala: %.4f contagens/g | Offset (tara): %ld | Ganho: %u | Pinos DT=%u SCK=%u\n",
                balanca.get_scale(), (long)balanca.get_offset(), balanca.get_gain(), PIN_DT, PIN_SCK);
}

void salvar() {
  prefs.begin("celula", false);
  prefs.putFloat("escala", balanca.get_scale());
  prefs.putLong("offset", balanca.get_offset());
  prefs.end();
}

void leituraUnica() {
  if (!balanca.wait_ready_timeout(500)) { Serial.println(F("HX711 sem resposta (confira DT/SCK e alimentação)")); return; }
  float g = balanca.get_units(5);
  Serial.printf("Massa: %9.2f g | %8.4f kg | Força: %8.3f N\n", g, g / 1000.0f, g / 1000.0f * G_LOCAL);
}

void tara() {
  Serial.println(F("Tirando o peso de cima... tara em 2 s (não encoste na célula)"));
  delay(2000);
  balanca.tare(30);
  salvar();
  Serial.printf("Tara feita. Offset = %ld\n", (long)balanca.get_offset());
}

void calibrar() {
  Serial.println(F("CALIBRAÇÃO"));
  Serial.println(F("1) Deixe a célula SEM peso e aperte Enter (faz a tara)."));
  lerLinha(); balanca.tare(30);
  Serial.printf("   Tara: offset = %ld\n", (long)balanca.get_offset());
  Serial.println(F("2) Coloque um peso conhecido e digite a massa em GRAMAS (ex.: 1000) e Enter:"));
  String s = lerLinha(120000);
  float massa = s.toFloat();
  if (massa <= 0) { Serial.println(F("   Valor inválido, calibração cancelada.")); return; }
  delay(1500);                                   // espera o peso parar de balançar
  balanca.calibrate_scale(massa, 30);
  salvar();
  Serial.printf("   Escala nova = %.4f contagens/g (salva na memória do ESP)\n", balanca.get_scale());
  Serial.printf("   Conferindo: %.2f g (deve dar ~%.0f g)\n", balanca.get_units(10), massa);
  Serial.println(F("   Dica: confira com um 2º peso diferente; se der errado, refaça com um peso maior."));
}

void crus() {
  if (!balanca.wait_ready_timeout(500)) { Serial.println(F("HX711 sem resposta")); return; }
  const int N = 20; float v[N];
  for (int i = 0; i < N; i++) v[i] = balanca.read();
  float m = media(v, N), d = desvio(v, N), esc = fabs(balanca.get_scale());
  Serial.printf("Cru: média %.0f | desvio %.1f contagens | ruído ≈ %.2f g\n", m, d, esc > 0 ? d / esc : 0);
  Serial.println(F("  Valores ~ +-8388607 fixos: célula aberta/fio solto. Ruído > 5 g: fios longos sem malha ou fonte ruidosa."));
}

void setup() {
  pinMode(PIN_LED, OUTPUT);
  Serial.begin(115200);
  delay(800);
  Serial.println(F("\nIniciando HX711..."));
  balanca.begin(PIN_DT, PIN_SCK);
  if (!balanca.wait_ready_timeout(2000)) {
    Serial.println(F("ERRO: HX711 não respondeu em 2 s."));
    Serial.println(F("  Confira: DT no GPIO21, SCK no GPIO22, VCC (3V3) e GND do módulo."));
  } else {
    Serial.println(F("HX711 OK"));
  }
  prefs.begin("celula", true);
  float esc = prefs.getFloat("escala", ESCALA_PADRAO);
  long off = prefs.getLong("offset", 0);
  bool temCal = prefs.isKey("escala");
  prefs.end();
  balanca.set_scale(esc);
  if (temCal && off != 0) balanca.set_offset(off); else if (balanca.is_ready()) balanca.tare(20);
  Serial.println(temCal ? F("Calibração carregada da memória.") : F("Usando escala padrão do código antigo (calibre com 'c')."));
  info();
  menu();
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    switch (c) {
      case 'l': lendo = !lendo; t0Leitura = millis();
                if (lendo) Serial.println(F("tempo_ms,massa_g,forca_N")); else Serial.println(F("Leitura parada."));
                break;
      case 'g': leituraUnica(); break;
      case 't': tara(); break;
      case 'c': lendo = false; calibrar(); break;
      case 'r': crus(); break;
      case 'i': info(); break;
      case 'z': balanca.set_scale(ESCALA_PADRAO); balanca.tare(20); salvar(); Serial.println(F("Calibração padrão restaurada.")); info(); break;
      case '\n': case '\r': break;
      default: menu();
    }
  }
  static uint32_t ultimo = 0;
  if (lendo && millis() - ultimo >= 100) {        // 10 Hz (taxa do HX711 com RATE em GND)
    ultimo = millis();
    if (balanca.is_ready()) {
      float g = balanca.get_units(1);
      digitalWrite(PIN_LED, !digitalRead(PIN_LED));
      Serial.printf("%lu,%.2f,%.4f\n", (unsigned long)(millis() - t0Leitura), g, g / 1000.0f * G_LOCAL);
    }
  }
}
