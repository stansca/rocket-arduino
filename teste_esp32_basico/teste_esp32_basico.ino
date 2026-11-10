/*
 * teste_esp32_basico.ino — Teste do ESP32 sozinho (DevKit, sem nada ligado)
 * Placa na IDE: "ESP32 Dev Module". Monitor Serial: 115200 baud.
 *
 * O que ele faz:
 *  - imprime os dados do chip (modelo, núcleos, flash, RAM, MAC);
 *  - pisca o LED do GPIO2 (DevKit V1 tem; o DevKitC não tem LED de uso, só o de energia);
 *  - a cada 1 s responde "ESP32 funcionando" com tempo ligado e RAM livre;
 *  - aperte o botão BOOT (GPIO0) e ele avisa que leu o botão;
 *  - digite qualquer coisa no Monitor Serial e ele devolve (eco);
 *  - digite w para procurar redes Wi-Fi (testa o rádio do chip).
 */
#include <WiFi.h>

const int LED = 2;
const int BOTAO_BOOT = 0;
uint32_t ultimo = 0, contador = 0;
bool estadoLed = false, botaoAntes = HIGH;

void scanWiFi() {
  Serial.println("Procurando redes Wi-Fi...");
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  int n = WiFi.scanNetworks();
  if (n <= 0) Serial.println("Nenhuma rede encontrada (o rádio pode estar com problema, ou não há redes por perto).");
  for (int i = 0; i < n; i++)
    Serial.printf("  %2d) %-28s  %4d dBm  canal %d\n", i + 1, WiFi.SSID(i).c_str(), WiFi.RSSI(i), WiFi.channel(i));
  WiFi.scanDelete();
  WiFi.mode(WIFI_OFF);
  Serial.printf("Rádio Wi-Fi: %s\n", n > 0 ? "OK" : "sem resposta");
}

void setup() {
  pinMode(LED, OUTPUT);
  pinMode(BOTAO_BOOT, INPUT_PULLUP);
  Serial.begin(115200);
  delay(1000);
  Serial.println();
  Serial.println("==============================================");
  Serial.println("  ESP32 OK! Teste básico iniciado");
  Serial.println("==============================================");
  Serial.printf("Modelo: %s (revisão %d), %d núcleos a %d MHz\n",
                ESP.getChipModel(), ESP.getChipRevision(), ESP.getChipCores(), ESP.getCpuFreqMHz());
  Serial.printf("Flash: %u MB | RAM livre: %u kB\n", ESP.getFlashChipSize() / 1048576, ESP.getFreeHeap() / 1024);
  Serial.printf("MAC: %012llX\n", ESP.getEfuseMac());
  Serial.println("Comandos: w = procurar Wi-Fi | qualquer texto = eco | botão BOOT = teste de entrada");
  Serial.println("----------------------------------------------");
}

void loop() {
  if (millis() - ultimo >= 1000) {
    ultimo = millis();
    contador++;
    estadoLed = !estadoLed;
    digitalWrite(LED, estadoLed);
    Serial.printf("[%lu] ESP32 funcionando | ligado há %lu s | RAM livre %u kB\n",
                  (unsigned long)contador, (unsigned long)(millis() / 1000), ESP.getFreeHeap() / 1024);
  }

  bool botao = digitalRead(BOTAO_BOOT);
  if (botao == LOW && botaoAntes == HIGH) Serial.println(">> Botão BOOT apertado: entrada digital OK");
  botaoAntes = botao;

  if (Serial.available()) {
    String txt = Serial.readStringUntil('\n');
    txt.trim();
    if (txt == "w") scanWiFi();
    else if (txt.length()) Serial.printf(">> Você digitou: \"%s\" (comunicação serial OK)\n", txt.c_str());
  }
}
