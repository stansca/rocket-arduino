# rocket-arduino — sketches da UTFPR Rocket

Sketches Arduino (ESP32) dos testes de bancada e dataloggers. Todos em C++ com o framework Arduino.

> A telemetria LoRa e a antena tracker (`foguete_tx` e `tracker_rx`) foram para o repositório próprio **[antena-tracker](https://github.com/stansca/antena-tracker)**.

| Sketch | Placa | O que faz | Projeto |
|---|---|---|---|
| [`teste_celula_carga/`](teste_celula_carga) | ESP32 Dev Module | Teste da célula de carga (HX711): tara, calibração, ruído, leitura em CSV | Placa do teste estático |
| [`Codigo/testeestatico.ino`](Codigo) | ESP32 | Aquisição antiga do teste estático (HX711 + BMP280 + SD, FreeRTOS). **Tem bugs conhecidos** (abaixo) | Teste estático |
| [`codigobmp/`](codigobmp) | ESP32 | Datalogger do BMP280 no cartão SD (FreeRTOS) | Telemetria |
| [`teste_esp32_basico/`](teste_esp32_basico) | ESP32 | Teste básico da placa (varredura de Wi-Fi etc.) | Bancada |

## Bibliotecas necessárias
Instale pelo Gerenciador de Bibliotecas da Arduino IDE (não estão neste repositório):
- **Adafruit BMP280 Library** (com Adafruit BusIO e Adafruit Unified Sensor)
- **HX711** (Rob Tillaart)

Placa: pacote **esp32** da Espressif no Gerenciador de Placas.

## Bugs conhecidos
- `Codigo/testeestatico.ino`: `while (bmp.begin(0x76))` está invertido (o certo é `while (!bmp.begin(0x76))`), e o HX711 usa os mesmos GPIO 21/22 do I2C do BMP280. Para o teste estático, use o `teste_celula_carga` como base.
