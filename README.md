# rocket-arduino — sketches da UTFPR Rocket

Sketches Arduino (ESP32) da telemetria, da antena tracker e dos testes de bancada. Todos em C++ com o framework Arduino.

| Sketch | Placa | O que faz | Projeto |
|---|---|---|---|
| [`foguete_tx/`](foguete_tx) | ESP32-C3 SuperMini | Transmissor embarcado: GPS u-blox, BMP280 e bateria; LoRa 917 MHz, pacote binário de 26 B a 5 Hz | Telemetria e antena tracker |
| [`tracker_rx/`](tracker_rx) | ESP32 DevKit V1 | Receptor da base e antena tracker pan/tilt (2 servos, OLED, calibração BASE/ALINHAR na NVS) | Telemetria e antena tracker |
| [`teste_celula_carga/`](teste_celula_carga) | ESP32 Dev Module | Teste da célula de carga (HX711): tara, calibração, ruído, leitura em CSV | Placa do teste estático |
| [`Codigo/testeestatico.ino`](Codigo) | ESP32 | Aquisição antiga do teste estático (HX711 + BMP280 + SD, FreeRTOS). **Tem bugs conhecidos** (abaixo) | Teste estático |
| [`codigobmp/`](codigobmp) | ESP32 | Datalogger do BMP280 no cartão SD (FreeRTOS) | Telemetria |
| [`teste_esp32_basico/`](teste_esp32_basico) | ESP32 | Teste básico da placa (varredura de Wi-Fi etc.) | Bancada |

## Bibliotecas necessárias
Instale pelo Gerenciador de Bibliotecas da Arduino IDE (não estão neste repositório):
- **Adafruit BMP280 Library** (com Adafruit BusIO e Adafruit Unified Sensor)
- **HX711** (Rob Tillaart)
- **RadioLib** e **SparkFun u-blox GNSS v3** (foguete_tx), **ESP32Servo**, **Adafruit SSD1306** e **Adafruit GFX** (tracker_rx)

Placa: pacote **esp32** da Espressif no Gerenciador de Placas.

## Telemetria (foguete_tx → tracker_rx)
LoRa 917,0 MHz, BW 250 kHz, SF7, CR 4/5, CRC, 5 Hz. Pacote `packed` de 26 bytes, little-endian: `magic 0xA5`, `flags`, `seq`, `lat`/`lon` (graus × 1e7), `altMsl` (cm), `baroRel` (dm, relativo ao boot), `vN`/`vE`/`vD` (cm/s), `sats`, `vbat` (× 20 mV).

## Bugs conhecidos
- `Codigo/testeestatico.ino`: `while (bmp.begin(0x76))` está invertido (o certo é `while (!bmp.begin(0x76))`), e o HX711 usa os mesmos GPIO 21/22 do I2C do BMP280. Para o teste estático, use o `teste_celula_carga` como base.
