#include <SPI.h>
#include "HX711.h"
#include <Adafruit_BMP280.h>
#include <SD.h>

// --- Pinos ---
#define CS_SD 5
const int HX711_DOUT_PIN = 21;
const int HX711_SCK_PIN = 22;

// --- Objetos ---
Adafruit_BMP280 bmp = Adafruit_BMP280();
HX711 balanca;

// --- Variáveis Globais ---
float altIni;
const float FATOR_CALIBRACAO = -0.07287;
const float GRAVIDADE = 9.789;

// --- Handles FreeRTOS ---
TaskHandle_t th_captacaoSensores = NULL;
TaskHandle_t th_armazenamento = NULL;

// Estrutura para organizar os dados na fila
struct DadosSensores {
    float altitude;
    float pressaoBar;
    float peso;
};

QueueHandle_t qh_dados = NULL;

void t_captacaoSensores(void *pvParameters);
void t_armazenamento(void *pvParameters);

void setup() {
    Serial.begin(115200);

    // 1. Inicializa BMP280
    while (bmp.begin(0x76)) {
        Serial.println("Erro no BMP280!");
    }
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL, 
                    Adafruit_BMP280::SAMPLING_X2, 
                    Adafruit_BMP280::SAMPLING_X16, 
                    Adafruit_BMP280::FILTER_X16, 
                    Adafruit_BMP280::STANDBY_MS_1);
    altIni = bmp.readAltitude(1013.25);

    // 2. Inicializa HX711
    balanca.begin(HX711_DOUT_PIN, HX711_SCK_PIN);
    while(!balanca.is_ready());
    
    balanca.set_scale();
    balanca.tare(50); // Tara com 50 amostras

    // 3. Inicializa Cartão SD e Pastas
    if (!SD.begin(CS_SD)) {
        Serial.println("Erro no Cartão SD!");
    } else {
        // Cria as pastas se não existirem
        if (!SD.exists("/flight")) SD.mkdir("/flight");
        if (!SD.exists("/static")) SD.mkdir("/static");
    }

    // 4. Criação da Fila (Queue)
    qh_dados = xQueueCreate(50, sizeof(DadosSensores));

    // 5. Criação das Tasks
    // Task de leitura no Core 0
    xTaskCreatePinnedToCore(t_captacaoSensores, "Leitura", 5000, NULL, 1, &th_captacaoSensores, 0);
    
    // Task de gravação no Core 1 (evita travar as leituras com a lentidão do SD)
    xTaskCreatePinnedToCore(t_armazenamento, "Gravacao", 8000, NULL, 1, &th_armazenamento, 1);
}

void loop() {
    // Vazio - O código roda nas Tasks
}

// --- Task de Leitura dos Sensores ---
void t_captacaoSensores(void *pvParameters) {
    DadosSensores d;
    while (1) {
        // Leitura BMP
        float pressaoPa = bmp.readPressure(); // Retorna em Pascal
        d.pressaoBar = pressaoPa / 100000.0;  // Converte Pa para Bar
        d.altitude = bmp.readAltitude(1013.25) - altIni;

        // Leitura HX711
        if (balanca.is_ready()) {
            d.peso = balanca.get_units(5) * FATOR_CALIBRACAO * GRAVIDADE;
        } else {
            d.peso = 0;
        }

        // Envia para a fila
        xQueueSend(qh_dados, &d, portMAX_DELAY);
        
        vTaskDelay(pdMS_TO_TICKS(100)); // Lê a cada 100ms (10Hz)
    }
}

// --- Task de Armazenamento no SD ---
void t_armazenamento(void *pvParameters) {
    DadosSensores recebidos;
    while (1) {
        if (xQueueReceive(qh_dados, &recebidos, portMAX_DELAY)) {
            
            // 1. Grava dados do BMP (Altitude e Pressão) na pasta /flight
            File fileFlight = SD.open("/flight/bmp.txt", FILE_APPEND);
            if (fileFlight) {
                fileFlight.print("Alt: "); fileFlight.print(recebidos.altitude);
                fileFlight.print(" m | Pres: "); fileFlight.print(recebidos.pressaoBar, 4);
                fileFlight.println(" bar");
                fileFlight.close();
            }

            // 2. Grava dados do HX711 na pasta /static
            File fileStatic = SD.open("/static/carga.txt", FILE_APPEND);
            if (fileStatic) {
                fileStatic.print("Peso: "); fileStatic.print(recebidos.peso);
                fileStatic.println(" g");
                fileStatic.close();
            }

            // Debug no Serial
            Serial.print("Altitude: "); Serial.print(recebidos.altitude);
            Serial.print("m | Peso: "); Serial.println(recebidos.peso);
        }
    }
}