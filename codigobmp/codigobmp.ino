#include <SPI.h>
#include <Adafruit_BMP280.h>
#include <SD.h>

#define CS 5

Adafruit_BMP280 bmp = Adafruit_BMP280();

float altIni;

TaskHandle_t th_captacaoDados = NULL, //Prefixo th_ para handles de tasks
             th_transmissaoDados = NULL;
QueueHandle_t qh_dadosAltitude = NULL; //Prefixo qh_ para handles de queues
//Todo: queue do GPS depois de adicionar o módulo

void t_captacaoDados(void);
void t_transmissaoDados(void);

void setup() {
  Serial.begin(115200);

  bmp.begin(0x76);

  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL, // modo de operação
                Adafruit_BMP280::SAMPLING_X2, // temperatura
                Adafruit_BMP280::SAMPLING_X16, // pressão
                Adafruit_BMP280::FILTER_X16, //filtro de correção de dados
                Adafruit_BMP280::STANDBY_MS_1);

  altIni = bmp.readAltitude(1013.25);

  SD.begin(CS);
  SD.open("/dados.txt", FILE_WRITE);

  qh_dadosAltitude = xQueueCreate(1024, sizeof(double));

  xTaskCreatePinnedToCore(
    t_captacaoDados,
    "Captacao de dados",
    5000,
    NULL,
    1,
    &th_captacaoDados,
    0
  );

  xTaskCreatePinnedToCore(
    t_transmissaoDados,
    "Transmissao de dados",
    6000,
    NULL,
    1,
    &th_transmissaoDados,
    1
  );
}

void loop() {
  
}

void t_captacaoDados(void *pvParameters){ //Prefixo t_ para tasks
  float alt = 0;
  
  while(1){
    alt = bmp.readAltitude(1013.25) - altIni;
    
    xQueueSend(qh_dadosAltitude, &alt, portMAX_DELAY);
  }
}

void t_transmissaoDados(void *pvParameters){
  float alt = 0;

  while(1){
    xQueueReceive(qh_dadosAltitude, &alt, portMAX_DELAY);
    Serial.println(alt);
    
    File arquivo = SD.open("/dados.txt", FILE_APPEND);

    if(arquivo){
      arquivo.print("sd corrompido: ");
      arquivo.println(alt);
      arquivo.close();
    }else{
      Serial.println("erro no arquivo");
    }
  }
}