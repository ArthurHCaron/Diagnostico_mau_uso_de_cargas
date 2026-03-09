#include <Arduino.h>
#include <esp_adc_cal.h>
#include <math.h>
#include <WiFi.h>
#include "HTTPClient.h"

#define INP_PIN 4
#define ADC_VREF 1100
#define TIMER0_PRESCALE 20
#define TIMER0_ALARM 520
#define TIMER1_PRESCALE 60000
#define TIMER1_ALARM 80000

const char* ssid = WIFI_SSID,
          * senha = WIFI_PASSWORD;
String chave_api = API_KEY,
       server = "http://api.thingspeak.com/update";
hw_timer_t *Timer0_Cfg = NULL,
           *Timer1_Cfg = NULL;
esp_adc_cal_characteristics_t adc;
esp_adc_cal_value_t tipo;
TaskHandle_t captacaoDados = NULL,
             exibicaoDados = NULL;
BaseType_t taskCaptacaoDados = pdFALSE,
           taskExibicaoDadosRelevantes = pdFALSE;
SemaphoreHandle_t dadosRelevantes;

volatile struct{
  float tensao2,
        corrente2,
        prodVI;
  int contador;
} dados;

void initDados(void);
static void IRAM_ATTR Timer0_ISR();
static void IRAM_ATTR Timer1_ISR();
static void taskCore0(void* pvParameters);
static void taskCore1(void* pvParameters);

void setup(){
  Serial.begin(921600);

  initDados();

  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(ADC1_CHANNEL_3, ADC_ATTEN_11db);
  adc1_config_channel_atten(ADC1_CHANNEL_9, ADC_ATTEN_11db);
  tipo = esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_11db, ADC_WIDTH_BIT_12, ADC_VREF, &adc);

  dadosRelevantes = xSemaphoreCreateBinary();
  xSemaphoreGive(dadosRelevantes);

  xTaskCreatePinnedToCore(
    taskCore1,
    "Exibicao de dados",
    5000,
    NULL,
    1,
    &exibicaoDados,
    1
  );
  xTaskCreatePinnedToCore(
    taskCore0,
    "Leitura de dados",
    5000,
    NULL,
    1,
    &captacaoDados,
    0
  );

  Timer0_Cfg = timerBegin(0, TIMER0_PRESCALE, true);
  timerAttachInterrupt(Timer0_Cfg, &Timer0_ISR, true);
  timerAlarmWrite(Timer0_Cfg, TIMER0_ALARM, true);
  timerAlarmEnable(Timer0_Cfg);

  Timer1_Cfg = timerBegin(1, TIMER1_PRESCALE, true);
  timerAttachInterrupt(Timer1_Cfg, &Timer1_ISR, true);
  timerAlarmWrite(Timer1_Cfg, TIMER1_ALARM, true);
  timerAlarmEnable(Timer1_Cfg);

  WiFi.begin(ssid, senha);

  while(WiFi.status() != WL_CONNECTED){
    Serial.println("Conectando wifi...");
    delay(1000);
  }

  Serial.println("Configuracao terminada");
}

void loop(){}

static void IRAM_ATTR Timer0_ISR(){
  taskCaptacaoDados = pdFALSE;

  vTaskNotifyGiveFromISR(captacaoDados, &taskCaptacaoDados);

  if(taskCaptacaoDados == pdTRUE) portYIELD_FROM_ISR();
}

static void IRAM_ATTR Timer1_ISR(){
  taskExibicaoDadosRelevantes = pdFALSE;
  vTaskNotifyGiveFromISR(exibicaoDados, &taskExibicaoDadosRelevantes);

  if(taskExibicaoDadosRelevantes == pdTRUE) portYIELD_FROM_ISR();
}

static void taskCore0(void* pvParameters){
  const float offset = 1.64,
              kPropor = 1001.922667;
  int leituraRaw = 0,
      correnteRaw = 0;
  static float sumV2 = 0,
               sumI2 = 0,
               cont = 0,
               leituraVolt = 0,
               leituraAmp = 0,
               prod = 0;

  while(1){
    ulTaskNotifyTake(taskCaptacaoDados, portMAX_DELAY);

    leituraRaw = adc1_get_raw(ADC1_CHANNEL_3);
    correnteRaw = adc1_get_raw(ADC1_CHANNEL_9);
    leituraVolt = esp_adc_cal_raw_to_voltage(leituraRaw, &adc) / 1000.0;
    leituraVolt = kPropor * (leituraVolt - offset);
    leituraAmp = 20.0 * (esp_adc_cal_raw_to_voltage(correnteRaw, &adc) / 1000.0 - 1.65);
    sumV2 += leituraVolt * leituraVolt;
    sumI2 += leituraAmp * leituraAmp;
    prod += leituraVolt * leituraAmp;
    cont++;

    if(xSemaphoreTake(dadosRelevantes, 0) == pdPASS){
      dados.tensao2 += sumV2;
      dados.corrente2 += sumI2;
      dados.contador += cont;
      dados.prodVI += prod;
      sumV2 = 0;
      sumI2 = 0;
      cont = 0;
      prod = 0;
      xSemaphoreGive(dadosRelevantes);
    }
  }
}

static void taskCore1(void* pvParameters){
  int codHttp = 0;
  float envio[5],
        vEficaz = 0,
        iEficaz = 0,
        potAp = 0,
        potAt = 0,
        potR = 0;
  String url = "";
  HTTPClient http;

  while(1){
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    xSemaphoreTake(dadosRelevantes, portMAX_DELAY);

    vEficaz = sqrtf(dados.tensao2 / dados.contador);
    iEficaz = sqrtf(dados.corrente2 / dados.contador);
    potAp = iEficaz * vEficaz;
    potR = dados.prodVI / dados.contador;
    potAt = sqrtf(potAp * potAp - potR * potR);
    envio[0] = vEficaz;
    envio[1] = iEficaz;
    envio[2] = potAt;
    envio[3] = potR;
    envio[4] = potAp;
    initDados();
    
    xSemaphoreGive(dadosRelevantes);

    if(!isnan(vEficaz)){
      if(WiFi.status() == WL_CONNECTED){
        url = server + "?api_key=" + chave_api + 
              "&field1=" + String(envio[0], 2) + 
              "&field2=" + String(envio[1], 2) +
              "&field3=" + String(envio[2], 2) + 
              "&field4=" + String(envio[3], 2) +
              "&field5=" + String(envio[4], 2);
        http.begin(url);

        codHttp = http.GET();
        http.end();
      }
      //V_rms;I_rms;Potencia_ativa;Potencia_reativa;Potencia_aparente;Envio
      Serial.printf("%.2f;%.2f;%.2f;%.2f;%.2f;%d\n", envio[0], envio[1], envio[2], envio[3], envio[4], codHttp);
    }
  }
}

void initDados(void){
  dados.tensao2 = 0;
  dados.corrente2 = 0;
  dados.prodVI = 0;
  dados.contador = 0;
}