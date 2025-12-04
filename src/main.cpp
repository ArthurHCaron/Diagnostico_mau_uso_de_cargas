#include <Arduino.h>
#include <esp_adc_cal.h>
#include <math.h>

#define INP_PIN 36

volatile bool flagLeitura = false,
              flagPrint = true;
int cont = 0;
float leitura[256],
      impressao[256];
hw_timer_t *Timer0_Cfg = NULL,
           *Timer1_Cfg = NULL;
esp_adc_cal_characteristics_t adc;
esp_adc_cal_value_t tipo;
TaskHandle_t captacaoDados = NULL,
             exibicaoDados = NULL;
BaseType_t taskCaptacaoDados = pdFALSE,
           bloqueioStructDados = pdFALSE;
SemaphoreHandle_t bufferImpressao,
                  dadosRelevantes;

volatile struct{
  float tensao,
        tensao2,
        pico,
        media,
        eficaz;
  int contador;
  void (*zerar)(void);  
} dados;

void initDados(void);
void zerar(void);

void IRAM_ATTR Timer0_ISR(){
  taskCaptacaoDados = pdFALSE;

  vTaskNotifyGiveFromISR(captacaoDados, &taskCaptacaoDados);

  if(taskCaptacaoDados == pdTRUE) portYIELD_FROM_ISR();
}

void IRAM_ATTR Timer1_ISR(){
  xSemaphoreTakeFromISR(dadosRelevantes, &bloqueioStructDados);

  dados.eficaz = sqrtf(dados.tensao2 / dados.contador);
  dados.media = dados.tensao / dados.contador;
  dados.zerar();

  xSemaphoreGiveFromISR(dadosRelevantes, &bloqueioStructDados);
  if(bloqueioStructDados == pdTRUE) portYIELD_FROM_ISR();
}

void taskCore0(void* pvParameters){
  const double offset = 1.66,
               kPropor = 921.113;
  int leituraRaw = 0;
  double leituraVolt = 0;

  while(1){
    ulTaskNotifyTake(taskCaptacaoDados, portMAX_DELAY);

    leituraRaw = adc1_get_raw(ADC1_CHANNEL_0);
    leituraVolt = esp_adc_cal_raw_to_voltage(leituraRaw, &adc) / 1000.0;
    leituraVolt = kPropor * (leituraVolt - offset);
    leitura[cont] = leituraVolt;
    cont++;
    
    if(xSemaphoreTake(dadosRelevantes, 0) == pdPASS){
      dados.tensao2 += leituraVolt * leituraVolt;
      dados.tensao += leituraVolt;
      dados.contador++;

      xSemaphoreGive(dadosRelevantes);
    }

    if(cont == 255){
      cont = 0;
      xSemaphoreTake(bufferImpressao, 0);
      memcpy(impressao, leitura, 256 * sizeof(float));
      xSemaphoreGive(bufferImpressao);
      xTaskNotifyGive(exibicaoDados);
    }
  }
}

void taskCore1(void* pvParameters){
  while(1){
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    xSemaphoreTake(bufferImpressao, portMAX_DELAY);

    for(int i = 0; i < 256; i++) Serial.println(impressao[i]);

    xSemaphoreGive(bufferImpressao);
  }
}

void setup(){
  Serial.begin(921600);

  initDados();

  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_11);
  tipo = esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1100, &adc);

  bufferImpressao = xSemaphoreCreateBinary();
  xSemaphoreGive(bufferImpressao);

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
    1000,
    NULL,
    1,
    &captacaoDados,
    0
  );

  Timer0_Cfg = timerBegin(0, 20, true);
  timerAttachInterrupt(Timer0_Cfg, &Timer0_ISR, true);
  timerAlarmWrite(Timer0_Cfg, 520, true);
  timerAlarmEnable(Timer0_Cfg);

  Timer1_Cfg = timerBegin(1, 60000, true);
  timerAttachInterrupt(Timer1_Cfg, &Timer1_ISR, true);
  timerAlarmWrite(Timer1_Cfg, 80000, true);
  timerAlarmEnable(Timer1_Cfg);

  Serial.println("Configuracao terminada");
}

void loop(){}

void initDados(void){
  dados.tensao2 = 0;
  dados.tensao = 0;
  dados.contador = 0;
  dados.media = 0;
  dados.eficaz = 0;
  dados.pico = 0;
  dados.zerar = &zerar;
}

void zerar(void){
  dados.tensao2 = 0;
  dados.tensao = 0;
  dados.contador = 0;
}