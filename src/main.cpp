#include <Arduino.h>
#include <esp_adc_cal.h>
#include <math.h>

#define INP_PIN 36
#define ADC_VREF 1100
#define TIMER0_PRESCALE 20
#define TIMER0_ALARM 520
#define TIMER1_PRESCALE 60000
#define TIMER1_ALARM 80000

hw_timer_t *Timer0_Cfg = NULL,
           *Timer1_Cfg = NULL;
esp_adc_cal_characteristics_t adc;
esp_adc_cal_value_t tipo;
TaskHandle_t captacaoDados = NULL,
             exibicaoDados = NULL;
BaseType_t taskCaptacaoDados = pdFALSE,
           taskExibicaoDadosRelevantes = pdFALSE;
SemaphoreHandle_t dadosRelevantes;

static struct{
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
static void IRAM_ATTR Timer0_ISR();
static void IRAM_ATTR Timer1_ISR();
static void taskCore0(void* pvParameters);
static void taskCore1(void* pvParameters);

void setup(){
  Serial.begin(921600);

  initDados();

  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_11);
  tipo = esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, ADC_VREF, &adc);

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

  Timer0_Cfg = timerBegin(0, TIMER0_PRESCALE, true);
  timerAttachInterrupt(Timer0_Cfg, &Timer0_ISR, true);
  timerAlarmWrite(Timer0_Cfg, TIMER0_ALARM, true);
  timerAlarmEnable(Timer0_Cfg);

  Timer1_Cfg = timerBegin(1, TIMER1_PRESCALE, true);
  timerAttachInterrupt(Timer1_Cfg, &Timer1_ISR, true);
  timerAlarmWrite(Timer1_Cfg, TIMER1_ALARM, true);
  timerAlarmEnable(Timer1_Cfg);

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
  const float offset = 1.66,
              kPropor = 921.113;
  int leituraRaw = 0;
  float sumV = 0,
        sumV2 = 0,
        cont = 0,
        leituraVolt = 0;

  while(1){
    ulTaskNotifyTake(taskCaptacaoDados, portMAX_DELAY);

    leituraRaw = adc1_get_raw(ADC1_CHANNEL_0);
    leituraVolt = esp_adc_cal_raw_to_voltage(leituraRaw, &adc) / 1000.0;
    leituraVolt = kPropor * (leituraVolt - offset);
    sumV += leituraVolt;
    sumV2 += leituraVolt * leituraVolt;
    cont++;

    if(xSemaphoreTake(dadosRelevantes, 0) == pdPASS){
      dados.tensao2 += sumV2;
      dados.tensao += sumV;
      dados.contador += cont;
      sumV = 0;
      sumV2 = 0;
      cont = 0;

      xSemaphoreGive(dadosRelevantes);
    }
  }
}

static void taskCore1(void* pvParameters){
  while(1){
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    xSemaphoreTake(dadosRelevantes, portMAX_DELAY);

    dados.eficaz = sqrtf(dados.tensao2 / dados.contador);
    dados.media = dados.tensao / dados.contador;
    dados.zerar();

    Serial.printf("E = %.2f\n", dados.eficaz);
    Serial.printf("M = %.2f\n", dados.media);
    Serial.printf("P = %.2f\n", dados.pico);
    
    xSemaphoreGive(dadosRelevantes);
  }
}

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