#include <Arduino.h>
#include <esp_adc_cal.h>
#include <math.h>

#define INP_PIN 36
#define ADC_VREF 1100
#define TIMER0_PRESCALE 20
#define TIMER0_ALARM 520

bool flagHabilitaLeitura = false;
int cont = 0;
float leituras[256];
hw_timer_t *Timer0_Cfg = NULL;
esp_adc_cal_characteristics_t adc;
esp_adc_cal_value_t tipo;

static void IRAM_ATTR Timer0_ISR();

void setup(){
  Serial.begin(115200);

  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_11);
  tipo = esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, ADC_VREF, &adc);

  Timer0_Cfg = timerBegin(0, TIMER0_PRESCALE, true);
  timerAttachInterrupt(Timer0_Cfg, &Timer0_ISR, true);
  timerAlarmWrite(Timer0_Cfg, TIMER0_ALARM, true);
  timerAlarmEnable(Timer0_Cfg);

  Serial.println("Configuracao terminada");
}

void loop(){
  static int leituraRaw = 0;
  float offset = 0,
        kPropor = 0,
        leituraV = 0;

  if(flagHabilitaLeitura == true){
    leituraRaw = adc1_get_raw(ADC1_CHANNEL_0);

  }
}

static void IRAM_ATTR Timer0_ISR(){
  flagHabilitaLeitura = true;
  portYIELD_FROM_ISR();
}