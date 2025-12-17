#include <Arduino.h>
#include <esp_adc_cal.h>
#include <math.h>

#define INP_PIN 36
#define ADC_VREF 1100
#define TIMER0_PRESCALE 20
#define TIMER0_ALARM 520

bool flagHabilitaLeitura = false,
     flagImpressao = true;
int cont = 0,
    leituras[256];
float leiturasV[256];
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
  float kPropor = 0,
        maior = 0,
        pico = 0,
        media = 0,
        eficaz = 0,
        offset = 0;

  if(flagHabilitaLeitura == true && cont < 256){
    leituras[cont] = adc1_get_raw(ADC1_CHANNEL_0);
    cont++;
    flagHabilitaLeitura = false;
  } else if(cont >= 256 && flagImpressao == true){
    flagHabilitaLeitura = false;  
    timerAlarmDisable(Timer0_Cfg);
    
    for(int i = 0; i < 256; i++){
      leiturasV[i] = esp_adc_cal_raw_to_voltage(leituras[i], &adc) / 1000.0;
      offset += leiturasV[i] / 256;

      if(leituras[i] > maior) maior = leituras[i];
    }

    for(int i = 0; i < 256; i++){
      leiturasV[i] = esp_adc_cal_raw_to_voltage(leituras[i], &adc) / 1000.0 - offset;
      eficaz += pow(leiturasV[i], 2);

      if(leituras[i] > maior) maior = leituras[i];
    }

    eficaz = sqrt(eficaz / 256);
    maior = esp_adc_cal_raw_to_voltage(maior, &adc) / 1000.0;
    kPropor = 129.0 / eficaz;
    media = 0;
    eficaz = 0;

    for(int i = 0; i < 256; i++){
      leiturasV[i] = kPropor * (esp_adc_cal_raw_to_voltage(leituras[i], &adc) / 1000.0 - offset);
      Serial.println(leiturasV[i]);
      
      media += leiturasV[i];
      eficaz += pow(leiturasV[i], 2);
    }

    media /= 256;
    eficaz = sqrt(eficaz / 256);
    pico = kPropor * (maior - offset);

    Serial.printf("K = %.2f\n", kPropor);
    Serial.printf("Offset = %.2f\n", offset);
    Serial.printf("Media = %.2f\n", media);
    Serial.printf("Eficaz = %.2f\n", eficaz);
    Serial.printf("Pico = %.2f", pico);

    flagImpressao = false;
  }
}

static void IRAM_ATTR Timer0_ISR(){
  flagHabilitaLeitura = true;
}