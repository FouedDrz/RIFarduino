#include <Arduino.h>
const int samplingFrequency = 32000;
const int bufferSize = 256;
volatile uint32_t adcBuffer[bufferSize];
uint32_t filteredBuffer[bufferSize];
#define FILTER_TAP_NUM 13
static int32_t filter_taps[FILTER_TAP_NUM] = {1, 2, 3, 4, 5, 6, 7, 6, 5, 4, 3, 2, 1};
// n'oublier pas de modifier les coefficients de filtre selon votre conception
// filtre passe haut/ passe bas / passe bande / coupe bande
//
void setupADC() {
  PMC->PMC_PCER1 |= PMC_PCER1_PID37; // Active le périphérique ADC
  ADC->ADC_MR = ADC_MR_PRESCAL(255)  // Définit le diviseur de fréquence à 255
              | ADC_MR_STARTUP_SUT64 // Définit le temps de démarrage à 64 périodes d'ADC_CLK
              | ADC_MR_TRACKTIM(15)  // Définit le temps de suivi à 15 périodes d'ADC_CLK
              | ADC_MR_SETTLING_AST3;// Définit le temps de stabilisation à 17 périodes d'ADC_CLK
  ADC->ADC_CHER = ADC_CHER_CH7;      // Active le canal 7 (A0)

  // Configure Timer Counter 0 Channel 0 (TC0) pour samplingFrequency
  PMC->PMC_PCER0 |= PMC_PCER0_PID27; // Active le périphérique TC0
  TC0->TC_CHANNEL[0].TC_CMR = TC_CMR_TCCLKS_TIMER_CLOCK4 | TC_CMR_CPCTRG; 
  // Définit la source d'horloge à TCLK4 (MCK / 128, 84 MHz / 128 = 656.25 kHz)
  // Active le déclenchement de comparaison RC
  // Définit la valeur RC pour une fréquence samplingFrequency Hz
  TC0->TC_CHANNEL[0].TC_RC = 656250 / samplingFrequency - 1;
  // Active l'interruption de comparaison RC
  TC0->TC_CHANNEL[0].TC_IER = TC_IER_CPCS;
  // Active l'interruption TC0_IRQn dans le NVIC
  NVIC_EnableIRQ(TC0_IRQn);

  // Configure le contrôleur DMA
  PMC->PMC_PCER1 |= PMC_PCER1_PID39; // Active le périphérique PDC
  ADC->ADC_PTCR = ADC_PTCR_RXTDIS | ADC_PTCR_TXTDIS; // Désactive le transfert PDC
  ADC->ADC_RPR = (uint32_t)adcBuffer; // Définit le pointeur de réception sur le tampon
  ADC->ADC_RCR = bufferSize; // Définit le compteur de réception à la taille du tampon
  ADC->ADC_RNPR = (uint32_t)adcBuffer; // Définit le prochain pointeur de réception sur le tampon
  ADC->ADC_RNCR = bufferSize; // Définit le prochain compteur de réception à la taille du tampon
  ADC->ADC_PTCR = ADC_PTCR_RXTEN; // Active le transfert PDC
}
void setupDAC() {
  // Active le périphérique DAC
  PMC->PMC_PCER1 |= PMC_PCER1_PID38;
  // Configure le DAC en mode normal
  DACC->DACC_MR = DACC_MR_REFRESH(1) | DACC_MR_STARTUP_8 | DACC_MR_MAXS;
  // Active le canal 1 du DAC
  DACC->DACC_CHER = DACC_CHER_CH1;
}

void TC0_Handler() {
  // Lit le registre d'état pour effacer le drapeau d'interruption
  TC0->TC_CHANNEL[0].TC_SR; 
  // Démarre une nouvelle conversion ADC
  ADC->ADC_CR = ADC_CR_START;
}
//
void filtersignalRIF2(uint32_t filteredBuffer[], uint32_t adcBuffer[], uint32_t F_TAP_NUM) {
  static uint32_t i = F_TAP_NUM - 1;
     for (int l = 0; l < F_TAP_NUM; l++)
    { if ((i - l) < 0) 
    { filteredBuffer[i] += filter_taps[l] * adcBuffer[i - l + F_TAP_NUM];
     } 
     else {
        filteredBuffer[i] += filter_taps[l] * adcBuffer[i - l];
         } 
         }
   i = (i + 1) % F_TAP_NUM;
 
}
//
void setup() {
  Serial.begin(115200);
  setupADC();
  setupDAC();
  TC0->TC_CHANNEL[0].TC_CCR = TC_CCR_CLKEN | TC_CCR_SWTRG;
}
void loop() {
  if (ADC->ADC_ISR & ADC_ISR_ENDRX) {
    ADC->ADC_PTCR = ADC_PTCR_RXTDIS | ADC_PTCR_TXTDIS;
    filtersignalRIF2((uint32_t *)filteredBuffer, (uint32_t *)adcBuffer, FILTER_TAP_NUM);
    for (int i = 0; i < bufferSize; i++) {
      Serial.println(filteredBuffer[i]);
      DACC->DACC_CDR = DACC_CDR_DATA(filteredBuffer[i]);
      while (!(DACC->DACC_ISR & DACC_ISR_TXRDY));
    }
    ADC->ADC_PTCR = ADC_PTCR_RXTEN;
    ADC->ADC_RPR = (uint32_t)adcBuffer;
    ADC->ADC_RCR = bufferSize;
    ADC->ADC_RNPR = (uint32_t)adcBuffer;
    ADC->ADC_RNCR = bufferSize;
  }
}