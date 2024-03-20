#include <Arduino.h>
const int samplingFrequency = 32000;
const int bufferSize = 1024;//256//512//
volatile uint16_t adcBuffer[bufferSize];
uint32_t filteredBuffer[bufferSize];
//#define FILTER_TAP_NUM 13
//
/*
sampling frequency: 32000 Hz

fixed point precision: 16 bits

* 0 Hz - 400 Hz
  gain = 1
  desired ripple = 5 dB
  actual ripple = n/a
* 600 Hz - 16000 Hz
  gain = 0
  desired attenuation = -40 dB
  actual attenuation = n/a
*/

#define FILTER_TAP_NUM 187
static int16_t filter_taps[FILTER_TAP_NUM] = {
  170,
  16,
  17,
  17,
  16,
  15,
  14,
  12,
  10,
  7,
  4,
  0,
  -4,
  -9,
  -15,
  -21,
  -28,
  -35,
  -43,
  -51,
  -60,
  -69,
  -79,
  -88,
  -99,
  -109,
  -120,
  -131,
  -142,
  -152,
  -163,
  -173,
  -183,
  -193,
  -202,
  -211,
  -219,
  -226,
  -232,
  -237,
  -241,
  -243,
  -245,
  -245,
  -243,
  -240,
  -236,
  -229,
  -221,
  -211,
  -200,
  -186,
  -171,
  -154,
  -134,
  -113,
  -91,
  -66,
  -40,
  -12,
  18,
  49,
  81,
  115,
  150,
  186,
  223,
  261,
  299,
  338,
  377,
  416,
  455,
  494,
  532,
  570,
  607,
  643,
  677,
  711,
  743,
  773,
  802,
  828,
  853,
  875,
  895,
  912,
  927,
  940,
  949,
  956,
  960,
  962,
  960,
  956,
  949,
  940,
  927,
  912,
  895,
  875,
  853,
  828,
  802,
  773,
  743,
  711,
  677,
  643,
  607,
  570,
  532,
  494,
  455,
  416,
  377,
  338,
  299,
  261,
  223,
  186,
  150,
  115,
  81,
  49,
  18,
  -12,
  -40,
  -66,
  -91,
  -113,
  -134,
  -154,
  -171,
  -186,
  -200,
  -211,
  -221,
  -229,
  -236,
  -240,
  -243,
  -245,
  -245,
  -243,
  -241,
  -237,
  -232,
  -226,
  -219,
  -211,
  -202,
  -193,
  -183,
  -173,
  -163,
  -152,
  -142,
  -131,
  -120,
  -109,
  -99,
  -88,
  -79,
  -69,
  -60,
  -51,
  -43,
  -35,
  -28,
  -21,
  -15,
  -9,
  -4,
  0,
  4,
  7,
  10,
  12,
  14,
  15,
  16,
  17,
  17,
  16,
  170
};



//




//static int32_t filter_taps[FILTER_TAP_NUM] = {1, 2, 3, 4, 5, 6, 7, 6, 5, 4, 3, 2, 1};
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
  DACC->DACC_MR = DACC_MR_TRGEN_DIS // Désactive le déclencheur externe
                  | DACC_MR_USER_SEL_CHANNEL1 // select canal 1
                  | DACC_MR_WORD_HALF // Largeur de mot de 16 bits (0 - 4095)
                  | DACC_MR_REFRESH(1) // Temps de rafraîchissement (dans les cycles de l'horloge du périphérique)
                  | DACC_MR_STARTUP_8 // Temps de démarrage (8 * 6 cycles)
                  | DACC_MR_MAXS; // Utilise le contrôleur DMA pour les transferts DAC
// Active le canal 1 du DAC
  DACC->DACC_CHER = DACC_CHER_CH1;
  DACC->DACC_IER |= DACC_IER_EOC;
// Active l'interruption DACC_IRQn dans le NVIC  
  NVIC_EnableIRQ(DACC_IRQn);     
}


void TC0_Handler() {
  // Lit le registre d'état pour effacer le drapeau d'interruption
  TC0->TC_CHANNEL[0].TC_SR; 
  // Démarre une nouvelle conversion ADC
  ADC->ADC_CR = ADC_CR_START;
}

void DACC_Handler() {

  DACC->DACC_ISR;  // Read and clear status register
  //DACC->DACC_CDR = 1000;  // or whatever you want between 0 and 4095

}


void filtersignalRIF(uint16_t filteredBuffer[], uint16_t adcBuffer[], uint16_t F_TAP_NUM) {
  static uint16_t i = F_TAP_NUM - 1;
  uint32_t temp;
  for (int n = 0; n < F_TAP_NUM; n++) {
    temp=filter_taps[i] * adcBuffer[i - n];
    temp=temp>>15;
    
    //filteredBuffer[i] += (uint32_t)filter_taps[i] * adcBuffer[i - n];
    filteredBuffer[i] += (uint16_t) temp;
    
    
    Serial.println(filteredBuffer[i]);
  }
  i++;
  if (i >= 2 * F_TAP_NUM) {
    for (int n = F_TAP_NUM - 2; n >= 0; n--) {
      adcBuffer[n + F_TAP_NUM + 1] = adcBuffer[n];
    }
    i = F_TAP_NUM - 1;
  }
}
void setup() {
  Serial.begin(115200);
  setupADC();
  setupDAC();
  TC0->TC_CHANNEL[0].TC_CCR = TC_CCR_CLKEN | TC_CCR_SWTRG;
}
void loop() {
  if (ADC->ADC_ISR & ADC_ISR_ENDRX) {
    ADC->ADC_PTCR = ADC_PTCR_RXTDIS | ADC_PTCR_TXTDIS;
    //filtersignalRIF((uint32_t *)filteredBuffer, (uint32_t *)adcBuffer, FILTER_TAP_NUM);
    for (int i = 0; i < bufferSize; i++) {
      Serial.println(adcBuffer[i]);
       adcBuffer[i] &= 0xFFF;
  
  // Écrit la valeur sur le canal 0 du DAC
    DACC->DACC_CDR = DACC_CDR_DATA(adcBuffer[i]) ; //| DACC_CDR_CHANSEL_CH0;
    Serial.println(DACC->DACC_CDR);
      //DACC->DACC_CDR = DACC_CDR_DATA(filteredBuffer[i]);
    while (!(DACC->DACC_ISR & DACC_ISR_TXRDY));
    }


    // Réactiver le transfert PDC et réinitialiser les pointeurs et compteurs
    ADC->ADC_PTCR = ADC_PTCR_RXTEN;
    ADC->ADC_RPR = (uint32_t)adcBuffer;
    ADC->ADC_RCR = bufferSize;
    ADC->ADC_RNPR = (uint32_t)adcBuffer;
    ADC->ADC_RNCR = bufferSize;
  }
}