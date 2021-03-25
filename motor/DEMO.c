// DEMO CODE
#include "MKL25Z4.h"                    // Device header
#include <stdio.h>

// LEd GPIO Constants
#define RED_LED     18          // PortB Pin 18 
#define GREEN_LED   19           // PortB Pin 19 
#define BLUE_LED     1           // PortD Pin 1 
#define MASK(x)     (1 << (x)) 

typedef enum {RED, GREEN, BLUE} colors_t;


// PWM Motor Constants
#define PTB0_Pin 0
#define PTB1_Pin 1

#define CLOCK_FREQ 48000000
#define PRESCALER  128

#define FREQ 50
#define DUTY_CYCLE 0.5


// UART Constants
#define BAUD_RATE 9600
#define UART_TX_PORTE22 22
#define UART_RX_PORTE23 23
#define UART2_INT_PRIO 128
 

/*----------------------------------------------------------------------------
 * LED Functions
 *---------------------------------------------------------------------------*/
void initGPIO() 
{ 
 // Enable Clock to PORTB and PORTD 
 SIM->SCGC5 |= ((SIM_SCGC5_PORTB_MASK) | (SIM_SCGC5_PORTD_MASK)); 
 
 // Configure MUX settings to make all 3 pins GPIO 
 
 PORTB->PCR[RED_LED] &= ~PORT_PCR_MUX_MASK; 
 PORTB->PCR[RED_LED] |= PORT_PCR_MUX(1); 
 
 PORTB->PCR[GREEN_LED] &= ~PORT_PCR_MUX_MASK; 
 PORTB->PCR[GREEN_LED] |= PORT_PCR_MUX(1); 
 
 PORTD->PCR[BLUE_LED] &= ~PORT_PCR_MUX_MASK; 
 PORTD->PCR[BLUE_LED] |= PORT_PCR_MUX(1); 
 
 // Set Data Direction Registers for PortB and PortD 
 PTB->PDDR |= (MASK(RED_LED) | MASK(GREEN_LED)); 
 PTD->PDDR |= MASK(BLUE_LED); 
} 


void offRGB(){
  PTB->PSOR = MASK(RED_LED) | MASK(GREEN_LED);
  PTD->PSOR = MASK(BLUE_LED);
}


void led_control(colors_t color, long on_off){
  if (on_off) {      // On
    switch(color){
    case RED:
      PTB->PCOR = MASK(RED_LED);
      break;
    case GREEN:
      PTB->PCOR = MASK(GREEN_LED);
      break;
    case BLUE:
      PTD->PCOR = MASK(BLUE_LED);
      break;
    }
  } else {           // Off
    switch(color){
    case RED:
      PTB->PSOR = MASK(RED_LED);
      break;
    case GREEN:
      PTB->PSOR = MASK(GREEN_LED);
      break;
    case BLUE:
      PTD->PSOR = MASK(BLUE_LED);
      break;
    }
  }
}

/*----------------------------------------------------------------------------
 * PWM Functions
 *---------------------------------------------------------------------------*/
void initPWM(uint32_t mod) {
  SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;
  
  PORTB->PCR[PTB0_Pin] &= ~PORT_PCR_MUX_MASK; 
  PORTB->PCR[PTB0_Pin] |= PORT_PCR_MUX(3);
  
  PORTB->PCR[PTB1_Pin] &= ~PORT_PCR_MUX_MASK;
  PORTB->PCR[PTB1_Pin] |= PORT_PCR_MUX(3);
  
  SIM_SCGC6 |= SIM_SCGC6_TPM1_MASK;
  
  SIM_SOPT2 &= ~SIM_SOPT2_TPMSRC_MASK;
  SIM_SOPT2 |= SIM_SOPT2_TPMSRC(1);
  
  TPM1->MOD = mod;
  
  TPM1->SC &= ~(TPM_SC_CMOD_MASK | TPM_SC_PS_MASK);   //Clearing
  TPM1->SC |= TPM_SC_CMOD(1) | TPM_SC_PS(7); 					//Increments in counter clock, 128 ps
  TPM1->SC &= ~TPM_SC_CPWMS_MASK;											
  
  TPM1_C0SC &= ~(TPM_CnSC_ELSB_MASK | TPM_CnSC_ELSA_MASK | TPM_CnSC_MSB_MASK | TPM_CnSC_MSA_MASK);
  TPM1_C0SC |= TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1);    //Toggle output on match
  
  TPM1_C1SC &= ~(TPM_CnSC_ELSB_MASK | TPM_CnSC_ELSA_MASK | TPM_CnSC_MSB_MASK | TPM_CnSC_MSA_MASK);
  TPM1_C1SC |= TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1);    //Toggle output on match
}  


void motor_control(long on_off, uint32_t mod) {
  
  if (on_off) {      // On
    TPM1_C0V = (uint32_t)(mod * DUTY_CYCLE);
    TPM1_C1V = (uint32_t)(mod * DUTY_CYCLE);
    
  } else {          // Off 
    TPM1_C0V = 0;
    TPM1_C1V = 0;
  }
}

/*----------------------------------------------------------------------------
 * UART Functions
 *---------------------------------------------------------------------------*/
/*
void initUART2(uint32_t baud_rate) {
  uint32_t divisor, bus_clock;
  
  SIM->SCGC4 |= SIM_SCGC4_UART2_MASK;
  SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;
    
  // Enable Port E Pins 22 and 23 and use their alt 4 function
  PORTE->PCR[UART_TX_PORTE22] &= ~PORT_PCR_MUX_MASK;
  PORTE->PCR[UART_TX_PORTE22] |= PORT_PCR_MUX(4);
  
  PORTE->PCR[UART_RX_PORTE23] &= ~PORT_PCR_MUX_MASK;
  PORTE->PCR[UART_RX_PORTE23] |= PORT_PCR_MUX(4);

  // Disable Tx and Rx before configuration
  UART2->C2 &= ~(UART_C2_TE_MASK | UART_C2_RE_MASK);

Kai Jie, [11.03.21 20:31]
// Set baud rate to desired value
  bus_clock = DEFAULT_SYSTEM_CLOCK/2;
  divisor = bus_clock/(baud_rate * 16);
  UART2->BDH = UART_BDH_SBR(divisor >> 8);
  UART2->BDL = UART_BDL_SBR(divisor);
  
    // No Parity, 8 bits
  UART2->C1 = 0;
  UART2->S2 = 0;
  UART2->C3 = 0;
  
  // Enable Tx and Rx
  UART2->C2 |= UART_C2_TE_MASK | UART_C2_RE_MASK;
}


void UART2_Transmit_Poll(uint8_t data) {
  while(!(UART2->S1 & UART_S1_TDRE_MASK));
  UART2->D = data;
}

uint8_t UART2_Receive_Poll() {
  while(!(UART2->S1 & UART_S1_RDRF_MASK));
  return UART2->D;
}

*/
// Delay 
static void delay(volatile uint32_t nof) {
  while(nof!=0) {
    __asm("NOP");
    nof--;
  }
}

void processData(uint8_t data, uint32_t mod) {
  colors_t colors[] = {RED, GREEN, BLUE};
  
  /*
  data[0] - 0 if LED, 1 if motor

  LED:
  data[1] - 0 if Off, 1 if On
  data[2:4] - 0 if Red, 1 if Green, 2 if Blue
  
  Motor:
  data[1] - 0 if Off, 1 if On
  */
  
  int led_motor = data % 2;   //0 if led, 1 if motor
  
  int on_off = (data >> 1) % 2;
  
  if (!led_motor) {            // Led
    uint8_t color_code = data >> 2;
    
    led_control(colors[color_code], on_off);
    
  } else {                    // Motor
    motor_control(on_off, mod);
  
  }
}

/*----------------------------------------------------------------------------
 * Main Function
 *---------------------------------------------------------------------------*/

int main() {  

  
  SystemCoreClockUpdate();
  
  //initUART2(BAUD_RATE);  
  
  initGPIO();
  //offRGB();
  
  uint32_t mod = CLOCK_FREQ / PRESCALER /FREQ;

  initPWM(mod);
  TPM1_C0V = 0;
  TPM1_C1V = 0;

  //volatile uint8_t data = 0x01;
  
  while(1) {
    // Receive commands for LED
   // data = UART2_Receive_Poll();
    //processData(data, mod);
		motor_control(0, mod);
  }
}