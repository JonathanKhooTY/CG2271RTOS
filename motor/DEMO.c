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
#define PTE20_Pin 20
#define PTE21_Pin 21
#define CLOCK_FREQ 48000000
#define PRESCALER  128

#define FREQ 50
#define DUTY_CYCLE 0.5


// UART Constants
#define BAUD_RATE 9600
#define UART_TX_PORTE22 22
#define UART_RX_PORTE23 23
#define UART2_INT_PRIO 128

#define SW_POS 6
int on_off = 0;


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

void initSwitch(void){
	
		//enabling clock for PortD
		SIM->SCGC5 |= SIM_SCGC5_PORTD_MASK;
	
		//Select GPIO and enable pullup resistors and interrupts on
		//falling edges of pin connected to switch
	
		PORTD->PCR[SW_POS] |= (PORT_PCR_MUX(1) |
													 PORT_PCR_PS_MASK |
													 PORT_PCR_PE_MASK |
													 PORT_PCR_IRQC(0x0a));
	
		//set PORTD switch bit to input
		PTD->PDDR &= ~MASK(SW_POS);
	
	//enable interrupts
	NVIC_SetPriority(PORTD_IRQn,2);
	NVIC_ClearPendingIRQ(PORTD_IRQn);
	NVIC_EnableIRQ(PORTD_IRQn);
}

void PORTD_IRQHandler() {
		//Clear pending IRQ
	NVIC_ClearPendingIRQ(PORTD_IRQn);
	
	on_off ^= 1;
	
	//clear INT flag. Interrupt Status Flag Register
	PORTD->ISFR |= MASK(SW_POS);
}





/*----------------------------------------------------------------------------
 * PWM Functions
 *---------------------------------------------------------------------------*/
void initPWM(uint32_t mod) {
  SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;
	SIM_SCGC5 |= SIM_SCGC5_PORTE_MASK;
	
  PORTE->PCR[PTE20_Pin] &= ~PORT_PCR_MUX_MASK; 
  PORTE->PCR[PTE20_Pin] |= PORT_PCR_MUX(3);
	
	PORTE->PCR[PTE21_Pin] &= ~PORT_PCR_MUX_MASK; 
  PORTE->PCR[PTE21_Pin] |= PORT_PCR_MUX(3);
	
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


// Delay 
static void delay(volatile uint32_t nof) {
  while(nof!=0) {
    __asm("NOP");
    nof--;
  }
}


/*----------------------------------------------------------------------------
 * Main Function
 *---------------------------------------------------------------------------*/

int main() {  

  
  SystemCoreClockUpdate();
  initSwitch();  
  initGPIO();
  
  uint32_t mod = CLOCK_FREQ / PRESCALER /FREQ;

  initPWM(mod);
  TPM1_C0V = 0;
  TPM1_C1V = 0;

 
  
  while(1) {	
		motor_control(on_off, mod);
  }
}