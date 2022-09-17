// ***** 0. Documentation Section *****
// TableTrafficLight.c for Lab 10
// Runs on LM4F120/TM4C123
// Index implementation of a Moore finite state machine to operate a traffic light.  
// Daniel Valvano, Jonathan Valvano
// January 15, 2016

// east/west red light connected to PB5
// east/west yellow light connected to PB4
// east/west green light connected to PB3
// north/south facing red light connected to PB2
// north/south facing yellow light connected to PB1
// north/south facing green light connected to PB0
// pedestrian detector connected to PE2 (1=pedestrian present)
// north/south car detector connected to PE1 (1=car present)
// east/west car detector connected to PE0 (1=car present)
// "walk" light connected to PF3 (built-in green LED)
// "don't walk" light connected to PF1 (built-in red LED)

// ***** 1. Pre-processor Directives Section *****
#include "TExaS.h"
#include "tm4c123gh6pm.h"

// ***** 2. Global Declarations Section *****
#define SENSOR  GPIO_PORTE_DATA_R
#define ROADLED GPIO_PORTB_DATA_R
#define PEDLED  GPIO_PORTF_DATA_R

struct State {
	unsigned long OutLED;
	unsigned long OutPed;
	unsigned long Time;
	unsigned long Next[8];
};
typedef const struct State STyp;

#define GoWest    0
#define WaitWest  1
#define GoSouth   2
#define WaitSouth 3
#define GoPed     4
#define FlashPed1 5
#define FlashPed2 6
#define FlashPed3 7
#define FlashPed4 8

STyp FSM[9] = {
	{0x0c, 0x02, 600, {GoWest, GoWest, WaitWest, WaitWest, WaitWest, WaitWest, WaitWest, WaitWest}},
	{0x14, 0x02, 600, {GoSouth, GoWest, GoSouth, GoSouth, GoPed, GoPed, GoSouth, GoSouth}},
	{0x21, 0x02, 600, {GoSouth, WaitSouth, GoSouth, WaitSouth, WaitSouth, WaitSouth, GoSouth, WaitSouth}},
	{0x22, 0x02, 600, {GoWest, GoWest, GoSouth, GoWest, GoPed, GoWest, GoPed, GoPed}},
	{0x24, 0x08, 600, {GoPed, FlashPed1, FlashPed1, FlashPed1, GoPed, FlashPed1, FlashPed1, FlashPed1}},
	{0x24, 0x02, 600, {FlashPed2, FlashPed2, FlashPed2, FlashPed2, FlashPed2, FlashPed2, FlashPed2, FlashPed2}},
	{0x24, 0x00, 600, {FlashPed3, FlashPed3, FlashPed3, FlashPed3, FlashPed3, FlashPed3, FlashPed3, FlashPed3}},
	{0x24, 0x02, 600, {FlashPed4, FlashPed4, FlashPed4, FlashPed4, FlashPed4, FlashPed4, FlashPed4, FlashPed4}},
	{0x24, 0x00, 600, {GoWest, GoWest, GoSouth, GoWest, GoPed, GoWest, GoSouth, GoWest}},
};

unsigned long S; // index to the curr state
unsigned long Input;

// FUNCTION PROTOTYPES: Each subroutine defined
void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts

void SysTick_Init(void);
void SysTick_Wait(unsigned long);
void SysTick_Wait1ms(unsigned long);

void Init_Ports(void);
void PortE_Init(void);
void PortB_Init(void);
void PortF_Init(void);

// ***** 3. Subroutines Section *****
void SysTick_Init(void){
	NVIC_ST_CTRL_R = 0;                   // disable SysTick during setup             
  NVIC_ST_CTRL_R = 0x00000005;          // enable SysTick with core clock
}

// The delay parameter is in units of the 80 MHz core clock. (12.5 ns)
void SysTick_Wait(unsigned long delay){
	NVIC_ST_RELOAD_R = delay - 1;
	NVIC_ST_CURRENT_R = 0;
	while((NVIC_ST_CTRL_R & 0x00010000) == 0){
	}
}

void SysTick_Wait1ms(unsigned long delay){
	unsigned long i;
	for(i=0; i<delay; i++){
		SysTick_Wait(80000); // wait 1ms
	}
}

void PortE_Init(void){
	GPIO_PORTE_AMSEL_R &= 0x00;        // 2) disable analog function
  GPIO_PORTE_PCTL_R &= 0x00000000;   // 3) GPIO clear bit PCTL  
  GPIO_PORTE_DIR_R &= ~0x07;         // 4.1) PE2-0 input, 
  GPIO_PORTE_AFSEL_R &= 0x00;        // 5) no alternate function       
  GPIO_PORTE_DEN_R |= 0x07;          // 7) enable digital pins PE2-PE0
}

void PortB_Init(void){
	GPIO_PORTB_AMSEL_R &= 0x00;        // 2) disable analog function
  GPIO_PORTB_PCTL_R &= 0x00000000;   // 3) GPIO clear bit PCTL  
  GPIO_PORTB_DIR_R |= 0x3F;          // 4.2) PB5-0 output  
  GPIO_PORTB_AFSEL_R &= 0x00;        // 5) no alternate function       
  GPIO_PORTB_DEN_R |= 0x3F;          // 7) enable digital pins PB5-0
}

void PortF_Init(void){ 
  GPIO_PORTF_LOCK_R = 0x4C4F434B;   // 2) unlock GPIO Port F
  GPIO_PORTF_CR_R = 0x0A;           // allow changes to PF3 and PF1
  // only PF0 needs to be unlocked, other bits can't be locked
  GPIO_PORTF_AMSEL_R = 0x00;        // 3) disable analog on PF
  GPIO_PORTF_PCTL_R = 0x00000000;   // 4) PCTL GPIO on PF4-0
  GPIO_PORTF_DIR_R = 0x0E;          // 5) PF4,PF0 in, PF3-1 out
  GPIO_PORTF_AFSEL_R = 0x00;        // 6) disable alt funct on PF7-0
  // GPIO_PORTF_PUR_R = 0x11;          // enable pull-up on PF0 and PF4
  GPIO_PORTF_DEN_R = 0x0A;          // 7) enable digital I/O on PF3 and PF1
}

void Init_Ports(void){ volatile unsigned long delay;
	SYSCTL_RCGC2_R |= 0x00000032;     // 1) activate clock for Ports E, B and F
  delay = SYSCTL_RCGC2_R;           // allow time for clock to start
	
	PortE_Init(); // Init port E for sensors
	PortB_Init(); // Init port B for road LEDs
	PortF_Init(); // Init port F for ped LEDs
}

int main(void){ 
  TExaS_Init(SW_PIN_PE210, LED_PIN_PB543210,ScopeOff); // activate grader and set system clock to 80 MHz
 
	SysTick_Init();
	Init_Ports();
  
  EnableInterrupts();
	S = GoWest;
  while(1){
		ROADLED = FSM[S].OutLED;
		PEDLED = FSM[S].OutPed;
		SysTick_Wait1ms(FSM[S].Time);
		Input = SENSOR;
		S = FSM[S].Next[Input];
  }
}

