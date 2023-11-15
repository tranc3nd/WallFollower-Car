// WallFollowerStarter.c
// Runs on TM4C123
// This is the starter file for CECS 347 Project 2 - A Wall Follower
// This project use PWM to control two DC Motors, SysTick timer 
// to control sampling rate, ADC to collect analog inputs from
// Sharp IR sensors and a potentiometer.
// Two GP2Y0A21YK0F analog IR distance sensors are used to allow
// the robot to follow a wall. A Minimum of two IR sensors are mounted
// looking forward to the left and forward to the right. 
// A optional third IR sensor looks directly forward can be used to avoid
// a head-on collision. Basically, the goal is to control power to each wheel so the
// left and right distances to the walls are equal.
// If an object is detected too close to the front of the robot,
// both wheels are immediately stopped.
/*
    ------------------------------------------wall---------
                      /
                     /
                    / 
                   /
         -----------
         |         |
         | Robot   | ---> direction of motion and third sensor
         |         |
         -----------
                   \
                    \
                     \
                      \
    ------------------------------------------wall---------
*/
// The original project is provided by Dr. Daniel Valvano, Jonathan Valvano
// September 12, 2013
// Modification is made by Dr. Min He to provide this starter project.

// PE1 connected to forward facing IR distance sensor
// PE4 connected to forward right IR distance sensor
// PE5 connected to forward left IR distance sensor

#include "ADCMultiSamples.h"
#include "PLL.h"
#include "tm4c123gh6pm.h"
#include "stdint.h"

// basic functions defined at end of startup.s
void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts
void WaitForInterrupt(void);  // low power mode
void PWM_PB76_Duty(unsigned long duty_L, unsigned long duty_R);

// High and low cycle counts for left and right wheels
long LeftH, LeftL, RightH, RightL;

uint8_t sample=0;

// You use datasheet to calculate the following ADC values
// then test your sensors to adjust the values 
#define WHEEL_DIR (*((volatile unsigned long *)0x400050F0)) // PB5432 are the four direction pins for L298
// Constant definitions based on the following hardware interface:
// PB5432 are used for direction control on L298.
// Motor 1 is connected to the left wheel, Motor 2 is connected to the right wheel.
#define FORWARD 0x28
#define BACKWARD 0x14
#define LEFTPIVOT 0x18
#define RIGHTPIVOT 0x24	

#define CRASH             IR15CM// if there is less than this distance ahead of the robot, it will immediately stop
#define IR15CM            2233  // ADC output for 15cm:1.8v -> (1.8/3.3)*4095=2233 
#define IR20CM            1724  // ADC output for 20cm:1.39v -> (1.39/3.3)*4095=1724
#define IR30CM            1116  // ADC output for 30cm:0.9v -> (0.9/3.3)*4095=1116
#define IR40CM            918   // ADC output for 40cm:0.74v -> (0.74/3.3)*4095=918
#define IR80CM            496   // ADC output for 80cm:0.4v -> (0.4/3.3)*4095=496
                                // with equal power to both motors (LeftH == RightH), the robot still may not drive straight
                                // due to mechanical differences in the motors, so bias the left wheel faster or slower than
                                // the constant right wheel
#define LEFTMINPCT        30    // minimum percent duty cycle of left wheel (10 to 90)
#define LEFTMAXPCT        50    // maximum percent duty cycle of left wheel (10 to 90)
#define RIGHTCONSTPCT     40    // constant percent duty cycle of right wheel (10 to 90)
#define period 					  80000
#define FIFTY_PERCENT 		80000*0.5

void System_Init(void);
void LEDSW_Init(void);
void Motor_Init(void);
void Car_Dir_Init(void);
void SysTick_Init(void);
void steering(uint16_t ahead_dist,uint16_t right_dist, uint16_t left_dist);
void ReadADCFIRFilter(uint16_t *ain2, uint16_t *ain9, uint16_t *ain8);
void ReadADCIIRFilter(uint16_t *ain2, uint16_t *ain9, uint16_t *ain8);
uint16_t median(uint16_t u1, uint16_t u2, uint16_t u3);
void ReadADCMedianFilter(uint16_t *ain2, uint16_t *ain9, uint16_t *ain8);// This function samples AIN2 (PE1), AIN9 (PE4), AIN8 (PE5) and

int main(void){
  uint16_t left, right, ahead;
  DisableInterrupts();  // disable interrupts while initializing
  System_Init();
	WHEEL_DIR = FORWARD;
  EnableInterrupts();   // enable after all initialization are done

  while(1){
    if(sample) {
      sample = 0;
      // choose one of the software filter methods
      ReadADCMedianFilter(&ahead, &right, &left);
      steering(ahead,right,left);
    }
  }
}

void System_Init(void) {
  PLL_Init();           // bus clock at 80 MHz
	SysTick_Init();       // Initialize SysTick timer with interrupt for smapling control
  ADC_Init298();        // initialize ADC to sample AIN2 (PE1), AIN9 (PE4), AIN8 (PE5)
  LEDSW_Init();         // configure onboard LEDs and push buttons
	//Car_Dir_Init();
  Motor_Init();         // Initialize signals for the two DC Motors
}

void LEDSW_Init(void){
	
}

void Motor_Init(void){
	/*SYSCTL_RCGC2_R |= 0x00000001; // activate clock for port A
  LeftH = (LEFTMAXPCT + LEFTMINPCT)*400;
  LeftL = 80000 - LeftH;        // value modified my controller
  RightH = RIGHTCONSTPCT*800;   // constant
  RightL = 80000 - RightH;
  GPIO_PORTA_AMSEL_R &= ~0xC0;      // disable analog functionality on PA6-5
  GPIO_PORTA_PCTL_R &= ~0xFF000000; // configure PA6-5 as GPIO
  GPIO_PORTA_DIR_R |= 0xC0;     // make PA6-5 out
  GPIO_PORTA_DR8R_R |= 0xC0;    // enable 8 mA drive on PA6-5
  GPIO_PORTA_AFSEL_R &= ~0xC0;  // disable alt funct on PA6-5
  GPIO_PORTA_DEN_R |= 0xC0;     // enable digital I/O on PA6-5
  GPIO_PORTA_DATA_R &= ~0xC0;   // make PA6-5 low 
	*/
	
	if ((SYSCTL_RCGC2_R&SYSCTL_RCGC2_GPIOB)==0) {
		SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOB;	// Activate B clocks
		while ((SYSCTL_RCGC2_R&SYSCTL_RCGC2_GPIOB)==0){};
	}
	
	LeftH = (LEFTMAXPCT + LEFTMINPCT)*400;
  LeftL = 80000 - LeftH;        // value modified my controller
  RightH = RIGHTCONSTPCT*800;   // constant
  RightL = 80000 - RightH;
	
	
	GPIO_PORTB_AFSEL_R |= 0xC0;	// enable alt funct: PB76 for PWM
  GPIO_PORTB_PCTL_R &= ~0xFF000000; // PWM to be used
  GPIO_PORTB_PCTL_R |= 0x44000000; // PWM to be used
  GPIO_PORTB_DEN_R |= 0xC0;	// enable digital I/O 
	
	// Initializes PWM settings
	SYSCTL_RCGCPWM_R |= 0x01;	// activate PWM0
	SYSCTL_RCC_R &= ~0x001E0000; // Clear any previous PWM divider values
	
	// PWM0_0 output A&B Initialization for PB76
	PWM0_0_CTL_R = 0;	// re-loading down-counting mode
	PWM0_0_GENA_R |= 0xC8;	// low on LOAD, high on CMPA down
	PWM0_0_GENB_R |= 0xC08;// low on LOAD, high on CMPB down
	PWM0_0_LOAD_R = period - 1;	// cycles needed to count down to 0
  PWM0_0_CMPA_R = 0;	// count value when output rises
	PWM0_0_CMPB_R = 0;	// count value when output rises
	
	PWM0_0_CTL_R |= 0x00000001;	// Enable PWM0 Generator 0 in Countdown mode
	PWM0_ENABLE_R |= 0x00000003;	// Disable PB76:PWM0 output 0&1 on initialization
}

void PWM_PB76_Duty(unsigned long duty_L, unsigned long duty_R){
	PWM0_0_CMPA_R = duty_L - 1;	// PA6 count value when output rises
  PWM0_0_CMPB_R = duty_R - 1;	// PA7 count value when output rises
}

void Car_Dir_Init(void){
	if ((SYSCTL_RCGC2_R&SYSCTL_RCGC2_GPIOB)==0) {
		SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOB;	// Activate B clocks
		while ((SYSCTL_RCGC2_R&SYSCTL_RCGC2_GPIOB)==0){};
	}
		
  GPIO_PORTB_AMSEL_R &= ~0x3C;	// disable analog function
	GPIO_PORTB_AFSEL_R &= ~0x3C;	// no alternate function
  GPIO_PORTB_PCTL_R &= ~0x00FFFF00;	// GPIO clear bit PCTL 
	GPIO_PORTB_DIR_R |= 0x3C; // output on pin(s)
  GPIO_PORTB_DEN_R |= 0x3C;	// enable digital I/O on pin(s)
}

void SysTick_Init(void){
	// Systick Init
	/*
	NVIC_ST_CTRL_R = 0;           // disable SysTick during setup
  NVIC_ST_RELOAD_R = period - 1; // reload value for 500us
  NVIC_ST_CURRENT_R = 0;        // any write to current clears it
  NVIC_SYS_PRI3_R = (NVIC_SYS_PRI3_R&0x00FFFFFF)|0x40000000; // priority 2
  NVIC_ST_CTRL_R = 0x00000007;  // enable with core clock and interrupts
	*/
}


void steering(uint16_t ahead_dist,uint16_t right_dist, uint16_t left_dist){
  // Suggest the following simple control as starting point:
  // 1. If any one of the senors see obstacle <20cm, stop
  // 2. If all sensors detect no obstacle within 35cm, stop
  // 3. If left sees obstacle within 30cm, turn right
  // 4. If right sees obstacle within 30cm, turn left
  // 5. If both sensors see no obstacle within 30cm, go straight  
	//if(right_dist >= IR30CM && left_dist >= IR30CM) {
		PWM1_ENABLE_R |= 0x00000003;	// Disable PA76:PWM1 output 0&1 on initialization
		PWM_PB76_Duty(FIFTY_PERCENT,FIFTY_PERCENT);
	//}
	
  // Feel free to add more controlls to fine tune your robot car.
  // Make sure to take care of both wheel movements and LED display here.
  
}
void SysTick_Handler(void){
  sample = 1;
}

// returns the results in the corresponding variables.  Some
// kind of filtering is required because the IR distance sensors
// output occasional erroneous spikes.  This is an FIR filter:
// y(n) = (x(n) + x(n-1))/2
// Assumes: ADC initialized by previously calling ADC_Init298()
void ReadADCFIRFilter(uint16_t *ain2, uint16_t *ain9, uint16_t *ain8){
  static uint16_t ain2previous=0; // after the first call, the value changed to 12
  static uint16_t ain9previous=0;
  static uint16_t ain8previous=0;
  // save some memory; these do not need to be 'static'
  //            x(n)
  uint16_t ain2newest;
  uint16_t ain9newest;
  uint16_t ain8newest;
  ADC_In298(&ain2newest, &ain9newest, &ain8newest); // sample AIN2(PE1), AIN9 (PE4), AIN8 (PE5)
  *ain2 = (ain2newest + ain2previous)/2;
  *ain9 = (ain9newest + ain9previous)/2;
  *ain8 = (ain8newest + ain8previous)/2;
  ain2previous = ain2newest; ain9previous = ain9newest; ain8previous = ain8newest;
}

// This function samples AIN2 (PE1), AIN9 (PE4), AIN8 (PE5) and
// returns the results in the corresponding variables.  Some
// kind of filtering is required because the IR distance sensors
// output occasional erroneous spikes.  This is an IIR filter:
// y(n) = (x(n) + y(n-1))/2
// Assumes: ADC initialized by previously calling ADC_Init298()
void ReadADCIIRFilter(uint16_t *ain2, uint16_t *ain9, uint16_t *ain8){
  //                   y(n-1)
  static uint16_t filter2previous=0;
  static uint16_t filter9previous=0;
  static uint16_t filter8previous=0;
  // save some memory; these do not need to be 'static'
  //            x(n)
  uint16_t ain2newest;
  uint16_t ain9newest;
  uint16_t ain8newest;
  ADC_In298(&ain2newest, &ain9newest, &ain8newest); // sample AIN2(PE1), AIN9 (PE4), AIN8 (PE5)
  *ain2 = filter2previous = (ain2newest + filter2previous)/2;
  *ain9 = filter9previous = (ain9newest + filter9previous)/2;
  *ain8 = filter8previous = (ain8newest + filter8previous)/2;
}

// Median function from EE345M Lab 7 2011; Program 5.1 from Volume 3
// helper function for ReadADCMedianFilter() but works for general use
uint16_t median(uint16_t u1, uint16_t u2, uint16_t u3){
uint16_t result;
  if(u1>u2)
    if(u2>u3)   result=u2;     // u1>u2,u2>u3       u1>u2>u3
      else
        if(u1>u3) result=u3;   // u1>u2,u3>u2,u1>u3 u1>u3>u2
        else      result=u1;   // u1>u2,u3>u2,u3>u1 u3>u1>u2
  else
    if(u3>u2)   result=u2;     // u2>u1,u3>u2       u3>u2>u1
      else
        if(u1>u3) result=u1;   // u2>u1,u2>u3,u1>u3 u2>u1>u3
        else      result=u3;   // u2>u1,u2>u3,u3>u1 u2>u3>u1
  return(result);
}

// This function samples AIN2 (PE1), AIN9 (PE4), AIN8 (PE5) and
// returns the results in the corresponding variables.  Some
// kind of filtering is required because the IR distance sensors
// output occasional erroneous spikes.  This is a median filter:
// y(n) = median(x(n), x(n-1), x(n-2))
// Assumes: ADC initialized by previously calling ADC_Init298()
void ReadADCMedianFilter(uint16_t *ain2, uint16_t *ain9, uint16_t *ain8){
  //                   x(n-2)        x(n-1)
  static uint16_t ain2oldest=0, ain2middle=0;
  static uint16_t ain9oldest=0, ain9middle=0;
  static uint16_t ain8oldest=0, ain8middle=0;
  // save some memory; these do not need to be 'static'
  //            x(n)
  uint16_t ain2newest;
  uint16_t ain9newest;
  uint16_t ain8newest;
  ADC_In298(&ain2newest, &ain9newest, &ain8newest); // sample AIN2(PE1), AIN9 (PE4), AIN8 (PE5)
  *ain2 = median(ain2newest, ain2middle, ain2oldest);
  *ain9 = median(ain9newest, ain9middle, ain9oldest);
  *ain8 = median(ain8newest, ain8middle, ain8oldest);
  ain2oldest = ain2middle; ain9oldest = ain9middle; ain8oldest = ain8middle;
  ain2middle = ain2newest; ain9middle = ain9newest; ain8middle = ain8newest;
}
