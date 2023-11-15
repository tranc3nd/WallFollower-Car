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
// PE2 connected to Potentiometer
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

uint8_t sample = 0, enableMotor = 0, speed_mode = 0;

#define LED (*((volatile unsigned long *)0x40025038))  // use onboard three LEDs: PF321
// You use datasheet to calculate the following ADC values
// then test your sensors to adjust the values 
#define CRASH             IR15CM// if there is less than this distance ahead of the robot, it will immediately stop
#define IR15CM            2233  // ADC output for 15cm:1.8v -> (1.8/3.3)*4095=2233 
#define IR20CM            1724  // ADC output for 20cm:1.39v -> (1.39/3.3)*4095=1724
#define IR30CM            1116  // ADC output for 30cm:0.9v -> (0.9/3.3)*4095=1116
#define IR40CM            918   // ADC output for 40cm:0.74v -> (0.74/3.3)*4095=918
#define IR80CM            496   // ADC output for 80cm:0.4v -> (0.4/3.3)*4095=496
#define IR60CM						707
                                // with equal power to both motors (LeftH == RightH), the robot still may not drive straight
                                // due to mechanical differences in the motors, so bias the left wheel faster or slower than
                                // the constant right wheel
#define LEFTMINPCT        30    // minimum percent duty cycle of left wheel (10 to 90)
#define LEFTMAXPCT        50    // maximum percent duty cycle of left wheel (10 to 90)
#define RIGHTCONSTPCT     40    // constant percent duty cycle of right wheel (10 to 90)
#define period 					  50000
#define FIFTY_PERCENT 		0.5
#define THIRTY_PERCENT 		0.3
#define TWENTY_PERCENT 		0.2
#define FIFTN_PERCENT 		0.15
#define TEN_PERCENT 			0.1
#define RELOAD 799999
#define NVIC_EN0_PORTF 	0x40000000     // (h) enable interrupt 30 in NVIC
#define WHEEL_DIR (*((volatile unsigned long *)0x400050F0)) // PB5432 are the four direction pins for L298
#define Dark    	0x00
#define Red     	0x02
#define Blue    	0x04
#define Green   	0x08
#define Yellow  	0x0A
#define Cran      0x0C
#define White   	0x0E
#define Purple  	0x06
// Constant definitions based on the following hardware interface:
// PB5432 are used for direction control on L298.
// Motor 1 is connected to the left wheel, Motor 2 is connected to the right wheel.
#define FORWARD 0x28
#define BACKWARD 0x14
#define LEFTPIVOT 0x18
#define RIGHTPIVOT 0x24	

void System_Init(void);
void LEDSW_Init(void);
void Motor_Init(void);
void SysTick_Init(void);
void steering(uint16_t ahead_dist,uint16_t right_dist, uint16_t left_dist);
void ReadADCFIRFilter(uint16_t *ain2, uint16_t *ain9, uint16_t *ain8);
void ReadADCIIRFilter(uint16_t *ain2, uint16_t *ain9, uint16_t *ain8);
uint16_t median(uint16_t u1, uint16_t u2, uint16_t u3);
void ReadADCMedianFilter(uint16_t *ain2, uint16_t *ain9, uint16_t *ain8);// This function samples AIN1 (PE2), AIN2 (PE1), AIN9 (PE4), AIN8 (PE5)
void PWM_PB76_Duty(unsigned long duty_L, unsigned long duty_R);
void Car_Dir_Init(void);
void Delay(void);
void GPIO_PORTF_Init(void);

unsigned char eq_calcution(unsigned int ADC_Value);
unsigned char tb_estimation(unsigned int ADC_Value);
int p_adcvalue = 0;
double p_speed = 0;


// High and low cycle counts for left and right wheels
long LeftH, LeftL, RightH, RightL;
uint16_t left, right, ahead, pot;

int main(void){
  DisableInterrupts();  // disable interrupts while initializing
  System_Init();
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
  PLL_Init();           // bus clock at 50 MHz
  SysTick_Init();       // Initialize SysTick timer with interrupt for smapling control
	GPIO_PORTF_Init();
  LEDSW_Init();         // configure onboard LEDs and push buttons
  Motor_Init();         // Initialize signals for the two DC Motors
	Car_Dir_Init();
  ADC_Init298();        // initialize ADC to sample AIN2 (PE1), AIN9 (PE4), AIN8 (PE5), AIN1 (PE2)
	WHEEL_DIR = FORWARD;
}

void LEDSW_Init(void){
	SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOF;	// Activate F clocks
	while ((SYSCTL_RCGC2_R&SYSCTL_RCGC2_GPIOF)==0){};
		
  GPIO_PORTF_AMSEL_R &= ~0x0E;      // 3) disable analog function
  GPIO_PORTF_PCTL_R &= ~0x0000FFF0; // 4) GPIO clear bit PCTL  
  GPIO_PORTF_DIR_R |= 0x0E;         // 6) PF1-PF3 output
  GPIO_PORTF_AFSEL_R &= ~0x0E;      // 7) no alternate function     
  GPIO_PORTF_DEN_R |= 0x0E;         // 8) enable digital pins PF3-PF1
  LED = Dark;                       // Turn off all LEDs.
}

void Motor_Init(void){
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
	PWM0_ENABLE_R &= ~0x00000003;	// Disable PB76:PWM0 output 0&1 on initialization	
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
  NVIC_ST_CTRL_R = 0;           // disable SysTick during setup
  NVIC_ST_RELOAD_R = RELOAD;       // reload value for 50% duty cycle
  NVIC_ST_CURRENT_R = 0;        // any write to current clears it
  NVIC_PRI1_R = (NVIC_PRI1_R&0xFFFFFF1F)|0x00000040; // bit 23-21 for SysTick PortE, set priority to 2
  NVIC_ST_CTRL_R |= 0x00000007;  // enable with core clock and interrupts, start systick timer
}

void steering(uint16_t ahead_dist,uint16_t right_dist, uint16_t left_dist){
  // Suggest the following simple control as starting point:
  // 1. If any one of the senors see obstacle <20cm, stop
  // 2. If all sensors detect no obstacle within 35cm, stop
  // 3. If left sees obstacle within 30cm, turn right
  // 4. If right sees obstacle within 30cm, turn left
  // 5. If both sensors see no obstacle within 30cm, go straight
	if(speed_mode) {	
		uint16_t ain2newest;
		uint16_t ain9newest;
		uint16_t ain8newest;
		uint16_t ain1newest;
		uint16_t *ain1;
		static uint16_t ain1oldest=0, ain1middle=0;
		ADC_In298(&ain2newest, &ain9newest, &ain8newest, &ain1newest); 
		ain1middle = ain1newest; ain1middle = ain1newest; ain1middle = ain1newest;
		*ain1 = median(ain1newest, ain1middle, ain1oldest);
		p_adcvalue = *ain1;
		
		if(p_adcvalue > 0 && p_adcvalue <= 360) p_speed = 0.05;
		else if(p_adcvalue > 360 && p_adcvalue <= 720) p_speed = 0.15;
		else if(p_adcvalue > 720 && p_adcvalue <= 1080) p_speed = 0.25;
		else if(p_adcvalue > 1080 && p_adcvalue <= 1440) p_speed = 0.35;
		else if(p_adcvalue > 1440) p_speed = 0.40;
		else p_speed = 0;	
	}else {
		p_adcvalue = 0;
		p_speed = 0;
	}
	
	if(enableMotor) {
			PWM0_ENABLE_R |= 0x00000003; // enable both wheel
			if((left_dist>IR15CM)&&(right_dist>IR15CM)&&(ahead_dist>IR15CM)){//stop moving
					PWM_PB76_Duty(0,0);
					LED=Red;
			}
			else if((left_dist>IR15CM)||(right_dist>IR15CM)){								//adjust left or right
				if(left_dist < right_dist){
					PWM_PB76_Duty((p_speed+FIFTN_PERCENT)*period,(p_speed+FIFTY_PERCENT)*period);
					LED=Green;
				}
				else{
					PWM_PB76_Duty((p_speed+FIFTY_PERCENT)*period,(p_speed+FIFTN_PERCENT)*period);
					LED=Blue;
				}
			}
			else if((left_dist<IR60CM)&&(right_dist<IR60CM)&&(ahead_dist<IR60CM)){	//open area
				PWM_PB76_Duty(0,0);
				LED=Purple;
			}
			
			else if((left_dist>IR20CM)){	//right moving
				PWM_PB76_Duty((p_speed+FIFTY_PERCENT)*period,(p_speed+FIFTN_PERCENT)*period);
				
			}
			else if((right_dist>IR20CM)){	// moving
				
				PWM_PB76_Duty((p_speed+FIFTN_PERCENT)*period,(p_speed+FIFTY_PERCENT)*period);
				
			}
			else if(left_dist<right_dist){
				LED=Dark;
				if((right_dist-left_dist) > IR20CM){
					PWM_PB76_Duty((p_speed+THIRTY_PERCENT)*period,(p_speed+THIRTY_PERCENT)*period);
				}
				else
					PWM_PB76_Duty((p_speed+FIFTN_PERCENT)*period,(p_speed+THIRTY_PERCENT)*period);
			}
			else if(left_dist>right_dist){
				LED=Dark;
				if((left_dist-right_dist) > IR20CM){
					PWM_PB76_Duty((p_speed+THIRTY_PERCENT)*period,(p_speed+THIRTY_PERCENT)*period);
				}
				else
					PWM_PB76_Duty((p_speed+THIRTY_PERCENT)*period,(p_speed+FIFTN_PERCENT)*period);
			}
			else {//if((right_dist>IR60CM)&&(left_dist>IR60CM)){
				LED=Dark;
				PWM_PB76_Duty((p_speed+THIRTY_PERCENT)*period,(p_speed+THIRTY_PERCENT)*period);
			}
	}else {
		LED = 0;
		PWM0_ENABLE_R &= ~0x00000003; // disable both wheels
	}

	//	}
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
	uint16_t ain1newest;
  ADC_In298(&ain2newest, &ain9newest, &ain8newest, &ain1newest); // sample AIN1 (PE2), AIN2(PE1), AIN9 (PE4), AIN8 (PE5)
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
	uint16_t ain1newest;
  ADC_In298(&ain2newest, &ain9newest, &ain8newest, &ain1newest); // sample AIN1 (PE2), AIN2(PE1), AIN9 (PE4), AIN8 (PE5)
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
	uint16_t ain1newest;
  ADC_In298(&ain2newest, &ain9newest, &ain8newest, &ain1newest); // sample AIN2(PE1), AIN9 (PE4), AIN8 (PE5)
  *ain2 = median(ain2newest, ain2middle, ain2oldest);
  *ain9 = median(ain9newest, ain9middle, ain9oldest);
  *ain8 = median(ain8newest, ain8middle, ain8oldest);
  ain2oldest = ain2middle; ain9oldest = ain9middle; ain8oldest = ain8middle;
  ain2middle = ain2newest; ain9middle = ain9newest; ain8middle = ain8newest;

}


void GPIO_PORTF_Init(void){	
	unsigned long volatile delay;
	SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOF; // activate clock for port F
	delay = SYSCTL_RCGC2_R;
	GPIO_PORTF_LOCK_R = 0x4C4F434B; 			// unlock GPIO Port F
	GPIO_PORTF_CR_R |= 0x11;      				// allow changes for PF4,PF0 take effect 
  GPIO_PORTF_AMSEL_R &= ~0x11;  				// disable analog functionality on PF4,PF0
  GPIO_PORTF_PCTL_R &= ~0x000F000F; 		// configure PF4, PF0 as GPIO
	GPIO_PORTF_DIR_R &= ~0x11;    				// make PF4,0 in (built-in button)
  GPIO_PORTF_DEN_R |= 0x11;     				// enable digital I/O on PF4, PF0
  GPIO_PORTF_AFSEL_R &= ~0x11;  				// disable alt funct on PF4, PF0
  GPIO_PORTF_PUR_R |= 0x11;     				// enable weak pull-up on PF4,PF0
  GPIO_PORTF_IS_R &= ~0x11;     				// PF4,PF0 is edge-sensitive
  GPIO_PORTF_IBE_R &= ~0x11;    				// PF4,PF0 is not both edges
  GPIO_PORTF_IEV_R &= ~0x11;    				// PF4,PF0 falling edge event
  GPIO_PORTF_ICR_R = 0x11;      				// clear flags 4,0
  GPIO_PORTF_IM_R |= 0x11;      				// arm interrupt on PF4,PF0
	NVIC_PRI7_R = (NVIC_PRI7_R&0xFF1FFFFF)|0x00400000; // bits:23-21 for PORTF, set priority to 2
  NVIC_EN0_R |= NVIC_EN0_PORTF;      		// enable interrupt 30 in NVIC
}


// PORTF ISR:
// Change delivered power based on switch press: 
void GPIOPortF_Handler(void){ // called on touch of either SW1 or SW2
  if(GPIO_PORTF_RIS_R&0x01) {  // SW2 touched - activates wheel speed mode
		Delay();
		GPIO_PORTF_ICR_R = 0x01;  // acknowledge flag0
		speed_mode ^= 1 + 0;
		LED = White;
		Delay();
  }
  if(GPIO_PORTF_RIS_R&0x10){  // SW1 touched - enables or disables wheel movement.
		Delay();
		GPIO_PORTF_ICR_R = 0x10;  // acknowledge flag4
		enableMotor ^= 1 + 0; // enable or disable wheels
		LED =Yellow;
		Delay();
	}
}

void Delay(void) {
	unsigned long volatile time;
  time = 727240*70/91;  // 0.03sec
  while(time){
		time--;
  }
}

//unsigned char eq_calcution(unsigned int ADC_Value){
//	double dist=0;
//	double A=-1.24836;			//from calculator
//	double B= 37878.6973; //from calculator
//	dist = A + (B/ADC_Value);
//	return dist;
//}

//unsigned char tb_estimation(unsigned int ADC_Value){
//	double output=0;
//	double values[]={4095,3830,3285,2850,2480,2170,1925,1720,1540,1380,
//									 1235,1150,1050,970, 930, 888, 867, 830, 805, 770,
//										754,732 ,712 ,692,670,650,640,620,603,585,
//										554,550};	
//	double distance[] = {3,10,12,14,16,18,20,22,24,26,
//											28,30,32,34,36,38,40,42,44,46,
//											48,50,52,54,56,58,60,62,64,66,
//											68,70};
//	unsigned char index;
//	unsigned int i;
//	for(i=0;i<sizeof(values)-1;i++){
//		if(ADC_Value>values[i+1] && ADC_Value<values[i]){
//			index=i;
//			break;
//		}
//	}
//	double ratio = ((values[index]-ADC_Value)/(values[index]-values[index+1]))*(distance[index]-distance[index+1]);
//	output = distance[index]+ratio;
//	return output;
//}
