#include "main.h"
#include "MPC_math.h"
#include "MPC_feedback.h"
#include "MPC_PWM.h"

// Variables used by SVPWM()
int T1 = 0;
int T2 = 0;
int T0 = 0;
uint16_t Ta = 0;
uint16_t Tb = 0;
uint16_t Tc = 0;
uint16_t Ts = 72;
uint16_t wt = 0;
uint8_t n = 0;
uint8_t run = 1;

uint16_t V = 200;
uint16_t xxx = 0;
/**
 * This function computes SVPWM timings for TIM1
 *
 */
void SVPWM(){
	if(run == 1){
		n = ((uint8_t)(wt/60))+1;

		T1 = (uint16_t)(V*(sinLowRes(n*60 - wt))/(1667));
		T2 = (uint16_t)(V*(sinLowRes(wt - ((n-1)*60)))/(1667));
		T0 = Ts - (T1+T2);

		switch(n){
			case 1:
				Ta = T1 + T2 + (T0/2);
				Tb = T2 + (T0/2);
				Tc = (T0/2);
				break;
			case 2:
				Ta = T1 + (T0/2);
				Tb = T1 + T2 + (T0/2);
				Tc = (T0/2);
				break;
			case 3:
				Ta = (T0/2);
				Tb = T1 + T2 + (T0/2);
				Tc = T2 + (T0/2);
				break;
			case 4:
				Ta = (T0/2);
				Tb = T1 + (T0/2);
				Tc = T1 + T2 + (T0/2);
				break;
			case 5:
				Ta = T2 + (T0/2);
				Tb = (T0/2);
				Tc = T1 + T2 + (T0/2);
				break;
			case 6:
				Ta = T1 + T2 + (T0/2);
				Tb = (T0/2);
				Tc = T1 + (T0/2);
				break;
			default:
				Ta = 0;
				Tb = 0;
				Tc = 0;
		}

		TIM1->CCR1 = Ta;
		TIM1->CCR2 = Tb;
		TIM1->CCR3 = Tc;

	} else {
		TIM1->CCR1 = 0;
		TIM1->CCR2 = 0;
		TIM1->CCR3 = 0;
	}
}
