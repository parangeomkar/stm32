#include "main.h"
#include "MPC_math.h"
#include "MPC_PWM.h"

// Variables used by SVPWM()
int T1 = 0;
int T2 = 0;
int T0 = 0;
uint16_t Ta = 0;
uint16_t Tb = 0;
uint16_t Tc = 0;
uint16_t Ts = 512;
uint16_t wt = 0;
uint8_t n = 0;
uint8_t run = 1;

uint16_t V = 300;

/**
 * This function computes SVPWM timings for TIM1
 *
 */
void SVPWM(){
	if(run == 1){
		n = ((uint8_t)(wt/60))+1;

		T1 = (uint16_t)(V*sin2(n*60 - wt)/1000);
		T2 = (uint16_t)(V*sin2(wt - ((n-1)*60))/1000);
		T0 = Ts - (T1+T2) + 5;


		if(wt < 60) {
			Ta = T1 + T2 + (T0/2);
			Tb = T2 + (T0/2);
			Tc = (T0/2);
		} else if(wt >= 60 && wt < 120) {
			Ta = T1 + (T0/2);
			Tb = T1 + T2 + (T0/2);
			Tc = (T0/2);
		} else if(wt >= 120 && wt < 180) {
			Ta = (T0/2);
			Tb = T1 + T2 + (T0/2);
			Tc = T2 + (T0/2);
		} else if(wt >= 180 && wt < 240) {
			Ta = (T0/2);
			Tb = T1 + (T0/2);
			Tc = T1 + T2 + (T0/2);
		} else if(wt >= 240 && wt < 300) {
			Ta = T2 + (T0/2);
			Tb = (T0/2);
			Tc = T1 + T2 + (T0/2);
		} else if(wt >= 300 && wt < 360) {
			Ta = T1 + T2 + (T0/2);
			Tb = (T0/2);
			Tc = T1 + (T0/2);
		} else {
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
