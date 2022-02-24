#include "main.h"
#include "MPC_core.h"
#include "MPC_math.h"
#include "MPC_PWM.h"
#include "MPC_feedback.h"
#include "MPC_communication.h"

// Variables used by executeSetAlgorithm()
uint8_t isOpenLoopComplete = 0;


/**
 * This function controls initial position of motor
 *
 */
uint16_t cnts = 0;
uint16_t executionCount = 101;
void initalPositionSet(){
	if(cnts < 1000){
		wt = 30;
	} else {
		wt = 0;
		thetaElecTemp = 0;
		thetaMech = 0;
		thetaElec = 0;
		TIM3->CNT = 0;
		speed = 0;
		wr = 0;
	}
}


/**
 * This function controls start/stop of motor with Blue button
 *
 */
void startStop(){
	if(!run){
		run = 1;
	} else {
		run = 0;
		cnts = 0;
	}
}

void testSVPWM(){
	V += 1;
	if(V>400){
		V = 200;
	}

	wt = 0;
	if(wt >= 360){
		wt = 0;
	}
}


/**
 * This function controls the execution and ADC measurement
 *
 */
uint16_t ex;
void executeAll(){
	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
	measureADC();

	if(run){
		if(cnts < 2000){
			initalPositionSet();
			cnts++;
		} else if(cnts >= 2000 && cnts < 3000){
			cnts++;
			V = 0;
		} else {
//			testSVPWM();
			modelPredictiveControl();
//			if(ex > 0){
//				V = 300;
//				openLoopControl();
//				ex = 0;
//			} else {
//				ex++;
//			}
		}
	} else {
		V = 0;
	}
	SVPWM();
	transferUART();
	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
}
