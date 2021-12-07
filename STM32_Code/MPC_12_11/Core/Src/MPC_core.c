#include "main.h"
#include "MPC_core.h"
#include "MPC_math.h"
#include "MPC_PWM.h"
#include "MPC_feedback.h"
#include "MPC_communication.h"

// Variables used by executeSetAlgorithm()
uint8_t isOpenLoopComplete = 0;


int states[6] = {1,3,2,6,4,5};

float Vdc = 12;

float Iterm = 0;
float iqRef = 0;

uint16_t cnts = 0;




/**
 * This function implements open loop control
 *
 */
void openLoopControl(){
	wt+=1;
	if(wt >= 360){
		wt = 0;
	}
}


/**
 * This function implements 6-step control
 *
 */
void sixStepControl(){
	wt = (uint16_t)(60*(uint8_t)((theta)/60));
}


/**
 * This function implements Speed PI controller
 *
 */
void PIController(){
	error = 100 - speed;

	if(error > -1500 && error < 1500){
		Kterm = error/10;
//		Iterm += (float)error*700/(256*10000); // error*Kp*Ts => error*2*0.0001 => error/5000
//
//		if(Iterm > 100){
//			Iterm = 100;
//		} else if(Iterm < -100){
//			Iterm = -100;
//		}

		iqRef = (Kterm+0)/100;

//		if(iqRef>500){
//			iqRef = 500;
//		} else if(iqRef < -500){
//			iqRef = -500;
//		}
	}
}


/**
 * This function implements model predictive control (MPC)
 *
 */
uint16_t penalty = 0;
uint16_t F = 1000;
void modelPredictiveControl(){
	computeSinCos();

	parkTransform(Ia,Ib,Ic,&Idq);
	parkTransform(Ea,Eb,Ec,&Edq);

	Idq.d = Idq.d*10/4096;
	Idq.q = Idq.q*10/4096;

	Edq.d = Edq.d/1241;
	Edq.q = Edq.q/1241;

	cost = 100000;

	for(i=0;i<6;i++){
		Va = states[i] & 0x01;
		Vb = (states[i]>>1) & 0x01;
		Vc = (states[i]>>2) & 0x01;
		parkTransform(Va,Vb,Vc,&Vdq);

		IdPred = (float)((3*Vdq.d) + 2*Idq.d - Edq.d)*1000/2375; // 1/(R*Ts+L) = 1/2.375
		IqPred = (float)((3*Vdq.q) + 2*Idq.q - Edq.q)*1000/2375;

//			IdPred = (8125*Idq.d/10000) + (speed*4*Idq.q/F) + (5*Vdq.d/2);
//			IqPred = (8125*Idq.q/10000) - (speed*4*Idq.d/F) + (5*Vdq.q/2) - (6*speed*4/(1000*F));

		if(IdPred > 0.6 || IqPred > 0.6 || IdPred < -0.6 || IqPred < -0.6 ){
			penalty = 10000;
		} else {
			penalty = 0;
		}

		costTemp = penalty + (square(mod(IdPred)) + 2*square(mod(0.2 - IqPred)));

		if(costTemp < cost){
			optimalVector = i;
			cost = costTemp;
		}
	}

	V = 300;

	wt = (optimalVector+1)*60;
	if(wt >= 360){
		wt = 0;
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
	}
}

/**
 * This function controls the execution and ADC measurement
 *
 */
uint16_t asd =0;
uint8_t p = 1;
void executeAll(){
	measureADC();
	if(run){
		if(executionCount > 15000 && p<2){
			sixStepControl();
		} else {
			openLoopControl();
//		}
//			if(cnts == p){
//				modelPredictiveControl();
//				cnts = 0;
//			} else {
//				cnts++;
//			}
//		} else {
			executionCount++;
//		}
		}
	}
	SVPWM();
	transferUART();
}
