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
short iqRef = 0;

/**
 * This function implements Speed PI controller
 *
 */
void PIController(){
	error = 500 - speed;
	Kterm = error;
	Iterm += (float)error/10000; // error*Kp*Ts => error*2*0.0001 => error/5000

	if(Iterm > 100){
		Iterm = 100;
	} else if(Iterm < -100){
		Iterm = -100;
	}
	iqRef = (Kterm+Iterm);

	if(iqRef>500){
		iqRef = 500;
	} else if(Iterm < -500){
		Iterm = -500;
	}
}



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
 * This function implements model predictive control (MPC)
 *
 */
void modelPredictiveControl(){
	computeSinCos();

	parkTransform(Ia,Ib,Ic,&Idq);
	parkTransform(Ea,Eb,Ec,&Edq);

	Idq.d = Idq.d/1241; // 3.3/4096 = 1/1241
	Idq.q = Idq.q/1241;

	Edq.d = Edq.d/1241; // 3.3/4096 = 1/1241
	Edq.q = Edq.q/1241;

	cost = 100000;

	for(i=0;i<6;i++){
		Va = states[i] & 0x01;
		Vb = (states[i]>>1) & 0x01;
		Vc = (states[i]>>2) & 0x01;
		parkTransform(Va,Vb,Vc,&Vdq);

		for(j=1;j<3;j++){
			IdPred = (float)((5/j)*Vdq.d + 2*Idq.d - Edq.d);
			IqPred = (float)((5/j)*Vdq.q + 2*Idq.q - Edq.q);

			costTemp = ((float)square(mod((short)(IdPred*1000))) + (float)square(mod(iqRef - (short)(IqPred*1000))))/1000000;

			if(costTemp < cost){
				optimalVector = i;
				optimalDuty = j;
				cost = costTemp;
			}
		}
	}

	if(optimalDuty == 2){
		V = 300;
//	}else if(optimalDuty == 3){
//		V = 200;
//	} else if(optimalDuty == 4){
//		V = 150;
	} else {
		V = 600;
	}

	wt = limitTheta((optimalVector+1)*60);
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
uint16_t executionCount, cnts = 0;
void executeAll(){
	measureADC();
	if(run){
		if(executionCount > 50000){
			if(cnts == 100){
				modelPredictiveControl();
				cnts = 0;
			} else {
				cnts++;
			}
		} else {
			openLoopControl();
			executionCount++;
		}
	}
	SVPWM();
}
