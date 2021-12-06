#include "main.h"
#include "math.h"
#include "MPC_math.h"
#include "MPC_PWM.h"
#include "MPC_feedback.h"
#include "MPC_communication.h"

// Variables used by executeSetAlgorithm()
uint8_t isOpenLoopComplete = 0;


/**
 * This function implements model predictive control (MPC)
 *
 */
int states[6] = {1,3,2,6,4,5};

float Vdc = 12;
int i,j;

float IalphaPred,IbetaPred,IdPred,IqPred;
float costTemp,cost;
uint8_t optimalVector,optimalDuty = 0;


short error;
uint16_t s=0;
float Kterm;
float Iterm = 0;
float D = 0.7;
void PIController(){
	error = 1000 - speed;
	Kterm = (float)error/1000;
	Iterm += (float)error/5000;

	if(Iterm > 1){
		Iterm = 1;
	} else if(Iterm < -1){
		Iterm = -1;
	}
	D = (Kterm+Iterm)/2;
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
	wt = (uint16_t)(60*floor((theta)/60));
}




/**
 * This function implements model predictive control
 *
 */
float d[4] = {1,0.75,0.5,0.25};

void predictCurrent(){
	Va = states[i] & 0x01;
	Vb = (states[i]>>1) & 0x01;
	Vc = (states[i]>>2) & 0x01;

	parkTransform(Va,Vb,Vc,&Vdq);

	IdPred = (float)(5*Vdq.d + 2*Idq.d - Edq.d);
	IqPred = (float)(5*Vdq.q + 2*Idq.q - Edq.q);
}


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
//			predictCurrent(i);

			IdPred = (float)((12/j)*Vdq.d + 2*Idq.d - Edq.d);
			IqPred = (float)((12/j)*Vdq.q + 2*Idq.q - Edq.q);

			costTemp = ((float)square(mod((short)(IdPred*1000))) + (float)square(mod(100 - (short)(IqPred*1000))))/1000000;

			if(costTemp < cost){
				optimalVector = i;
				optimalDuty = j;
				cost = costTemp;
			}
		}
	}

	if(optimalDuty == 2){
		V = 200;
//	}else if(optimalDuty == 3){
//		V = 100;
//	} else if(optimalDuty == 4){
//		V = 75;
	} else {
		V = 400;
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
		//transferUART();
	}
	SVPWM();
}
