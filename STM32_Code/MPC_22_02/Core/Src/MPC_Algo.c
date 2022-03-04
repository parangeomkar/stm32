#include "main.h"
#include "MPC_core.h"
#include "MPC_math.h"
#include "MPC_PWM.h"
#include "MPC_feedback.h"
#include "MPC_communication.h"



/**
 * This function implements model predictive control (MPC)
 *
 */
uint8_t Sa,Sb,Sc;
int states[7] = {1,3,2,6,4,5,0};

void modelPredictiveControl(){
	computeSinCos();
	parkTransform(Ia,Ib,Ic,&Idq);

	cost = 100000;

	IdPredTemp = Idq.d/1230;
	IqPredTemp = Idq.q/1230;

//	V = (Vbus/65);

	for(i=0;i<6;i++){
		Sa = states[i] & 0x01;
		Sb = (states[i]>>1) & 0x01;
		Sc = (states[i]>>2) & 0x01;

	    Va = 8*((2*Sa-Sb-Sc))/3;
	    Vb = 8*((2*Sb-Sa-Sc))/3;
	    Vc = 8*((2*Sc-Sb-Sa))/3;

		parkTransform(Va,Vb,Vc,&Vdq);

		costTemp = 0;

//		for(j=0;j<1;j++){
//		IdPred = (int)((8650*IdPredTemp) + (wr*IqPredTemp*2) + (1250*Vdq.d));
		IqPred = (int)((9062*IqPredTemp) - (wr*IdPredTemp/2) + (1250*Vdq.q));

		costTemp = sqr(mod((IqRef - IqPred)))/10000;
//		}

		if(costTemp < cost){
			optimalVector = i;
			cost = costTemp;
			IqTx = IqPred*1230;
		}
	}


	wt = (optimalVector)*60;
	if(wt >= 360){
		wt -= 360;
	}
}
