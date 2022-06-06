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

int states[7] = {1,3,2,6,4,5,0};
void initModelPredictiveControl(){
	C1 = 10000*(1 - (0.75/(16130*0.0004)));
	C2 = 10000*(1/(16130*0.0004));

	for(i=0;i<6;i++){
		Sa = states[i] & 0x01;
		Sb = (states[i]>>1) & 0x01;
		Sc = (states[i]>>2) & 0x01;

		Varray[i][0] = (V/100)*((2*Sa-Sb-Sc))/3;
		Varray[i][1] = (V/100)*((2*Sb-Sa-Sc))/3;
		Varray[i][2] = (V/100)*((2*Sc-Sb-Sa))/3;
	}
}

int16_t lambda1 = 10;
void modelPredictiveControl(){
	computeSinCos();
	parkTransform(Ia,Ib,Ic,&Idq);

	IdTemp = Idq.d/1230;
	IqTemp = Idq.q/1230;

	cost = 100000;


//	V = (Vbus/65);

	for(i=0;i<6;i++){
		Va = Varray[i][0];
		Vb = Varray[i][1];
		Vc = Varray[i][2];

		parkTransform(Va,Vb,Vc,&Vdq);

		costTemp = 0;

		for(j=0;j<1;j++){
			IdPred = (int)((C1*IdTemp) + (wr*IqTemp/2) + (C2*Vdq.d));
			IqPred = (int)((C1*IqTemp) - (wr*IdTemp/2) + (C2*Vdq.q));

			costTemp = lambda1*sqr(mod(IdPred))/100000 + sqr(mod((IqRef - IqPred)))/10000;
		}

		if(costTemp < cost){
			optimalVector = i;
			cost = costTemp;
			IqTx = IqPred*123/1000;
		}
	}


	wt = (optimalVector)*60;
	if(wt >= 360){
		wt -= 360;
	}
}
