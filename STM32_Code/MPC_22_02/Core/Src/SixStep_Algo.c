#include "main.h"
#include "MPC_core.h"
#include "MPC_math.h"
#include "MPC_PWM.h"
#include "MPC_feedback.h"
#include "MPC_communication.h"


/**
 * This function implements 6-step control
 *
 */
void sixStepControl(){
	thetaElec = 360*TIM3->CNT/600;
	wt = 60*(thetaElec/60) + 90;

	if(wt >= 360){
		wt -= 360;
	}
}
