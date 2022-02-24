#include "main.h"
#include "MPC_core.h"
#include "MPC_math.h"
#include "MPC_PWM.h"
#include "MPC_feedback.h"
#include "MPC_communication.h"


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
