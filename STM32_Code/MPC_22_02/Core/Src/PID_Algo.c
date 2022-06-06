#include "main.h"
#include "MPC_core.h"
#include "MPC_math.h"
#include "MPC_PWM.h"
#include "MPC_feedback.h"
#include "MPC_communication.h"



/**
 * This function implements Speed PI controller
 *
 */

float Iterm = 0;
int16_t speedReq = 1500;
void SpeedPIController(){
	float Kp = 10;
	float Ki = 0.1;
	error = speedReq - speed;
	Kterm = (float)(error)*Kp;
	Iterm += (float)(error)*Ki;

	if(Iterm > 3250){
		Iterm = 3250;
	} else if(Iterm < -3250){
		Iterm = -3250;
	}

	IqRef = (int16_t)(Kterm+Iterm);

	if(IqRef > 4500){
		IqRef = 4500;
	} else if(IqRef < -4500){
		IqRef = -4500;
	}
}
