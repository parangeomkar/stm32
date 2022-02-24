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
int16_t Kp = 20;
int16_t Ki = 20;

float Iterm = 0;
void SpeedPIController(){
	speedReq = 2000;
	error = speedReq - speed;
	Kterm = (float)(error)*0.002;
	Iterm += (float)(error)*0.00001;

	if(Iterm > 1){
		Iterm = 1;
	} else if(Iterm < -1){
		Iterm = -1;
	}

	IqRef = (Kterm+Iterm);

	if(IqRef > 3){
		IqRef = 3;
	} else if(IqRef < -3){
		IqRef = -3;
	}
}