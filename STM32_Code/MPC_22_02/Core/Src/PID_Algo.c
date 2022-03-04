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
int16_t Kp = 512;
int16_t Ki = 512;

float Iterm = 0;
int16_t speedReq = 1500;
void SpeedPIController(){
	error = speedReq - speed;
	Kterm = (float)(error*Kp)/256;
	Iterm += (float)(error*Ki)/2560;

	if(Iterm > 10000){
		Iterm = 10000;
	} else if(Iterm < -10000){
		Iterm = -10000;
	}

	IqRef = (int16_t)(Kterm+Iterm);

	if(IqRef > 20000){
		IqRef = 20000;
	} else if(IqRef < -20000){
		IqRef = -20000;
	}
}
