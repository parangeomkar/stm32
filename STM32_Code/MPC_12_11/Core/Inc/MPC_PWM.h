#ifndef MPC_PWM_dec
	#define MPC_PWM_dec 1

	int T1;
	int T2;
	int T0;
	uint16_t Ta;
	uint16_t Tb;
	uint16_t Tc;
	uint16_t Ts;
	uint8_t n;

	uint8_t run;
	uint16_t wt;
	uint16_t V;

	/**
	 * This function computes SVPWM timings for TIM1
	 *
	 */
	void SVPWM();
#endif
