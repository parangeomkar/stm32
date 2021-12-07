#ifndef MPC_core_dec
	#define MPC_core_dec 1
	uint8_t isOpenLoopComplete;

	float Vdc;
	int i,j;

	float IalphaPred,IbetaPred,IdPred,IqPred;
	float costTemp,cost;
	uint8_t optimalVector,optimalDuty;


	short error;
	float Kterm;
	float Iterm;
	uint16_t D;

	uint16_t executionCount, cnts;

	/**
	 * This function implements Speed PI controller
	 *
	 */
	void PIController();

	/**
	 * This function implements open loop control
	 *
	 */
	void openLoopControl();


	/**
	 * This function implements 6-step control
	 *
	 */
	void sixStepControl();


	/**
	 * This function implements model predictive control (MPC)
	 *
	 */
	void modelPredictiveControl();


	/**
	 * This function controls start/stop of motor with Blue button
	 *
	 */
	void startStop();


	/**
	 * This function controls the execution and ADC measurement
	 *
	 */
	void executeAll();
#endif
