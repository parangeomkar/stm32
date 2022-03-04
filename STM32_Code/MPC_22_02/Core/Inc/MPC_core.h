#ifndef MPC_core_dec
	#define MPC_core_dec 1
	uint8_t isOpenLoopComplete;

	int i,j;

	float IalphaPred,IbetaPred,IdPredTemp,IqPredTemp,IdPred,IqPred,IqTx;
	int costTemp,cost;
	uint8_t optimalVector,optimalDuty,startOCwatch;
	int16_t polePairs;


	int16_t error;
	float Kterm;
	float Iterm;
	int16_t Kp,Ki,sigma,delta,speedReq;

	float IqRef;

	/**
	 * This function implements Speed PI controller
	 *
	 */
	void SpeedPIController();


	/**
	 * This function implements model predictive control (MPC)
	 *
	 */
	void modelPredictiveControl();

	/**
	 * This function implements 6-step control
	 *
	 */
	void sixStepControl();

	/**
	 * This function controls starting of motor with Blue button
	 *
	 */
	void startMotor();

	/**
	 * This function controls stopping of motor with Blue button
	 *
	 */
	void stopMotor();

	/**
	 * This function implements open loop control
	 *
	 */
	void openLoopControl();
#endif
