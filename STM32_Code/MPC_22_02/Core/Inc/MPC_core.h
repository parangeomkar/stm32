#ifndef MPC_core_dec
	#define MPC_core_dec 1
	uint8_t isOpenLoopComplete;

	int i,j;

	float IalphaPred,IbetaPred,IdTemp,IqTemp,IdPred,IqPred,IqTx;
	int costTemp,cost,C1,C2;
	uint8_t optimalVector,optimalDuty,startOCwatch;
	int16_t polePairs;
	uint8_t Sa,Sb,Sc;

	float Varray[7][3];
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
	 * This function initializes model predictive control variables
	 *
	 */
	void initModelPredictiveControl();

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
