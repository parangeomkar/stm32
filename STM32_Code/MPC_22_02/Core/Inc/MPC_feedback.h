#ifndef MPC_feedback_dec
	#define MPC_feedback_dec 1

	extern ADC_HandleTypeDef hadc1;
	extern ADC_HandleTypeDef hadc2;
	extern TIM_HandleTypeDef htim2;


	// Variables used by computePosition()
	int16_t thetaElec,thetaElecOld,thetaElecTemp,thetaElecEncoder;
	uint16_t thetaMech,x;
	uint16_t dT;
	int16_t speed;
	float wr;

	int16_t speedTemp, sum, speedArr[20];
	uint16_t speedPIrate, k, e;

	struct alphaBeta Ealbt,Valbt,Ialbt,IalbtReq;
	struct directQuad Idq,Edq,Vdq;


	int16_t Va,Vb,Vc, Ea,Eb,Ec;
	int16_t Ia,Ib,Ic,Vbus;


	// Variables used by measureADC()
	uint32_t Eab[2],Iabc[4];


	// Variables used by computePosition()
	int16_t dTheta;
	int16_t thetaOld;

	/**
	 * This function computes average rotor speed
	 *
	 */
	void computeSpeed();


	/**
	 * This function computes rotor position
	 *
	 * @param uint8_t x in range 0 to 128
	 * @return uint8_t arcsin(x) range 0 to 90
	 */
	void computePosition();



	/**
	 * This function computes 2 phase currents and 2 BEMFs
	 *
	 */
	void measureADC();



	void SpeedPIController();
#endif
