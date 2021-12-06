#ifndef MPC_feedback_dec
	#define MPC_feedback_dec 1

	extern ADC_HandleTypeDef hadc1;
	extern ADC_HandleTypeDef hadc2;


	// Variables used by computePosition()
	short theta;
	uint16_t speed;


	struct alphaBeta Ealbt,Valbt,Ialbt,IalbtReq;
	struct directQuad Idq,Edq,Vdq;


	short Va,Vb,Vc, Ea,Eb,Ec, Ia,Ib,Ic;


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


	/**
	 * This function sine cos values for given theta used in park transform
	 *
	 */
	void computeSinCos();
#endif
