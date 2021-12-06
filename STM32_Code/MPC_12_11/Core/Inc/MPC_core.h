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

#endif
