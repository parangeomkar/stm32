#include "main.h"
#include "MPC_math.h"
#include "MPC_feedback.h"


// Variables used by measureADC()
uint32_t Eab[2],Iab[2];


// Variables used by computePosition()
float dTheta;
short thetaOld;


/**
 * This function computes average rotor speed
 *
 */
void computeSpeed(){
	dTheta = theta - thetaOld;
	dTheta = mod(dTheta);

	if(dTheta < 100){
		/*
		 * Need to fix this
		 *
		 * */
		speed = (uint16_t)(0.999*speed + dTheta*0.486);
	}
}


/**
 * This function computes rotor position
 *
 * @param uint8_t x in range 0 to 128
 * @return uint8_t arcsin(x) range 0 to 90
 */
void computePosition(){
	// Compute alpha-beta BEMFS
	clarkeTransform(Ea,Eb,Ec, &Ealbt);

	// Compute theta
	theta = arctan2(Ealbt.beta,Ealbt.alpha);


	// Include error +/- 5 degrees
	//theta = theta - atan2Error[theta] + 5;

	// Limit theta value
	if(theta < 0){
		theta += 360;
	}

	if(theta > 360){
		theta = thetaOld;
	}

	// Compute rotor speed
	computeSpeed();

	thetaOld = theta;
}

/**
 * This function computes 2 phase currents and 2 BEMFs
 *
 */
void measureADC(){
	HAL_ADC_Start_DMA(&hadc2, Eab, 2);
	HAL_ADC_Start_DMA(&hadc1, Iab, 2);

	// Compute abc currents
	Ia = ((short)Iab[0] - 1945);
	Ib = ((short)Iab[1] - 1923);
	Ic = -(Ia+Ib);


	// Compute abc BEMFs
	Ea = ((short)Eab[0]-1885);
	Eb =  (short)Eab[1]-1745;
	Ec = -(Ea+Eb);

	// Compute rotor position
	computePosition();
}


/**
 * This function sine cos values for given theta used in park transform
 *
 */
void computeSinCos(){
	sin000 = sin2(theta);
	cos000 = cos2(theta);
	sin120 = sin2(theta+120);
	sin240 = sin2(theta+240);
	cos120 = cos2(theta+120);
	cos240 = cos2(theta+240);
}
