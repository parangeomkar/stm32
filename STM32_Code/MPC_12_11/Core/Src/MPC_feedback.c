#include "main.h"
#include "MPC_math.h"
#include "MPC_feedback.h"


/**
 * This function computes average rotor speed
 *
 */
uint16_t clockTicks = 1;
void computeSpeed(){
	dTheta = thetaOld - theta;

	// check if angle roll over has occurred
	if(dTheta > 250){
		if(clockTicks > 0){
			speed = 150000/clockTicks; 	// speed = 60/(Ts*p*clockTicks) = 150000/clockTicks --> p = 4 || Ts = 1 clockTick = 100uS
		}
		clockTicks = 0;
		PIController();
	} else {
		clockTicks++;
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
