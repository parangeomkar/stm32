#include "main.h"
#include "MPC_math.h"
#include "MPC_core.h"
#include "MPC_feedback.h"


/**
 * This function computes instantaneous rotor speed
 *
 *	******** Needs Fix ***********
 */
void computeSpeed(){
	dTheta = tempTheta - thetaOld;

	if(dTheta > 0 && dTheta < 10){
		speed = (0.999*speed + (dTheta*1250/3000));
		PIController();
	}

	thetaOld = tempTheta;
}



/**
 * This function computes average rotor speed
 *
 */
uint16_t clockTicks = 1;
void computeAverageSpeed(){
	dTheta = thetaOld - theta;

	speed = theta;
	// check if angle roll over has occurred
//	if(dTheta > 200){
//		if(clockTicks > 0){
//			speed = theta; 	// speed = 60/(Ts*p*clockTicks) = 150000/clockTicks --> p = 4 || Ts = 1 clockTick = 100uS
//			//PIController();
//		}
//		clockTicks = 0;
//	} else {
//		clockTicks++;
//	}

	thetaOld = theta;
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

	// Compute rotor speed
	computeAverageSpeed();
}

/**
 * This function computes 2 phase currents and 2 BEMFs
 *
 */
void measureADC(){
	HAL_ADC_Start_DMA(&hadc2, Eab, 2);
	HAL_ADC_Start_DMA(&hadc1, Iab, 2);

	// Compute abc currents
	Ia = ((short)Iab[0] - 1950);
	Ib = ((short)Iab[1] - 1916);
	Ic = -(Ia+Ib);


	// Compute abc BEMFs
	Ea = ((short)Eab[0]-1880);
	Eb =  (short)Eab[1]-1743;
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
