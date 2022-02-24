#include "main.h"
#include "MPC_math.h"
#include "MPC_feedback.h"


/**
 * This function computes rotor position
 *
 */
int16_t speedTemp, sum, speedArr[20];
uint16_t j, k, e;
uint16_t x = 1;
void computePositionWithEncoder(){
	thetaElec = 90 + 360*TIM3->CNT/300;


	if(thetaElec >= 360){
		thetaElec -= 360;
	}

	if(thetaElec != thetaElecOld){
		dTheta = thetaElec - thetaElecOld;
		if(dTheta > 300){
			dTheta -= 360;
		} else if (dTheta < -300){
			dTheta += 360;
		}

		thetaElecOld = thetaElec;

		speedTemp = dTheta*833/x;

		x = 1;

//		if(k>19){
//			sum += (speedTemp - speedArr[e]);
//			speedArr[e] = speedTemp;
//			e++;
//
//			if(e>19){
//				e = 0;
//			}
//		} else {
//			sum += speedTemp;
//			speedArr[k] = speedTemp;
//			k++;
//		}

		speed = (98*speed + speedTemp)/100;
		wr = (float)speed*1047/10000;

		if(j>9){
			SpeedPIController();
		} else {
			j++;
		}

	} else {
		x++;
	}
}



/**
 * This function computes 2 phase currents and 2 BEMFs
 *
 */
void measureADC(){
	HAL_ADC_Start_DMA(&hadc1, Iabc, 4);

	// Compute abc currents
	Ia = -(((int16_t)Iabc[0] - 1951));
	Ib = -((int16_t)Iabc[1] - 1924);
	Ic = -((int16_t)Iabc[2] - 1955);
	Vbus = ((int16_t)Iabc[3]) + 1;

	// Compute rotor position
	computePositionWithEncoder();
//	fluxObserver();
}
