#include "main.h"
#include "MPC_math.h"
#include "MPC_core.h"
#include "MPC_feedback.h"


/**
 * This function computes rotor position
 *
 */
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

		speedTemp = dTheta*672/x;

		x = 1;

		if(k>9){
			sum += (speedTemp - speedArr[e]);
			speedArr[e] = speedTemp;
			e++;

			if(e>9){
				e = 0;
			}
		} else {
			sum += speedTemp;
			speedArr[k] = speedTemp;
			k++;
		}

		speed = (2*speed + sum/10)/3;
		wr = (float)speed*1047/10000;

		if(speedPIrate>9){
			SpeedPIController();
			speedPIrate = 0;
		} else {
			speedPIrate++;
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
	Ia = -(((int16_t)Iabc[0])-1956)*103/100;
	Ib = -((int16_t)Iabc[1]-1916);
	Ic = -((int16_t)Iabc[2]-1955);
	Vbus = ((int16_t)Iabc[3]) + 1;

	if(startOCwatch && (Ia > 1000 || Ia < -1000
	  || Ib > 1000 || Ib < -1000
	  || Ic > 1000 || Ic < -1000)){
		stopMotor();
	}

	// Compute rotor position
	computePositionWithEncoder();
//	fluxObserver();
}
