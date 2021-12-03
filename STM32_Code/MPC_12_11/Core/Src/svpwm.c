#include "main.h"
#include "arm_math.h"

#define min(x,y) (((x) < (y)) ? (x) : (y))
#define max(x,y) (((x) > (y)) ? (x) : (y))
#define mod(x) (((x) > 0) ? (x) : (-x))
#define square(x) (x*x)


UART_HandleTypeDef huart2;
TIM_HandleTypeDef htim1;
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

uint16_t sinTable[] = {0,9,18,27,36,45,54,62,71,80,89,98,106,115,124,133,141,150,158,167,175,183,192,200,208,216,224,232,240,248,256,264,271,279,286,294,301,308,315,322,329,336,343,349,356,362,368,374,380,386,392,398,403,409,414,419,424,429,434,439,443,448,452,456,460,464,468,471,475,478,481,484,487,490,492,495,497,499,501,503,504,506,507,508,509,510,511,511,512,512};
uint8_t asinTable[] = {0,0,1,1,2,2,3,3,4,4,4,5,5,6,6,7,7,8,8,9,9,9,10,10,11,11,12,12,13,13,14,14,14,15,15,16,16,17,17,18,18,19,19,20,20,21,21,22,22,23,23,23,24,24,25,25,26,26,27,27,28,28,29,29,30,31,31,32,32,33,33,34,34,35,35,36,36,37,38,38,39,39,40,40,41,42,42,43,43,44,45,45,46,47,47,48,49,49,50,51,51,52,53,54,54,55,56,57,58,58,59,60,61,62,63,64,65,66,67,68,70,71,72,74,76,78,80,83,90};


float V = 0.7;

// Variables used by SVPWM()
int T1 = 0;
int T2 = 0;
int T0 = 0;
uint16_t Ta = 0;
uint16_t Tb = 0;
uint16_t Tc = 0;
uint16_t Ts = 512;
uint16_t wt = 0;
uint8_t n = 0;
uint8_t run = 1;

// Variables used by executeSetAlgorithm()
uint8_t isOpenLoopComplete = 0;


// Variables used by computePosition()
short theta;
float dTheta;
short thetaOld;
short E1,E2,E3;
uint16_t speed;


// Variables used by measureADC()
uint32_t Eab[2],Iab[2];

// Variables used by transferUART()
uint8_t txData[7];

// Variables used by arctan2
float c1 = 0.7854;
float c2 = 2.3562;
float r = 0;
float abs_y = 0;
float angle = 0;


/**
 * This function implements model predictive control (MPC)
 *
 */
uint8_t states[8][3] = {{0,0,0},
				{1,0,0},
				{1,1,0},
				{0,1,0},
				{0,1,1},
				{0,0,1},
				{1,0,1},
				{1,1,1}};

struct alphaBeta {
	short alpha,beta;
};

struct  directQuad {
	short d,q;
};


struct alphaBeta Ealbt,Valbt,Ialbt,IalbtReq;
struct directQuad Idq;

short sin000,sin120,sin240,cos000,cos120,cos240,id_pred,iq_pred;
short Vdc = 12;
uint8_t i,j;

/**
 * This function limits the range of theta between 0 and 360 degrees
 *
 * @param short theta
 * @return short angle between 0 to 360 degrees
 */
short limitTheta(short theta){
	if(theta < 0){
		return ((360+theta) - 360*(1+(theta/360)));
	} else {
		return (theta - 360*(theta/360));
	}
}


/**
 * This function computes sin(theta) using LUT
 *
 * @param short theta in degrees
 * @return short sin(x) range -512 to +512
 */
short sin2(short theta){
  theta = limitTheta(theta);

  if(theta <= 90){
    return sinTable[theta];
  } else if(theta > 90 && theta <=180){
    return sinTable[180 - theta];
  } else if(theta > 180 && theta <= 270){
    return -sinTable[theta - 180];
  } else {
    return -sinTable[360 - theta];
  }
}


/**
 * This function computes cos(theta) using LUT
 *
 * @param short theta in degrees
 * @return short cos(x) range -512 to +512
 */
short cos2(short theta){
  return sin2(theta+90);
}


/**
 * This function computes arcsin(x) using LUT
 *
 * @param uint8_t x in range 0 to 128
 * @return uint8_t arcsin(x) range 0 to 90
 */
uint8_t asin2(uint8_t value){
	return asinTable[value];
}



/**
 * This function computes arctan2(x)
 *
 * @param float x, y
 * @return uint18_t arctan2(x) range 0 to 360 degrees
 */
uint16_t arctan2(float y,float x){
   abs_y = mod(y)+0.0000001;

   if (x>=0){
      r = (x - abs_y)/(x + abs_y);
      angle = (1-r)*c1;
   } else {
      r = (x + abs_y) / (abs_y - x);
      angle = c2 - (r*c1);
   }

   if (y < 0){
	   return((uint16_t)(180-(angle*57)));
   } else {
	   return((uint16_t)(180+(angle*57)));
   }
}



/**
 * This function sine cos values for given theta used in park transform
 *
 */
void computeSinCos(){
	sin000 = sin2(theta);
	cos000 = cos2(theta);
//	sin120 = sin2(theta+120);
//	sin240 = sin2(theta+240);
//	cos120 = cos2(theta+120);
//	cos240 = cos2(theta+240);
}


/**
 * This function computes Park transform
 *
 */
void parkTransform(short a, short b, short c, struct directQuad *Xdq){
	Xdq->d = (sin000*a + sin240*b + sin120*c)*Vdc*2/3;
	Xdq->q = (cos000*a + cos240*b + cos240*c)*Vdc*2/3;
}

/**
 * This function computes Clarke transform
 *
 */
void inverseParkTransform(float d, float q, struct alphaBeta *Xalbt){
	Xalbt->alpha = d*cos000 - q*sin000;
	Xalbt->beta = d*sin000 + q*cos000;
}


/**
 * This function computes Clarke transform
 *
 */
void clarkeTransform(float a, float b, float c, struct alphaBeta *Xalbt){
	Xalbt->alpha = (a*2/3) - ((b+c)/3);
	Xalbt->beta = (b-c)*250/433;
}


/**
 * This function transfers data over UART
 *
 */
void transferUART(){
	HAL_UART_Transmit(&huart2, txData, 3, 10);

	txData[0] = 123;
	txData[1] = (uint16_t)(wt) & 0xff;
	txData[2] = ((uint16_t)(wt) >> 8) & 0xff;
//	txData[3] = (uint16_t)(wt) & 0xff;
//	txData[4] = ((uint16_t)(wt) >> 8) & 0xff;
}

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
		speed = 0.999*speed + (dTheta)*0.486;
	}
}


/**
 * This function computes rotor position
 *
 * @param uint8_t x in range 0 to 128
 * @return uint8_t arcsin(x) range 0 to 90
 */
void computePosition(){

	// Compute abc BEMFs
	E1 = ((short)Eab[0]-1880)*81/100;
	E2 =  (short)Eab[1]-1768;
	E3 = -(E1+E2);

	// Compute alpha-beta BEMFS
	clarkeTransform(E1,E2,E3, &Ealbt);

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

	// Compute rotor position
	computePosition();
}




short error;
uint16_t s=0;
float Kterm;
float Iterm = 0;
float D = 0.7;
void PIController(){
	error = 1000 - speed;
	Kterm = (float)error/1000;
	Iterm += (float)error/5000;

	if(Iterm > 1){
		Iterm = 1;
	} else if(Iterm < -1){
		Iterm = -1;
	}
	D = (Kterm+Iterm)/2;
}

/**
 * This function computes SVPWM timings for TIM1
 *
 */
void SVPWM(){
	//	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 1);
	if(run == 1){
		n = floor(wt/60)+1;

		T1 = V*sin2(n*60 - wt);
		T2 = V*sin2(wt - ((n-1)*60));
		T0 = Ts - (T1+T2) + 5;

		if(wt >= 0 && wt < 60) {
			Ta = T1 + T2 + (T0/2);
			Tb = T2 + (T0/2);
			Tc = (T0/2);
		} else if(wt >= 60 && wt < 120) {
			Ta = T1 + (T0/2);
			Tb = T1 + T2 + (T0/2);
			Tc = (T0/2);
		} else if(wt >= 120 && wt < 180) {
			Ta = (T0/2);
			Tb = T1 + T2 + (T0/2);
			Tc = T2 + (T0/2);
		} else if(wt >= 180 && wt < 240) {
			Ta = (T0/2);
			Tb = T1 + (T0/2);
			Tc = T1 + T2 + (T0/2);
		} else if(wt >= 240 && wt < 300) {
			Ta = T2 + (T0/2);
			Tb = (T0/2);
			Tc = T1 + T2 + (T0/2);
		} else if(wt >= 300 && wt < 360) {
			Ta = T1 + T2 + (T0/2);
			Tb = (T0/2);
			Tc = T1 + (T0/2);
		} else {
			Ta = 0;
			Tb = 0;
			Tc = 0;
		}
		TIM1->CCR1 = Ta;
		TIM1->CCR2 = Tb;
		TIM1->CCR3 = Tc;
	} else {
		TIM1->CCR1 = 0;
		TIM1->CCR2 = 0;
		TIM1->CCR3 = 0;
	}
	//	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 0);
}


/**
 * This function checks and resets the Timer 2 count if it is greater than preset.
 *
 */
void checkTimer(){
	if(TIM2->ARR < TIM2->CNT){
		TIM2->CNT = 0;
	}
}


/**
 * This function implements open loop control
 *
 */
void openLoopControl(){
	wt+=60;
	if(wt >= 360){
		wt = 0;
		if(TIM2->ARR > 9000){
			TIM2->ARR -= 10;
		} else {
			isOpenLoopComplete = 1;
		}
	}
	checkTimer();
}


/**
 * This function implements 6-step control
 *
 */
void sixStepControl(){
	wt = 60*floor((theta)/60);
	if(TIM2->ARR > 4800){
		TIM2->ARR -= 1;
	}
	checkTimer();
}

short IalphaPred,IbetaPred;
int costTemp[7],cost,asd;
float Ia,Ib,Ic;
uint8_t optimalVector = 0;

void modelPredictiveControl(){
	computeSinCos();
	Ia = (float)Iab[0]/4096;
	Ib = (float)Iab[1]/4096;
	Ic = -(Ia+Ib);
	clarkeTransform(Ia,Ib,Ic,&Ialbt);
	inverseParkTransform(0,0.3,&IalbtReq);
	cost = 1000000;
	i = 0;
	for(i=0;i<7;i++){
		clarkeTransform(states[i][0],states[i][1],states[i][2],&Valbt);

		IalphaPred = (Valbt.alpha + 2*Ialbt.alpha - Ealbt.alpha)*421/1000;
		IbetaPred = (Valbt.beta + 2*Ialbt.beta - Ealbt.beta)*421/1000;

		costTemp[i] = square(mod(IalbtReq.alpha - IalphaPred)) + square(mod(IalbtReq.beta - IbetaPred));

		if(costTemp[i] < cost){
			optimalVector = i;
			cost = costTemp[i];
		}
	}

	wt = optimalVector*60;

	if(optimalVector == 0){
		V = 0;
	} else {
		V = 0.7;
	}

	checkTimer();
}

/**
 * This function executes set algorithms at regular interval
 * determined by TIM2
 *
 */
void executeSetAlgorithm(){
	if(isOpenLoopComplete){
//		sixStepControl();
		modelPredictiveControl();
	} else {
		openLoopControl();
	}
}


/**
 * This function controls start/stop of motor with Blue button
 *
 */
void startStop(){
	if(!run){
		run = 1;
	} else {
		run = 0;
	}
}
