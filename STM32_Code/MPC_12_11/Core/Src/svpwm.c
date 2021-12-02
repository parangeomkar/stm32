#include "main.h"
#include "arm_math.h"

#define min(x,y) (((x) < (y)) ? (x) : (y))
#define max(x,y) (((x) > (y)) ? (x) : (y))
#define mod(x) (((x) > 0) ? (x) : (-x))


UART_HandleTypeDef huart2;
TIM_HandleTypeDef htim1;
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;

uint8_t RX_data[2];
uint8_t TX_data[1];

uint16_t sinTable[] = {0,9,18,27,36,45,54,62,71,80,89,98,106,115,124,133,141,150,158,167,175,183,192,200,208,216,224,232,240,248,256,264,271,279,286,294,301,308,315,322,329,336,343,349,356,362,368,374,380,386,392,398,403,409,414,419,424,429,434,439,443,448,452,456,460,464,468,471,475,478,481,484,487,490,492,495,497,499,501,503,504,506,507,508,509,510,511,511,512,512};
uint8_t asinTable[] = {0,0,1,1,2,2,3,3,4,4,4,5,5,6,6,7,7,8,8,9,9,9,10,10,11,11,12,12,13,13,14,14,14,15,15,16,16,17,17,18,18,19,19,20,20,21,21,22,22,23,23,23,24,24,25,25,26,26,27,27,28,28,29,29,30,31,31,32,32,33,33,34,34,35,35,36,36,37,38,38,39,39,40,40,41,42,42,43,43,44,45,45,46,47,47,48,49,49,50,51,51,52,53,54,54,55,56,57,58,58,59,60,61,62,63,64,65,66,67,68,70,71,72,74,76,78,80,83,90};


short limitTheta(short theta){
	return (theta - 360*(theta/360));
}


int sin2(short theta){
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


int cos2(uint16_t theta){
  return sin2(theta+90);
}

uint8_t asin2(uint8_t value){
	return asinTable[value];
}


float V = 0.6;
int T1=0;
int T2=0;
int T0=0;

uint16_t Ta=0;
uint16_t Tb=0;
uint16_t Tc=0;
uint16_t Ts=512;

short theta;
float dTheta;
short thetaOld;
uint16_t speed;

uint16_t wt = 0;
uint8_t n = 0;
uint8_t run = 1;


uint32_t Eab[2],Iab[2],ea,eb;
uint32_t Err[2];
uint8_t pData[7];

short E1,E2,E3;

float32_t Valpha = 0;
float32_t Vbeta = 0;
float32_t Vsqrt;


void transferUART(){
	HAL_UART_Transmit(&huart2, pData, 3, 10);

	pData[0] = 123;
	pData[1] = (uint16_t)(wt) & 0xff;
	pData[2] = ((uint16_t)(wt) >> 8) & 0xff;
//	pData[3] = (uint16_t)(wt) & 0xff;
//	pData[4] = ((uint16_t)(wt) >> 8) & 0xff;
}

float c1 = 0.7854;
float c2 = 2.3562;
float r = 0;
float abs_y = 0;
float angle = 0;

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

void measureTheta(){
	E1 = ((short)Eab[0]-1880)*81/100;
	E2 =  (short)Eab[1]-1768;
	E3 = -(E1+E2);

	// Estimate Valpha and Vbeta
   	Valpha = (E1*2/3) - ((E2+E3)/3);
	Vbeta = (E2-E3)*250/433;

	theta = arctan2(Vbeta,Valpha);

	if(theta < 0){
		theta += 360;
	}

	if(theta > 360){
		theta = thetaOld;
	}

	dTheta = theta - thetaOld;
	dTheta = mod(dTheta);

	if(dTheta < 200){
		speed = 0.999*speed + (dTheta)*0.486;
	} else {
		speed = 0.999*speed + (360-dTheta)*0.486;
	}

	thetaOld = theta;
}

void measureADC(){
	HAL_ADC_Start_DMA(&hadc2, Eab, 2);
	HAL_ADC_Start_DMA(&hadc1, Iab, 2);
	measureTheta();
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
//	transferUART();
}




uint8_t startPID = 0;
short temp;
void SVPWM(){
//	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 1);
	if(run == 1){

		//transferUART();
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

////			PIController();
////			V = D;
//
//			temp = (short)theta+3;
//
//			if(temp < 0){
//				wt = temp + 360;
//			} else if(temp > 360){
//				wt = temp - 360;
//			} else {
//				wt =  temp;
//			}
//
//		} else {
//		}

		if (V<0){
			V = 0;
		} else if(V > 0.95){
			V = 0.95;
		} else {
			V+=0.000001;
		}

		//transferUART();

	} else {
		TIM1->CCR1 = 0;
		TIM1->CCR2 = 0;
		TIM1->CCR3 = 0;
	}
//	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 0);
}

void incWt(){
	if(startPID){
		wt = 60*floor((theta+15)/60);
		if(TIM2->ARR > 3800){
			TIM2->ARR -= 1;
		}
	} else {
		wt+=60;
		if(wt >= 360){
			wt = 0;
			if(TIM2->ARR > 9000){
				startPID = 0;
				TIM2->ARR -= 10;
			} else {
				startPID = 1;
			}
		}
	}

	if(wt >= 310){
		wt = 0;
	}

	if(TIM2->ARR < TIM2->CNT){
		TIM2->CNT = 0;
	}
}

void startStop(){
	if(!run){
		run = 1;
	} else {
		run = 0;
	}
}
