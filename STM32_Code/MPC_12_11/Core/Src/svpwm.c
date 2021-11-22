#include "main.h"

#define min(x,y) (((x) < (y)) ? (x) : (y))
#define max(x,y) (((x) > (y)) ? (x) : (y))
#define mod(x) (((x) > 0) ? (x) : (-x))



UART_HandleTypeDef huart2;
TIM_HandleTypeDef htim3;
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;


uint8_t RX_data[1];
uint8_t TX_data[1];

uint16_t sinTable[] = {0,9,18,27,36,45,54,62,71,80,89,98,106,115,124,133,141,150,158,167,175,183,192,200,208,216,224,232,240,248,256,264,271,279,286,294,301,308,315,322,329,336,343,349,356,362,368,374,380,386,392,398,403,409,414,419,424,429,434,439,443,448,452,456,460,464,468,471,475,478,481,484,487,490,492,495,497,499,501,503,504,506,507,508,509,510,511,511,512,512};



uint16_t sine(uint16_t theta){
  theta = theta - 360*(theta/360);

  if(theta <= 90){
    return sinTable[theta]+512;
  } else if(theta > 90 && theta <=180){
    return sinTable[180 - theta] + 512;
  } else if(theta > 180 && theta <= 270){
    return 512 - sinTable[theta - 180];
  } else {
    return 512 - sinTable[360 - theta];
  }
}


int sin2(uint16_t theta){
  theta = theta - 360*(theta/360);

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
  theta = (theta+90) - 360*((theta+90)/360);

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



uint16_t triangle = 0;
uint8_t up = 1;
void incTriangle(){
	if(up == 1){
		triangle++;
		if(triangle == 1024){
		  up = 0;
		}
	} else {
		triangle--;
		if(triangle == 0){
		  up = 1;
		}
	}
}



float V = 0.6;
int T1=0;
int T2=0;
int T0=0;

uint16_t Ta=0;
uint16_t Tb=0;
uint16_t Tc=0;
uint16_t Ts=512;

uint16_t theta;

void measure();

uint16_t wt = 300;
uint8_t n=0;
uint8_t run = 0;




void SVPWM_PC(){
	TIM1->CCR1 = Ta;
	TIM1->CCR2 = Tb;
	TIM1->CCR3 = Tc;
}



uint32_t value[3];
uint32_t value1[1],value2[2];
uint32_t v1_err,v2_err;
uint8_t pData[9];

//void fastAtan(){
//	x = (float)beta/(float)alpha;
//	theta = 360 + 180*x - x*((abs(x) - 1)*(0.2447 + 0.0663*abs(x)));
//}


int v1,v2;

void measure(){
//	if(RX_data[0] == 202){
//		RX_data[0] = 0;
	HAL_UART_Transmit(&huart2, pData, 9, 10);
//	}

	HAL_ADC_Start_DMA(&hadc2, value2, 2);
	HAL_ADC_Start_DMA(&hadc1, value, 3);

	pData[0] = 123;
	pData[1] = value[0] & 0xff;
	pData[2] = (value[0] >> 8) & 0xff;
	pData[3] = value[1] & 0xff;
	pData[4] = (value[1] >> 8) & 0xff;
	pData[5] = value2[0] & 0xff;
	pData[6] = (value2[0] >> 8) & 0xff;
	pData[7] = value2[1] & 0xff;
	pData[8] = (value2[1] >> 8) & 0xff;

}

uint8_t startFlag = 0;

void startSending(){
	if(RX_data[0] == 201 && startFlag == 0){
		startFlag = 1;
		HAL_TIM_Base_Start_IT(&htim3);

		TX_data[0] = 101;
		HAL_UART_Transmit(&huart2, TX_data, 1, 10);
	}
}


void SVPWM(){
	if(run == 1){
		//	measure();
		n = floor(wt/60) + 1;

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

		wt++;
		if(wt == 360){
			wt = 0;
			if(TIM2->ARR > 250){
				TIM2->ARR -= 1;
			}
			if(TIM2->ARR < TIM2->CNT){
				TIM2->CNT = 0;
			}
		}
	} else {
		TIM1->CCR1 = 0;
		TIM1->CCR2 = 0;
		TIM1->CCR3 = 0;

	}
	send();
}

void startStop(){
	if(run == 0){
		run = 1;
	} else {
		run = 0;
	}
}
