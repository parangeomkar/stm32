#include "main.h"
#include "MPC_math.h"
#include "MPC_core.h"
#include "MPC_PWM.h"
#include "MPC_feedback.h"
#include "MPC_communication.h"

// Variables used by transferUART()
uint8_t txData[6];
uint8_t comCode[7],startTx,testEnd;
uint8_t c = 0;
/**
 * This function transfers data over UART
 *
 */
int16_t arr[10000], idx;
uint32_t err,asdsda;
void transferUART(){
	if(startTx){
			txData[0] = ((uint16_t)(speed + 30000)) & 0xff;
			txData[1] = (((uint16_t)(speed + 30000)) >> 8) & 0xff;
			HAL_UART_Transmit(&huart2, txData,2,10);
//			asdsda++;
//			if(asdsda > 5000){
//				speedReq = 1500;
//			}
//		err += error;
//		if(testEnd){
//			txData[0] = (err) & 0xff;
//			txData[1] = (err >> 8) & 0xff;
//			txData[2] = (err >> 16) & 0xff;
//			txData[3] = (err >> 24) & 0xff;
//
//			HAL_UART_Transmit(&huart2, txData,4,10);
//			startTx = 0;
//			testEnd = 0;
//			err = 0;
//		}
	}
}

/**
 * This function initializes UART DMA receive
 *
 */
void receiveUART(){
	HAL_UART_Receive_DMA(&huart2, comCode, 3);
}


/**
 * This function is a handler for received data
 *
 */
void handleRxCommands(){
	if(comCode[0] == 101){
		startTx = 1;
	} else if(comCode[0] == 102){
//		testEnd = 1;
		startTx = 0;
	} else if(comCode[0] == 103){
		stopMotor();
	} else if(comCode[0] == 104){
		startMotor();
	} else if(comCode[0] == 105){
		sigma = comCode[1] + 256*comCode[2];
	} else if(comCode[0] == 106){
		delta = comCode[1] + 256*comCode[2];
	} else if(comCode[0] == 107){
		Kp = comCode[1] + 256*comCode[2];
	} else if(comCode[0] == 108){
		Ki = comCode[1] + 256*comCode[2];
	} else if(comCode[0] == 109){
		speedReq = comCode[1] + 256*comCode[2];
	} else if(comCode[0] == 110){
		HAL_NVIC_SystemReset();
	}
}
