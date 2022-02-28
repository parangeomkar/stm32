#include "main.h"
#include "MPC_math.h"
#include "MPC_core.h"
#include "MPC_PWM.h"
#include "MPC_feedback.h"
#include "MPC_communication.h"

// Variables used by transferUART()
uint8_t txData[6];
uint8_t comCode[7],startTx;
uint8_t c = 0;
/**
 * This function transfers data over UART
 *
 */
void transferUART(){
	if(startTx){
		txData[0] = ((int16_t)(speed) + 10000) & 0xff;
		txData[1] = (((int16_t)(speed) + 10000) >> 8) & 0xff;

		txData[2] = ((int16_t)(IqTx) + 10000) & 0xff;
		txData[3] = (((int16_t)(IqTx) + 10000) >> 8) & 0xff;
//
//		txData[4] = ((int16_t)(Ib) + 10000) & 0xff;
//		txData[5] = (((int16_t)(Ib) + 10000) >> 8) & 0xff;
//
//		txData[6] = (optimalVector) & 0xff;

		HAL_UART_Transmit(&huart2, txData, 4, 10);
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
		startTx = 0;
	} else if(comCode[0] == 103){
		run = 0;
	} else if(comCode[0] == 104){
		run = 1;
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
	}
}
