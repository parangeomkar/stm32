#include "main.h"
#include "MPC_math.h"
#include "MPC_PWM.h"
#include "MPC_feedback.h"
#include "MPC_communication.h"

// Variables used by transferUART()
uint8_t txData[9];

/**
 * This function transfers data over UART
 *
 */
void transferUART(){
	HAL_UART_Transmit(&huart2, txData, 8, 10);

	txData[0] = 123;
	txData[1] = (uint16_t)(Ia+2000) & 0xff;
	txData[2] = ((uint16_t)(Ia+2000) >> 8) & 0xff;
	txData[3] = (uint16_t)(Ib+2000) & 0xff;
	txData[4] = ((uint16_t)(Ib+2000) >> 8) & 0xff;
	txData[5] = (uint16_t)(Idq.q*100+100) & 0xff;
	txData[6] = (uint16_t)(theta+2000) & 0xff;
	txData[7] = ((uint16_t)(theta+2000) >> 8) & 0xff;
}
