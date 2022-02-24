#ifndef MPC_communication_dec
	#define MPC_communication_dec 1

	UART_HandleTypeDef huart2;

	/**
	 * This function transfers data over UART
	 *
	 */
	void transferUART();


	/**
	 * This function initializes UART DMA receive
	 *
	 */
	void receiveUART();

	/**
	 * This function is a handler for UART received data
	 *
	 */
	void handleRxCommands();
#endif
