#ifndef MPC_math_dec
	#define MPC_math_dec 1

	#define min(x,y) (((x) < (y)) ? (x) : (y))
	#define max(x,y) (((x) > (y)) ? (x) : (y))
	#define mod(x) (((x) > 0) ? (x) : (-x))
	#define square(x) (x*x)

	struct alphaBeta {
		float alpha,beta;
	};

	struct  directQuad {
		float d,q;
	};

	short sin000,sin120,sin240,cos000,cos120,cos240,id_pred,iq_pred;


	/**
	 * This function limits the range of theta between 0 and 360 degrees
	 *
	 * @param short theta
	 * @return short angle between 0 to 360 degrees
	 */
	short limitTheta(short theta);


	/**
	 * This function computes sin(theta) using LUT
	 *
	 * @param short theta in degrees
	 * @return short sin(x) range -512 to +512
	 */
	short sin2(short theta);


	/**
	 * This function computes cos(theta) using LUT
	 *
	 * @param short theta in degrees
	 * @return short cos(x) range -512 to +512
	 */
	short cos2(short theta);


	/**
	 * This function computes arcsin(x) using LUT
	 *
	 * @param uint8_t x in range 0 to 128
	 * @return uint8_t arcsin(x) range 0 to 90
	 */
	uint8_t asin2(uint8_t value);


	/**
	 * This function computes arctan2(x)
	 *
	 * @param float x, y
	 * @return uint18_t arctan2(x) range 0 to 360 degrees
	 */
	uint16_t arctan2(float y,float x);


	/**
	 * This function computes Park transform
	 *
	 */
	void parkTransform(short a, short b, short c, struct directQuad *Xdq);


	/**
	 * This function computes Clarke transform
	 *
	 */
	void inverseParkTransform(float d, float q, struct alphaBeta *Xalbt);


	/**
	 * This function computes Clarke transform
	 *
	 */
	void clarkeTransform(float a, float b, float c, struct alphaBeta *Xalbt);
#endif
