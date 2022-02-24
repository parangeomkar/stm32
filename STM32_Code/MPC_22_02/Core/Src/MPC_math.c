#include "main.h"
#include "MPC_math.h"
#include "MPC_feedback.h"

const uint16_t sinTable[] = {0,9,18,27,36,45,54,62,71,80,89,98,106,115,124,133,141,150,158,167,175,183,192,200,208,216,224,232,240,248,256,264,271,279,286,294,301,308,315,322,329,336,343,349,356,362,368,374,380,386,392,398,403,409,414,419,424,429,434,439,443,448,452,456,460,464,468,471,475,478,481,484,487,490,492,495,497,499,501,503,504,506,507,508,509,510,511,511,512,512,512};
const uint8_t asinTable[] = {0,0,1,1,2,2,3,3,4,4,4,5,5,6,6,7,7,8,8,9,9,9,10,10,11,11,12,12,13,13,14,14,14,15,15,16,16,17,17,18,18,19,19,20,20,21,21,22,22,23,23,23,24,24,25,25,26,26,27,27,28,28,29,29,30,31,31,32,32,33,33,34,34,35,35,36,36,37,38,38,39,39,40,40,41,42,42,43,43,44,45,45,46,47,47,48,49,49,50,51,51,52,53,54,54,55,56,57,58,58,59,60,61,62,63,64,65,66,67,68,70,71,72,74,76,78,80,83,90};
const uint8_t atan2Error[360] =  {5, 4, 6, 5, 6, 7, 5, 7, 8, 8, 5, 8, 9, 9, 9, 5, 9,10,10,10,10,10,10,10,10,10,10,10,10,10, 9, 9, 9, 9, 9, 9, 8, 8, 8, 8, 7, 7, 7, 7, 6, 6, 6, 5, 5, 5, 5, 4, 4, 4, 4, 3, 3, 3, 3, 3, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 5, 2, 3, 3, 5, 3, 4, 4, 5, 4, 5, 5, 5, 6, 5, 6, 7, 5, 7, 8, 8, 8, 5, 8, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 8, 8, 8, 8, 8, 7, 7, 7, 7, 6, 6, 6, 5, 5, 5, 5, 4, 4, 4, 4, 3, 3, 3, 3, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 5, 1, 2, 2, 2, 2, 5, 2, 3, 5, 3, 4, 4, 5, 4, 5, 5, 6, 6, 5, 6, 7, 5, 7, 8, 8, 8, 8, 5, 8, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 8, 8, 8, 8, 8, 8, 7, 7, 7, 7, 6, 6, 6, 6, 5, 5, 5, 5, 4, 4, 4, 3, 3, 3, 3, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 5, 1, 2, 2, 2, 5, 2, 3, 5, 3, 4, 5, 4, 5, 5, 5, 6, 6, 5, 6, 7, 7, 5, 7, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 7, 7, 7, 7, 7, 6, 6, 6, 6, 5, 5, 5, 5, 4, 4, 4, 3, 3, 3, 3, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 5, 0, 1, 1, 1, 5, 1, 2, 2, 5, 2, 3, 5, 3, 5};

// Variables used by arctan2
float c1 = 0.7854;
float c2 = 2.3562;
float r = 0;
float abs_y = 0;
float angle = 0;
uint16_t tempTheta = 0;

/**
 * This function limits the range of thetaElec between 0 and 360 degrees
 *
 * @param short thetaElec
 * @return short angle between 0 to 360 degrees
 */
short limitTheta(short thetaElec){
	if(thetaElec < 0){
		return ((360+thetaElec) - 360*(1+(thetaElec/360)));
	} else {
		return (thetaElec - 360*(thetaElec/360));
	}
}


/**
 * This function computes sin(thetaElec) using LUT
 *
 * @param short thetaElec in degrees
 * @return short sin(x) range -512 to +512
 */
short sin2(short thetaElec){
  thetaElec = limitTheta(thetaElec);

  if(thetaElec <= 90){
    return sinTable[thetaElec];
  } else if(thetaElec > 90 && thetaElec <=180){
    return sinTable[180 - thetaElec];
  } else if(thetaElec > 180 && thetaElec <= 270){
    return -sinTable[thetaElec - 180];
  } else {
    return -sinTable[360 - thetaElec];
  }
}


/**
 * This function computes cos(thetaElec) using LUT
 *
 * @param short thetaElec in degrees
 * @return short cos(x) range -512 to +512
 */
short cos2(short thetaElec){
  return sin2(thetaElec+90);
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
	   tempTheta = (uint16_t)(180-(angle*57));
   } else {
	   tempTheta = (uint16_t)(180+(angle*57));
   }

   return (tempTheta - atan2Error[tempTheta] + 5);
}


/**
 * This function computes Park transform
 *
 */
void parkTransform(short a, short b, short c, struct directQuad *Xdq){
	Xdq->d = (float)(sin000*a + sin240*b + sin120*c)/768; // (2/3)*(1/512) = 1/768
	Xdq->q = (float)(cos000*a + cos240*b + cos120*c)/768;
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
 * This function computes sine & cosine values for given thetaElec to be used in park transform
 *
 */
void computeSinCos(){
	sin000 = sin2(thetaElec);
	cos000 = cos2(thetaElec);
	sin120 = sin2(thetaElec+120);
	sin240 = sin2(thetaElec+240);
	cos120 = cos2(thetaElec+120);
	cos240 = cos2(thetaElec+240);
}
