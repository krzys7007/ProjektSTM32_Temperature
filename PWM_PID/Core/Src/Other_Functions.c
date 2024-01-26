/*
 * Other_Functions.c
 *
 *  Created on: Jan 24, 2024
 *      Author: krzys
 */
#include "Other_Functions.h"
#include "string.h"
#include "stdio.h"


int convertTempPx(float Temp){
	int px = (int)(Temp * (-2.5) + 110);
	return px;
}


