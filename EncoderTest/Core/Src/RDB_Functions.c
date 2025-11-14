/*
 * RDB_Functions.c
 *
 *  Created on: Nov 7, 2025
 *      Author: Sam Neumann
 */


#include "RDB_depth_digits_36x58.h"
#include "ST7565.h"
#include"string.h"

const unsigned char *depthDigitArray[10]= {BM0_36x58_block, BM1_36x58_block, BM2_36x58_block, BM3_36x58_block, BM4_36x58_block, BM5_36x58_block, BM6_36x58_block,BM7_36x58_block,BM8_36x58_block,BM9_36x58_block};

int number;
//test function to increments all values at the same time.
void countTestRDBlcd(){

	for(int i=0; i<10; i++){
		number=i;

		updateDisplay();
		HAL_Delay(300);
	ST7565_fillrect(5, 5, 36, 58, WHITE);  // left digit rect cover
   	ST7565_fillrect(40, 5, 36, 58, WHITE); // middle rect cover
	ST7565_fillrect(75,5, 36, 58, WHITE);  // right rect cover

	ST7565_drawbitmapNew(5,15, depthDigitArray[i],  36, 58, 1); //left digit
    ST7565_drawbitmapNew(40, 15, depthDigitArray[i], 36, 58, 1); //middle digit
    ST7565_drawbitmapNew(75, 15, depthDigitArray[i], 36, 58, 1); //right digit


	}

}

//function that takes calculated depth value integer and parses it for display
// needs to hide leadiing zeros
void parseDepthVal(int depthValue){

	int ones = depthValue % 10;
	int tens = (depthValue/10) %10;
	int hundred = depthValue/100;


	ST7565_fillrect(5, 0, 36, 48, WHITE);  // left digit rect cover
   	ST7565_fillrect(40, 0, 36, 48, WHITE); // middle rect cover
	ST7565_fillrect(75,0, 36, 48, WHITE);  // right rect cover

	ST7565_drawbitmapNew(5,47, depthDigitArray[hundred],  36, 48, 1); //left digit
    ST7565_drawbitmapNew(40, 47, depthDigitArray[tens], 36, 58, 1); //middle digit
    ST7565_drawbitmapNew(75, 47, depthDigitArray[ones], 36, 58, 1); //right digit
   //HAL_Delay(100);
  	ST7565_drawstring_anywhere(0, 7, "DEPTH"); // x=pixel,y=page
  	ST7565_drawstring_anywhere(115, 0, "ft");  //
    updateDisplay();

}





