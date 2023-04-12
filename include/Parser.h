#ifndef __PARSER_H__
#define __PARSER_H__

#include "stm32f10x.h"
#include "config.h"
#include "utils.h"
#include "uart.h"

union hexToFloat
{
	char c[4];
	float fval;
	int ival;
};
void Parser(char* receivedBytes, PositionsAngles_s* UAVdata, simpleLogger* log);
void requestData(uartHandler *huart1);
void sendAck(uartHandler *huart1);

#endif
