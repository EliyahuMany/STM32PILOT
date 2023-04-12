#include "Parser.h"

hexToFloat conv;

void Parser(char* receivedBytes, PositionsAngles_s* UAVdata, simpleLogger* log)
{
	if ((receivedBytes[0] == 'R') && (receivedBytes[1] == 'R') && (receivedBytes[2] == 'E') && (receivedBytes[3] == 'F') && (receivedBytes[4] == ','))
	{
		// TODO: add crc check of the data4 byte
		uint8_t byteCounter = 1;
		float parsedValue = -999.0;
		bool indexFlag = true;
		uint8_t packetIndex = 0;
		for (int i=5;i<SIMULATION_MSG_LEN;i++)
		{
			if (byteCounter == 4) // after reading 4 bytes
			{
				conv.c[byteCounter-1] = receivedBytes[i];
				parsedValue = conv.fval;
				if (indexFlag)
					packetIndex = conv.ival;
				else
				{
					switch (packetIndex)
					{
						case 0:
							// Pitch deg
							if (parsedValue >= -90 && parsedValue <= 90)
							{
								UAVdata->pitch = parsedValue;
								log->pitch = UAVdata->pitch;
							}
							break;
						case 1:
							// Roll deg
							if (parsedValue >= -90 && parsedValue <= 90)
							{
								UAVdata->roll = parsedValue;
								log->roll = UAVdata->roll;
							}
							break;
						case 2:
							// Alt msl
							if (parsedValue >= -500 && parsedValue <= 20000)
							{	
								UAVdata->alt = parsedValue;
								log->alt = UAVdata->alt;
							}
							break;
						case 3:
							// throttle
							if (parsedValue >= -0.0 && parsedValue <= 1.0)
							{	
								UAVdata->throttle = parsedValue;
								log->throttle = UAVdata->throttle;
							}
							break;
						case 4:
							// heading
							if (parsedValue >= 0.0 && parsedValue <= 360.0)
							{	
								UAVdata->yaw = parsedValue;
								log->heading = parsedValue;
							}
							break;
						case 5:
							// latitude
							if (parsedValue >= -90.0 && parsedValue <= 90.0)
							{	
								UAVdata->gps.lat = parsedValue;
							}
							break;
						case 6:
							// longitude
							if (parsedValue >= -180.0 && parsedValue <= 180.0)
							{	
								UAVdata->gps.lon = parsedValue;
							}
							break;
						case 7:
							// P
							if (parsedValue >= -180.0 && parsedValue <= 180.0)
							{	
								UAVdata->rollRate = parsedValue;
							}
							break;
						case 8:
							// Q
							if (parsedValue >= -180.0 && parsedValue <= 180.0)
							{	
								UAVdata->pitchRate = parsedValue;
							}
							break;
						case 9:
							// R
							if (parsedValue >= -180.0 && parsedValue <= 180.0)
							{	
								UAVdata->yawRate = parsedValue;
							}
							break;
						case 10:
							// true airspeed
							if (parsedValue >= -180.0 && parsedValue <= 180.0)
							{	
								UAVdata->speed = parsedValue;
							}
							break;
					}
				}
				byteCounter = 1;
				indexFlag = !indexFlag;
			}
			else
			{
				conv.c[byteCounter-1] = receivedBytes[i]; // Little Endian
				byteCounter += 1;
			}
		}
	}
}

void requestData(uartHandler *huart1)
{
	huart1->write('S');
	huart1->write('e');
	huart1->write('n');
	huart1->write('d');
	huart1->write('P');
	huart1->write('\n');
}
void sendAck(uartHandler *huart1)
{
	huart1->write('A');
	huart1->write('c');
	huart1->write('k');
	huart1->write('\n');
}

