#ifndef __DATASTATUS_H__  //만약 DATASTATUS가 선언 안되어 있다면 밑을 실행해라
#define __DATASTATUS_H__

typedef unsigned char BYTE; //signed -127~128, unsigned 0~255

typedef enum
{
	EVENT_NULL = 0,
	EVENT_100ms,
	EVENT_1s,
	EVENT_2s,
	EVENT_UART2_RX,
	EVENT_UART5_RX
}EVENT_ID;
// 자료형 - 집합

typedef enum
{
	eUart0,
	eUart1,
	eUart2,
	eUart3,
	eUart4,
	eUart5,
	eUart6,
	eUart7,
	eUart8
}eUart_ID;

void initDataStatus();

void setEventData(EVENT_ID val);
EVENT_ID getEventData();



#endif // __DATASTATUS_H__
