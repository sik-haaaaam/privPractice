#ifndef __DATASTATUS_H__  //만약 DATASTATUS가 선언 안되어 있다면 밑을 실행해라
#define __DATASTATUS_H__

<<<<<<< HEAD
typedef unsigned char BYTE; //signed -127~128, unsigned 0~255

typedef enum
{
	EVENT_NULL = 0,
	EVENT_100ms,
	EVENT_1s,
	EVENT_2s
}EVENT_ID;
// 자료형 - 집합


void initDataStatus();

void setEventData(EVENT_ID val);
EVENT_ID getEventData();

=======
void initDataStatus();

>>>>>>> ca186cf2a1434c7b84e07f79659d574897666919
#endif // __DATASTATUS_H__
