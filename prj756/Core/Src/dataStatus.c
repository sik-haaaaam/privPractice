#include "dataStatus.h"
<<<<<<< HEAD
#include "string.h"


EVENT_ID mEventData[256]; // m 붙여준 이유 -> member 변수
BYTE mEVENTDataCnt;


void initDataStatus()
{
	memset(mEventData, EVENT_NULL, sizeof(EVENT_ID)*256);  //초기화(메모리세팅)
	mEventDataCnt = 0;
}

// val:input, mEventData:output
void setEventData(EVENT_ID val)
{
	mEventData[mEventDataCnt++] = val;
}

EVENT_ID getEventData()
{
	EVENT_ID returnVal = EVENT_NULL; //초기화(init), val : local 변수
	returnVal = mEventData[0];
	memcpy(mEventData[0], &mEventData[1], sizeof(EVENT_ID)*255); // 첫번째꺼 썼으니까 두번째부터 마지막까지 한칸 앞으로 이동

	mEventData[255] = EVENT_NULL;
	mEventDataCnt--;
	return returnVal;
=======

void initDataStatus()
{

>>>>>>> ca186cf2a1434c7b84e07f79659d574897666919
}
