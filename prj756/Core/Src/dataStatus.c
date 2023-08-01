#include "dataStatus.h"
#include "string.h"


EVENT_ID mEventData[256]; // m 붙여준 이유 -> member 변수
BYTE mEVENTDataCnt;
BYTE mUartData;

void initDataStatus()
{
	memset(mEventData, EVENT_NULL, sizeof(EVENT_ID)*256);  //초기화(메모리세팅)
	mEVENTDataCnt = 0;
}


void setEventData(EVENT_ID val)
{
	mEventData[mEVENTDataCnt++] = val;
	// val:input, mEventData:output
}

EVENT_ID getEventData()
{
	EVENT_ID returnVal = EVENT_NULL; //초기화(init), val : local 변수

	if(mEVENTDataCnt == 0)
		return returnVal;

	returnVal = mEventData[0];
	memcpy(&mEventData[0], &mEventData[1], sizeof(EVENT_ID)*255); // 첫번째꺼 썼으니까 두번째부터 마지막까지 한칸 앞으로 이동

	mEventData[255] = EVENT_NULL;
	mEVENTDataCnt--;
	return returnVal;
}
