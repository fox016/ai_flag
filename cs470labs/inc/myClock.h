#include <sys/timeb.h>

class MyClock
{
public:
	static int getMilliCount()
	{
		timeb tb;
		ftime(&tb);
		int nCount = tb.millitm + (tb.time & 0xfffff) * 1000;
		return nCount;
	}

	static int getMilliSpan(int nTimeStart, int nTimeEnd)
	{
		int nSpan = nTimeEnd - nTimeStart;
		if(nSpan < 0)
			nSpan += 0x100000 * 1000;
		return nSpan;
	}
};
