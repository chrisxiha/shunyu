#ifdef WIN32

	#include <Windows.h>
	#include <direct.h>
	#include <psapi.h>
	#include <io.h>

	#define R_OK 4
	#define W_OK 2
	#define X_OK 1
	#define F_OK 0

#elif defined LINUX 

	#include <unistd.h>
	#include <sys/stat.h>
	#include <sys/time.h>
	#include <sys/sysinfo.h>

#endif
#include <time.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string>
#include <iostream>
#include <thread>
#include <list>
#include <mutex>
#include <vector>
#include <chrono>
#include "tof_error.h"
#include "typedef.h"
#include "tof_dev_sdk.h"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
using namespace cv;


//
#define SAFE_DELETE(p) if(p){delete p; p=NULL;}
#define SAFE_DELETE_ARRY(p) if(p){delete [] p; p=NULL;}
#define SAFE_FREE(p) if(p){free(p); p=NULL;}
#define SAFE_CLOSE_FP(fp) if(fp){fclose(fp); fp=NULL;}



#ifdef WIN32
static int gettimeofday(struct timeval *tp, void *tzp)
{
	time_t clock;
	struct tm tm;
	SYSTEMTIME wtm;
	GetLocalTime(&wtm);
	tm.tm_year = wtm.wYear - 1900;
	tm.tm_mon = wtm.wMonth - 1;
	tm.tm_mday = wtm.wDay;
	tm.tm_hour = wtm.wHour;
	tm.tm_min = wtm.wMinute;
	tm.tm_sec = wtm.wSecond;
	tm.tm_isdst = -1;
	clock = mktime(&tm);
	tp->tv_sec = clock;
	tp->tv_usec = wtm.wMilliseconds * 1000;
	return (0);
}
#endif

static unsigned long long Utils_GetTickCount(void)
{
	unsigned long long tick = 0;

#ifdef WIN32
	//tick = GetTickCount();//实际精度只有15ms左右; 返回的是一个32位的无符号整数，Windows连续运行49.710天后，它将再次从零开始计时; 
	//tick = GetTickCount64();//返回一个64位的无符号整数。Windows连续运行5.8亿年后，其计时才会归零; 
	//tick = clock();//该程序从启动到函数调用占用CPU的时间, 是C/C++中的计时函数

	//struct timeval tv;
	//gettimeofday(&tv, 0);
	//tick = (tv.tv_sec * 1000 + tv.tv_usec / 1000);
		
	auto timePoint = std::chrono::steady_clock::now(); // std::chrono::time_point
	tick = std::chrono::duration_cast<std::chrono::milliseconds>(timePoint.time_since_epoch()).count();

#elif defined LINUX
	//struct timeval tv;
	//gettimeofday(&tv, 0);
	//tick = (tv.tv_sec * 1000 + tv.tv_usec/1000);
	
    struct timespec tv;
    clock_gettime(CLOCK_MONOTONIC, &tv);
	tick = (tv.tv_sec * 1000 + tv.tv_nsec/1000000);
	
#else
	printf("unknown platform in getting tick cnt, error!\n");
#endif // WIN32

	return tick;
}


static long Utils_GetFileLen(const char * filename)
{
	if (NULL == filename)
	{
		printf("NULL == filename\n");
		return 0;
	}

	FILE * fd = fopen(filename, "rb");
	if (NULL == fd)
	{
		printf("open file (%s) failed, errno=%d(%s).\n", filename, errno, strerror(errno));
		return 0;
	}

	fseek(fd, 0L, SEEK_END); /* 定位到文件末尾 */
	const long len = ftell(fd);
	fclose(fd);

	return len;

}

static void Utils_SaveBufToFile(void* pData, const unsigned int nDataLen, const char* pFile, const bool bAppend)
{
	if ((NULL == pData) || (0 >= nDataLen) || (NULL == pFile))
	{
		return;
	}

	FILE* fp = fopen(pFile, (bAppend ? "ab" : "wb"));
	if (NULL == fp)
	{
		printf("open file(%s) failed, error=%d(%s).\n", pFile, errno, strerror(errno));
		return;
	}

	fwrite(pData, 1, nDataLen, fp);
	fclose(fp);
}

template <class T>
T Utils_FindMaxValue(T* pData, const int nCnt)
{
	T max = pData[0];

	for (int i = 0; i < nCnt; i++)
	{
		if (max < pData[i])
		{
			max = pData[i];
		}
	}

	return max;
}

template <class T>
T Utils_FindMinValue(T* pData, const int nCnt)
{
	T min = pData[0];

	for (int i = 0; i < nCnt; i++)
	{
		if (min > pData[i])
		{
			min = pData[i];
		}
	}

	return min;
}

static int CaculateGrayPixelBytes(const GRAY_FORMAT format)
{
	int grayByte = 1;

	switch (format)
	{
	case GRAY_FORMAT_UINT8: grayByte = 1; break;
	case GRAY_FORMAT_UINT16: grayByte = 2; break;
	case GRAY_FORMAT_FLOAT: grayByte = 4; break;
	case GRAY_FORMAT_BGRD: grayByte = 4; break;
	default:break;
	}

	return grayByte;

}
static const char* StringGrayFormat(const GRAY_FORMAT format)
{
	const char* pStr = "UnknownGRAY";

	switch (format)
	{
	case GRAY_FORMAT_UINT8: pStr = "U8"; break;
	case GRAY_FORMAT_UINT16: pStr = "U16"; break;
	case GRAY_FORMAT_FLOAT: pStr = "FLOAT"; break;
	case GRAY_FORMAT_BGRD: pStr = "BGRD"; break;

	default: break;
	}

	return pStr;
}
static float CalCenterPointDataZAvg(PointData *pPointData, const UINT32 width, const UINT32 height)
{
	if (NULL == pPointData)
	{
		return 0;
	}

	const int start_h = (10<height) ? ((height / 2) - 5) : 0;
	const int end_h = (10<height) ? ((height / 2) + 5) : (height);
	const int start_w = (10<width) ? ((width / 2) - 5) : 0;
	const int end_w = (10<width) ? ((width / 2) + 5) : (width);


	float sum = 0.0;
	int cnt = 0;
	for (int h = start_h; h < end_h; h++)
	{
		PointData *pTmp = pPointData + h*width;
		for (int w = start_w; w < end_w; w++)
		{
			if (0.00001 < pTmp[w].z)
			{
				sum += pTmp[w].z;
				cnt++;
			}
		}
	}

	return ((0 < cnt) ? (sum / cnt) : 0);
}

static const char* StringColorFormat(const COLOR_FORMAT type)
{
	const char* pStr = "UnknownRGB";

	switch (type)
	{
	case COLOR_FORMAT_MJPG: pStr = "JPG"; break;

	case COLOR_FORMAT_H264: pStr = "H264"; break;

	case COLOR_FORMAT_YUV422: pStr = "YUV422"; break;
	case COLOR_FORMAT_YUYV: pStr = "YUYV"; break;
	case COLOR_FORMAT_I420: pStr = "I420"; break;
	case COLOR_FORMAT_YV12: pStr = "YV12"; break;
	case COLOR_FORMAT_NV12: pStr = "NV12"; break;
	case COLOR_FORMAT_NV21: pStr = "NV21"; break;

	case COLOR_FORMAT_BGR: pStr = "BGR"; break;
	case COLOR_FORMAT_RGB: pStr = "RGB"; break;
	case COLOR_FORMAT_BGRA: pStr = "BGRA"; break;
	case COLOR_FORMAT_RGBA: pStr = "RGBA"; break;

	default: break;
	}

	return pStr;
}
static const char* TofMode2Str(const TOF_MODE mode)
{
	const char* pStr = "Unknown";

	switch (mode)
	{
	case TOF_MODE_STERO_5FPS: pStr = "STERO_5FPS"; break;
	case TOF_MODE_STERO_10FPS: pStr = "STERO_10FPS"; break;
	case TOF_MODE_STERO_15FPS: pStr = "STERO_15FPS"; break;
	case TOF_MODE_STERO_30FPS: pStr = "STERO_30FPS"; break;
	case TOF_MODE_STERO_45FPS: pStr = "STERO_45FPS"; break;
	case TOF_MODE_STERO_60FPS: pStr = "STERO_60FPS"; break;

	case TOF_MODE_MONO_5FPS: pStr = "MONO_5FPS"; break;
	case TOF_MODE_MONO_10FPS: pStr = "MONO_10FPS"; break;
	case TOF_MODE_MONO_15FPS: pStr = "MONO_15FPS"; break;
	case TOF_MODE_MONO_30FPS: pStr = "MONO_30FPS"; break;
	case TOF_MODE_MONO_45FPS: pStr = "MONO_45FPS"; break;
	case TOF_MODE_MONO_60FPS: pStr = "MONO_60FPS"; break;

	case TOF_MODE_HDRZ_5FPS: pStr = "HDRZ_5FPS"; break;
	case TOF_MODE_HDRZ_10FPS: pStr = "HDRZ_10FPS"; break;
	case TOF_MODE_HDRZ_15FPS: pStr = "HDRZ_15FPS"; break;
	case TOF_MODE_HDRZ_30FPS: pStr = "HDRZ_30FPS"; break;
	case TOF_MODE_HDRZ_45FPS: pStr = "HDRZ_45FPS"; break;
	case TOF_MODE_HDRZ_60FPS: pStr = "HDRZ_60FPS"; break;

	case TOF_MODE_5FPS: pStr = "5FPS"; break;
	case TOF_MODE_10FPS: pStr = "10FPS"; break;
	case TOF_MODE_20FPS: pStr = "20FPS"; break;
	case TOF_MODE_30FPS: pStr = "30FPS"; break;
	case TOF_MODE_45FPS: pStr = "45FPS"; break;
	case TOF_MODE_60FPS: pStr = "60FPS"; break;

	case TOF_MODE_ADI_1M5: pStr = "ADI_1M5"; break;
	case TOF_MODE_ADI_5M: pStr = "ADI_5M"; break;

	default: break;
	}

	return pStr;
}


class CGrayConvert
{
	CGrayConvert();
	~CGrayConvert();

public:
	static bool Gray_2_Bgr32(void* pGray, const int width, const int height, const GRAY_FORMAT format, unsigned int* pBgr32);
	static bool Gray_2_U16(void* pGray, const int width, const int height, const GRAY_FORMAT format, unsigned short* pU16);
	static bool Gray_2_U8(void* pGray, const int width, const int height, const GRAY_FORMAT format, unsigned char* pU8);

private:
	static bool ToBgr32(unsigned char* pGray, const int width, const int height, unsigned int* pBgr32);
	static bool ToBgr32(unsigned short* pGray, const int width, const int height, unsigned int* pBgr32);
	static bool ToBgr32(float* pGray, const int width, const int height, unsigned int* pBgr32);
	static bool ToBgr32(unsigned int* pGray, const int width, const int height, unsigned int* pBgr32);
private:
	static bool ToU16(unsigned char* pGray, const int width, const int height, unsigned short* pU16);
	static bool ToU16(unsigned short* pGray, const int width, const int height, unsigned short* pU16);
	static bool ToU16(float* pGray, const int width, const int height, unsigned short* pU16);
	static bool ToU16(unsigned int* pGray, const int width, const int height, unsigned short* pU16);
private:
	static bool ToU8(unsigned char* pGray, const int width, const int height, unsigned char* pU8);
	static bool ToU8(unsigned short* pGray, const int width, const int height, unsigned char* pU8);
	static bool ToU8(float* pGray, const int width, const int height, unsigned char* pU8);
	static bool ToU8(unsigned int* pGray, const int width, const int height, unsigned char* pU8);
};
CGrayConvert::CGrayConvert()
{
}
CGrayConvert::~CGrayConvert()
{
}

bool CGrayConvert::Gray_2_Bgr32(void* pGray, const int width, const int height, const GRAY_FORMAT format, unsigned int* pBgr32)
{
	bool retVal = false;

	switch (format)
	{
	case GRAY_FORMAT_UINT8:  retVal = ToBgr32((unsigned char*)pGray, width, height, pBgr32); break;
	case GRAY_FORMAT_UINT16: retVal = ToBgr32((unsigned short*)pGray, width, height, pBgr32); break;
	case GRAY_FORMAT_FLOAT:  retVal = ToBgr32((float*)pGray, width, height, pBgr32); break;
	case GRAY_FORMAT_BGRD:   retVal = ToBgr32((unsigned int*)pGray, width, height, pBgr32); break;
	default: break;
	}

	return retVal;
}

bool CGrayConvert::Gray_2_U16(void* pGray, const int width, const int height, const GRAY_FORMAT format, unsigned short* pU16)
{
	bool retVal = false;

	switch (format)
	{
	case GRAY_FORMAT_UINT8:  retVal = ToU16((unsigned char*)pGray, width, height, pU16); break;
	case GRAY_FORMAT_UINT16: retVal = ToU16((unsigned short*)pGray, width, height, pU16); break;
	case GRAY_FORMAT_FLOAT:  retVal = ToU16((float*)pGray, width, height, pU16); break;
	case GRAY_FORMAT_BGRD:   retVal = ToU16((unsigned int*)pGray, width, height, pU16); break;
	default: break;
	}

	return retVal;
}

bool CGrayConvert::Gray_2_U8(void* pGray, const int width, const int height, const GRAY_FORMAT format, unsigned char* pU8)
{
	bool retVal = false;

	switch (format)
	{
	case GRAY_FORMAT_UINT8:  retVal = ToU8((unsigned char*)pGray, width, height, pU8); break;
	case GRAY_FORMAT_UINT16: retVal = ToU8((unsigned short*)pGray, width, height, pU8); break;
	case GRAY_FORMAT_FLOAT:  retVal = ToU8((float*)pGray, width, height, pU8); break;
	case GRAY_FORMAT_BGRD:   retVal = ToU8((unsigned int*)pGray, width, height, pU8); break;
	default: break;
	}

	return retVal;
}
bool CGrayConvert::ToBgr32(unsigned char* pGray, const int width, const int height, unsigned int* pBgr32)
{
	const int pixel_cnt = width*height;

	for (int i = 0; i < pixel_cnt; i++)
	{
		pBgr32[i] = ((pGray[i] << 24) | (pGray[i] << 16) | (pGray[i] << 8) | pGray[i]);
	}

	return true;
}
bool CGrayConvert::ToBgr32(unsigned short* pGray, const int width, const int height, unsigned int* pBgr32)
{
	const int pixel_cnt = width*height;
	//const unsigned short min = Utils_FindMinValue(pGray, pixel_cnt);//*min_element(pGray, pGray + pixel_cnt);//min_element耗时长
	const unsigned short max = Utils_FindMaxValue(pGray, pixel_cnt);// *max_element(pGray, pGray + pixel_cnt);//max_element耗时长

	if (0 >= max)
	{
		memset(pBgr32, 0, pixel_cnt * sizeof(pBgr32[0]));
		return true;
	}

	const float K = (255 * 1.0 / max);//最大值是255的多少倍

	for (int i = 0; i < pixel_cnt; i++)
	{
		const unsigned char tmp = (unsigned char)(pGray[i] * K);
		pBgr32[i] = ((tmp << 24) | (tmp << 16) | (tmp << 8) | tmp);
	}

	return true;
}
bool CGrayConvert::ToBgr32(float* pGray, const int width, const int height, unsigned int* pBgr32)
{
	const int pixel_cnt = width*height;
	//const float min = Utils_FindMinValue(pGray, pixel_cnt);//*min_element(pGray, pGray + pixel_cnt);//min_element耗时长
	const float max = Utils_FindMaxValue(pGray, pixel_cnt);// *max_element(pGray, pGray + pixel_cnt);//max_element耗时长

	if (0.001 >= max)//0值用黑色表示
	{
		memset(pBgr32, 0, pixel_cnt * sizeof(pBgr32[0]));
		return true;
	}

	const float K = (255 * 1.0 / max);//最大值是255的多少倍

	for (int i = 0; i < pixel_cnt; i++)
	{
		unsigned char tmp = 0;//0值用黑色表示
		if (0.001 < pGray[i])
		{
			tmp = (unsigned char)(pGray[i] * K);
		}
		pBgr32[i] = ((tmp << 24) | (tmp << 16) | (tmp << 8) | tmp);
	}

	return true;
}
bool CGrayConvert::ToBgr32(unsigned int* pGray, const int width, const int height, unsigned int* pBgr32)
{
	const int pixel_cnt = width*height;

	memcpy(pBgr32, pGray, pixel_cnt * sizeof(pBgr32[0]));

	return true;
}

bool CGrayConvert::ToU16(unsigned char* pGray, const int width, const int height, unsigned short* pU16)
{
	const int pixel_cnt = width*height;
	//const unsigned char min = Utils_FindMinValue(pGray, pixel_cnt);//*min_element(pGray, pGray + pixel_cnt);//min_element耗时长
	const unsigned char max = Utils_FindMaxValue(pGray, pixel_cnt);// *max_element(pGray, pGray + pixel_cnt);//max_element耗时长

	if (0 >= max)
	{
		memset(pU16, 0, pixel_cnt * sizeof(pU16[0]));
		return true;
	}

	const float K = (65535 * 1.0 / max);//最大值是65535的多少倍

	for (int i = 0; i < pixel_cnt; i++)
	{
		const unsigned short tmp = (unsigned short)(pGray[i] * K);
		pU16[i] = tmp;
	}

	return true;
}
bool CGrayConvert::ToU16(unsigned short* pGray, const int width, const int height, unsigned short* pU16)
{
	const int pixel_cnt = width*height;

	memcpy(pU16, pGray, pixel_cnt * sizeof(pU16[0]));

	return true;
}
bool CGrayConvert::ToU16(float* pGray, const int width, const int height, unsigned short* pU16)
{
#if 1
	//方法1：灰度直接数据类型强转

	const int pixel_cnt = width*height;

	for (int i = 0; i < pixel_cnt; i++)
	{
		pU16[i] = ((65535.0 < pGray[i]) ? 65535 : pGray[i]);
	}

#else
	//方法2：灰度按照等比例压缩

	const int pixel_cnt = width*height;
	//const float min = Utils_FindMinValue(pGray, pixel_cnt);//*min_element(pGray, pGray + pixel_cnt);//min_element耗时长
	const float max = Utils_FindMaxValue(pGray, pixel_cnt);// *max_element(pGray, pGray + pixel_cnt);//max_element耗时长

	if (0.001 >= max)//0值用黑色表示
	{
		memset(pU16, 0, pixel_cnt * sizeof(pU16[0]));
		return true;
	}

	const float K = (65535 * 1.0 / max);//最大值是65535的多少倍

	for (int i = 0; i < pixel_cnt; i++)
	{
		unsigned short tmp = 0;//0值用黑色表示
		if (0.001 < pGray[i])
		{
			tmp = (unsigned short)(pGray[i] * K);
		}
		pU16[i] = tmp;
	}
#endif
	return true;
}
bool CGrayConvert::ToU16(unsigned int* pGray, const int width, const int height, unsigned short* pU16)
{
	const int pixel_cnt = width*height;

	for (int i = 0; i < pixel_cnt; i++)
	{
		unsigned char* pTmp = (unsigned char*)(pGray + i);//BGRD排列
		pU16[i] = (pTmp[0] << 8);//放大到65535
	}

	return true;
}

bool CGrayConvert::ToU8(unsigned char* pGray, const int width, const int height, unsigned char* pU8)
{
	printf("h1");
	const int pixel_cnt = width*height;

	memcpy(pU8, pGray, pixel_cnt * sizeof(pU8[0]));

	return true;
}
bool CGrayConvert::ToU8(unsigned short* pGray, const int width, const int height, unsigned char* pU8)
{
	printf("h2");
	const int pixel_cnt = width*height;
	//const unsigned short min = Utils_FindMinValue(pGray, pixel_cnt);//*min_element(pGray, pGray + pixel_cnt);//min_element耗时长
	const unsigned short max = Utils_FindMaxValue(pGray, pixel_cnt);// *max_element(pGray, pGray + pixel_cnt);//max_element耗时长

	if (0 >= max)
	{
		memset(pU8, 0, pixel_cnt * sizeof(pU8[0]));
		return true;
	}

	const float K = (255 * 1.0 / max);//最大值是255的多少倍

	for (int i = 0; i < pixel_cnt; i++)
	{
		const unsigned char tmp = (unsigned char)(pGray[i] * K);
		pU8[i] = tmp;
	}

	return true;
}
bool CGrayConvert::ToU8(float* pGray, const int width, const int height, unsigned char* pU8)
{
	printf("h3");
	const int pixel_cnt = width*height;
	//const float min = Utils_FindMinValue(pGray, pixel_cnt);//*min_element(pGray, pGray + pixel_cnt);//min_element耗时长
	const float max = Utils_FindMaxValue(pGray, pixel_cnt);// *max_element(pGray, pGray + pixel_cnt);//max_element耗时长

	if (0.001 >= max)//0值用黑色表示
	{
		memset(pU8, 0, pixel_cnt * sizeof(pU8[0]));
		return true;
	}

	const float K = (255 * 1.0 / max);//最大值是255的多少倍

	for (int i = 0; i < pixel_cnt; i++)
	{
		unsigned char tmp = 0;//0值用黑色表示
		if (0.001 < pGray[i])
		{
			tmp = (unsigned char)(pGray[i] * K);
		}
		pU8[i] = tmp;
	}

	return true;
}
bool CGrayConvert::ToU8(unsigned int* pGray, const int width, const int height, unsigned char* pU8)
{
	printf("h4");
	const int pixel_cnt = width*height;

	for (int i = 0; i < pixel_cnt; i++)
	{
		unsigned char* pTmp = (unsigned char*)(pGray + i);//BGRD排列
		pU8[i] = pTmp[0];
	}

	return true;
}

static bool SaveGray_2_BGR32(void* pGray, const UINT32 width, const UINT32 height, const GRAY_FORMAT format, char* pFile)
{
	if ((NULL == pGray) || (0 >= width) || (0 >= height) || (NULL == pFile))
	{
		return false;
	}

	unsigned int* pData = new unsigned int[width * height];//bgra
	CGrayConvert::Gray_2_Bgr32(pGray, width, height, format, pData);
	Utils_SaveBufToFile(pData, width * height * sizeof(pData[0]), (const char*)pFile, false);
	SAFE_DELETE_ARRY(pData);

	return true;

}
static bool SaveGray_2_U16(void* pGray, const UINT32 width, const UINT32 height, const GRAY_FORMAT format, char* pFile)
{
	if ((NULL == pGray) || (0 >= width) || (0 >= height) || (NULL == pFile))
	{
		return false;
	}

	unsigned short* pData = new unsigned short[width * height];//U16
	CGrayConvert::Gray_2_U16(pGray, width, height, format, pData);
	Utils_SaveBufToFile(pData, width * height * sizeof(pData[0]), (const char*)pFile, false);
	SAFE_DELETE_ARRY(pData);

	return true;

}
static bool SaveGray_2_U8(void* pGray, const UINT32 width, const UINT32 height, const GRAY_FORMAT format, char* pFile)
{
	if ((NULL == pGray) || (0 >= width) || (0 >= height) || (NULL == pFile))
	{
		return false;
	}

	unsigned char* pData = new unsigned char[width * height];//U8
	CGrayConvert::Gray_2_U8(pGray, width, height, format, pData);
	// Utils_SaveBufToFile(pData, width * height * sizeof(pData[0]), (const char*)pFile, false);
	SAFE_DELETE_ARRY(pData);

	return true;

}

static bool SaveDepthText(float* pDepthData, const UINT32 width, const UINT32 height, char* pTxtFile, const bool bWH)
{
	if ((NULL == pDepthData) || (0 >= width) || (0 >= height) || (NULL == pTxtFile))
	{
		return false;
	}

	FILE* fp = fopen(pTxtFile, "w");
	if (NULL == fp)
	{
		return false;
	}

	if (bWH)//1：W列、H行排列
	{
		UINT32 nPos = 0;
		for (UINT32 h = 0; h < height; h++)
		{
			for (UINT32 w = 0; w < (width - 1); w++)
			{
				fprintf(fp, "%0.6f,", pDepthData[nPos]);
				nPos++;
			}
			fprintf(fp, "%0.6f\n", pDepthData[nPos]);
			nPos++;
		}
	}
	else//2：1行、W*H行排列
	{
		const UINT32 nCnt = width *height;
		for (UINT32 nPos = 0; nPos < nCnt; nPos++)
		{
			fprintf(fp, "%0.6f\n", pDepthData[nPos]);
		}
	}

	fclose(fp);
	return true;
}
static bool SavePointDataXYZText(PointData *pPointData, const UINT32 width, const UINT32 height, char* pTxtFile)
{
	if ((NULL == pPointData) || (0 >= width) || (0 >= height) || (NULL == pTxtFile))
	{
		return false;
	}

	FILE* fp = fopen(pTxtFile, "w");
	if (NULL == fp)
	{
		return false;
	}

	const UINT32 nCnt = width *height;
	for (UINT32 nPos = 0; nPos < nCnt; nPos++)
	{
		fprintf(fp, "%0.6f;%0.6f;%0.6f\n", pPointData[nPos].x, pPointData[nPos].y, pPointData[nPos].z);
	}

	fclose(fp);
	return true;
}
static bool SavePointDataZWHText(PointData *pPointData, const UINT32 width, const UINT32 height, char* pTxtFile)
{
	if ((NULL == pPointData) || (0 >= width) || (0 >= height) || (NULL == pTxtFile))
	{
		return false;
	}

	FILE* fp = fopen(pTxtFile, "w");
	if (NULL == fp)
	{
		return false;
	}

	UINT32 nPos = 0;
	for (UINT32 h = 0; h < height; h++)
	{
		for (UINT32 w = 0; w < width; w++)
		{
			fprintf(fp, "%0.6f", pPointData[nPos].z);
			nPos++;
		}
		fprintf(fp, "\n");
	}

	fclose(fp);
	return true;
}

static bool SaveImuText(ImuFrameData *imuFrameData, const char* pFile, const bool bAppend)
{
	if ((NULL == imuFrameData) || (NULL == pFile))
	{
		return false;
	}

	FILE* fp = fopen(pFile, (bAppend ? "ab" : "wb"));
	if (NULL == fp)
	{
		printf("open file(%s) failed, error=%d(%s).\n", pFile, errno, strerror(errno));
		return false;
	}

	fprintf(fp, "timeStamp:%lld", imuFrameData->timeStamp);

	fprintf(fp, ",accelData:%0.6f,%0.6f,%0.6f", imuFrameData->accelData_x, imuFrameData->accelData_y, imuFrameData->accelData_z);
	fprintf(fp, ",gyrData:%0.6f,%0.6f,%0.6f", imuFrameData->gyrData_x, imuFrameData->gyrData_y, imuFrameData->gyrData_z);
	fprintf(fp, ",magData:%0.6f,%0.6f,%0.6f\n", imuFrameData->magData_x, imuFrameData->magData_y, imuFrameData->magData_z);

	fclose(fp);

	return true;
}

static void CaptureTofFrame(const std::string& strDir, const unsigned int nCaptureIndex, TofFrameData *tofFrameData)
{
	const unsigned int nPixelCnt = tofFrameData->frameWidth * tofFrameData->frameHeight;
	char szFile[512] = { 0 };

	//
	// if (NULL != tofFrameData->pPointData)
	// {
	// 	sprintf(szFile, "%s/%u-PointData.dat", strDir.c_str(), nCaptureIndex);
	// 	Utils_SaveBufToFile(tofFrameData->pPointData, nPixelCnt  * sizeof(tofFrameData->pPointData[0]), szFile, false);

	// 	sprintf(szFile, "%s/%u-PointData.txt", strDir.c_str(), nCaptureIndex);
	// 	SavePointDataXYZText(tofFrameData->pPointData, tofFrameData->frameWidth, tofFrameData->frameHeight, szFile);
	// }

	//
	if (NULL != tofFrameData->pGrayData)
	{
		// sprintf(szFile, "%s/%u-Gray.%s", strDir.c_str(), nCaptureIndex, StringGrayFormat(tofFrameData->grayFormat));
		// Utils_SaveBufToFile(tofFrameData->pGrayData, nPixelCnt * CaculateGrayPixelBytes(tofFrameData->grayFormat), szFile, false);

		sprintf(szFile, "%s/%u-Gray.u8", strDir.c_str(), nCaptureIndex);
		SaveGray_2_U8(tofFrameData->pGrayData, tofFrameData->frameWidth, tofFrameData->frameHeight, tofFrameData->grayFormat, szFile);
	}
	//
	// if (NULL != tofFrameData->pConfidence)
	// {
	// 	sprintf(szFile, "%s/%u-Confidence.dat", strDir.c_str(), nCaptureIndex);
	// 	Utils_SaveBufToFile(tofFrameData->pConfidence, nPixelCnt  * sizeof(tofFrameData->pConfidence[0]), szFile, false);

	// 	sprintf(szFile, "%s/%u-Confidence.txt", strDir.c_str(), nCaptureIndex);
	// 	SaveDepthText(tofFrameData->pConfidence, tofFrameData->frameWidth, tofFrameData->frameHeight, szFile, false);

	// 	sprintf(szFile, "%s/%u-Confidence.u8", strDir.c_str(), nCaptureIndex);
	// 	SaveGray_2_U8(tofFrameData->pConfidence, tofFrameData->frameWidth, tofFrameData->frameHeight, GRAY_FORMAT_FLOAT, szFile);
	// }
	// //
	// if (NULL != tofFrameData->pRgbD)
	// {
	// 	sprintf(szFile, "%s/%u-RgbD.%s", strDir.c_str(), nCaptureIndex, "rgb");
	// 	Utils_SaveBufToFile(tofFrameData->pRgbD, nPixelCnt * sizeof(tofFrameData->pRgbD[0]), szFile, false);
	// }
	// //
	// if ((NULL != tofFrameData->pRawData) && (0 < tofFrameData->nRawDataLen))
	// {
	// 	sprintf(szFile, "%s/%u-Tof.%s", strDir.c_str(), nCaptureIndex, "raw");
	// 	Utils_SaveBufToFile(tofFrameData->pRawData, tofFrameData->nRawDataLen, szFile, false);
	// }
	// //
	// if ((NULL != tofFrameData->pExtData) && (0 < tofFrameData->nExtDataLen))
	// {
	// 	sprintf(szFile, "%s/%u-Tof.%s", strDir.c_str(), nCaptureIndex, "extdata");
	// 	Utils_SaveBufToFile(tofFrameData->pExtData, tofFrameData->nExtDataLen, szFile, false);
	// }
}
static void CaptureRgbFrame(const std::string& strDir, const unsigned int nCaptureIndex, RgbFrameData *rgbFrameData)
{
	char szFile[512] = { 0 };

	//
	if ((NULL != rgbFrameData->pFrameData) && (0 < rgbFrameData->nFrameLen))
	{
		sprintf(szFile, "%s/%u-Rgb.%s", strDir.c_str(), nCaptureIndex, StringColorFormat(rgbFrameData->formatType));
		Utils_SaveBufToFile(rgbFrameData->pFrameData, rgbFrameData->nFrameLen, szFile, false);
	}
	//
	if ((NULL != rgbFrameData->pExtData) && (0 < rgbFrameData->nExtDataLen))
	{
		sprintf(szFile, "%s/%u-Rgb.%s", strDir.c_str(), nCaptureIndex, "extdata");
		Utils_SaveBufToFile(rgbFrameData->pExtData, rgbFrameData->nExtDataLen, szFile, false);
	}
}
static void CaptureImuFrame(const std::string& strDir, const unsigned int nCaptureIndex, ImuFrameData *imuFrameData)
{
	char szFile[512] = { 0 };

	//
	if (NULL != imuFrameData)
	{
		sprintf(szFile, "%s/%u-Imu.dat", strDir.c_str(), /*nCaptureIndex*/1);
		Utils_SaveBufToFile(imuFrameData, sizeof(ImuFrameData), szFile, /*false*/true);//固定写到同一个文件里就好了

		sprintf(szFile, "%s/%u-Imu.txt", strDir.c_str(), /*nCaptureIndex*/1);//固定写到同一个文件里就好了
		SaveImuText(imuFrameData, szFile, /*false*/true);
	}
}



static void CallBackTofDeviceStatus(TOFDEV_STATUS tofDevStatus, void *pUserData)
{
	printf("device status: %d.\n", tofDevStatus);

}

static void fnTofStream(TofFrameData *tofFrameData, void* pUserData)
{
	UINT32* frame_count = (UINT32*)pUserData;
	const UINT32 width = tofFrameData->frameWidth;
	const UINT32 height = tofFrameData->frameHeight;

	(*frame_count)++;

	//
	const float fDepthZAvg = CalCenterPointDataZAvg(tofFrameData->pPointData, tofFrameData->frameWidth, tofFrameData->frameHeight);
	printf("[%d], one TOF frame, time=%llu, center depthZ = %0.3f m.\n", *frame_count, Utils_GetTickCount(), fDepthZAvg);
	// return;

	CaptureTofFrame(std::string("./Capture"), (*frame_count), tofFrameData);

}

static void fnRgbStream(RgbFrameData *rgbFrameData, void* pUserData)
{
	UINT32* frame_count = (UINT32*)pUserData;
	const UINT32 width = rgbFrameData->frameWidth;
	const UINT32 height = rgbFrameData->frameHeight;

	(*frame_count)++;

	printf("[%d], one RGB frame, time=%llu, formatType=%d, %d.\n", *frame_count, Utils_GetTickCount(), rgbFrameData->formatType, rgbFrameData->formatTypeOrg);
	return;

	CaptureRgbFrame(std::string("./Capture"), (*frame_count), rgbFrameData);

}

static void fnImuStream(ImuFrameData *imuFrameData, void* pUserData)
{
	UINT32* frame_count = (UINT32*)pUserData;

	(*frame_count)++;

	printf("[%d], one IMU frame, time=%llu.\n", *frame_count, Utils_GetTickCount());
	return;

	CaptureImuFrame(std::string("./Capture"), (*frame_count), imuFrameData);

}

static void PrintDevInfo(TofDeviceInfo *pTofDeviceInfo)
{
	printf("Dev Info:==================================\n");
	printf(">>  szDevName=%s.\n", pTofDeviceInfo->szDevName);
	printf(">>  szDevId=%s.\n", pTofDeviceInfo->szDevId);
	printf(">>  szFirmwareVersion=%s.\n", pTofDeviceInfo->szFirmwareVersion);
	printf("Dev Info==================================\n\n");
}


static TOF_MODE ChoseTofMode(const UINT32 supportedTOFMode)
{
#if 0
	//这里简单处理，选择第一种支持的模式

	const int bitCnt = (sizeof(supportedTOFMode) * 8);
	int tofMode = 0;

	for (int i = 0; i < bitCnt; i++)
	{
		tofMode = 1 << i;
		if (supportedTOFMode & tofMode)
		{
			return (TOF_MODE)tofMode;
		}
	}

	tofMode = 1 << 0;
	return (TOF_MODE)tofMode;

#else
	UINT32 tofMode = 0;
	UINT32 mode_index = 0;

	//打印出所有支持的TOF模式，供用户选择
	printf("chose tof mode from list: \n");
	printf(">>  number: mode.\n");


	const UINT32 min_index = 0;
	const UINT32 max_index = 8 * sizeof(supportedTOFMode);
	for (mode_index = min_index; mode_index < max_index; mode_index++)
	{
		tofMode = (1 << mode_index);
		if (supportedTOFMode & tofMode)
		{
			printf(">>  %d: %s.\n", mode_index, TofMode2Str((TOF_MODE)tofMode));
		}
	}

	//用于选择哪一种tof模式
	while (1)
	{
		printf("input mode (number) >>");

		std::string strInput;
		std::cin >> strInput;

		mode_index = (UINT32)strtol(strInput.c_str(), NULL, 10);
		if (max_index >= mode_index)
		{
			tofMode = (1 << mode_index);
			if (supportedTOFMode & tofMode)
			{
				break;
			}
		}
	}

	return ((TOF_MODE)tofMode);

#endif

}


static bool OpenStream(HTOFD hTofD, TofDeviceInfo* pCaps, const TOF_MODE tofMode, 
	UINT32* tof_frame_count, UINT32* rgb_frame_count, UINT32* imu_frame_count)
{
	(*tof_frame_count) = 0;
	TOFRET retVal = TOFD_StartTofStream(hTofD, tofMode, fnTofStream, tof_frame_count);
	if ((TOFRET_SUCCESS != retVal) && (TOFRET_SUCCESS_READING_CALIB != retVal))
	{
		printf("start TOF stream, [ FAILED ], retVal=0x%08x!!!!!!!!!!!!!!!!!!!!!\n\n", retVal);
		return false;
	}

	if (pCaps->bRgbSupported)//支持RGB的情况下
	{
		(*rgb_frame_count) = 0;
		TOFRET retVal = TOFD_StartRgbStream(hTofD, fnRgbStream, rgb_frame_count);
		if (TOFRET_SUCCESS != retVal)
		{
			printf("start RGB stream, [ FAILED ], retVal=0x%08x!!!!!!!!!!!!!!!!!!!!!\n\n", retVal);
			return false;
		}
	}

	if (pCaps->bImuSupported)//支持IMU的情况下
	{
		(*imu_frame_count) = 0;
		TOFRET retVal = TOFD_StartImuStream(hTofD, fnImuStream, imu_frame_count);
		if (TOFRET_SUCCESS != retVal)
		{
			printf("start IMU stream, [ FAILED ], retVal=0x%08x!!!!!!!!!!!!!!!!!!!!!\n\n", retVal);
			return false;
		}
	}

	return true;

}

static void CloseStream(HTOFD hTofD, TofDeviceInfo* pCaps)
{
	TOFD_StopTofStream(hTofD);

	if (pCaps->bRgbSupported)//支持RGB的情况下
	{
		TOFD_StopRgbStream(hTofD);
	}
	if (pCaps->bImuSupported)//支持IMU的情况下
	{
		TOFD_StopImuStream(hTofD);
	}
}


static void GetOrSetSomeParam(HTOFD hTofD, TofDeviceInfo* pCaps, const TOF_MODE tofMode)
{
	TOFRET retVal = TOFRET_ERROR_OTHER;

#if 1
	if (pCaps->supportedTofExpMode & EXP_MODE_AUTO)
	{
		if (TOFRET_SUCCESS != (retVal = TOFD_SetTofAE(hTofD, true)))//按需，或者联系沟通后确认
		{
			printf("TOFD_SetTofAE failed, retVal=0x%08x.\n", retVal);
		}
	}

	if (TOFRET_SUCCESS != (retVal = TOFD_SetTofHDRZ(hTofD, false)))//按需，或者联系沟通后确认
	{
		printf("TOFD_SetTofHDRZ failed, retVal=0x%08x.\n", retVal);
	}
#endif

#if 1 //滤波，按需，或者联系沟通后确认
	for (UINT32 i = 0; i < 32; i++)
	{
		UINT32 type = (1 << i);
		if (0 != (pCaps->supportedTOFFilter & type))
		{
			if (TOFRET_SUCCESS != (retVal = TOFD_SetTofFilter(hTofD, (const TOF_FILTER)type, true)))
			{
				printf("TOFD_SetTofFilter failed, retVal=0x%08x.\n", retVal);
			}
		}
	}
#endif

#if 0
	TofExpouse struExp;
	memset(&struExp, 0, sizeof(struExp));
	if (TOFRET_SUCCESS != (retVal = TOFD_GetTofExpTime(hTofD, &struExp)))
	{
		printf("TOFD_GetTofExpTime failed, retVal=0x%08x.\n", retVal);
	}

	if (TOFRET_SUCCESS != (retVal = TOFD_SetTofExpTime(hTofD, struExp.nCurrent)))
	{
		printf("TOFD_SetTofExpTime failed, retVal=0x%08x.\n", retVal);
	}
#endif
}

static void ThreadTestDemo(HTOFD hTofD)
{
	TofDeviceInfo struCaps;
	memset(&struCaps, 0, sizeof(struCaps));
	TOFRET retVal = TOFD_GetDeviceInfo(hTofD, &struCaps);
	if (TOFRET_SUCCESS != retVal)
	{
		printf("Get Device Info, [ FAILED ], retVal=0x%08x!!!!!!!!!!!!!!!!!!!!!\n\n", retVal);
	}
	PrintDevInfo(&struCaps);

	const TOF_MODE tofMode = ChoseTofMode(struCaps.supportedTOFMode);//选择其中一种TOF模式出TOF数据

	//
	UINT32 tof_frame_count = 0, rgb_frame_count = 0, imu_frame_count = 0;
	const bool bSuc = OpenStream(hTofD, &struCaps, tofMode, &tof_frame_count, &rgb_frame_count, &imu_frame_count);
	if (bSuc)
	{
		//成功后，等到数据流拿到后再去设置参数，这样才能保证生效
		while (10 > tof_frame_count)
		{
			printf("Waiting for stream data...\n");
			std::this_thread::sleep_for(std::chrono::milliseconds(1000)); //单位是毫秒
		}

		printf("Get Or Set Some Params...\n");
		GetOrSetSomeParam(hTofD, &struCaps, tofMode);
	}

	//一旦键盘输入字符s，就退出
	const std::string strExitCode = "s";
	printf("input %s to exit demo >>\n", strExitCode.c_str());
	while (1)
	{
	    std::string strInput;
		std::cin >> strInput;
		if (strExitCode == strInput) break;
		
		printf("input %s to exit demo >>\n", strExitCode.c_str());
	}

	//等待线程退出
	CloseStream(hTofD, &struCaps);

}


static void DoTestDemo(TofDeviceDescriptor* pDevsDesc)
{
	HTOFD hTofD = TOFD_OpenDevice(pDevsDesc, CallBackTofDeviceStatus, NULL);
	if (NULL == hTofD)
	{
		printf("Open Tof Device failed.\n");
		return;
	}

	std::thread thread_test = std::thread(ThreadTestDemo, hTofD);
	thread_test.join();

	TOFD_CloseDevice(hTofD);

}

static UINT32 ChoseDev(const UINT32  dev_cnt)
{
	const UINT32 min_index = 1;
	const UINT32 max_index = dev_cnt;

	if (1 == max_index)//只有一个设备的话就不用选择了
	{
		return max_index;
	}

	UINT32 dev_index = 1;
	while (1)
	{
		printf("please chose a dev (min=%d, max:%d):\n", min_index, max_index);
		printf(">>");

		std::string strInput;
		std::cin >> strInput;

		dev_index = (UINT32)strtol(strInput.c_str(), NULL, 10);
		if ((min_index <= dev_index) && (max_index >= dev_index))
		{
			break;
		}
		printf("invalid dev index:%d.\n", dev_index);
	}

	return dev_index;
}

int main()
{
	printf("*********************start test device*************************\n");

	TofDevInitParam struInitParam;
	memset(&struInitParam, 0, sizeof(struInitParam));
	strncpy(struInitParam.szDepthCalcCfgFileDir, "./parameter", sizeof(struInitParam.szDepthCalcCfgFileDir) - 1);
	TOFD_Init(&struInitParam);

	printf("SDK Version: %s.\n", TOFD_GetSDKVersion());

	TofDeviceDescriptor* pDevsDescList = NULL;
	UINT32  dev_num = 0;
	TOFD_SearchDevice(&pDevsDescList, &dev_num);
	if (0 < dev_num )
	{
		const UINT32 dev_index = ChoseDev(dev_num) - 1;//决定测试哪一个设备
		DoTestDemo(pDevsDescList + dev_index);
	}
	else
	{
		printf("can not find tof device!\n");
	}

	printf("*********************stop test device*********************\n");
	TOFD_Uninit();

#ifdef WIN32
	system("pause");
#endif
	return 0;
}




