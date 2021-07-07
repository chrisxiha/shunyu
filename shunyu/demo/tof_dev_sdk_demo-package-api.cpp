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

using namespace std;

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <sensor_msgs/PointCloud2.h>

ros::Publisher all_pointcloud_publisher_; 
ros::Publisher gray_publisher;
ros::Publisher depth_publisher;
ros::Publisher raw_publisher;

bool normalize_pub_image = false;
bool ae_enable = true;
bool hd_enable = true;
bool filter_enable = true;

double depth_range_min = 0.001;
double depth_range_max = 4.0;
float bad_point = std::numeric_limits<float>::quiet_NaN();

UINT32 tof_seq = 0;
ros::Time tof_timestamp;
std::string tof_frame_id = std::string("shunyu");






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
	const int pixel_cnt = width*height;

	memcpy(pU8, pGray, pixel_cnt * sizeof(pU8[0]));

	return true;
}
bool CGrayConvert::ToU8(unsigned short* pGray, const int width, const int height, unsigned char* pU8)
{
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
	Utils_SaveBufToFile(pData, width * height * sizeof(pData[0]), (const char*)pFile, false);
	SAFE_DELETE_ARRY(pData);

	return true;

}

class CDataTools
{
public:
	CDataTools();
	~CDataTools();

public:

	static void ReleaseList(std::list<TofFrameData*>& listData);
	static void CloneBuf(TofFrameData* dst, TofFrameData* src);
	static void ReleaseBuf(TofFrameData* data);

	static void ReleaseList(std::list<RgbFrameData*>& listData);
	static void CloneBuf(RgbFrameData* dst, RgbFrameData* src);
	static void ReleaseBuf(RgbFrameData* data);

	static void ReleaseList(std::list<ImuFrameData*>& listData);
	static void CloneBuf(ImuFrameData* dst, ImuFrameData* src);
	static void ReleaseBuf(ImuFrameData* data);

};
CDataTools::CDataTools()
{
}
CDataTools::~CDataTools()
{
}

void CDataTools::ReleaseList(std::list<TofFrameData*>& listData)
{
	for (UINT32 i = 0; i < listData.size(); i++)
	{
		TofFrameData* tmp = listData.front();
		listData.pop_front();

		ReleaseBuf(tmp);
		SAFE_DELETE(tmp);
	}
	listData.clear();
}
void CDataTools::CloneBuf(TofFrameData* dst, TofFrameData* src)
{
	if ((NULL == dst) || (NULL == src))
	{
		return;
	}

	memcpy(dst, src, sizeof(src[0]));
	const UINT32 nPixelCnt = src->frameWidth * src->frameHeight;

	if (src->pPointData)
	{
		dst->pPointData = new PointData[nPixelCnt];
		memcpy(dst->pPointData, src->pPointData, nPixelCnt * sizeof(src->pPointData[0]));
	}

	if (src->pGrayData)
	{
		const int nPixelBytes = CaculateGrayPixelBytes(src->grayFormat);
		dst->pGrayData = new UINT8[nPixelCnt * nPixelBytes];
		memcpy(dst->pGrayData, src->pGrayData, nPixelCnt * nPixelBytes * sizeof(UINT8));
	}

	if (src->pConfidence)
	{
		dst->pConfidence = new FLOAT32[nPixelCnt];
		memcpy(dst->pConfidence, src->pConfidence, nPixelCnt * sizeof(src->pConfidence[0]));
	}

	if (src->pRgbD)
	{
		dst->pRgbD = new RgbDData[nPixelCnt];
		memcpy(dst->pRgbD, src->pRgbD, nPixelCnt * sizeof(src->pRgbD[0]));
	}

	if ((src->pRawData) && (0 < src->nRawDataLen))
	{
		dst->pRawData = new UINT8[src->nRawDataLen];
		memcpy(dst->pRawData, src->pRawData, src->nRawDataLen);
	}

	if ((src->pExtData) && (0 < src->nExtDataLen))
	{
		dst->pExtData = new UINT8[src->nExtDataLen];
		memcpy(dst->pExtData, src->pExtData, src->nExtDataLen);
	}

}
void CDataTools::ReleaseBuf(TofFrameData* data)
{
	if (NULL == data)
	{
		return;
	}

	SAFE_DELETE_ARRY(data->pPointData);

	UINT8* pGray = (UINT8*)(data->pGrayData);
	SAFE_DELETE_ARRY(pGray);
	data->pGrayData = NULL;

	SAFE_DELETE_ARRY(data->pConfidence);

	SAFE_DELETE_ARRY(data->pRgbD);

	UINT8* pRaw = (UINT8*)(data->pRawData);
	SAFE_DELETE_ARRY(pRaw);
	data->pRawData = NULL;

	UINT8* pExt = (UINT8*)(data->pExtData);
	SAFE_DELETE_ARRY(pExt);
	data->pExtData = NULL;
}

void CDataTools::ReleaseList(std::list<RgbFrameData*>& listData)
{
	for (UINT32 i = 0; i < listData.size(); i++)
	{
		RgbFrameData* tmp = listData.front();
		listData.pop_front();

		ReleaseBuf(tmp);
		SAFE_DELETE(tmp);
	}
	listData.clear();
}
void CDataTools::CloneBuf(RgbFrameData* dst, RgbFrameData* src)
{
	if ((NULL == dst) || (NULL == src))
	{
		return;
	}

	memcpy(dst, src, sizeof(src[0]));

	if ((src->pFrameData) && (0 < src->nFrameLen))
	{
		dst->pFrameData = new UINT8[src->nFrameLen];
		memcpy(dst->pFrameData, src->pFrameData, src->nFrameLen);
	}

	if ((src->pExtData) && (0 < src->nExtDataLen))
	{
		dst->pExtData = new UINT8[src->nExtDataLen];
		memcpy(dst->pExtData, src->pExtData, src->nExtDataLen);
	}
}
void CDataTools::ReleaseBuf(RgbFrameData* data)
{
	if (NULL == data)
	{
		return;
	}

	SAFE_DELETE_ARRY(data->pFrameData);

	UINT8* pExt = (UINT8*)(data->pExtData);
	SAFE_DELETE_ARRY(pExt);
	data->pExtData = NULL;
}

void CDataTools::ReleaseList(std::list<ImuFrameData*>& listData)
{
	for (UINT32 i = 0; i < listData.size(); i++)
	{
		ImuFrameData* tmp = listData.front();
		listData.pop_front();

		ReleaseBuf(tmp);
		SAFE_DELETE(tmp);
	}
	listData.clear();
}
void CDataTools::CloneBuf(ImuFrameData* dst, ImuFrameData* src)
{
	if ((NULL == dst) || (NULL == src))
	{
		return;
	}

	memcpy(dst, src, sizeof(src[0]));

}
void CDataTools::ReleaseBuf(ImuFrameData* data)
{
	if (NULL == data)
	{
		return;
	}

}



class CDevData
{
public:
	CDevData()
	{
		m_nIndex = 0xffffffff;
		m_hTofD = NULL;
		memset(&m_struCaps, 0, sizeof(m_struCaps));

		//TOF帧
		m_nTofFrameRecvTotalCnt = 0;
		m_listTofDataMaxLen = 0;
		m_listTofData.clear();
		m_pTofData = NULL;

		//RGB帧
		m_nRgbFrameRecvTotalCnt = 0;
		m_listRgbDataMaxLen = 0;
		m_listRgbData.clear();
		m_pRgbData = NULL;

		//IMU帧
		m_nImuFrameRecvTotalCnt = 0;
		m_listImuDataMaxLen = 0;
		m_listImuData.clear();
		m_pImuData = NULL;

	}
	~CDevData()
	{
		Init(0, 0, 0);
	}

public:
	void Init(const UINT32 listTofMaxLen = 20, const UINT32 listRgbMaxLen = 20, const UINT32 listImuMaxLen = 200)
	{
		std::lock_guard<std::mutex> locker(m_lock);//构造里自动上锁，析构里自动解锁（涉及软件算法）

		m_nIndex = 0xffffffff;
		m_hTofD = NULL;
		memset(&m_struCaps, 0, sizeof(m_struCaps));

		//TOF帧
		m_nTofFrameRecvTotalCnt = 0;
		m_listTofDataMaxLen = listTofMaxLen;
		CDataTools::ReleaseList(m_listTofData);
		CDataTools::ReleaseBuf(m_pTofData);
		SAFE_DELETE(m_pTofData);

		//RGB帧
		m_nRgbFrameRecvTotalCnt = 0;
		m_listRgbDataMaxLen = listRgbMaxLen;
		CDataTools::ReleaseList(m_listRgbData);
		CDataTools::ReleaseBuf(m_pRgbData);
		SAFE_DELETE(m_pRgbData);

		//IMU帧
		m_nImuFrameRecvTotalCnt = 0;
		m_listImuDataMaxLen = listImuMaxLen;
		CDataTools::ReleaseList(m_listImuData);
		CDataTools::ReleaseBuf(m_pImuData);
		SAFE_DELETE(m_pImuData);
	}

	SBOOL BindDev(const UINT32 index, HTOFD hTofD, TofDeviceInfo& struCaps)
	{
		std::lock_guard<std::mutex> locker(m_lock);//构造里自动上锁，析构里自动解锁（涉及软件算法）

		m_nIndex = index;//设备索引
		m_hTofD = hTofD;//设备句柄
		memcpy(&m_struCaps, &struCaps, sizeof(struCaps));//设备信息

		return true;
	}


	SBOOL PutTofData(TofFrameData* in)
	{
		std::lock_guard<std::mutex> locker(m_lock);//构造里自动上锁，析构里自动解锁（涉及软件算法）

		if (NULL == m_hTofD)
		{
			printf("dev is closed!!!!!!!!!!!!!!!!!!!!!!!!!.\n");
			return false;
		}

		if (m_listTofDataMaxLen <= m_listTofData.size())//链表不要太长，以防内存爆掉
		{
			printf("tof list over flow!!!!!!!!!!!!!!!!!!!!!!!!!.\n");
			return false;
		}

		TofFrameData* tmp = new TofFrameData;
		CDataTools::CloneBuf(tmp, in);
		m_listTofData.push_back(tmp);

		return true;
	}
	SBOOL GetTofData(TofFrameData* out)
	{
		std::lock_guard<std::mutex> locker(m_lock);//构造里自动上锁，析构里自动解锁（涉及软件算法）

		if (NULL == m_hTofD)
		{
			printf("dev is closed!!!!!!!!!!!!!!!!!!!!!!!!!.\n");
			return false;
		}

		if (m_listTofData.empty())
		{
			return false;
		}

		//先释放上一次存着的内存
		CDataTools::ReleaseBuf(m_pTofData);
		SAFE_DELETE(m_pTofData);

		//
		m_pTofData = m_listTofData.front();//返回第一个元素的值。
		m_listTofData.pop_front();//删除第一个元素。

		memcpy(out, m_pTofData, sizeof(m_pTofData[0]));

		return true;
	}

	SBOOL PutRgbData(RgbFrameData* in)
	{
		std::lock_guard<std::mutex> locker(m_lock);//构造里自动上锁，析构里自动解锁（涉及软件算法）

		if (NULL == m_hTofD)
		{
			printf("dev is closed!!!!!!!!!!!!!!!!!!!!!!!!!.\n");
			return false;
		}

		if (m_listRgbDataMaxLen <= m_listRgbData.size())//链表不要太长，以防内存爆掉
		{
			printf("rgb list over flow!!!!!!!!!!!!!!!!!!!!!!!!!.\n");
			return false;
		}

		RgbFrameData* tmp = new RgbFrameData;
		CDataTools::CloneBuf(tmp, in);
		m_listRgbData.push_back(tmp);

		return true;
	}
	SBOOL GetRgbData(RgbFrameData* out)
	{
		std::lock_guard<std::mutex> locker(m_lock);//构造里自动上锁，析构里自动解锁（涉及软件算法）

		if (NULL == m_hTofD)
		{
			printf("dev is closed!!!!!!!!!!!!!!!!!!!!!!!!!.\n");
			return false;
		}

		if (m_listRgbData.empty())
		{
			return false;
		}

		//先释放上一次存着的内存
		CDataTools::ReleaseBuf(m_pRgbData);
		SAFE_DELETE(m_pRgbData);

		//
		m_pRgbData = m_listRgbData.front();//返回第一个元素的值。
		m_listRgbData.pop_front();//删除第一个元素。

		memcpy(out, m_pRgbData, sizeof(m_pRgbData[0]));

		return true;
	}

	SBOOL PutImuData(ImuFrameData* in)
	{
		std::lock_guard<std::mutex> locker(m_lock);//构造里自动上锁，析构里自动解锁（涉及软件算法）

		if (NULL == m_hTofD)
		{
			printf("dev is closed!!!!!!!!!!!!!!!!!!!!!!!!!.\n");
			return false;
		}

		if (m_listImuDataMaxLen <= m_listImuData.size())//链表不要太长，以防内存爆掉
		{
			printf("imu list over flow!!!!!!!!!!!!!!!!!!!!!!!!!.\n");
			return false;
		}

		ImuFrameData* tmp = new ImuFrameData;
		CDataTools::CloneBuf(tmp, in);
		m_listImuData.push_back(tmp);

		return true;
	}
	SBOOL GetImuData(ImuFrameData* out)
	{
		std::lock_guard<std::mutex> locker(m_lock);//构造里自动上锁，析构里自动解锁（涉及软件算法）

		if (NULL == m_hTofD)
		{
			printf("dev is closed!!!!!!!!!!!!!!!!!!!!!!!!!.\n");
			return false;
		}

		if (m_listImuData.empty())
		{
			return false;
		}

		//先释放上一次存着的内存
		CDataTools::ReleaseBuf(m_pImuData);
		SAFE_DELETE(m_pImuData);

		//
		m_pImuData = m_listImuData.front();//返回第一个元素的值。
		m_listImuData.pop_front();//删除第一个元素。

		memcpy(out, m_pImuData, sizeof(m_pImuData[0]));

		return true;
	}


public:
	std::mutex m_lock;

	UINT32 m_nIndex;//设备索引
	HTOFD m_hTofD;//设备句柄
	TofDeviceInfo m_struCaps;//设备信息

	//TOF帧
	UINT32 m_nTofFrameRecvTotalCnt;//接收到的TOF帧记数
	UINT32 m_listTofDataMaxLen;//m_listTofData最大长度
	std::list<TofFrameData*> m_listTofData;//TOF帧缓存
	TofFrameData* m_pTofData;//给用户的一帧TOF帧缓存

	//RGB帧
	UINT32 m_nRgbFrameRecvTotalCnt;//接收到的RGB帧记数
	UINT32 m_listRgbDataMaxLen;//m_listRgbData最大长度
	std::list<RgbFrameData*> m_listRgbData;//RGB帧缓存
	RgbFrameData* m_pRgbData;//给用户的一帧RGB帧缓存

	//IMU帧
	UINT32 m_nImuFrameRecvTotalCnt;//接收到的Imu帧记数
	UINT32 m_listImuDataMaxLen;//m_listImuData最大长度
	std::list<ImuFrameData*> m_listImuData;//Imu帧缓存
	ImuFrameData* m_pImuData;//给用户的一帧IMU帧缓存


};


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

static bool VisualizePoint(PointData *pPointData, const UINT32 width, const UINT32 height)
{
	if ((NULL == pPointData) || (0 >= width) || (0 >= height) )
	{
		return false;
	}
	cv::Mat depth_image = cv::Mat::zeros(cv::Size(width, height), CV_16UC1);


	const UINT32 nCnt = width *height;

	pcl::PointCloud<pcl::PointXYZ>::Ptr pclpointcloud(new pcl::PointCloud<pcl::PointXYZ>);


	for (UINT32 nPos = 0; nPos < nCnt; nPos++)
	{
		// fprintf(fp, "%0.6f;%0.6f;%0.6f\n", pPointData[nPos].x, pPointData[nPos].y, pPointData[nPos].z);
		pcl::PointXYZ pt;
		

		

		if( pPointData[nPos].z > depth_range_min && pPointData[nPos].z < depth_range_max){

			pt.x = pPointData[nPos].x;
			pt.y = pPointData[nPos].y;
			pt.z = pPointData[nPos].z;

			unsigned short this_depth = pt.z * 1000;
			if(this_depth>65535) this_depth = 65535;
			if(this_depth<0) this_depth = 0;
			int v = nPos / width;
			int u = nPos % width;
			depth_image.at<unsigned short>(v, u) = this_depth;

			

		}else{

			pt.x = bad_point;
			pt.y = bad_point;
			pt.z = bad_point;

		}

		pclpointcloud->points.push_back(pt);
		

	}

	sensor_msgs::ImagePtr depth_msg = cv_bridge::CvImage(std_msgs::Header(),
                                                                            sensor_msgs::image_encodings::TYPE_16UC1,
                                                                            depth_image).toImageMsg();
	depth_msg->header.seq = tof_seq;
	depth_msg->header.frame_id = tof_frame_id;
	depth_msg->header.stamp = tof_timestamp;
	if (depth_publisher.getNumSubscribers() > 0) {
		// depth_publisher.publish(*depth_msg, *camera_info_); 
		depth_publisher.publish(depth_msg);
	}



    sensor_msgs::PointCloud2 msg_pointcloud_all;
    pcl::toROSMsg(*pclpointcloud, msg_pointcloud_all);
    msg_pointcloud_all.header.stamp = tof_timestamp;
    msg_pointcloud_all.header.seq = tof_seq;
	msg_pointcloud_all.header.frame_id = tof_frame_id;
    all_pointcloud_publisher_.publish(msg_pointcloud_all);

	return true;

}


 bool VisualizeGray(float* pGray, const UINT32 width, const UINT32 height){

	cv::Mat gray_image = cv::Mat::zeros(cv::Size(width, height), CV_16UC1);

	const int pixel_cnt = width*height;

	float K = 1.0;
	if(normalize_pub_image){
		const float max = Utils_FindMaxValue(pGray, pixel_cnt);
		// const float max = 2000;
		K = (65535 * 1.0 / max);

		printf("gray max: %f, K: %f \n", max, K);
	}

	for (int i = 0; i < pixel_cnt; i++)
	{
		unsigned short tmp = 0;
		if (0.001 < pGray[i])
		{
			tmp = (unsigned short)(pGray[i] * K );

			if(tmp>65535) tmp = 65535;
			if(tmp<0) tmp = 0;

			int v = i / width;
			int u = i % width;
			gray_image.at<unsigned short>(v, u) = tmp;
		}
	}
	// cv::namedWindow("Mat", CV_WINDOW_AUTOSIZE);    
	// cv::imshow("Mat", image);
	// cv::waitKey(1);

	sensor_msgs::ImagePtr gray_msg = cv_bridge::CvImage(std_msgs::Header(),
                                                                            sensor_msgs::image_encodings::TYPE_16UC1,
                                                                            gray_image).toImageMsg();
        
    gray_msg->header.seq = tof_seq;
	gray_msg->header.frame_id = tof_frame_id;
	gray_msg->header.stamp = tof_timestamp;
        // depth_msg->width = depth_img_.cols;
        // depth_msg->height = depth_img_.rows;
        // depth_msg->is_bigendian = false;
        // depth_msg->step = depth_msg->width * sizeof(uint16_t);
        if (gray_publisher.getNumSubscribers() > 0) {
            // depth_publisher.publish(*depth_msg, *camera_info_); 
            gray_publisher.publish(gray_msg);
        }

	return true;

}

 bool VisualizeRaw(float* pRaw, const UINT32 width, const UINT32 height){

	cv::Mat raw_image = cv::Mat::zeros(cv::Size(width, height), CV_16UC1);

	const int pixel_cnt = width*height;

	float K = 1.0;
	if(normalize_pub_image){
		const float max = Utils_FindMaxValue(pRaw, pixel_cnt);
		// const float max = 2000;
		K = (65535 * 1.0 / max);

		printf("raw max: %f, K: %f \n", max, K);
	}


	for (int i = 0; i < pixel_cnt; i++)
	{
		unsigned short tmp = 0;
		// if (0.001 < pRaw[i])
		// {
			tmp = (unsigned short)(pRaw[i] * K );

			if(tmp>65535) tmp = 65535;
			if(tmp<0) tmp = 0;

			int v = i / width;
			int u = i % width;
			raw_image.at<unsigned short>(v, u) = tmp;
		// }
	}
	

	// cv::namedWindow("Mat2", CV_WINDOW_AUTOSIZE);    
	// cv::imshow("Mat2", image);
	// cv::waitKey(1);


	sensor_msgs::ImagePtr raw_msg = cv_bridge::CvImage(std_msgs::Header(),
                                                                            sensor_msgs::image_encodings::TYPE_16UC1,
                                                                            raw_image).toImageMsg();
        
    raw_msg->header.seq = tof_seq;
	raw_msg->header.frame_id = tof_frame_id;
	raw_msg->header.stamp = tof_timestamp;
        // depth_msg->width = depth_img_.cols;
        // depth_msg->height = depth_img_.rows;
        // depth_msg->is_bigendian = false;
        // depth_msg->step = depth_msg->width * sizeof(uint16_t);
        if (raw_publisher.getNumSubscribers() > 0) {
            // depth_publisher.publish(*depth_msg, *camera_info_); 
            raw_publisher.publish(raw_msg);
        }

	return true;

}


static void Visualize( const unsigned int nCaptureIndex, TofFrameData *tofFrameData)
{
	// printf( "tt timeStamp:%lld", tofFrameData->timeStamp);

	tof_seq = nCaptureIndex;
	tof_timestamp = ros::Time::now();

	if (NULL != tofFrameData->pPointData)
	{
		VisualizePoint(tofFrameData->pPointData, tofFrameData->frameWidth, tofFrameData->frameHeight);
	}
	
	if (NULL != tofFrameData->pGrayData)
	{
		VisualizeGray((float*)tofFrameData->pGrayData, tofFrameData->frameWidth, tofFrameData->frameHeight);
	}

	// if ((NULL != tofFrameData->pRawData) && (0 < tofFrameData->nRawDataLen))
	// {
	// 	VisualizeRaw((float*)tofFrameData->pRawData, tofFrameData->frameWidth, tofFrameData->frameHeight);
	// }

}



static void CaptureTofFrame(const std::string& strDir, const unsigned int nCaptureIndex, TofFrameData *tofFrameData)
{
	const unsigned int nPixelCnt = tofFrameData->frameWidth * tofFrameData->frameHeight;
	char szFile[512] = { 0 };

	//
	if (NULL != tofFrameData->pPointData)
	{
		sprintf(szFile, "%s/%u-PointData.dat", strDir.c_str(), nCaptureIndex);
		Utils_SaveBufToFile(tofFrameData->pPointData, nPixelCnt  * sizeof(tofFrameData->pPointData[0]), szFile, false);

		sprintf(szFile, "%s/%u-PointData.txt", strDir.c_str(), nCaptureIndex);
		SavePointDataXYZText(tofFrameData->pPointData, tofFrameData->frameWidth, tofFrameData->frameHeight, szFile);
	}

	//
	if (NULL != tofFrameData->pGrayData)
	{
		sprintf(szFile, "%s/%u-Gray.%s", strDir.c_str(), nCaptureIndex, StringGrayFormat(tofFrameData->grayFormat));
		Utils_SaveBufToFile(tofFrameData->pGrayData, nPixelCnt * CaculateGrayPixelBytes(tofFrameData->grayFormat), szFile, false);

		sprintf(szFile, "%s/%u-Gray.u8", strDir.c_str(), nCaptureIndex);
		SaveGray_2_U8(tofFrameData->pGrayData, tofFrameData->frameWidth, tofFrameData->frameHeight, tofFrameData->grayFormat, szFile);
	}
	//
	if (NULL != tofFrameData->pConfidence)
	{
		sprintf(szFile, "%s/%u-Confidence.dat", strDir.c_str(), nCaptureIndex);
		Utils_SaveBufToFile(tofFrameData->pConfidence, nPixelCnt  * sizeof(tofFrameData->pConfidence[0]), szFile, false);

		sprintf(szFile, "%s/%u-Confidence.txt", strDir.c_str(), nCaptureIndex);
		SaveDepthText(tofFrameData->pConfidence, tofFrameData->frameWidth, tofFrameData->frameHeight, szFile, false);

		sprintf(szFile, "%s/%u-Confidence.u8", strDir.c_str(), nCaptureIndex);
		SaveGray_2_U8(tofFrameData->pConfidence, tofFrameData->frameWidth, tofFrameData->frameHeight, GRAY_FORMAT_FLOAT, szFile);
	}
	//
	if (NULL != tofFrameData->pRgbD)
	{
		sprintf(szFile, "%s/%u-RgbD.%s", strDir.c_str(), nCaptureIndex, "rgb");
		Utils_SaveBufToFile(tofFrameData->pRgbD, nPixelCnt * sizeof(tofFrameData->pRgbD[0]), szFile, false);
	}
	//
	if ((NULL != tofFrameData->pRawData) && (0 < tofFrameData->nRawDataLen))
	{
		sprintf(szFile, "%s/%u-Tof.%s", strDir.c_str(), nCaptureIndex, "raw");
		Utils_SaveBufToFile(tofFrameData->pRawData, tofFrameData->nRawDataLen, szFile, false);
	}
	//
	if ((NULL != tofFrameData->pExtData) && (0 < tofFrameData->nExtDataLen))
	{
		sprintf(szFile, "%s/%u-Tof.%s", strDir.c_str(), nCaptureIndex, "extdata");
		Utils_SaveBufToFile(tofFrameData->pExtData, tofFrameData->nExtDataLen, szFile, false);
	}
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
	CDevData* pDevData = (CDevData*)pUserData;
	const UINT32 width = tofFrameData->frameWidth;
	const UINT32 height = tofFrameData->frameHeight;

	pDevData->PutTofData(tofFrameData);
	pDevData->m_nTofFrameRecvTotalCnt++;

	return;
	//
	// const float fDepthZAvg = CalCenterPointDataZAvg(tofFrameData->pPointData, tofFrameData->frameWidth, tofFrameData->frameHeight);
	// printf("[%d], one TOF frame, time=%llu, center depthZ = %0.3f m.\n", pDevData->m_nTofFrameRecvTotalCnt, Utils_GetTickCount(), fDepthZAvg);

	// CaptureTofFrame(std::string("./Capture"), pDevData->m_nTofFrameRecvTotalCnt, tofFrameData);

}

static void fnRgbStream(RgbFrameData *rgbFrameData, void* pUserData)
{
	CDevData* pDevData = (CDevData*)pUserData;
	const UINT32 width = rgbFrameData->frameWidth;
	const UINT32 height = rgbFrameData->frameHeight;

	pDevData->PutRgbData(rgbFrameData);
	pDevData->m_nRgbFrameRecvTotalCnt++;

	return;
	//
	printf("[%d], one RGB frame, time=%llu, formatType=%d, %d.\n", pDevData->m_nRgbFrameRecvTotalCnt, Utils_GetTickCount(), rgbFrameData->formatType, rgbFrameData->formatTypeOrg);

	CaptureRgbFrame(std::string("./Capture"), pDevData->m_nRgbFrameRecvTotalCnt, rgbFrameData);

}

static void fnImuStream(ImuFrameData *imuFrameData, void* pUserData)
{
	CDevData* pDevData = (CDevData*)pUserData;

	pDevData->PutImuData(imuFrameData);
	pDevData->m_nImuFrameRecvTotalCnt++;

	return;
	//
	printf("[%d], one IMU frame, time=%llu.\n", pDevData->m_nImuFrameRecvTotalCnt, Utils_GetTickCount());

	CaptureImuFrame(std::string("./Capture"), pDevData->m_nImuFrameRecvTotalCnt, imuFrameData);

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
#if 1
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


static bool OpenStream(CDevData* pDevData, const TOF_MODE tofMode)
{
	HTOFD hTofD = pDevData->m_hTofD;
	TofDeviceInfo* pCaps = &(pDevData->m_struCaps);

	TOFRET retVal = TOFD_StartTofStream(hTofD, tofMode, fnTofStream, pDevData);
	if ((TOFRET_SUCCESS != retVal) && (TOFRET_SUCCESS_READING_CALIB != retVal))
	{
		printf("start TOF stream, [ FAILED ], retVal=0x%08x!!!!!!!!!!!!!!!!!!!!!\n\n", retVal);
		return false;
	}

	if (pCaps->bRgbSupported)//支持RGB的情况下
	{
		TOFRET retVal = TOFD_StartRgbStream(hTofD, fnRgbStream, pDevData);
		if (TOFRET_SUCCESS != retVal)
		{
			printf("start RGB stream, [ FAILED ], retVal=0x%08x!!!!!!!!!!!!!!!!!!!!!\n\n", retVal);
			return false;
		}
	}

	if (pCaps->bImuSupported)//支持IMU的情况下
	{
		TOFRET retVal = TOFD_StartImuStream(hTofD, fnImuStream, pDevData);
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
	if(ae_enable){
		if (pCaps->supportedTofExpMode & EXP_MODE_AUTO)
		{
			if (TOFRET_SUCCESS != (retVal = TOFD_SetTofAE(hTofD, true)))//按需，或者联系沟通后确认
			{
				printf("TOFD_SetTofAE failed, retVal=0x%08x.\n", retVal);
			}
		}
	}

	if(hd_enable){
		if (TOFRET_SUCCESS != (retVal = TOFD_SetTofHDRZ(hTofD, false)))//按需，或者联系沟通后确认
		{
			printf("TOFD_SetTofHDRZ failed, retVal=0x%08x.\n", retVal);
		}
	}
#endif

#if 1 //滤波，按需，或者联系沟通后确认
	if(filter_enable){
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





static TofDeviceDescriptor* g_pDevsDescList = NULL;
static UINT32 g_nDevNum = 0;

#define MAX_DEV_CNT (2)//这里只开了2个设备的空间，按需开
static CDevData* g_pDevData = NULL;

static SBOOL TofDevSdk_Init(SCHAR *pCfgFileDir)
{
	g_pDevsDescList = NULL;
	g_nDevNum = 0;
	g_pDevData = new CDevData[MAX_DEV_CNT];
	for (UINT32 i = 0; i < MAX_DEV_CNT; i++)
	{
		g_pDevData[i].Init();
	}

	TofDevInitParam struInitParam;
	memset(&struInitParam, 0, sizeof(struInitParam));
	strncpy(struInitParam.szDepthCalcCfgFileDir, pCfgFileDir, sizeof(struInitParam.szDepthCalcCfgFileDir) - 1);
	TOFD_Init(&struInitParam);

	printf("SDK Version: %s.\n", TOFD_GetSDKVersion());

	return true;
}
static SBOOL TofDevSdk_UnInit(void)
{
	TOFD_Uninit();

	g_pDevsDescList = NULL;
	g_nDevNum = 0;
	SAFE_DELETE_ARRY(g_pDevData);

	return true;
}
static UINT32 TofDevSdk_SearchDev(void)//返回设备个数
{
	g_pDevsDescList = NULL;
	g_nDevNum = 0;

	TOFD_SearchDevice(&g_pDevsDescList, &g_nDevNum);

	return g_nDevNum;
}
static SBOOL TofDevSdk_OpenDev(const UINT32 index)//index取值: 0 ~ (设备个数-1)
{
	if (NULL == g_pDevsDescList)
	{
		printf("dev list is empty.\n");
		return false;
	}
	if (0 == g_nDevNum)
	{
		printf("no device.\n");
		return false;
	}
	if ((index >= g_nDevNum) || (index >= MAX_DEV_CNT))
	{
		printf("index (%d) is out of range.\n", index);
		return false;
	}

	CDevData* pDevData = (g_pDevData + index);

	HTOFD hTofD = TOFD_OpenDevice(g_pDevsDescList + index, CallBackTofDeviceStatus, NULL);
	if (NULL == hTofD)
	{
		printf("Open Tof Device(index=%d) failed.\n", index);
		return false;
	}

	TofDeviceInfo struCaps;
	memset(&struCaps, 0, sizeof(struCaps));
	TOFRET retVal = TOFD_GetDeviceInfo(hTofD, &struCaps);
	if (TOFRET_SUCCESS != retVal)
	{
		printf("Get Device Info(index=%d), [ FAILED ], retVal=0x%08x!!!!!!!!!!!!!!!!!!!!!\n\n", index, retVal);
	}
	PrintDevInfo(&struCaps);

	pDevData->Init();
	pDevData->BindDev(index, hTofD, struCaps);

	const TOF_MODE tofMode = ChoseTofMode(struCaps.supportedTOFMode);//选择其中一种TOF模式出TOF数据

	//
	const bool bSuc = OpenStream(pDevData, tofMode);
	if (bSuc)
	{
		//成功后，等到数据流拿到后再去设置参数，这样才能保证生效
		while (0 >= pDevData->m_listTofData.size())
		{
			printf("Waiting for stream data...\n");
			std::this_thread::sleep_for(std::chrono::milliseconds(1000)); //单位是毫秒
		}

		printf("Get Or Set Some Params...\n");
		GetOrSetSomeParam(hTofD, &struCaps, tofMode);
	}
	else
	{
		CloseStream(hTofD, &struCaps);

		TOFD_CloseDevice(hTofD);

		pDevData->Init();
	}

	return bSuc;
}
static SBOOL TofDevSdk_CloseDev(const UINT32 index)//index取值: 0 ~ (设备个数-1)
{
	if ((index >= g_nDevNum) || (index >= MAX_DEV_CNT))
	{
		printf("index (%d) is out of range.\n", index);
		return false;
	}

	CDevData* pDevData = (g_pDevData + index);

	if (pDevData->m_hTofD)
	{
		CloseStream(pDevData->m_hTofD, &(pDevData->m_struCaps));

		TOFD_CloseDevice(pDevData->m_hTofD);

		pDevData->Init();
	}

	return true;
}
static SBOOL TofDevSdk_ReadTofData(const UINT32 index, TofFrameData* data)//index取值: 0 ~ (设备个数-1)
{
	if ((index >= g_nDevNum) || (index >= MAX_DEV_CNT))
	{
		printf("index (%d) is out of range.\n", index);
		return false;
	}

	CDevData* pDevData = (g_pDevData + index);

	TofFrameData outTmp;
	if (!pDevData->GetTofData(&outTmp))
	{
		return false;
	}

	memcpy(data, &outTmp, sizeof(outTmp));

	return true;
}
static SBOOL TofDevSdk_ReadRgbData(const UINT32 index, RgbFrameData* data)//index取值: 0 ~ (设备个数-1)
{
	if ((index >= g_nDevNum) || (index >= MAX_DEV_CNT))
	{
		printf("index (%d) is out of range.\n", index);
		return false;
	}

	CDevData* pDevData = (g_pDevData + index);

	RgbFrameData outTmp;
	if (!pDevData->GetRgbData(&outTmp))
	{
		return false;
	}

	memcpy(data, &outTmp, sizeof(outTmp));

	return true;
}
static SBOOL TofDevSdk_ReadImuData(const UINT32 index, ImuFrameData* data)//index取值: 0 ~ (设备个数-1)
{
	if ((index >= g_nDevNum) || (index >= MAX_DEV_CNT))
	{
		printf("index (%d) is out of range.\n", index);
		return false;
	}

	CDevData* pDevData = (g_pDevData + index);

	ImuFrameData outTmp;
	if (!pDevData->GetImuData(&outTmp))
	{
		return false;
	}

	memcpy(data, &outTmp, sizeof(outTmp));

	return true;
}



static void ThreadTofStream(const UINT32 index, const UINT64 nDuration)
{
	TofFrameData struData;

	UINT32 nLoop = 0;
	const UINT64 start_tick = Utils_GetTickCount();

	// while (nDuration > (Utils_GetTickCount() - start_tick))
	while(ros::ok())
	{
		if (TofDevSdk_ReadTofData(index, &struData))
		{
			//todo
			nLoop++;

	        const float fDepthZAvg = CalCenterPointDataZAvg(struData.pPointData, struData.frameWidth, struData.frameHeight);
			printf("[%d], one TOF frame, %d x %d time=%llu, center depthZ = %0.3f m.\n", nLoop, struData.frameWidth, struData.frameHeight, Utils_GetTickCount(), fDepthZAvg);

			Visualize(nLoop, &struData);

			//CaptureTofFrame(std::string("./Capture"), nLoop, &struData);
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(5)); //单位是毫秒
	}
}
static void ThreadRgbStream(const UINT32 index, const UINT64 nDuration)
{
	RgbFrameData struData;

	UINT32 nLoop = 0;
	const UINT64 start_tick = Utils_GetTickCount();

	// while (nDuration > (Utils_GetTickCount() - start_tick))
	while(ros::ok())
	{
		if (TofDevSdk_ReadRgbData(index, &struData))
		{
			//todo
			nLoop++;
			printf("[%d], one RGB frame, time=%llu, formatType=%d, %d.\n", nLoop, Utils_GetTickCount(), struData.formatType, struData.formatTypeOrg);

			//CaptureRgbFrame(std::string("./Capture"), nLoop, &struData);
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(5)); //单位是毫秒
	}
}
static void ThreadImuStream(const UINT32 index, const UINT64 nDuration)
{
	ImuFrameData struData;

	UINT32 nLoop = 0;
	const UINT64 start_tick = Utils_GetTickCount();

	// while (nDuration > (Utils_GetTickCount() - start_tick))
	while(ros::ok())
	{
		if (TofDevSdk_ReadImuData(index, &struData))
		{
			//todo
			nLoop++;
			printf("[%d], one IMU frame, time=%llu.\n", nLoop, Utils_GetTickCount());

			//CaptureImuFrame(std::string("./Capture"), nLoop, &struData);
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(5)); //单位是毫秒
	}
}


int main(int argc, char** argv)
{

	ros::init(argc, argv, "~");
    ros::NodeHandle nh;
    ros::NodeHandle priv_nh("~");
    ros::NodeHandle camera_nh("/shunyu/");

	priv_nh.param("normalize_pub_image", normalize_pub_image, false);
	priv_nh.param("ae_enable", ae_enable, true);
	priv_nh.param("hd_enable", hd_enable, true);
	priv_nh.param("filter_enable", filter_enable, true);

	stringstream ss;
	ss.str("");
    ss << "all_pointcloud";
    all_pointcloud_publisher_ = camera_nh.advertise<sensor_msgs::PointCloud2>(ss.str(), 1);
	ss.str("");
    ss << "gray_image";
	gray_publisher = camera_nh.advertise<sensor_msgs::Image>(ss.str(), 1);
	ss.str("");
    ss << "depth_image";
	depth_publisher = camera_nh.advertise<sensor_msgs::Image>(ss.str(), 1);
	ss.str("");
    ss << "raw_image";
	raw_publisher = camera_nh.advertise<sensor_msgs::Image>(ss.str(), 1);


	printf("*********************start test device*************************\n");

	TofDevSdk_Init((SCHAR*)("./parameter"));

	const UINT32 dev_num = TofDevSdk_SearchDev();
	if (0 < dev_num)
	{
		const UINT32 index = 0;//这里只简单处理第一个设备
		if (TofDevSdk_OpenDev(index))
		{
			const UINT64 nDuration = 10 * 1000;//线程持续时间长度（毫秒），按需修改
			std::thread thread_tof = std::thread(ThreadTofStream, index, nDuration);
			std::thread thread_rgb = std::thread(ThreadRgbStream, index, nDuration);
			std::thread thread_imu = std::thread(ThreadImuStream, index, nDuration);
			thread_tof.join();
			thread_rgb.join();
			thread_imu.join();

			TofDevSdk_CloseDev(index);
		}
	}
	else
	{
		printf("can not find tof device!\n");
	}

	printf("*********************stop test device*********************\n");
	TofDevSdk_UnInit();

#ifdef WIN32
	system("pause");
#endif
	return 0;
}
