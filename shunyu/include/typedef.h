#ifndef __TYPEDEF_H__
#define __TYPEDEF_H__


typedef unsigned int       UINT32;
typedef unsigned short     UINT16;
typedef unsigned char	   UINT8;
typedef unsigned long long UINT64;

typedef signed int    	   SINT32;
typedef signed short  	   SINT16;
typedef signed char		   SINT8;
typedef signed long long   SINT64;

typedef float              FLOAT32;
typedef double 		       FLOAT64;
typedef bool			   SBOOL;
typedef char			   SCHAR;	


#ifndef TRUE
    #define TRUE 1
#endif

#ifndef FALSE
    #define FALSE 0
#endif

#ifndef NULL
    #define NULL 0
#endif


#define MAKE_UNIQUE_ID(major, sub, a, b) ((major<<24) | (sub<<16) | (a<<8) | (b))


typedef enum tagTOF_MODE
{
	//双频
	TOF_MODE_STERO_5FPS  = 0x00000001,
	TOF_MODE_STERO_10FPS = 0x00000002,
	TOF_MODE_STERO_15FPS = 0x00000004,
	TOF_MODE_STERO_30FPS = 0x00000008,
	TOF_MODE_STERO_45FPS = 0x00000010,
	TOF_MODE_STERO_60FPS = 0x00000020,

	//单频
	TOF_MODE_MONO_5FPS   = 0x00000040,
	TOF_MODE_MONO_10FPS  = 0x00000080,
	TOF_MODE_MONO_15FPS  = 0x00000100,
	TOF_MODE_MONO_30FPS  = 0x00000200,
	TOF_MODE_MONO_45FPS  = 0x00000400,
	TOF_MODE_MONO_60FPS  = 0x00000800,

	//HDRZ：这几个模式代表具有raw数据的HDRZ融合的
	TOF_MODE_HDRZ_5FPS   = 0x00001000,
	TOF_MODE_HDRZ_10FPS  = 0x00002000,
	TOF_MODE_HDRZ_15FPS  = 0x00004000,
	TOF_MODE_HDRZ_30FPS  = 0x00008000,
	TOF_MODE_HDRZ_45FPS  = 0x00010000,
	TOF_MODE_HDRZ_60FPS  = 0x00020000,

	//一种频
	TOF_MODE_5FPS        = 0x00040000,
	TOF_MODE_10FPS       = 0x00080000,
	TOF_MODE_20FPS       = 0x00100000,
	TOF_MODE_30FPS       = 0x00200000,
	TOF_MODE_45FPS       = 0x00400000,
	TOF_MODE_60FPS       = 0x00800000,

	//名称待定
	TOF_MODE_ADI_1M5     = 0x01000000,
	TOF_MODE_ADI_5M      = 0x02000000,


}TOF_MODE;


typedef enum tagTOF_FILTER
{
	TOF_FILTER_RemoveFlyingPixel   = 0x00000001,
	TOF_FILTER_AdaptiveNoiseFilter = 0x00000002,
	TOF_FILTER_InterFrameFilter    = 0x00000004,
	TOF_FILTER_PointCloudFilter    = 0x00000008,
	TOF_FILTER_StraylightFilter    = 0x00000010,
	TOF_FILTER_CalcIntensities     = 0x00000020,
	TOF_FILTER_MPIFlagAverage      = 0x00000040,
	TOF_FILTER_MPIFlagAmplitude    = 0x00000080,
	TOF_FILTER_MPIFlagDistance     = 0x00000100,
	TOF_FILTER_ValidateImage       = 0x00000200,
	TOF_FILTER_SparsePointCloud    = 0x00000400,
	TOF_FILTER_Average             = 0x00000800,
	TOF_FILTER_Median              = 0x00001000,
	TOF_FILTER_Confidence          = 0x00002000,
	TOF_FILTER_MPIFilter           = 0x00004000,
	TOF_FILTER_PointCloudCorrect   = 0x00008000,
	TOF_FILTER_LineRecognition     = 0x00010000,

}TOF_FILTER;


typedef struct tagTofFilterCfg_RemoveFlyingPixel
{
	FLOAT32 f0;
	FLOAT32 f1;
	FLOAT32 nearDist;
	FLOAT32 farDist;
}TofFilterCfg_RemoveFlyingPixel;

typedef struct tagTofFilterCfg_AdaptiveNoiseFilter
{
	SINT32 kernel;
	SINT32 thr_value;
}TofFilterCfg_AdaptiveNoiseFilter;

typedef struct tagTofFilterCfg_InterFrameFilter
{
	FLOAT32 MotionDetectGain;
	FLOAT32 MotionDetectThre;
	FLOAT32 interframeGain1;
	FLOAT32 interframeGain2;
}TofFilterCfg_InterFrameFilter;

typedef struct tagTofFilterCfg_PointCloudFilter
{
	SINT32 kernel;
}TofFilterCfg_PointCloudFilter;

typedef struct tagTofFilterCfg_StraylightFilter
{
	FLOAT32 dist[16];
	FLOAT32 Thre[16];
}TofFilterCfg_StraylightFilter;

typedef struct tagTofFilterCfg_CalcIntensities
{
	UINT8 szRes[4];//预留,4字节对齐
}TofFilterCfg_CalcIntensities;

typedef struct tagTofFilterCfg_MPIFlagAverage
{
	UINT8 szRes[4];//预留,4字节对齐
}TofFilterCfg_MPIFlagAverage;

typedef struct tagTofFilterCfg_MPIFlagAmplitude
{
	UINT8 szRes[4];//预留,4字节对齐
}TofFilterCfg_MPIFlagAmplitude;

typedef struct tagTofFilterCfg_MPIFlagDistance
{
	UINT8 szRes[4];//预留,4字节对齐
}TofFilterCfg_MPIFlagDistance;

typedef struct tagTofFilterCfg_ValidateImage
{
	UINT8 szRes[4];//预留,4字节对齐
}TofFilterCfg_ValidateImage;

typedef struct tagTofFilterCfg_SparsePointCloud
{
	UINT8 szRes[4];//预留,4字节对齐
}TofFilterCfg_SparsePointCloud;

typedef struct tagTofFilterCfg_Average
{
	UINT8 szRes[4];//预留,4字节对齐
}TofFilterCfg_Average;

typedef struct tagTofFilterCfg_Median
{
	UINT8 szRes[4];//预留,4字节对齐
}TofFilterCfg_Median;

typedef struct tagTofFilterCfg_Confidence
{
	FLOAT32 Thre;
}TofFilterCfg_Confidence;

typedef struct tagTofFilterCfg_MPIFilter
{
	FLOAT32 amplitudeThreshold;
	FLOAT32 distanceThreshold;
	FLOAT32 noiseDistance;
}TofFilterCfg_MPIFilter;

typedef struct tagTofFilterCfg_PointCloudCorrect
{
	FLOAT32 dipAngle;//扫地机上模组下斜的角度
	FLOAT32 topGroundDistance;//模组距地面的高度
	//FLOAT32 highThreshold;//电线高度
	FLOAT32 threshold1;
	FLOAT32 threshold2;
}TofFilterCfg_PointCloudCorrect;

typedef struct tagTofFilterCfg
{
	TOF_FILTER type;//某一种滤波类型，一般是输入参数（只读）

	SBOOL bEnable;//是否启用，可以是输入或者输出参数
	UINT8 szRes[3];//预留,4字节对齐
	union
	{
		TofFilterCfg_RemoveFlyingPixel struRemoveFlyingPixel;
		TofFilterCfg_AdaptiveNoiseFilter struAdaptiveNoiseFilter;
		TofFilterCfg_InterFrameFilter struInterFrameFilter;
		TofFilterCfg_PointCloudFilter struPointCloudFilter;
		TofFilterCfg_StraylightFilter struStraylightFilter;
		TofFilterCfg_CalcIntensities struCalcIntensities;
		TofFilterCfg_MPIFlagAverage struMPIFlagAverage;
		TofFilterCfg_MPIFlagAmplitude struMPIFlagAmplitude;
		TofFilterCfg_MPIFlagDistance struMPIFlagDistance;
		TofFilterCfg_ValidateImage struValidateImage;
		TofFilterCfg_SparsePointCloud struSparsePointCloud;
		TofFilterCfg_Average struAverage;
		TofFilterCfg_Median struMedian;
		TofFilterCfg_Confidence struConfidence;
		TofFilterCfg_MPIFilter struMPIFilter;
		TofFilterCfg_PointCloudCorrect struPointCloudCorrect;
	}uCfg;//某种滤波的具体配置，可以是输入或者输出参数

}TofFilterCfg;

typedef enum tagEXP_MODE
{
	EXP_MODE_MANUAL = 0x00000001,//手动曝光
	EXP_MODE_AUTO   = 0x00000002,//自动曝光(AE)
}EXP_MODE;



//灰度数据格式
typedef enum tagGRAY_FORMAT
{
	GRAY_FORMAT_UINT8  = 0,//8位数据
	GRAY_FORMAT_UINT16,//无符号16位数据
	GRAY_FORMAT_FLOAT,//浮点型数据
	GRAY_FORMAT_BGRD,//每像素32位， 按B/G/R/D顺序存放

}GRAY_FORMAT;


typedef struct tagPointData
{
	FLOAT32 x;
	FLOAT32 y;
	FLOAT32 z;
}PointData;


//TOF Expouse
typedef struct tagTofExpouse
{
	UINT32  	nCurrent;//当前值，可读写
	UINT32  	nDefault;//默认值，只读
	UINT32  	nStep;//步进值，只读
	UINT32  	nMax;//最大值，只读
	UINT32  	nMin;//最小值，只读
}TofExpouse;


//TOF Expouse Group1
typedef struct tagTofExpouseGroup1
{
	TofExpouse exp;//曝光参数
}TofExpouseGroup1;


//TOF Expouse Group2
typedef struct tagTofExpouseGroup2
{
	TofExpouse exp_AEF;//自动曝光帧曝光参数
	TofExpouse exp_FEF;//固定曝光帧曝光参数
}TofExpouseGroup2;

//TOF Expouse Items
typedef struct tagTofExpouseItems
{
	UINT32 nIndex;//1---g1有效, 2---g2有效

	union
	{
		//[第1种]: 仅适用于只有单频或者双频raw数据的时候
		TofExpouseGroup1 g1;//曝光参数

		//[第2种]: 仅适用于具有自动曝光帧和固定曝光帧的raw数据的时候（帧内HDRZ融合时）
		TofExpouseGroup2 g2;//曝光参数
	}uParam;

}TofExpouseItems;

//TOF Expouse Current Group1
typedef struct tagTofExpouseCurrentGroup1
{
	UINT32 exp;//曝光值
}TofExpouseCurrentGroup1;


//TOF Expouse Current Group2
typedef struct tagTofExpouseCurrentGroup2
{
	UINT32 exp_AEF;//自动曝光帧曝光值
	UINT32 exp_FEF;//固定曝光帧曝光值
}TofExpouseCurrentGroup2;

//TOF Expouse Current Items
typedef struct tagTofExpouseCurrentItems
{
	UINT32 nIndex;//1---g1有效, 2---g2有效

	union
	{
		//[第1种]: 仅适用于只有单频或者双频raw数据的时候
		TofExpouseCurrentGroup1 g1;//曝光值

		//[第2种]: 仅适用于具有自动曝光帧和固定曝光帧的raw数据的时候（帧内HDRZ融合时）
		TofExpouseCurrentGroup2 g2;//曝光值
	}uParam;

}TofExpouseCurrentItems;


//TOF Expouse Range Group1
typedef struct tagTofExpouseRangeGroup1
{
	UINT32 min;//曝光值(最小)
	UINT32 max;//曝光值(最大)
}TofExpouseRangeGroup1;


//TOF Expouse Range Group2
typedef struct tagTofExpouseRangeGroup2
{
	UINT32 min_AEF;//自动曝光帧曝光值(最小)
	UINT32 max_AEF;//自动曝光帧曝光值(最大)
	
	UINT32 min_FEF;//固定曝光帧曝光值(最小)
	UINT32 max_FEF;//固定曝光帧曝光值(最大)
}TofExpouseRangeGroup2;


//TOF Expouse Range Items
typedef struct tagTofExpouseRangeItems
{
	UINT32 nIndex;//1---g1有效, 2---g2有效

	union
	{
		//[第1种]: 仅适用于只有单频或者双频raw数据的时候
		TofExpouseRangeGroup1 g1;//曝光范围

		//[第2种]: 仅适用于具有自动曝光帧和固定曝光帧的raw数据的时候（帧内HDRZ融合时）
		TofExpouseRangeGroup2 g2;//曝光范围
	}uParam;

}TofExpouseRangeItems;


//自定义参数客户识别号
typedef enum tagCUSTOM_PARAM_GUEST_ID
{
	CUSTOM_PARAM_GUEST_ID_1 = 1,//客户1
	CUSTOM_PARAM_GUEST_ID_2 = 2,//客户2

}CUSTOM_PARAM_GUEST_ID;

//客户1自定义的参数
typedef struct tagCustomParamGuest1
{
	SINT32 quantileThreshold;//AE 比例
	FLOAT32 referenceAmplitude;//参考幅度
	FLOAT32 amplitudeThreshold;//幅度阈值
	UINT8 szRes[496];//总长508字节，4字节对齐，预留
}CustomParamGuest1;

//客户2自定义的参数
typedef struct tagCustomParamGuest2
{
	UINT8 szRes[508];//总长508字节，4字节对齐，预留
}CustomParamGuest2;


//客户自定义的参数
typedef struct tagGuestCustomParam
{
	CUSTOM_PARAM_GUEST_ID id;//输入参数，只读

	union
	{
		CustomParamGuest1 p1;//当id为CUSTOM_PARAM_GUEST_ID_1时有效；
		CustomParamGuest2 p2;//当id为CUSTOM_PARAM_GUEST_ID_2时有效；


		UINT8 data[508];//限定联合体为508字节长度（该字段不使用，仅用于数据结构长度定义）
	}uParam;
}GuestCustomParam;




//TOF模组内参和畸变
typedef struct tagTofModuleLensParameter
{
	FLOAT32 fx;
	FLOAT32 fy;
	FLOAT32 cx;
	FLOAT32 cy;
	FLOAT32 k1;
	FLOAT32 k2;
	FLOAT32 p1;
	FLOAT32 p2;
	FLOAT32 k3;
	//FLOAT32 k4;
}TofModuleLensParameter;

//TOF模组标定数据
typedef struct tagTofCalibData
{
	UINT8* pData;//指向标定数据
	UINT32 nDataLen;//pData内标定数据长度
}TofCalibData;

typedef struct tagTofRawData
{
	//RAW数据
	UINT8* pRaw;//一帧RAW数据
	UINT32 nRawLen;//RAW数据长度（字节数）

	//RAW数据其他属性参数
	FLOAT32 fTemperature;//出RAW数据时模组温度（注意：部分型号模组不需要该字段、部分模组RAW数据自带该数据，那么可以输入0值）

}TofRawData;


typedef struct tagExterntionHooks
{
	void* pUserData;//用户自定义数据

	/**********用于提前送出计算出来的曝光值**********/
	//@    pExp:  计算出的曝光值信息;
	//@    user_data:  用户自定义数据，与pUserData属于同一个;
	//@    【特别注意】:  对于在该回调函数内调用TOFM_XXX接口时，只允许调用软件算法部分接口，否则会死锁！！！！！
	void(*RecvTofExpTime)(TofExpouseCurrentItems* pExp, void*user_data);//根据模组实际情况选择是否实现


}ExterntionHooks;



#endif //__TYPEDEF_H__


