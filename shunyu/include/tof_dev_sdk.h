#ifndef __TOF_DEVICE_H__
#define __TOF_DEVICE_H__

#include "typedef.h"
#include "tof_error.h"

#ifdef WIN32
    #ifdef TOF_DEVICE_SDK_EXPORT
        #define TOFDDLL __declspec(dllexport)
    #else
        #define TOFDDLL __declspec(dllimport)
    #endif
#else
    #define TOFDDLL 
#endif


typedef enum tagTOF_DEV_TYPE
{
	TOF_DEV_CLEANER01A         = MAKE_UNIQUE_ID('C', 0x01, 'A', 0x00),//Cleaner01A
	TOF_DEV_CLEANER01APLUS     = MAKE_UNIQUE_ID('C', 0x01, 'A', 0x01),//Cleaner01A（Plus版）
	TOF_DEV_CLEANER01A_NET     = MAKE_UNIQUE_ID('C', 0x01, 'A', 0x02),//Cleaner01A（网络版）
	TOF_DEV_CLEANER01B         = MAKE_UNIQUE_ID('C', 0x01, 'B', 0x00),//Cleaner01B
	TOF_DEV_CLEANER01D         = MAKE_UNIQUE_ID('C', 0x01, 'D', 0x00),//Cleaner01D
	TOF_DEV_CLEANER01D_NET     = MAKE_UNIQUE_ID('C', 0x01, 'D', 0x01),//Cleaner01D（网络版）
	TOF_DEV_CLEANER01E_NET     = MAKE_UNIQUE_ID('C', 0x01, 'E', 0x01),//Cleaner01E（网络版）
	TOF_DEV_CLEANER01F         = MAKE_UNIQUE_ID('C', 0x01, 'F', 0x00),//Cleaner01F
	TOF_DEV_CLEANER01G         = MAKE_UNIQUE_ID('C', 0x01, 'G', 0x00),//Cleaner01G
	TOF_DEV_CLEANER02A         = MAKE_UNIQUE_ID('C', 0x02, 'A', 0x00),//Cleaner02A
	TOF_DEV_CLEANER02A_NET     = MAKE_UNIQUE_ID('C', 0x02, 'A', 0x01),//Cleaner02A（网络版）
	TOF_DEV_MARS01A            = MAKE_UNIQUE_ID('M', 0x01, 'A', 0x00),//Mars01A
	TOF_DEV_MARS01B            = MAKE_UNIQUE_ID('M', 0x01, 'B', 0x00),//Mars01B
	TOF_DEV_MARS01C            = MAKE_UNIQUE_ID('M', 0x01, 'C', 0x00),//Mars01C
	TOF_DEV_MARS01D            = MAKE_UNIQUE_ID('M', 0x01, 'D', 0x00),//Mars01D
	TOF_DEV_MARS01E            = MAKE_UNIQUE_ID('M', 0x01, 'E', 0x00),//Mars01E
	TOF_DEV_MARS04             = MAKE_UNIQUE_ID('M', 0x04, 0x00, 0x00),//Mars04
	TOF_DEV_MARS04A            = MAKE_UNIQUE_ID('M', 0x04, 'A', 0x00),//Mars04A
	TOF_DEV_MARS04B            = MAKE_UNIQUE_ID('M', 0x04, 'B', 0x00),//Mars04B
	TOF_DEV_MARS05             = MAKE_UNIQUE_ID('M', 0x05, 0x00, 0x00),//Mars05
	TOF_DEV_MARS05A            = MAKE_UNIQUE_ID('M', 0x05, 'A', 0x00),//Mars05A
	TOF_DEV_MARS05B            = MAKE_UNIQUE_ID('M', 0x05, 'B', 0x00),//Mars05B
	TOF_DEV_MARS05B_BCTC       = MAKE_UNIQUE_ID('M', 0x05, 'B', 0x01),//Mars05B(BCTC版本)
	TOF_DEV_MARS05B_BCTC_SUNNY = MAKE_UNIQUE_ID('M', 0x05, 'B', 0x02),//Mars05B(BCTC版本_sunny)
	TOF_DEV_USBTOF_HI          = MAKE_UNIQUE_ID('U', 'T', 'H', 0x00),//UsbTof-Hi
	TOF_DEV_DREAM              = MAKE_UNIQUE_ID('D', 'R', 'M', 0x00),//DREAM
	TOF_DEV_HOT002             = MAKE_UNIQUE_ID('H', 'O', 'T', 0x02),//HOT002

}TOF_DEV_TYPE;



typedef struct tagRgbDData
{
	UINT8 r;
	UINT8 g;
	UINT8 b;
}RgbDData;


typedef struct tagTofFrameData
{
	UINT64  timeStamp;
	UINT32  frameWidth;
	UINT32  frameHeight;

	PointData *pPointData;//点云数据

	GRAY_FORMAT grayFormat;//pGrayData内数据格式
	void   *pGrayData;//灰度数据

	FLOAT32 *pConfidence;//置信度（支持的板子才可以）

	RgbDData* pRgbD;//RgbD数据

	void   *pRawData;//raw数据（支持raw数据的板子才可以）
	UINT32 nRawDataLen;//pRawData内raw数据长度，字节数

	//扩展数据(一般针对客户特殊需求)，不同设备/不同客户均不同，可能为空；
	void   *pExtData;//扩展数据
	UINT32 nExtDataLen;//pExtData内扩展数据长度，字节数

}TofFrameData;


typedef enum tagCOLOR_FORMAT
{
	//MJPG格式
	COLOR_FORMAT_MJPG   = MAKE_UNIQUE_ID('M', 'J', 'P', 'G'),

	//H264格式
	COLOR_FORMAT_H264   = MAKE_UNIQUE_ID('H', '2', '6', '4'),

	//YUV格式
	COLOR_FORMAT_YUV422 = MAKE_UNIQUE_ID('Y', 'U', 'V', 0x22),
	COLOR_FORMAT_YUYV   = MAKE_UNIQUE_ID('Y', 'U', 'Y', 'V'),
	COLOR_FORMAT_I420   = MAKE_UNIQUE_ID('I', '4', '2', '0'),
	COLOR_FORMAT_YV12   = MAKE_UNIQUE_ID('Y', 'V', '1', '2'),
	COLOR_FORMAT_NV12   = MAKE_UNIQUE_ID('N', 'V', '1', '2'),
	COLOR_FORMAT_NV21   = MAKE_UNIQUE_ID('N', 'V', '2', '1'),

	//RGB格式
	COLOR_FORMAT_BGR    = MAKE_UNIQUE_ID('B', 'G', 'R', 0x00), //RGB24（每个像素占3个字节，按照B、G、R的顺序存放）
	COLOR_FORMAT_RGB    = MAKE_UNIQUE_ID('R', 'G', 'B', 0x00), //RGB24（每个像素占3个字节，按照R、G、B的顺序存放）
	COLOR_FORMAT_BGRA   = MAKE_UNIQUE_ID('B', 'G', 'R', 'A'), //RGB32（每个像素占4个字节，按照B、G、R、A的顺序存放）
	COLOR_FORMAT_RGBA   = MAKE_UNIQUE_ID('R', 'G', 'B', 'A'), //RGB32（每个像素占4个字节，按照R、G、B、A的顺序存放）

}COLOR_FORMAT;


typedef enum tagRgbVideoControlProperty
{
	RgbVideoControl_Exposure = MAKE_UNIQUE_ID('E', 'X', 'P', 0x00),//RGB模组的曝光属性
	RgbVideoControl_Gain     = MAKE_UNIQUE_ID('G', 'A', 'I', 'N'),//RGB模组的增益属性

}RgbVideoControlProperty;

typedef enum tagRgbVideoControlFlags
{
	RgbVideoControlFlags_Auto   = MAKE_UNIQUE_ID('A', 'U', 'T', 'O'),//自动
	RgbVideoControlFlags_Manual = MAKE_UNIQUE_ID('M', 'A', 'N', 'U'),//手动

}RgbVideoControlFlags;


typedef struct tagRgbVideoControl
{
	SINT32 lDefault;//默认值
	SINT32 lStep;//步进值
	SINT32 lMax;//最大值
	SINT32 lMin;//最小值
	SINT32 lCapsFlags;//支持的值，是RgbVideoControlFlags的一种或多种组合

	SINT32 lCurrent;//当前值
	RgbVideoControlFlags lFlags;//当前Flag值

}RgbVideoControl;


typedef struct tagRgbData
{
	UINT8 r;
	UINT8 g;
	UINT8 b;
}RgbData;


typedef struct tagRgbFrameData
{
	UINT64  timeStamp;
	UINT32  frameWidth;
	UINT32  frameHeight;

	COLOR_FORMAT formatType;//指明pFrameData内数据帧的格式
	COLOR_FORMAT formatTypeOrg;//指明pFrameData内数据帧的格式(编码压缩之前的格式)
	UINT32  nFrameLen;
	UINT8*  pFrameData;

	//扩展数据(一般针对客户特殊需求)，不同设备/不同客户均不同，可能为空；
	void   *pExtData;//扩展数据
	UINT32 nExtDataLen;//pExtData内扩展数据长度，字节数

}RgbFrameData;

typedef struct tagImuFrameData
{
	UINT64 timeStamp;

	FLOAT32 accelData_x;
	FLOAT32 accelData_y;
	FLOAT32 accelData_z;

	FLOAT32 gyrData_x;
	FLOAT32 gyrData_y;
	FLOAT32 gyrData_z;

	FLOAT32 magData_x;
	FLOAT32 magData_y;
	FLOAT32 magData_z;

}ImuFrameData;


typedef struct tagTofDevInitParam
{
	SCHAR szDepthCalcCfgFileDir[200];//深度计算所需配置文件的目录，如home/user/temp
	UINT8 nLogLevel;//日志打印级别
	SCHAR szHostIPAddr[32];		//本地主机网卡IP地址

}TofDevInitParam;



typedef struct tagTofDeviceDescriptor
{
	void*  	hDevice;
	void*  	hDriver;

}TofDeviceDescriptor;

typedef struct tagTofDeviceInfo
{
	//BASIC information
	TOF_DEV_TYPE devType;//用于区分是哪款设备
	SCHAR szDevName[32];
	SCHAR szDevId[64];//设备/模块的序列号（标识设备唯一性）
	SCHAR szFirmwareVersion[32];//固件版本信息
	
	//TOF
	UINT32 supportedTOFMode;//TOF_MODE的组合
	UINT32 tofResWidth;
	UINT32 tofResHeight;
	GRAY_FORMAT grayFormat;//灰度数据格式
	
	//TOF Expouse
	UINT32 supportedTofExpMode;//EXP_MODE的组合

	//TOF Filter
	UINT32 supportedTOFFilter; //TOF_FILTER的组合

	//TOF HDRZ
	SBOOL bTofHDRZSupported;
	UINT8 byRes1[3];//字节对齐，预留

	//TOF RemoveINS
	SBOOL bTofRemoveINSSupported;//[该字段已作废]
	UINT8 byRes5[3];//字节对齐，预留

	//TOF MPIFlag
	SBOOL bTofMPIFlagSupported;//[该字段已作废]
	UINT8 byRes6[3];//字节对齐，预留

	//RGB
	SBOOL bRgbSupported;
	UINT8 byRes2[3];//字节对齐，预留
	COLOR_FORMAT rgbColorFormat;//传出的RGB数据格式
	COLOR_FORMAT rgbColorFormatOrg;//传出的RGB数据格式(编码压缩之前的格式)
	UINT32 rgbResWidth;
	UINT32 rgbResHeight;
	UINT32 supportedRgbProperty;// RgbVideoControlProperty的组合

	//RGBD
	SBOOL bRgbDSupported;
	UINT8 byRes3[3];//字节对齐，预留

	//IMU
	SBOOL bImuSupported;
	UINT8 byRes4[3];//字节对齐，预留

	//远程抓图
	SBOOL bRemoteCaptureSupported;
	//固件升级
	SBOOL bUpgradeFirmwareSupported;
	//设备重启
	SBOOL bRebootDevSupported;
	//主从机间同步时间
	SBOOL bMasterSlaveSyncTimeSupported;

	//

}TofDeviceInfo;

typedef struct tagTofDeviceParam
{
	FLOAT32 fBoardTemp;//主板温度(需要设备支持)
	FLOAT32 fSensorTemp;//senseor温度(需要设备支持)
	FLOAT32 fImuTemp;//Imu温度(需要设备支持)
}TofDeviceParam;

typedef struct tagTofDeviceTemperature
{
	FLOAT32 fBoardTemp;//主板温度(需要设备支持)
	FLOAT32 fSensorTemp;//senseor温度(需要设备支持)
	FLOAT32 fImuTemp;//Imu温度(需要设备支持)
}TofDeviceTemperature;

typedef struct tagNetDevInfo
{
	SBOOL bDHCP;//是否是自动获取IP
	UINT8 byRes[3];//字节对齐，预留
	SCHAR szIPv4Address[32];//设备IP地址
	SCHAR szIPv4SubnetMask[32];//设备子网掩码
	SCHAR szIPv4Gateway[32];//设备网关
	SCHAR szMAC[32];//设备MAC地址

}NetDevInfo_t;

typedef struct tagRemoteCapture
{
	UINT8 szRes[4];//预留,4字节对齐
}RemoteCapture;


//RGB模组内参和畸变
typedef struct tagRgbModuleLensParameter
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
}RgbModuleLensParameter;

//固件升级的实时状态
typedef enum tagFIRMWARE_UPGRADE_STATUS
{
	FIRMWARE_UPGRADE_STATUS_FINISHED   = 1,//升级完成
	FIRMWARE_UPGRADE_STATUS_RUNNING    = 2,//正在升级
	FIRMWARE_UPGRADE_STATUS_FAILED     = 3,//升级失败
	FIRMWARE_UPGRADE_STATUS_UNKNOWN    = 4,//升级失败(未知错误)
	FIRMWARE_UPGRADE_STATUS_ERROR_DATA = 5,//升级失败(固件包错误)
	FIRMWARE_UPGRADE_STATUS_IO         = 6,//升级失败(IO读写失败)

}FIRMWARE_UPGRADE_STATUS;

//固件升级的实时状态信息
typedef struct tagFirmwareUpgradeStatus
{
	FIRMWARE_UPGRADE_STATUS status;//升级的状态
	UINT8 nProgress;//实时进度，取值必须处于：0-100
	UINT8 byRes[3];//字节对齐，预留
}FirmwareUpgradeStatus;

//固件升级的实时状态回调函数
typedef void (*FNFirmwareUpgradeStatus)(FirmwareUpgradeStatus *statusData, void* pUserData);

//固件升级数据
typedef struct tagFirmwareUpgradeData
{
	UINT8* pData;//指向固件数据（完整的固件数据首地址）
	UINT32 nDataLen;//pData内固件数据长度（完整的固件数据长度）

	FNFirmwareUpgradeStatus fnUpgradeStatus;//固件升级实时状态回调函数
	void* pUpgradeStatusUserData;//fnUpgradeStatus的pUserData参数
}FirmwareUpgradeData;

//设备重启
typedef struct tagRebootDev
{
	UINT8 byRes[4];//字节对齐，预留
}RebootDev;

//双目相机参数
typedef struct tagStereoLensParameter
{
	FLOAT32 szRotationMatrix[3][3];//双目旋转矩阵
	FLOAT32 szTranslationMatrix[3];//双目平移矩阵

}StereoLensParameter;

//主从机间同步时间
typedef struct tagMasterSlaveSyncTime
{
	UINT64 hostSendTimestamp;//主机发送命令的时间（主机的本地时间）
	UINT64 slaveRecvTimestamp;//从机接收到命令的时间（从机的本地的时间）
	UINT64 slaveSendTimestamp;//从机发送命令的时间（从机的本地时间）
	UINT64 hostRecvTimestamp;//主机接收到命令的时间（主机的本地的时间）

}MasterSlaveSyncTime;

typedef enum tagTOF_DEV_PARAM_TYPE
{
	TOF_DEV_PARAM_Temperature           = MAKE_UNIQUE_ID(0x00, 0x00, 0x00, 0x00),//温度信息
	TOF_DEV_PARAM_TofLensParameter      = MAKE_UNIQUE_ID(0x00, 0x00, 0x00, 0x01),//TOF模组内参和畸变
	TOF_DEV_PARAM_TofCalibData          = MAKE_UNIQUE_ID(0x00, 0x00, 0x00, 0x02),//TOF模组标定数据
	TOF_DEV_PARAM_netdevinfo            = MAKE_UNIQUE_ID(0x00, 0x00, 0x00, 0x03),//网络接入设备信息
	TOF_DEV_PARAM_ReplaceTofCalibData   = MAKE_UNIQUE_ID(0x00, 0x00, 0x00, 0x04),//替换SDK里TOF模组标定数据（仅仅是替换SDK里标定数据，并非烧写到模组）
	TOF_DEV_PARAM_RemoteCapture         = MAKE_UNIQUE_ID(0x00, 0x00, 0x00, 0x05), //远程抓图：控制模块抓取数据并保存在模块内部；
	TOF_DEV_PARAM_ExportRaw             = MAKE_UNIQUE_ID(0x00, 0x00, 0x00, 0x06), //导出一帧RAW数据：实时的从模块里导出一帧RAW数据（适用于RAW数据和深度数据异步传输的情况）；
	TOF_DEV_PARAM_RgbLensParameter      = MAKE_UNIQUE_ID(0x00, 0x00, 0x00, 0x07),//RGB模组内参和畸变
	TOF_DEV_PARAM_UpgradeFirmware       = MAKE_UNIQUE_ID(0x00, 0x00, 0x00, 0x08),//升级固件
	TOF_DEV_PARAM_RebootDev             = MAKE_UNIQUE_ID(0x00, 0x00, 0x00, 0x09),//设备重启
	TOF_DEV_PARAM_StereoLensParameter   = MAKE_UNIQUE_ID(0x00, 0x00, 0x00, 0x0a),//双目相机参数
	TOF_DEV_PARAM_GetMasterSlaveSyncTime = MAKE_UNIQUE_ID(0x00, 0x00, 0x00, 0x0b),//获取主从机间同步时间

}TOF_DEV_PARAM_TYPE;

typedef struct tagTofDeviceParamV20
{
	TOF_DEV_PARAM_TYPE type;//输入参数，只读
	
	union
	{
		TofDeviceTemperature struTemperature;//温度信息【当type为TOF_DEV_PARAM_Temperature时有效】
		TofModuleLensParameter struTofLensParameter;//TOF模组内参和畸变【当type为TOF_DEV_PARAM_TofLensParameter时有效】
		RgbModuleLensParameter struRgbLensParameter;//RGB模组内参和畸变【当type为TOF_DEV_PARAM_RgbLensParameter时有效】
		TofCalibData struTofCalibData;//TOF模组标定数据【当type为TOF_DEV_PARAM_TofCalibData时有效】
		NetDevInfo_t stuNetDevData;	//网络接入设备信息【当type为TOF_DEV_PARAM_netdevinfo时有效】
		TofCalibData struReplaceTofCalibData;//替换SDK里TOF模组标定数据【当type为TOF_DEV_PARAM_ReplaceTofCalibData时有效】
		RemoteCapture struRemoteCapture; //远程抓图：控制模块抓取数据并保存在模块内部；【当type为TOF_DEV_PARAM_RemoteCapture时有效】
		TofRawData struExportRaw; //导出一帧RAW数据：实时的从模块里导出一帧RAW数据（适用于RAW数据和深度数据异步传输的情况）；【当type为TOF_DEV_PARAM_ExportRaw时有效】
		FirmwareUpgradeData struFirmware;//固件升级数据【当type为TOF_DEV_PARAM_UpgradeFirmware时有效】
		RebootDev struRebootDev;//设备重启【当type为TOF_DEV_PARAM_RebootDev时有效】
		StereoLensParameter struStereoLensParameter;//双目相机参数【当type为TOF_DEV_PARAM_StereoLensParameter时有效】
		MasterSlaveSyncTime struMasterSlaveSyncTime;//主从机间同步时间【当type为TOF_DEV_PARAM_GetMasterSlaveSyncTime时有效】
	}uParam;
}TofDeviceParamV20;



typedef enum tagTOFDEV_STATUS
{
	TOFDEV_STATUS_UNUSED                 = MAKE_UNIQUE_ID('U', 'U', 'S', 'E'),//（该值未使用，有效的设备状态从1开始）

	TOFDEV_STATUS_DEV_BROKEN             = MAKE_UNIQUE_ID('D', 'E', 'V', 'B'),//设备异常断开
	//
	TOFDEV_STATUS_READ_CALIB_DATA_SUC    = MAKE_UNIQUE_ID('R', 'C', 'D', 'S'),//读取标定数据成功
	TOFDEV_STATUS_READ_CALIB_DATA_FAILED = MAKE_UNIQUE_ID('R', 'C', 'D', 'F'),//读取标定数据失败
	//
	TOFDEV_STATUS_TOF_STREAM_FAILED      = MAKE_UNIQUE_ID('T', 'S', 'F', 0x00),//取TOF流失败

}TOFDEV_STATUS;

typedef void* HTOFD;


typedef void (*FNTofStream)(TofFrameData *tofFrameData, void* pUserData);
typedef void (*FNTofDeviceStatus)(TOFDEV_STATUS tofDevStatus, void* pUserData);
typedef void (*FNRgbStream)(RgbFrameData *rgbFrameData, void* pUserData);
typedef void (*FNImuStream)(ImuFrameData *imuFrameData, void* pUserData);

#ifdef __cplusplus
extern "C" {
#endif

//初始化/反初始化SDK（其他任何接口的使用都必须介于这两个接口之间）
TOFDDLL TOFRET TOFD_Init(TofDevInitParam* pInitParam);
TOFDDLL TOFRET TOFD_Uninit(void);

//获取SDK版本号（返回值为字符串型版本号）
TOFDDLL SCHAR* TOFD_GetSDKVersion(void);

//搜索到系统里SDK支持的所有设备
TOFDDLL TOFRET TOFD_SearchDevice(TofDeviceDescriptor **ppDevsDesc, UINT32* pDevNum);

//打开/关闭设备
TOFDDLL HTOFD  TOFD_OpenDevice(TofDeviceDescriptor *pDevDesc, FNTofDeviceStatus fnTofDevStatus, void* pUserData);
TOFDDLL TOFRET TOFD_CloseDevice(HTOFD hTofDev);

//获取设备能力
TOFDDLL TOFRET TOFD_GetDeviceInfo(HTOFD hTofDev, TofDeviceInfo *pTofDeviceInfo);

//获取/设置设备参数（已逐步废弃）
TOFDDLL TOFRET TOFD_GetDeviceParam(HTOFD hTofDev, TofDeviceParam *pTofDeviceParam);
TOFDDLL TOFRET TOFD_SetDeviceParam(HTOFD hTofDev, TofDeviceParam *pTofDeviceParam);

//获取/设置设备参数（V2.0版本）
TOFDDLL TOFRET TOFD_GetDeviceParamV20(HTOFD hTofDev, TofDeviceParamV20 *pTofDeviceParam);
TOFDDLL TOFRET TOFD_SetDeviceParamV20(HTOFD hTofDev, TofDeviceParamV20 *pTofDeviceParam);

//启用/禁用自动设置TOF模组曝光
TOFDDLL TOFRET TOFD_SetTofAE(HTOFD hTofDev, const SBOOL bEnable);

//获取/设置TOF模组曝光
TOFDDLL TOFRET TOFD_SetTofExpTime(HTOFD hTofDev, const UINT32 expTime);
TOFDDLL TOFRET TOFD_GetTofExpTime(HTOFD hTofDev, TofExpouse *pExp);

//获取/启用/禁用某种滤波算法
TOFDDLL TOFRET TOFD_SetTofFilter(HTOFD hTofDev, const TOF_FILTER type, const SBOOL bEnable);
TOFDDLL TOFRET TOFD_GetTofFilter(HTOFD hTofDev, const TOF_FILTER type, SBOOL* pbEnable);

//启用/禁用HDRZ算法
TOFDDLL TOFRET TOFD_SetTofHDRZ(HTOFD hTofDev, const SBOOL bEnable);

//启用/禁用RemoveINS算法[已弃用，SDK内部会按需自动启用]
TOFDDLL TOFRET TOFD_SetTofRemoveINS(HTOFD hTofDev, const SBOOL bEnable);

//启用/禁用MPIFlag算法（已废弃，请使用TOFD_SetTofFilter(xxx, TOF_FILTER_MPIFilter, xxx)）
TOFDDLL TOFRET TOFD_SetTofMPIFlag(HTOFD hTofDev, const SBOOL bEnable);

//启动/关闭TOF取流
TOFDDLL TOFRET TOFD_StartTofStream(HTOFD hTofDev, const TOF_MODE tofMode, FNTofStream fnTofStream, void* pUserData);
TOFDDLL TOFRET TOFD_StopTofStream(HTOFD hTofDev);

//获取/设置RGB模组属性
TOFDDLL TOFRET TOFD_GetRgbProperty(HTOFD hTofDev, const RgbVideoControlProperty Property, RgbVideoControl *pValue);
TOFDDLL TOFRET TOFD_SetRgbProperty(HTOFD hTofDev, const RgbVideoControlProperty Property, const SINT32 lValue, const RgbVideoControlFlags lFlag);

//启动/关闭RGB取流
TOFDDLL TOFRET TOFD_StartRgbStream(HTOFD hTofDev, FNRgbStream fnRgbStream, void* pUserData);
TOFDDLL TOFRET TOFD_StopRgbStream(HTOFD hTofDev);

//启动/关闭IMU取流
TOFDDLL TOFRET TOFD_StartImuStream(HTOFD hTofDev, FNImuStream fnImuStream, void* pUserData);
TOFDDLL TOFRET TOFD_StopImuStream(HTOFD hTofDev);

#ifdef __cplusplus
}
#endif

#endif


