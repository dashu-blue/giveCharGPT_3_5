#ifndef APPINTERFACE_H
#define APPINTERFACE_H

#ifdef __cplusplus
extern "C"
{
#endif

#ifdef _MSC_VER
#if defined DEF_EXPORTS
#define DLLOPTION __declspec(dllexport) 	
#elif defined DEF_IMPORTS
#define DLLOPTION __declspec(dllimport)
#else 
#define DLLOPTION
#endif 
#else
#define DLLOPTION
#endif

#define APPSOUTHRTK_MAXSYS		 6								// GPS BD2 BD3 GLO GAL QZS
#define APPSOUTHRTK_MAXSAT		 170							// 卫星号1, 2, 3, ...
#define APPSOUTHRTK_MAXCHN		 10								// 通道最大个数, 一对载波+伪距视作一个通道
#define APPSOUTHRTK_MAXOBS		 100							// 历元内最大卫星个数
#define APPSOUTHRTK_MAXLABEL	 6								// 字符串标签长度
#define APPSOUTHRTK_MAXBASENUM	 20								// 基站最大个数
#define APPSOUTHRTK_MAXOBSERRNUM 200							// 观测值错误信息最大个数

// 时间结构体
typedef struct tagTIMETAG
{
	unsigned long threadID;
	int cmdID;

	int solQuality;												// 0-无效解, 1-固定解, 2-浮点解, 3-差分解, 5-单点解
	int year, month, day, hour, minute;							// (GPST)年月日时分
	double second;												// 秒
} CALLBACK_TIMETAG;

// 卫星状态
typedef struct tagSVSATINFO
{
	int svid;													// 连续计数卫星号, svid==0表示无效
	char satid[APPSOUTHRTK_MAXLABEL];							// 字符串卫星号, satid==""表示无效
	double elv, azh;											// 高度角(度), 方位角(度)
	
	double psrMP[APPSOUTHRTK_MAXCHN];							// 伪距多路径(m)
	float snr[APPSOUTHRTK_MAXCHN];								// 信噪比(db/Hz)
	char cycleSlip[APPSOUTHRTK_MAXCHN];							// 周跳标识符, >0表示有周跳
} SVSATINFO;

// 历元三次差分统计
typedef struct tagEPOCHDIFFVAL
{
	double sig[2];												// 历元三次差分精度(m), 0:载波, 1:伪距
	float rate[2];												// 历元三次差分合格率(比例)
} EPOCHDIFFVAL;
typedef struct tagEPOCHDIFFSTAT
{
	EPOCHDIFFVAL tol;											// 历元三次差分总体统计
	EPOCHDIFFVAL sys[APPSOUTHRTK_MAXSYS];						// 历元三次差分逐系统统计
	EPOCHDIFFVAL chn[APPSOUTHRTK_MAXSYS][APPSOUTHRTK_MAXCHN];	// 历元三次差分逐通道统计
} EPOCHDIFFSTAT;

// 观测值完整性(观测时长/采样间隔), 应出现但未出现记为无效
typedef struct tagOBSINTSTAT
{
	int	cntTotalEpo, cntValidEpo;								// 历元可能数, 历元有效数, 一个历元记为一个观测数
	int	cntTotalChn[APPSOUTHRTK_MAXSYS][APPSOUTHRTK_MAXCHN];	// 通道观测可能数, 一对载波/伪距记为一个观测数
	int	cntValidChn[APPSOUTHRTK_MAXSYS][APPSOUTHRTK_MAXCHN];	// 通道观测有效数, 一对载波/伪距记为一个观测数

	int cntTotalSat[APPSOUTHRTK_MAXSAT];						// 卫星观测量可能数, (一对载波/伪距记为一个观测数, 或历元数)
	int cntValidSat[APPSOUTHRTK_MAXSAT];						// 卫星观测量有效数, (一对载波/伪距记为一个观测数, 或历元数)
} OBSINTSTAT;

// 通道索引, 对应APPSOUTHRTK_MAXCHN数组, 一个通道记为一对载波/伪距
typedef struct tagCHNINDEX
{
	char chnRinexLn[APPSOUTHRTK_MAXSYS][APPSOUTHRTK_MAXCHN];	// 每个系统的每个通道对应的RINEX L?, 类型为整数[0, 9], 0表示无效
	char chnCode[APPSOUTHRTK_MAXSYS][APPSOUTHRTK_MAXCHN];		// 每个系统的每个通道对应的码/跟踪方式, 类型为字母A~Z
	int chnCnt[APPSOUTHRTK_MAXSYS];								// 每个系统的通道个数, 用于表示APPSOUTHRTK_MAXCHN数组长度
} CHNINDEX;

// 解算结果
typedef struct tagCALLBACK_SOLUTION
{
	unsigned long threadID;
	int cmdID;

	int solQuality;												// 0-无效解, 1-固定解, 2-浮点解, 3-差分解, 5-单点解
	int year, month, day, hour, minute;							// (GPST)年月日时分
	double second, rcvClkBias[APPSOUTHRTK_MAXSYS];				// 秒, 接收机钟差(m)

	int satNum;													// 参与计算的卫星个数
	double blh[3], ecef[3];										// (lat lon hgt), (XYZ)
	double gdop, pdop, hdop, vdop;								// 精度因子

	CHNINDEX chnIndex;											// 通道索引
	int satObsFlg[APPSOUTHRTK_MAXSAT];							// 拥有观测值的卫星标识, 下标是svid(连续计数卫星号), -1表示不可用, 0表示可用, >0表示错误码
	int satEphFlg[APPSOUTHRTK_MAXSAT];							// 拥有星历的卫星标识, 下标是svid(连续计数卫星号), -1表示不可用, 0表示可用, >0表示错误码
	SVSATINFO svSatInfo[APPSOUTHRTK_MAXOBS];					// 卫星状态信息, 当sv==""表示数组结束

	double clkBiasRMS;											// 接收机钟差均方根误差(m)
	OBSINTSTAT obsIntStat;										// 观测值完整性统计
	EPOCHDIFFSTAT epoDifStat;									// 观测值历元三次差分统计
} CALLBACK_SOLUTION;

typedef struct tagCALLBACK_PROGRESS 
{
	int cmdID;
	int type;
	int rate;
} CALLBACK_PROGRESS;

typedef void(*CALLBACK_FUNCS_PROGRESS)(CALLBACK_PROGRESS);
typedef void(*CALLBACK_POSITION_TIMETAG)(CALLBACK_TIMETAG*);
typedef void(*CALLBACK_POSITION_EPOCH)(CALLBACK_SOLUTION*);

typedef struct tagCALLBACK_FUNCS
{
	CALLBACK_FUNCS_PROGRESS cbProgress;
	CALLBACK_POSITION_TIMETAG cbPositionTimetag;
	CALLBACK_POSITION_EPOCH cbPositionEpoch;
} CALLBACK_FUNCS;

typedef struct tagOBS_HANDLE
{
	int   errCode;
	void *data;								   // 保存智能指针的指针CPosStreamConstPtr*
} OBS_HANDLE;

typedef struct tagBASEINFO
{
	int num;                                   // 基站数
	double antHeight[APPSOUTHRTK_MAXBASENUM];  // 天线高   
	double coord[APPSOUTHRTK_MAXBASENUM][3];   // 坐标（根据下标和天线高对应）
} BASEINFO;

// 错误数据信息结构体,存放内容如下
// sec(时间）     val(具体值)        errCode(类型)
//                对应的时间间隔     错误的观测时间间隔 
//                无                 重叠的观测数据
// -1(某一段数据)          

typedef struct tagERRINFO
{
	double sec;
	double val;                    
	int    errCode;
}ERRINFO;
typedef struct tagERROBSINFO
{
	int	    nErr;
	ERRINFO errInfo[APPSOUTHRTK_MAXOBSERRNUM];
}ERROBSINFO;


DLLOPTION OBS_HANDLE obshandle_init(const char *cmdstr);
DLLOPTION OBS_HANDLE obshandle_init_callback(const char *cmdstr, CALLBACK_FUNCS *pCbfs);
DLLOPTION OBS_HANDLE obshandle_merge(OBS_HANDLE *pObsHdl1, OBS_HANDLE *pObsHdl2);
DLLOPTION BASEINFO   obshandle_get_baseinfo(OBS_HANDLE obsHdl);
DLLOPTION ERROBSINFO obshandle_get_errobsinfo(OBS_HANDLE obsHdl);
DLLOPTION void	     obshandle_release(OBS_HANDLE *pObsHdl);
DLLOPTION int rnx2rtkp_cmd_obs(const char *cmdstr, OBS_HANDLE *pObsHdl);
DLLOPTION int rnx2rtkp_cmd_obs_callback(const char *cmdstr, CALLBACK_FUNCS *pCbfs, OBS_HANDLE *pObsHdl);

DLLOPTION int rnx2rtkp(int argc, char *argv[]);
DLLOPTION int rnx2rtkp_cmd(const char *cmdstr);
DLLOPTION int rnx2rtkp_cmd_callback(const char *cmdstr, CALLBACK_FUNCS *pCbfs);

//返回支持的IMU类型列表
DLLOPTION const char *get_imu_list();
//返回错误码列表
DLLOPTION const char *get_err_list();
//返回输出项码列表
DLLOPTION const char *get_outitm_list();
//返回回调进度类型列表
DLLOPTION const char *get_progress_list();
//返回场景列表
DLLOPTION const char* get_scene_list();
//OEM数据转IMR
DLLOPTION int log2imr(char *filepath);
//OEM数据转文本格式
DLLOPTION int log2txt(char *filepath);
//返回南方读取库Windows平台版本字符串, 格式为日期, 如"20220811"
DLLOPTION const char *get_south_readlib_windows_ver();
//返回南方读取库Linux平台版本字符串, 格式为日期, 如"20220315"
DLLOPTION const char *get_south_readlib_linux_ver();
//返回RTK算法版本字符串, 格式为SVN号+日期, 如"168.20220315"
DLLOPTION const char *get_south_rtk_ver();
//返回卫星号的字符串形式
DLLOPTION void conv_svid_to_satid(int svid, char *satid, int len);
//返回卫星的系统号: -1-无效, 0-GPS, 1-BD2, 2-BD3, 3-GLO, 4-GAL, 5-QZS, 输入连续计数卫星号
DLLOPTION int conv_svid_to_sysid(int svid);
//返回频点名称字符串, 输入系统号sysid, RINEX L?号
DLLOPTION const char *conv_rinexLn_to_frqstr(int sysid, int rinexLn);

#ifdef __cplusplus
}
#endif

#endif // !APPINTERFACE_H
