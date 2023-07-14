#include "SouthRTKPosCalculation.h"

#include <sag_sdk/CoordSystem.h>

#include <QMutex>
#include <QMutexLocker>

using namespace std;

static double dddddd_to_ddmmss(double angle)
{
	int dd = floor(angle);
	int mm = floor((angle - dd) * 60.0);
	double ss = (angle - dd - mm / 60.0) * 3600.0;
	return (dd + mm / 100.0 + ss / 10000.0);
}

void _handle_callbackFuncsProgress(CALLBACK_PROGRESS progressInfo)
{
	emit SouthRTKPosCalculation::instance()->notifyProgress(progressInfo);
}

void _handle_callbackPositionTimetag(CALLBACK_TIMETAG * timetagInfo)
{
	emit SouthRTKPosCalculation::instance()->notifyTimetag(*timetagInfo);
}

void _handle_callbackPositionEpoch(CALLBACK_SOLUTION * solutionInfo)
{
	emit SouthRTKPosCalculation::instance()->notifySolution(*solutionInfo);
}

unique_ptr<SouthRTKPosCalculation> SouthRTKPosCalculation::sm_instance = nullptr;

SouthRTKPosCalculation * SouthRTKPosCalculation::instance()
{
	if (sm_instance == nullptr)
	{
		sm_instance = std::make_unique<SouthRTKPosCalculation>();
	}
	return sm_instance.get();
}

int SouthRTKPosCalculation::getBaseStationCoordinate(double & b, double & l, double & h, double & antHeight, const QString & baseStationFilePath)
{
	OBS_HANDLE baseStationHandle;
	QStringList parmList;
	parmList.append("-bobs '" + baseStationFilePath + "'");
	parmList.append("-headonly");
	QString cmdQStr = parmList.join(" ");
	cmdQStr.replace("/", "\\");
	QByteArray dataArray = cmdQStr.toLocal8Bit();
	baseStationHandle = obshandle_init(dataArray.data());
	int ret = baseStationHandle.errCode;
	if (ret)
	{
		return ret;
	}
	BASEINFO baseInfo = obshandle_get_baseinfo(baseStationHandle);
	ICoordTransform *trans = extensible_factory_get_ICoordTransform("coord_lxn", 0);
	if (trans && baseInfo.num)
	{
		trans->transfrom(COORD_SRC, COORD_TYPE_XYZ, COORD_SRC, COORD_TYPE_BLH, baseInfo.coord[0][0], baseInfo.coord[0][1], baseInfo.coord[0][2], b, l, h);
		b = dddddd_to_ddmmss(b);
		l = dddddd_to_ddmmss(l);
		antHeight = baseInfo.antHeight[0];
		extensible_factory_release_ICoordTransform(trans);
	}
	obshandle_release(&baseStationHandle);
	return ret;
}

void SouthRTKPosCalculation::rnx2rtkpCmd(int& returnCode, QString& returnCodeDescription, const char* cmdstr)
{
	static QMutex mutexLock(QMutex::NonRecursive);
	QMutexLocker mutexLocker(&mutexLock);
	const bool isUseCallback = static_cast<bool>(1);
	try
	{
		const char* south_readlib_windows_ver = get_south_readlib_windows_ver();
		const char* south_rtk_ver = get_south_rtk_ver();
		const char* err_list = get_err_list();
		if (isUseCallback)
		{
			CALLBACK_FUNCS cbfun;
			cbfun.cbProgress = NULL;
			cbfun.cbPositionTimetag = NULL;
			cbfun.cbPositionEpoch = NULL;
			returnCode = rnx2rtkp_cmd_callback(cmdstr, &cbfun);
		}
		else
		{
			returnCode = rnx2rtkp_cmd(cmdstr);
		}
	}
	catch (const std::exception& ex)
	{
		QString error = SouthRTKPosCalculation::tr("Throwing an exception error, %1").arg(QString::fromLocal8Bit(ex.what()));// 抛出异常错误，%1
		returnCodeDescription = error;
		return;
	}
	catch (...)
	{
		QString error = SouthRTKPosCalculation::tr("Throwing unknown exception error");// 抛出不明异常错误
		returnCodeDescription = error;
		return;
	}
	returnCodeDescription = m_returnCodeInfomationMap.value(returnCode, SouthRTKPosCalculation::tr("Undefined new error, error code %1").arg(returnCode));// 未定义的新错误，错误代码%1
}

SouthRTKPosCalculation::SouthRTKPosCalculation()
	: QObject()
{
	m_returnCodeInfomationMap.insert(APP_OK,							SouthRTKPosCalculation::tr("Normal end"));// 正常结束
	m_returnCodeInfomationMap.insert(APP_FATAL_NOCMD,					SouthRTKPosCalculation::tr("No command input"));// 无命令输入
	m_returnCodeInfomationMap.insert(APP_FATAL_INVALID_FILEPATH,		SouthRTKPosCalculation::tr("Invalid path"));// 无效路径
	m_returnCodeInfomationMap.insert(APP_FATAL_OUTFILENAME_EMPTY,		SouthRTKPosCalculation::tr("No output file name specified"));// 未指定输出文件名
	m_returnCodeInfomationMap.insert(APP_FATAL_UNDETECTED,				SouthRTKPosCalculation::tr("Unknown error"));// 未知错误
	m_returnCodeInfomationMap.insert(APP_FATAL_LOADBANK_ERR,			SouthRTKPosCalculation::tr("Cache data read error"));// 缓存数据读取错误
	m_returnCodeInfomationMap.insert(APP_FATAL_ROVEFILE_OVERFLOW,		SouthRTKPosCalculation::tr("Excessive input of mobile station files"));// 流动站文件输入过多
	m_returnCodeInfomationMap.insert(APP_FATAL_BASEFILE_OVERFLOW,		SouthRTKPosCalculation::tr("Excessive input of base station files"));// 基准站文件输入过多
	m_returnCodeInfomationMap.insert(APP_FATAL_EPHFILE_OVERFLOW,		SouthRTKPosCalculation::tr("Excessive input of ephemeris files"));// 星历文件输入过多
	m_returnCodeInfomationMap.insert(APP_FATAL_COMBINE_FAILED,			SouthRTKPosCalculation::tr("Forward and backward smoothing failed"));// 前后向平滑失败
	m_returnCodeInfomationMap.insert(APP_FATAL_SMOOTH_FAILED,			SouthRTKPosCalculation::tr("RTS smoothing failed"));// RTS平滑失败
	m_returnCodeInfomationMap.insert(GNSS_FATAL_ROVE_BASE_MISALIGN,		SouthRTKPosCalculation::tr("Mobile station reference station has no common data"));// 流动站基准站无共同数据
	m_returnCodeInfomationMap.insert(GNSS_FATAL_BASECOORD_WRONG,		SouthRTKPosCalculation::tr("Base station coordinate setting error"));// 基站坐标设置错误
	m_returnCodeInfomationMap.insert(GNSS_FATAL_ROVE_OBS_EMPTY,			SouthRTKPosCalculation::tr("Mobile station without measurement data"));// 流动站无量测数据
	m_returnCodeInfomationMap.insert(GNSS_FATAL_BASE_OBS_EMPTY,			SouthRTKPosCalculation::tr("No measurement data from the reference station"));// 基准站无量测数据
	m_returnCodeInfomationMap.insert(INS_FATAL_OBS_EMPTY,				SouthRTKPosCalculation::tr("Inertial navigation without measurement data"));// 惯导无量测数据
	m_returnCodeInfomationMap.insert(INS_FATAL_ALIGNMENT_FAILED,		SouthRTKPosCalculation::tr("Inertial Navigation Alignment Failure"));// 惯导对准失败
	m_returnCodeInfomationMap.insert(INS_FATAL_UNSUPPORTEDIMUTYPE,		SouthRTKPosCalculation::tr("Unsupported inertial navigation type"));// 不支持的惯导类型
	m_returnCodeInfomationMap.insert(INS_FATAL_UNDETECTED,				SouthRTKPosCalculation::tr("Inertial Navigation Unknown Error"));// 惯导未知错误
	m_returnCodeInfomationMap.insert(INS_WARN_OBS_LARGEGAPS,			SouthRTKPosCalculation::tr("There is an excessive data interval"));// 存在过大的数据间隔
	m_returnCodeInfomationMap.insert(INS_WARN_OBS_OVERLAP,				SouthRTKPosCalculation::tr("There is overlapping data"));// 存在重叠的数据
	m_returnCodeInfomationMap.insert(INS_WARN_OBS_SECOUTRANGE,			SouthRTKPosCalculation::tr("Seconds out of time range"));// 秒数超出时间范围
	m_returnCodeInfomationMap.insert(INS_WARN_OBS_DISORDER,				SouthRTKPosCalculation::tr("Time disordered data exists"));// 存在时间乱序的数据
	m_returnCodeInfomationMap.insert(GNSS_WARN_ROVE_BEGIN_UNCOVERED,	SouthRTKPosCalculation::tr("The starting time period data of the mobile station is not covered by the reference station"));// 流动站开始时间段数据不被基准站覆盖
	m_returnCodeInfomationMap.insert(GNSS_WARN_ROVE_END_UNCOVERED,		SouthRTKPosCalculation::tr("The data of the end time period of the mobile station is not covered by the reference station"));// 流动站结束时间段数据不被基准站覆盖
}

SouthRTKPosCalculation::~SouthRTKPosCalculation()
{
}
