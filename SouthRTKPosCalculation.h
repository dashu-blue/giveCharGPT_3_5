#ifndef __UAVSTAR_PPK_SOUTHRTK_POS_CALCULATION_H__
#define __UAVSTAR_PPK_SOUTHRTK_POS_CALCULATION_H__

#include <rtklib_qc\InterfaceApp.h>

#include <memory> 

#include <QObject>
#include <QMap>

// 定义返回码说明(源于SOUTHRTK说明.docx文档2023-6-8版)
#define APP_OK							1	// 正常结束
#define APP_FATAL_NOCMD					2	// 无命令输入
#define APP_FATAL_INVALID_FILEPATH		3	// 无效路径
#define APP_FATAL_OUTFILENAME_EMPTY		4	// 未指定输出文件名
#define APP_FATAL_UNDETECTED			5	// 未知错误
#define APP_FATAL_LOADBANK_ERR			6	// 缓存数据读取错误
#define APP_FATAL_ROVEFILE_OVERFLOW		7	// 流动站文件输入过多
#define APP_FATAL_BASEFILE_OVERFLOW		8	// 基准站文件输入过多
#define APP_FATAL_EPHFILE_OVERFLOW		9	// 星历文件输入过多
#define APP_FATAL_COMBINE_FAILED		10	// 前后向平滑失败
#define APP_FATAL_SMOOTH_FAILED			11	// RTS平滑失败
#define GNSS_FATAL_ROVE_BASE_MISALIGN	12	// 流动站基准站无共同数据
#define GNSS_FATAL_BASECOORD_WRONG		13	// 基站坐标设置错误
#define GNSS_FATAL_ROVE_OBS_EMPTY		14	// 流动站无量测数据
#define GNSS_FATAL_BASE_OBS_EMPTY		15	// 基准站无量测数据
#define INS_FATAL_OBS_EMPTY				16	// 惯导无量测数据
#define INS_FATAL_ALIGNMENT_FAILED		17	// 惯导对准失败
#define INS_FATAL_UNSUPPORTEDIMUTYPE	18	// 不支持的惯导类型
#define INS_FATAL_UNDETECTED			19	// 惯导未知错误
#define INS_WARN_OBS_LARGEGAPS			20	// 存在过大的数据间隔
#define INS_WARN_OBS_OVERLAP			21	// 存在重叠的数据
#define INS_WARN_OBS_SECOUTRANGE		22	// 秒数超出时间范围
#define INS_WARN_OBS_DISORDER			23	// 存在时间乱序的数据
#define GNSS_WARN_ROVE_BEGIN_UNCOVERED	24	// 流动站开始时间段数据不被基准站覆盖
#define GNSS_WARN_ROVE_END_UNCOVERED	25	// 流动站结束时间段数据不被基准站覆盖

// 包装了部分SOUTHRTK解算接口的处理类
class UAVSTAR_PPK SouthRTKPosCalculation : public QObject
{
	Q_OBJECT
public:
	static SouthRTKPosCalculation* instance();
	static int getBaseStationCoordinate(double& b, double& l, double& h, double& antHeight, const QString& baseStationFilePath);
	
	// 输入命令开始解算
	// @param[in&out]	returnCode				解算接口返回码
	// @param[in&out]	returnCodeDescription	返回码对应的错误描述
	// @param[in]		cmdstr					解算命令
	// @tip 该函数会阻塞执行
	void rnx2rtkpCmd(int& returnCode, QString& returnCodeDescription, const char* cmdstr);
signals:
	void notifyProgress(CALLBACK_PROGRESS progressInfo);
	void notifyTimetag(CALLBACK_TIMETAG timetagInfo);
	void notifySolution(CALLBACK_SOLUTION solutionInfo);
private:
	explicit SouthRTKPosCalculation();
	virtual ~SouthRTKPosCalculation();
	SouthRTKPosCalculation(const SouthRTKPosCalculation& copy) = delete;
	SouthRTKPosCalculation& operator=(const SouthRTKPosCalculation& copy) = delete;
private:
	static std::unique_ptr<SouthRTKPosCalculation> sm_instance;
	QMap<qint32, QString> m_returnCodeInfomationMap;
private:
	friend class std::default_delete<SouthRTKPosCalculation>;
	friend std::unique_ptr<SouthRTKPosCalculation> std::make_unique<SouthRTKPosCalculation>();
};

#endif