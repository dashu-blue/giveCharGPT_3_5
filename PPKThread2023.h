#ifndef __UAVSTAR_PPK_THREAD_2023_H__
#define __UAVSTAR_PPK_THREAD_2023_H__

#include "GNSSSettings.h"

#include <rtklib_qc/InterfaceApp.h>

#include <UAVstar_earth/CameraFromAerialPos.h>

#include <QThread>
#include <QMap>
#include <QList>
#include <QFileInfo>
#include <QDateTime>

#define SYSTEM_EPSG_4326 "+proj=longlat +datum=WGS84 +k=1 +no_defs"	// 定义wgs84椭球体的地理坐标系proj字符串
#define SYSTEM_EPSG_4328 "+proj=geocent +datum=WGS84 +k=1 +no_defs"	// 定义wgs84椭球体的空间直角坐标系proj字符串

// 新定义相机与天线的相对位置的结构体
struct UAVSTAR_PPK PPKCameraOffset
{
	qreal eastX = 0.0;
	qreal northY = 0.0;
	qreal upZ = 0.0;
};

// 新定义基站数据信息结构体
struct UAVSTAR_PPK PPKBaseStationInfo
{
	QString filePath = "";		// 基站文件的绝对路径
	qreal X = 0.0;				// 基站的纬度(北)坐标
	qreal Y = 0.0;				// 基站的经度(东)坐标
	qreal Z = 0.0;				// 基站的椭球高(高)
	qreal antennaHeight = 0.0;	// 基站的天线高
	bool operator<(const PPKBaseStationInfo& other) const;
	bool operator==(const PPKBaseStationInfo& other) const;
};

// 新定义移动站数据信息结构体
struct UAVSTAR_PPK PPKMobileStationInfo
{
	QString filePath = "";
	bool operator<(const PPKMobileStationInfo& other) const;
	bool operator==(const PPKMobileStationInfo& other) const;
};

class UAVSTAR_PPK PPK2023Thread : public QThread
{
	Q_OBJECT
public:
	explicit PPK2023Thread(QObject* parent = nullptr);
	virtual ~PPK2023Thread();

	// 设置参数
	void setData(
		const QMap<PPKMobileStationInfo, QList<PPKBaseStationInfo>>& stationInfoMap,
		const PPKCameraOffset& offset,
		const GNSSSettings& gnssSettings,
		const QString& outputDir
	);
signals:
	// ppk进度信息信号
	void progressed(QString progressDescription);
public slots:
	// 设置取消ppk解算标志，表示使用方需要中断当前ppk解算工作
	void onCancel();
private:
	virtual void run();
private:
	QMap<PPKMobileStationInfo, QList<PPKBaseStationInfo>> m_stationInfoMap;
	PPKCameraOffset m_offset;
	GNSSSettings m_gnssSettings;
	QString m_outputDir;
	bool m_cancel;// 取消ppk解算工作的标志，为了实现终止正在进行ppk工作的目的，需要在各个ppk工作环节中最耗时的代码处检查这个标志

	// PPK预处理环节
	// @param[in&out]	stationInfoMap	返回将基站坐标系坐标转换为wgs84空间直角坐标系坐标的数据集合
	// @tip 检查参数有效性
	// @tip 将基站坐标系坐标转换为wgs84空间直角坐标系坐标
	bool _preprocessingPPK(QMap<PPKMobileStationInfo, QList<PPKBaseStationInfo>>& stationInfoMap);
	// PPK正式解算环节
	// @param[in&out]	solutionInfoMap	解算接口生成的所有解算结果集合
	// @param[in]		stationInfoMap		经过预处理的数据集合			  
	// @tip 构建解算接口调用命令
	// @tip 调用解算接口，开始解算工作
	bool _calculatingPPK(QMap<PPKMobileStationInfo, CALLBACK_SOLUTION>& solutionInfoMap, const QMap<PPKMobileStationInfo, QList<PPKBaseStationInfo>>& stationInfoMap);
	// PPK尾处理环节
	// ???
	// @tip 将飞机姿态数据IMU(spos文件)因素和相机天线相对偏移位置因素，融合处理(计算)到解算结果的碎步点集合(pos集合)中，生成新的碎步点集合(pos集合)
	bool _postProcessingPPK(const QMap<PPKMobileStationInfo, CALLBACK_SOLUTION>& solutionInfoMap);

	// 检查参数完整性
	bool _checkDataIntegrity(QString& error);
	// 基站坐标系坐标转换为wgs84空间直角坐标系坐标
	bool _baseStationCsToWgs84XYZCs(QString& error, PPKBaseStationInfo& outBaseStationInfo, const PPKBaseStationInfo& inputBaseStationInfo, const GNSSSettings& gnssSettings);
	// 统一处理进度信息
	void _processProgressInfo(const QString& progressInfo);
private slots:
	// 解析解算接口信号发送来的进度信息，然后调用信号发送出去
	void _onSouthRTKInterFaceProgress(CALLBACK_PROGRESS progressInfo);
};

#endif