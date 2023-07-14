#include "PPKThread2023.h"
#include "SouthRTKPosCalculation.h"

#include <gnss00_utilities/dictionary_utils.h>

#include <UAVstar_app/CommonMethod.h>

#include <functional>

#include <QDebug>

bool PPKBaseStationInfo::operator<(const PPKBaseStationInfo & other) const
{
	QFileInfo fileInfo(this->filePath);
	QFileInfo otherFileInfo(other.filePath);
	if (fileInfo.isFile() && otherFileInfo.isFile())
	{
		return fileInfo.created() < otherFileInfo.created();
	}
	else
	{
		return this->filePath < other.filePath;
	}
}

bool PPKBaseStationInfo::operator==(const PPKBaseStationInfo & other) const
{
	bool isSame = this->filePath.compare(other.filePath) == 0
		&& this->X == other.X
		&& this->Y == other.Y
		&& this->Z == other.Z
		&& this->antennaHeight == other.antennaHeight;
	return isSame;
}

bool PPKMobileStationInfo::operator<(const PPKMobileStationInfo & other) const
{
	QFileInfo fileInfo(this->filePath);
	QFileInfo otherFileInfo(other.filePath);
	if (fileInfo.isFile() && otherFileInfo.isFile())
	{
		return fileInfo.created() < otherFileInfo.created();
	}
	else
	{
		return this->filePath < other.filePath;
	}
}

bool PPKMobileStationInfo::operator==(const PPKMobileStationInfo & other) const
{
	bool isSame = this->filePath.compare(other.filePath) == 0;
	return isSame;
}

PPK2023Thread::PPK2023Thread(QObject* parent)
	: QThread(parent)
	, m_cancel(false)
{
}

void PPK2023Thread::onCancel()
{
	m_cancel = true;
}

PPK2023Thread::~PPK2023Thread()
{
}

void PPK2023Thread::setData(const QMap<PPKMobileStationInfo, QList<PPKBaseStationInfo>>& stationInfoMap, const PPKCameraOffset & offset, const GNSSSettings & gnssSettings, const QString & outputDir)
{
	m_stationInfoMap = stationInfoMap;
	m_offset = offset;
	m_gnssSettings = gnssSettings;
	m_outputDir = outputDir;
}

void PPK2023Thread::run()
{
	QMap<PPKMobileStationInfo, QList<PPKBaseStationInfo>> stationInfoMap;
	QMap<PPKMobileStationInfo, CALLBACK_SOLUTION> solutionInfoMap;
	// PPK预处理环节
	bool isPreprocessingPPK = _preprocessingPPK(stationInfoMap);
	if (!isPreprocessingPPK)
	{
		return;
	}
	// PPK解算环节
	bool isCalculatingPPK = _calculatingPPK(solutionInfoMap, stationInfoMap);
	if (!isCalculatingPPK)
	{
		return;
	}
	// PPK生成解算报告环节
	bool isGenerateSolutionReport = _postProcessingPPK(solutionInfoMap);
	if (!isGenerateSolutionReport)
	{
		return;
	}
}

bool PPK2023Thread::_preprocessingPPK(QMap<PPKMobileStationInfo, QList<PPKBaseStationInfo>>& stationInfoMap)
{
	bool reply = false;
	const QString preprocessingStage = PPK2023Thread::tr("PPK preprocessing stage:");							// PPK预处理环节：
	_processProgressInfo(preprocessingStage + PPK2023Thread::tr("Start PPK preprocessing phase"));			// 开始PPK预处理环节
	/// 1. 检查参数完整性
	_processProgressInfo(preprocessingStage + PPK2023Thread::tr("Start checking parameter integrity"));		// 开始检查参数完整性
	QString checkParamIntegrityError = "";
	bool isParamIntegrity = _checkDataIntegrity(checkParamIntegrityError);
	if (!isParamIntegrity)
	{
		_processProgressInfo(preprocessingStage + checkParamIntegrityError);
		_processProgressInfo(PPK2023Thread::tr("Parameter check failed"));// 参数检查不通过				
		stationInfoMap.clear();
		return reply = false;
	}
	_processProgressInfo(preprocessingStage + PPK2023Thread::tr("Complete parameter integrity check"));		// 完成检查参数完整性
	/// 2. 将基站坐标系坐标转换为wgs84空间直角坐标系坐标，存放在新的QMap中
	_processProgressInfo(preprocessingStage + PPK2023Thread::tr("Starting the conversion of base station coordinate system coordinates to wgs84 spatial Cartesian coordinate system coordinates"));// 开始基站坐标系坐标转换为wgs84空间直角坐标系坐标
	for (auto cit = m_stationInfoMap.cbegin(); cit != m_stationInfoMap.cend(); ++cit)
	{
		QList<PPKBaseStationInfo> baseStationWgs84XYZCsInfoList = {};
		for (const PPKBaseStationInfo& singleBaseStationInfo : cit.value())
		{
			// 检查取消ppk解算工作标志
			if (m_cancel)
			{
				_processProgressInfo(PPK2023Thread::tr("Manually cancel PPK"));// 手动取消PPK
				return reply = false;
			}

			PPKBaseStationInfo baseStationWgs84XYZCsInfo;
			QString transfromCsError = "";
			bool isConvertBaseStationCs = _baseStationCsToWgs84XYZCs(transfromCsError, baseStationWgs84XYZCsInfo, singleBaseStationInfo, m_gnssSettings);
			if (!isConvertBaseStationCs)
			{
				_processProgressInfo(preprocessingStage + transfromCsError);
				_processProgressInfo(PPK2023Thread::tr("Failed to convert base station coordinates to wgs84 spatial Cartesian coordinates"));// 基站坐标转wgs84空间直角坐标失败
				stationInfoMap.clear();
				return reply = false;
			}
			baseStationWgs84XYZCsInfoList.push_back(baseStationWgs84XYZCsInfo);
		}
		stationInfoMap.insert(cit.key(), baseStationWgs84XYZCsInfoList);
	}
	_processProgressInfo(preprocessingStage + PPK2023Thread::tr("Complete the conversion of base station coordinate system coordinates to wgs84 spatial Cartesian coordinate system coordinates"));// 完成基站坐标系坐标转换为wgs84空间直角坐标系坐标
	_processProgressInfo(preprocessingStage + PPK2023Thread::tr("End PPK preprocessing phase"));				// 结束PPK预处理环节
	return reply = true;
}

bool PPK2023Thread::_calculatingPPK(QMap<PPKMobileStationInfo, CALLBACK_SOLUTION>& solutionInfoMap, const QMap<PPKMobileStationInfo, QList<PPKBaseStationInfo>>& stationInfoMap)
{
	bool reply = false;
	// 定义构建调用解算接口的单行命令的局部函数
	std::function<QString(const QString&, const QString&)> createSingleCmd = [](const QString& key, const QString& value)
	{
		// 如果是带路径的参数，路径用单引号括起来，可避免路径中存在的空格导致文件解析失败
		QString singleCmd = QFileInfo(value).exists() ? QString("%1 '%2'") : QString("%1 %2");
		singleCmd = singleCmd.arg(key).arg(value);
		return singleCmd;
	};
	const QString calculatingStage = PPK2023Thread::tr("PPK calculating stage:");	// PPK解算环节
	
	// 多基站解算依据移动站个数循环调用解算接口
	for (auto cit = stationInfoMap.cbegin(); cit != stationInfoMap.cend(); ++cit)
	{
		// 检查取消ppk解算工作标志
		if (m_cancel)
		{
			_processProgressInfo(PPK2023Thread::tr("Manually cancel PPK"));// 手动取消PPK
			return reply = false;
		}

		QStringList totalCmdList = {};
		PPKMobileStationInfo mobileStationInfo = cit.key();
		QList<PPKBaseStationInfo> baseStationInfoList = cit.value();

		QString info = PPK2023Thread::tr("%1 Mobile Station Preprocessing").arg(mobileStationInfo.filePath);// %1 移动站预处理
		_processProgressInfo(calculatingStage + info);

		// 构建解算接口调用指令，解算接口指令与说明源于SOUTHRTK说明.docx文档2023-6-8版
		// 指令：-robs
		// 说明： 流动站路径，前后加上单引号'以避免路径中出现空格导致不识别的问题
		totalCmdList.push_back(createSingleCmd("-robs", mobileStationInfo.filePath));

		for (const PPKBaseStationInfo& singleBaseStationInfo : baseStationInfoList)
		{
			// 指令：-bobs
			// 说明： 基准站路径，前后加上单引号'以避免路径中出现空格导致不识别的问题
			totalCmdList.push_back(createSingleCmd("-bobs", singleBaseStationInfo.filePath));

			// 指令：-rbxyz
			// 说明：reference (base) receiver ecef pos (m) [rinex head]
			QString rbxyzCmdValue = QString("%1 %2 %3")
				.arg(singleBaseStationInfo.X, 0, 'f', 12)
				.arg(singleBaseStationInfo.Y, 0, 'f', 12)
				.arg(singleBaseStationInfo.Z, 0, 'f', 12);
			totalCmdList.push_back(createSingleCmd("-rbxyz", rbxyzCmdValue));

			// 指令：-banth
			// 说明：antenna height(m)
			totalCmdList.push_back(createSingleCmd("-banth", QString::number(singleBaseStationInfo.antennaHeight, 'f', 12)));
		}

		// 指令：-o
		// 说明：set output file [stdout]
		totalCmdList.push_back(createSingleCmd("-o", m_outputDir));

		// 指令：-scene
		// 说明：选择场景（7：默认， 8：静态，9：手持，10：机载，11：车载）
		totalCmdList.push_back(createSingleCmd("-scene", "10"));

		// 指令：-ifileform
		// 说明：imu file format (0:EUROC,1:IMR,2:GINav,4:Novatel OEM) [EUROC]
		totalCmdList.push_back(createSingleCmd("-ifileform", "1"));

		// 指令：-smooth
		// 说明：smooth solution (0:not smooth, 0x1:forward 0x2:backward, 0x3:both direction) [0]
		totalCmdList.push_back(createSingleCmd("-smooth", "0x3"));

		// 指令：-combine
		// 说明：choose two solution to combine (0x1:forward and backward, 0x2:foward smooth and backward smooth) [0x0]
		totalCmdList.push_back(createSingleCmd("-combine", "0x2"));

		// 指令：-multipass
		// 说明：multipass process (0:off, 1:on) [off]
		totalCmdList.push_back(createSingleCmd("-multipass", "1"));

		// 指令：-outsol
		// 说明：utput solution(0x1:foward, 0x2:backward, 0x4:combine, 0x8:forward smooth, 0x10:backward smooth, 0x20 : combine smooth)[0x0]
		totalCmdList.push_back(createSingleCmd("-outsol", "0x20"));

		// 指令：-outitm
		// 说明：choose output item(0x1:gpssec, 0x2 : gpssecond, 0x4 : pos, 0x8 : vel, 0x : 10 : att, 0x20 : bg, 0x40 : ba, 0x80 : pos accuracy, 0x100 : vel accuracy, 0x200 : att accuracy, 0x400 : bg accuracy, 0x800 : ba accuracy, 0x1000 : quality, 0x2000 : satnum, 0x4000 : ratio, 0x8000 : pdop, 0x10000 : worklabel[0]
		totalCmdList.push_back(createSingleCmd("-outitm", "0x16"));

		// 指令：-prcpass
		// 说明：direction to process (0x0: read bank only, 0x1:forward, 0x2:backward, 0x3:both) [0x1]
		totalCmdList.push_back(createSingleCmd("-prcpass", "0x3"));

		// 指令：-namemod
		// 说明：mode of naming output file (0:default 1:follow the rover obs file 2:self define) [0]
		totalCmdList.push_back(createSingleCmd("-namemod", "1"));

		// 指令：-crdfmt
		// 说明：选择输出的坐标格式（0：无，1：直角坐标系，2：大地坐标系，3：站心坐标系）[1]
		totalCmdList.push_back(createSingleCmd("-crdfmt", "2"));

		// 指令：-ti
		// 说明：time interval (sec) [all]
		// 说明：设置频率
		totalCmdList.push_back(createSingleCmd("-ti", "1.0"));

		// 指令：-tritm
		// 说明：debug trace items(0x100:function flow, 0x8000: processing flow, 0x4000: raw imu data)
		totalCmdList.push_back(createSingleCmd("-tritm", "0x0"));

		// 指令：-gapmax
		// 说明：支持的最大中断间隔（秒）[1.0]
		totalCmdList.push_back(createSingleCmd("-gapmax", "2.0"));

		// 指令：-nav
		// 说明：星历路径，前后加上单引号'以避免路径中出现空格导致不识别的问题
		//totalCmdList.push_back(createSingleCmd("-nav", "?"));

		// 指令：-imufile
		// 说明：IMU文件路径，前后加上单引号'以避免路径中出现空格导致不识别的问题
		//totalCmdList.push_back(createSingleCmd("-imufile", "?"));
		
		// 指令：-imutype
		// 说明：imu type (0:FSAS 1:KVH1750 2:HG4930 3:STIM300 4:CPT)
		//totalCmdList.push_back(createSingleCmd("-imutype", "?"));

		// 指令：-lever
		// 说明：lever arm from GNSS antenna center to IMU center in body frame, the input order is E, N, U
		//totalCmdList.push_back(createSingleCmd("-lever", "?"));

		// 指令：-rotate
		// 说明：rotation from body to IMU, the input order is pitch, roll, yaw, the unit is deg
		//totalCmdList.push_back(createSingleCmd("-rotate", "?"));

		// 指令：-algomod
		// 说明：mode of postprocess (0:RTK 2:LCI 3:TCI) [RTK]
		//totalCmdList.push_back(createSingleCmd("-algomod", "?"));

		// 指令：-timefmt
		// 说明：solution time format (1:gpssecond of week, 2:gpssecond of day, 3:通用时)[1]
		//totalCmdList.push_back(createSingleCmd("-timefmt", "?"));

		// 指令：-outinterval
		// 说明：intervals of pos result [0.1]
		//totalCmdList.push_back(createSingleCmd("-outinterval", "?"));

		// 指令：-cmdid
		// 说明：命令对应的ID，用于区分不同命令解算[0]
		//totalCmdList.push_back(createSingleCmd("-cmdid", "?"));

		// 组合解算接口调用指令
		QString cmdQStr = totalCmdList.join(" ");
		cmdQStr.replace("/", "\\\\");// 文件路径分隔符"/"换为"\\\\"
		QByteArray cmdByteArray = cmdQStr.toLocal8Bit();
		const char* cmd = cmdByteArray.data();
		QString info2 = PPK2023Thread::tr("%1 Mobile Station Command Check %2").arg(mobileStationInfo.filePath).arg(cmdQStr);// %1 移动站指令检查 %2
		_processProgressInfo(calculatingStage + info2);

		// 开始调用解算接口
		_processProgressInfo(calculatingStage + PPK2023Thread::tr("Start of mobile station %1 calculation").arg(mobileStationInfo.filePath));// %1 移动站解算开始
		int returnCode = -1;
		QString returnCodeDescription = "-1";		
		QObject::connect(SouthRTKPosCalculation::instance(), &SouthRTKPosCalculation::notifyProgress, this, &PPK2023Thread::_onSouthRTKInterFaceProgress, Qt::ConnectionType::AutoConnection);
		QObject::connect(SouthRTKPosCalculation::instance(), &SouthRTKPosCalculation::notifySolution, this, [&](CALLBACK_SOLUTION solutionInfo)
		{ 
			solutionInfoMap.insert(cit.key(), solutionInfo);
		}, Qt::ConnectionType::AutoConnection);// 统计解算接口信息发送来的解算结果，后面用于生成解算报告
		SouthRTKPosCalculation::instance()->rnx2rtkpCmd(returnCode, returnCodeDescription, cmd);
		QObject::disconnect(SouthRTKPosCalculation::instance(), NULL, this, NULL);
		_processProgressInfo(calculatingStage + PPK2023Thread::tr("PPK return code:%1; Description:%2").arg(returnCode).arg(returnCodeDescription));
		_processProgressInfo(calculatingStage + PPK2023Thread::tr("End of mobile station %1 calculation").arg(mobileStationInfo.filePath));// %1 移动站解算结束
	}
	if (solutionInfoMap.isEmpty())
	{
		_processProgressInfo(PPK2023Thread::tr("Did not solve any pos points"));// 没有解算出来任何一个碎步点
		return reply = false;
	}
	return reply = true;
}

bool PPK2023Thread::_postProcessingPPK(const QMap<PPKMobileStationInfo, CALLBACK_SOLUTION>& solutionInfoMap)
{
	bool reply = false;
	const QString generateSolutionReportStage = PPK2023Thread::tr("PPK Generate solution report:");	// PPK生成解算报告环节：

	// 检查取消ppk解算工作标志
	if (m_cancel)
	{
		_processProgressInfo(PPK2023Thread::tr("Manually cancel PPK"));// 手动取消PPK
		return reply = false;
	}

	// 个人认为CameraFromAerialPos的作用应该是通过设置无人机天线的地理坐标、无人机三个姿态角和相机与天线相对相对偏移值(PPKCameraOffset)，计算(转换)出无人机的相机的地理坐标
	//CameraFromAerialPos* m_cameraFromAerialPosHandler;
	return reply = false;
}

bool PPK2023Thread::_checkDataIntegrity(QString & error)
{
	bool reply = false;
	if (m_stationInfoMap.isEmpty())
	{
		error = PPK2023Thread::tr("No data files were imported");//没有导入任何数据文件
		return reply = false;
	}
	for (const PPKMobileStationInfo& singleMobileStationInfo : m_stationInfoMap.keys())
	{
		bool isExist = QFileInfo(singleMobileStationInfo.filePath).isFile();
		if (!isExist)
		{
			error = PPK2023Thread::tr("The %1 mobile station file under the path does not exist").arg(singleMobileStationInfo.filePath);// 路径下%1移动站文件不存在
			return reply = false;
		}
	}
	for (const QList<PPKBaseStationInfo>& singleBaseStationInfoList : m_stationInfoMap.values())
	{
		for (const PPKBaseStationInfo& singleBaseStationInfo : singleBaseStationInfoList)
		{
			bool isExist = QFileInfo(singleBaseStationInfo.filePath).isFile();
			if (!isExist)
			{
				error = PPK2023Thread::tr("The %1 base station file under the path does not exist").arg(singleBaseStationInfo.filePath);// 路径下%1基站文件不存在
				return reply = false;
			}
		}
	}
	if (m_outputDir.isEmpty())
	{
		error = PPK2023Thread::tr("The export path is empty");// 导出路径为空
		return reply = false;
	}
	if (!QDir().mkpath(m_outputDir))
	{
		error = PPK2023Thread::tr("Failed to create export path %1").arg(m_outputDir);// 创建导出路径%1失败
		return reply = false;
	}
	DictionaryW inputCsDictionaryer, outputCsXYZDictionaryer, outputCsBLHDictionaryer;
	IDictionary& inputCsDictionary = inputCsDictionaryer.getInnerObject();
	IDictionary& outputCsXYZDictionary = outputCsXYZDictionaryer.getInnerObject();
	IDictionary& outputCsBLHDictionary = outputCsBLHDictionaryer.getInnerObject();
	CoordType inputCsType, outputCsXYZType, outputCsBLHType;
	m_gnssSettings.getInputCsDictionary(inputCsDictionary, inputCsType);
	m_gnssSettings.getOutputCsDictionary_XYZ(outputCsXYZDictionary, outputCsXYZType);
	m_gnssSettings.getOutputCsDictionary_BLH(outputCsBLHDictionary, outputCsBLHType);
	if (inputCsDictionary.size() <= 0 || outputCsXYZDictionary.size() <= 0 || outputCsBLHDictionary.size() <= 0)
	{
		error = PPK2023Thread::tr("Abnormal coordinate system setting");// 坐标系设置异常
		return reply = false;
	}
	error = "";
	return reply = true;
}

bool PPK2023Thread::_baseStationCsToWgs84XYZCs(QString & error, PPKBaseStationInfo & outBaseStationInfo, const PPKBaseStationInfo & inputBaseStationInfo, const GNSSSettings & gnssSettings)
{
	bool reply = false;
	DictionaryW wgs84XYZCsDictionaryer, baseStationCsDictionaryer;
	IDictionary& wgs84XYZCsDictionary = wgs84XYZCsDictionaryer.getInnerObject();
	IDictionary& baseStationCsDictionary = baseStationCsDictionaryer.getInnerObject();
	bool isConvert = proj4_to_dictionary(SYSTEM_EPSG_4328, wgs84XYZCsDictionary);
	if (!isConvert)
	{
		error = PPK2023Thread::tr("Failed to convert wgs84 spatial Cartesian coordinate system proj string to corresponding coordinate system dictionary");// wgs84空间直角坐标系proj字符串转为对应坐标系字典失败
		return reply = false;
	}
	CoordType baseStationCsType;
	const_cast<GNSSSettings&>(gnssSettings).getInputCsDictionary(baseStationCsDictionary, baseStationCsType);
	bool isEmpty = baseStationCsDictionary.size() == 0;
	if (isEmpty)
	{
		error = PPK2023Thread::tr("The base station coordinate system dictionary is empty");// 基站坐标系字典为空
		return reply = false;
	}
	ICoordTransform *trans = extensible_factory_get_ICoordTransform("coord_lxn", 0);
	int isSetting = trans->setParam(&wgs84XYZCsDictionary, &baseStationCsDictionary);
	int isTransfrom = trans->transfrom(
		CoordSrcDest::COORD_SRC, baseStationCsType,
		CoordSrcDest::COORD_DEST, CoordType::COORD_TYPE_XYZ,
		inputBaseStationInfo.X, inputBaseStationInfo.Y, inputBaseStationInfo.Z,
		outBaseStationInfo.X, outBaseStationInfo.Y, outBaseStationInfo.Z
	);
	outBaseStationInfo.filePath = inputBaseStationInfo.filePath;
	outBaseStationInfo.antennaHeight = inputBaseStationInfo.antennaHeight;
	extensible_factory_release_ICoordTransform(trans);
	trans = nullptr;
	error = "";
	return reply = true;
}

void PPK2023Thread::_processProgressInfo(const QString & progressInfo)
{
	qDebug() << progressInfo;
	emit progressed(progressInfo);
}

void PPK2023Thread::_onSouthRTKInterFaceProgress(CALLBACK_PROGRESS progressInfo)
{
	const QString calculatingStage = PPK2023Thread::tr("PPK calculating stage:");//ppk解算环节：
	const int type = progressInfo.type;
	const int rate = progressInfo.rate;
	QString text = "";
	if (type == 0)//5%
	{
		text = PPK2023Thread::tr("Reading data");
		text += "-" + QString::number(rate) + "%";
	}
	else if (type == 1)//3%
	{
		text = PPK2023Thread::tr("Reading data");
		text += "-" + QString::number(rate) + "%";
	}
	else if (type == 2)//5%
	{
		text = PPK2023Thread::tr("Reading data");
		text += "-" + QString::number(rate) + "%";
	}
	else if (type == 3 || type == 4 || type == 7 || type == 8)//88%
	{
		text = PPK2023Thread::tr("Is calculating");
		text += "-" + QString::number(rate) + "%";
	}
	else if (type == 6)//2%
	{
		text = PPK2023Thread::tr("Is calculating");
		text += "-" + QString::number(rate) + "%";
	}
	else if (type == 9)//完成
	{
		text = PPK2023Thread::tr("Calculating complete");
		text += "-100%";
	}
	_processProgressInfo(calculatingStage + text);
}
