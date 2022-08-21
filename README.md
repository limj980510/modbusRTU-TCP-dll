# modbusRTU-TCP-dll
a dll of modbusRTU/TCP
just some library functions of modbusRTU/TCP,you can use this dynamic library in your projects.

Contains two parts of the program,
one is AutoGroup_BlkDev.cpp, which is responsible for dividing the data point tags into blocks, and then grouping
the second is modbusdrv.cpp, which is responsible for periodically constructing and sending modbus request messages, and is responsible for parsing the received response messages.

The functions included in the library are as follows：

1、AutoGroup_BlkDev.cpp
//解析连续Tag地址，在modbusdrv.cpp中实现
extern int AnalyzeTagAddr_Continuous(char *szAddressInDevice, int nLenBits, char *szBlockName, int nBlockNameLen, int *pnStartBits, int *pnEndBits);
CAutoGroupBlkDev::CAutoGroupBlkDev(vector<PKTAG *> &vecTags) //构造函数
CAutoGroupBlkDev::~CAutoGroupBlkDev(void) //析构函数
// 取地址的第一部分，即冒号、点号、#号、逗号前，作为分块块名，AO;AI;DO;DI
int GetBlockName(PKTAG *pTag, char *szBlockName, int nBlockNameLen)
void CAutoGroupBlkDev::calcTagsStartAndEndBits() //计算tag起始和结束地址
const char * CAutoGroupBlkDev::GenerateGroupName(const char *szBlockName) //生成分组名，一个块一组
//对所有tags进行成组操作
void CAutoGroupBlkDev::AutoGroupTags(GroupVector &vecTagGrpInfo, GROUP_OPTION *pGroupOption)
bool compareDrvTag(PKTAG* left, PKTAG* right) //起始地址位的比较
// 结合上面的成组函数，进行关于组大小的调整，进行分组操作
void CAutoGroupBlkDev::GroupTagsOfGroupType(string strGrpType, vector<PKTAG *> & vecTags, GroupVector &vecGroups, GROUP_OPTION *pGroupOption)
// 将tags放入对应的组内
long TagsToGroups(vector<PKTAG *> &vecAllDevTags, GROUP_OPTION *pGroupOption, GroupVector &vecTagGroup)
// 计算分组的寄存器起始地址、寄存器个数、初始有效位
long CalcGroupRegisterInfo(DRVGROUP *pTagGroup, int nRegisterLenBits)
//更新组的质量
long UpdateGroupQuality(PKDEVICE *pDevice, DRVGROUP *pTagGroup, int nQuality, const char *szQualityFormat, ...)
//更新组的数据
long UpdateGroupData(PKDEVICE *pDevice, DRVGROUP *pTagGroup, const char *szBuffer, long lBufLen, short nStatus){

2、modbusdrv.cpp
//打印日志<<<设备:%s, 收:%s，长度:%d，>>>设备:%s, 发:%s，长度:%d
void LogHex(bool bIsReceive, int nLogLevel, PKDEVICE *pDevice, char *szBuf, int nBufLen)
bool NeedSwap(char *szBlockType) //是否需要高低字节转换
int GetBlockTypeId(char *szBlockType) //获取块名
unsigned char GetReadFuncCodeNo(DRVGROUP *pGroup) //获取功能码
//计算CRC校验值
unsigned short CRC16( unsigned char * puchMsg, unsigned short usDataLen )
unsigned char GetStationID(PKDEVICE *pDevice,DRVGROUP *pTagGroup) //获取站号，也就是从机di'zh
void SetClearRecvBufferFlag(PKDEVICE *pDevice) //进行清空标志位
bool GetClearRecvBufferFlag(PKDEVICE *pDevice) //获取清空标志位
PKDRIVER_EXPORTS long InitDriver(PKDRIVER *pDriver) //初始化驱动,这里面负责定期任务。
PKDRIVER_EXPORTS long InitDevice(PKDEVICE *pDevice) //初始化设备
PKDRIVER_EXPORTS long UnInitDevice(PKDEVICE *pDevice) //反初始化设备
PKDRIVER_EXPORTS long UnInitDriver(PKDRIVER *pDriver) //反初始化驱动
//构件一个读请求数据的modbus包, 如果返回0，说明构建包成功
long BuildReadRequestPacket(PKDEVICE *pDevice, DRVGROUP *pTagGroup, char *szRequestBuff, int nPackBufLen, int &nTotalPackLen, unsigned short &uTransID)
//构件一个控制的modbus包; 如果返回0，说明构建包成功;
long BuildWriteRequestPacket(PKDEVICE *pDevice, DRVGROUP *pTagGroup, PKTAG *pTag, char *szRequestBuff, int nPackBufLen, char *szStrValue, int &nTotalPackLen, int &nFunctionCode, unsigned short &uTransID)
//得到读取指令的数据区的长度（字节为单位）
int GetReadTagGroupDataLen(PKDEVICE *pDevice, DRVGROUP *pTagGroup)
//是否是一个有效的包, 如果是则返回包的总长度
bool IsValidPackage(PKDEVICE *pDevice, DRVGROUP *pTagGroup, char *pBuff, int nBufLen, int &nTotalPackLen, int nReqStationNo, int nReqFuncCode, int nReqTransId)
//找到有效的满足条件的响应包
bool FindValidResponsePackage(PKDEVICE *pDevice, DRVGROUP *pTagGroup, bool bModbusTCP, char *pBuff, int lCurRecvLen, int nReqStationNo, int nReqFuncCode, int nReqTransId)
//解析一个完整的数据包
int ParseOnePackage(PKDEVICE *pDevice, DRVGROUP *pTagGroup, char *pPackBuffer, int nTotalPackLen)
//接收某个事务号（modbustcp）的数据包，modbusrtu的话。应答包中应该包含功能码、子站，modbustcp还包括事务号
//long RecvAndParseReadPacket(PKDEVICE *pDevice, DRVGROUP *pTagGroup, int nReqFuncCode, unsigned short nReqTransId)
//定时器周期性函数
PKDRIVER_EXPORTS long OnTimer(PKDEVICE *pDevice, PKTIMER *timerInfo)
//当有控制命令时该函数被调用
//PKDRIVER_EXPORTS long OnControl(PKDEVICE *pDevice, PKTAG *pTag, const char *szStrValue, long lCmdId)
//解析连续Tag地址
int AnalyzeTagAddr_Continuous(char *szAddressInDevice, int nLenBits, char *szBlockName, int nBlockNameLen, int *pnStartBits, int *pnEndBits)
