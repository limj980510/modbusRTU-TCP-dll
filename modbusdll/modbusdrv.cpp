// Modbus 驱动。设备参数1为0表示ModbusRTU驱动，为1表示ModbusTCP驱动
// 数据块参数1表示站号。站号必须大于等于1
// 参数2：读写用哪些指令；
// 参数3：每个块的最大字节数
// 变量也具有一个参数：表示站号（如果一个设备接入了多个不同的组，每个组的站号可能是不同的）
// DRVTAG的nData1是起始地址位（相对于AI的0地址），nData2是结束地址位，nData3是在该块内的起始位数
// AO:1,DO:1
// AO:1.1,DI:1.1
// 5#AO:1.1
#include "ace/Time_Value.h"
#include "ace/ACE.h"
#include "modbusdrv.h"
#include "math.h"
#include "AutoGroup_BlkDev.h"
#include <memory.h>
#include <cstring>
#include <string.h> // for sprintf
#include <stdlib.h>
#include <cstdio>
#include <time.h>
#include "time.h"
#include "pkcomm/pkcomm.h"

#define EC_ICV_INVALID_PARAMETER                    100
#define EC_ICV_DRIVER_DATABLOCK_TYPECANNOTWRITE		101
#define EC_ICV_DRIVER_DATABLOCK_UNKNOWNTYPE			103
#define EC_ICV_DRIVER_DATABLOCK_INVALIDCMDLENGTH	104
#define EC_ICV_BUFFER_TOO_SMALL                     105

#define DEVPARAM_TRANSID							0	// 事务号
#define DEVPARAM_STATION_NO							1	// 站号
#define DEVPARAM_RWCMD								2	// 读写用什么指令
#define DEVPARAM_MAXBYTES_ONEBLOCK					3	// 每个块的最大ID
#define DEVPARAM_CLEARFLAG							4	// 是否需要先清除缓冲区
#define DEVPARAM_DATA_LEFTBUFFER_LEN				5	// 上次分析完包后，还剩下半个包的缓冲区长度

#define DEVPARAM_DATAPOINTER_RECVBUFF				0	// 保存上次分析完后的半个缓冲区
#define PK_TAGDATA_MAXLEN							4096
#define PACKAGE_LEN                                 1024  //打印包最大长度

void LogHex(bool bIsReceive, int nLogLevel, PKDEVICE *pDevice, char *szBuf, int nBufLen)
{
	char szLog[PACKAGE_LEN] = { 0 };
	unsigned int nLogBufLen = sizeof(szLog);
	PKStringHelper::HexDumpBuf(szBuf, nBufLen, szLog, sizeof(szLog), &nLogBufLen);
	if (bIsReceive)
		Drv_LogMessage(nLogLevel, "<<<设备:%s, 收:%s，长度:%d", pDevice->szName, szLog, nBufLen);
	else
		Drv_LogMessage(nLogLevel, ">>>设备:%s, 发:%s，长度:%d", pDevice->szName, szLog, nBufLen);
	//DWORD thid = GetCurrentThreadId();
	//Drv_LogMessage(PK_LOGLEVEL_NOTICE, "threadid=%ld", thid);  //xhzw-debug
}

/**
*  是否需要高低字节转换.- little_endian needs
*  @param  -[in]  {char*}  szBlockType: [comment]
*  @retval { bool }
*  @version     08/21/2008  xingxing  Initial Version.
*/
bool NeedSwap(char *szBlockType)
{
	if (!is_little_endian())
		return false;

	//@version
	string  strBlockType = szBlockType;
	vector<string> vecTemp = PKStringHelper::StriSplit(strBlockType, "#");
	if (vecTemp.size() >= 2)
	{
		strBlockType = vecTemp[1];
	}

	if (PKStringHelper::StriCmp(strBlockType.c_str(), BLOCK_TYPE_AI) == 0 || PKStringHelper::StriCmp(strBlockType.c_str(), BLOCK_TYPE_AO) == 0)
		return true;
	else
		return false;
}

int GetBlockTypeId(char *szBlockType)
{
	//@version 2019/07/30  在此判断下是否有# 对从站号的解析
	string  strHWBlockName = szBlockType;
	vector<string> vecTemp = PKStringHelper::StriSplit(strHWBlockName, "#");
	if (vecTemp.size() >= 2)
	{
		strHWBlockName = vecTemp[1];
	}
	if (PKStringHelper::StriCmp(strHWBlockName.c_str(), BLOCK_TYPE_AI) == 0)
		return BLOCK_TYPE_ID_AI;
	if (PKStringHelper::StriCmp(strHWBlockName.c_str(), BLOCK_TYPE_AO) == 0)
		return BLOCK_TYPE_ID_AO;
	if (PKStringHelper::StriCmp(strHWBlockName.c_str(), BLOCK_TYPE_DI) == 0)
		return BLOCK_TYPE_ID_DI;
	if (PKStringHelper::StriCmp(strHWBlockName.c_str(), BLOCK_TYPE_DO) == 0)
		return BLOCK_TYPE_ID_DO;

	return BLOCK_TYPE_ID_UNDEFINED;
}

unsigned char GetReadFuncCodeNo(DRVGROUP *pGroup)
{
	return GetBlockTypeId(pGroup->szHWBlockName);
}
/******************************************************************************************************
* FunName : CRC16
* Function : The function returns the CRC16 as a unsigned short type
* Input    : puchMsg - message to calculate CRC upon; usDataLen - quantity of bytes in message
* Output   : CRC value
******************************************************************************************************/
unsigned short CRC16(unsigned char * puchMsg, unsigned short usDataLen)
{
	unsigned char uchCRCHi = 0xFF; /* high byte of CRC initialized */
	unsigned char uchCRCLo = 0xFF; /* low byte of CRC initialized */
	unsigned uIndex; /* will index into CRC lookup table */
	while (usDataLen--) /* pass through message buffer */
	{
		uIndex = uchCRCLo ^ *puchMsg++; /* calculate the CRC */
		uchCRCLo = uchCRCHi ^ auchCRCHi[uIndex];
		uchCRCHi = auchCRCLo[uIndex];
	}
	return (uchCRCHi << 8 | uchCRCLo);
}

//notice ModbusTCP 需加预处理器 定义宏  MODBUSTYPE_TCP
bool IsModbusTCP(PKDEVICE *pDevice)
{
#ifdef MODBUSTYPE_TCP
	return true;
#else
	return false;
#endif
}

// ModbusRTU时，设备ID不能为0，至少是1
// 先取tag点的第三个参数，如果没有则取设备的第三个参数。如果都没有则缺省为1
// 控制时处理了站号，但是读写时未处理站号？？？？？？？？？？？？？？？？？？？
//DRVGROUP *pTagGroup = (DRVGROUP *)pTag->pData1;
unsigned char GetStationID(PKDEVICE *pDevice, DRVGROUP *pTagGroup)
{
	string strBlockName = pTagGroup->szHWBlockName; // 5#AO
	vector<string> vecTemp = PKStringHelper::StriSplit(strBlockName, "#");
	if (vecTemp.size() >= 2)
	{
		string strStationNo = vecTemp[0];
		int nStationNo = ::atoi(strStationNo.c_str());
		return nStationNo;
	}

	//if (pTag && strlen(pTag->szParam) > 0 && ::atoi(pTag->szParam) > 0)
	//	nStationNo = ::atoi(pTag->szParam);
	unsigned char nStationNo = pDevice->nUserData[DEVPARAM_STATION_NO];
	return nStationNo;
}

//进行清空标志位
void SetClearRecvBufferFlag(PKDEVICE *pDevice)
{
	pDevice->nUserData[DEVPARAM_CLEARFLAG] = 1;
}

//获取清空标志位
bool GetClearRecvBufferFlag(PKDEVICE *pDevice)
{
	return pDevice->nUserData[DEVPARAM_CLEARFLAG] != 0;
}

void CheckBlockStatus(PKDEVICE *pDevice, DRVGROUP *pTagGroup, long lSuccess)
{
	if (lSuccess == PK_SUCCESS)
		pTagGroup->nFailCountRecent = 0;
	else
	{
		if (pTagGroup->nFailCountRecent > 10)	// 最近失败次数
		{
			char szTip[1024] = { 0 };
			sprintf(szTip, "read failcount:%d", pTagGroup->nFailCountRecent);
			UpdateGroupQuality(pDevice, pTagGroup, TAG_QUALITY_COMMUNICATE_FAILURE, szTip);
			pTagGroup->nFailCountRecent = 0; // 避免计数太大导致循环
		}
		else
			pTagGroup->nFailCountRecent += 1;
	}
}

/*
初始化驱动
*/
PKDRIVER_EXPORTS long InitDriver(PKDRIVER *pDriver)
{
	Drv_LogMessage(PK_LOGLEVEL_NOTICE, "InitDriver(driver:%s)", pDriver->szName);
	Drv_LogMessage(PK_LOGLEVEL_NOTICE, "设备param1:站号,缺省为1. 参数2:0表示正常指令读写,1表示多寄存器读写,缺省为0.  参数3:每个包最大字节数,缺省不限制.  参数4:未定义");
	return 0;
}
/*
初始化设备
*/
PKDRIVER_EXPORTS long InitDevice(PKDEVICE *pDevice)
{
	Drv_LogMessage(PK_LOGLEVEL_NOTICE, "InitDevice(device:%s)", pDevice->szName);
	pDevice->nUserData[DEVPARAM_TRANSID] = 0;				// 事务号
	pDevice->pUserData[DEVPARAM_DATAPOINTER_RECVBUFF] = new char[DEFAULT_RESPONSE_MAXLEN]; // 保存上次剩余的半个包的缓冲区
	pDevice->nUserData[DEVPARAM_DATA_LEFTBUFFER_LEN] = 0; // 上次剩余的半个包的长度

	// 参数1：站号
	pDevice->nUserData[DEVPARAM_STATION_NO] = 1;			// 站号，缺省为1
	if (pDevice->szParam1 != NULL && atoi(pDevice->szParam1) != 0)
		pDevice->nUserData[DEVPARAM_STATION_NO] = atoi(pDevice->szParam1);

	// 参数2：采用什么指令
	pDevice->nUserData[DEVPARAM_RWCMD] = 0;				// 读写用什么指令,0表示正常的指令，1表示多寄存器指令
	if (pDevice->szParam2 != NULL && atoi(pDevice->szParam2) != 0)
		pDevice->nUserData[DEVPARAM_RWCMD] = atoi(pDevice->szParam2);

	// 参数3：每个块最大字节数  modbus每种设备都有所不同，应该是作为参数传过来比较合适
	if (strlen(pDevice->szParam3) > 0)
	{
		int nCfgMaxBytes = ::atoi(pDevice->szParam3);
		if (nCfgMaxBytes > 0)
			pDevice->nUserData[DEVPARAM_MAXBYTES_ONEBLOCK] = nCfgMaxBytes;
	}

	// 获取到所有的tag点。需要在tag点存储块内偏移（位）、长度（位），组包含的tag点对象列表（以便计算）

	// 进行自组块处理，将所有的tag点自组块成BLOCK
	GroupVector vecTagGroup;
	GROUP_OPTION groupOption;
	groupOption.nIsGroupTag = 1;
	groupOption.nMaxBytesOneGroup = pDevice->nUserData[DEVPARAM_MAXBYTES_ONEBLOCK]; // modbus每种设备都有所不同，应该是作为参数传过来比较合适
	if (groupOption.nMaxBytesOneGroup <= 0) //如果没有输入，则取缺省230个字节
		groupOption.nMaxBytesOneGroup = 230;

	vector<PKTAG *> vecTags;
	for (int i = 0; i < pDevice->nTagNum; i++)
		vecTags.push_back(pDevice->ppTags[i]);
	TagsToGroups(vecTags, &groupOption, vecTagGroup);
	vecTags.clear();


	unsigned int i = 0;
	for (; i < vecTagGroup.size(); i++)
	{
		DRVGROUP *pTagGrp = vecTagGroup[i];

		int nBlockTypeId = GetBlockTypeId(pTagGrp->szHWBlockName);
		int nRegisterLenBits = 1;

		//这里判断一下是否有站号  
		string  strHWBlockName = pTagGrp->szHWBlockName;
		vector<string> vecTemp = PKStringHelper::StriSplit(strHWBlockName, "#");
		if (vecTemp.size() >= 2)
		{
			strHWBlockName = vecTemp[1];
		}

		if (PKStringHelper::StriCmp(strHWBlockName.c_str(), BLOCK_TYPE_AO) == 0 || PKStringHelper::StriCmp(strHWBlockName.c_str(), BLOCK_TYPE_AI) == 0)
			nRegisterLenBits = 16;
		CalcGroupRegisterInfo(pTagGrp, nRegisterLenBits); // AI,AO按照字组块，DI、DO按照位组块

		//获取系统时间
		const time_t t = time(NULL);
		struct tm* systemTime = localtime(&t);
		int intervalSec = (60 - systemTime->tm_sec) * 1000;


		char szBaseTime[32] = { 0 };
		unsigned int nMSec = 0;
		int nSec = PKTimeHelper::GetHighResTime(&nMSec);
		PKTimeHelper::HighResTime2String(szBaseTime, sizeof(szBaseTime), nSec, nMSec);

		string strTime = szBaseTime;
		int curSec = stoi(strTime.substr(17, 2).c_str()); //获取当前秒

		Drv_LogMessage(PK_LOGLEVEL_NOTICE, "!!!!!!!!!   %d ---------  %d   !!!!!!!!!!!!!!!!!!", curSec, systemTime->tm_sec);
		//PKTIMER timerInfo;
		//// 定时周期，单位毫秒 TODO
		//timerInfo.nPeriodMS = pTagGrp->nPollRate;

		//// 设置相位, 定时器的起始时间，可以合理设置做到整分整秒  TODO
		//timerInfo.nPhaseMS = (60 - curSec) * 1000;
		//

		PKTIMER timerInfo;
		//timerInfo.nPeriodMS = pTagGrp->nPollRate;
		timerInfo.nPeriodMS = 60000;
		//修改相位做到整点
		timerInfo.nPhaseMS = intervalSec;


		timerInfo.pUserData[0] = pTagGrp;
		void *pTimerHandle = Drv_CreateTimer(pDevice, &timerInfo); // 设定定时器
		Drv_LogMessage(PK_LOGLEVEL_NOTICE, "modbus, device:%s, autogroup:%s,tagcount:%d, cycle:%d ms,registertype:%s,startregno:%d,endregno:%d",
			pDevice->szName, pTagGrp->szAutoGroupName, pTagGrp->vecTags.size(), pTagGrp->nPollRate, pTagGrp->szHWBlockName, pTagGrp->nBeginRegister, pTagGrp->nEndRegister);
		string strTags = "";
		for (int iTag = 0; iTag < pTagGrp->vecTags.size(); iTag++)
		{
			PKTAG *pTag = pTagGrp->vecTags[iTag];
			if (strTags.empty())
				strTags = pTag->szName;
			else
				strTags = strTags + "," + pTag->szName;
		}
		Drv_LogMessage(PK_LOGLEVEL_NOTICE, "modbus, device:%s, groupname:%s, tagnames:%s", pDevice->szName, pTagGrp->szAutoGroupName, strTags.c_str());
	}
	return 0;
}

/*
反初始化设备
*/
PKDRIVER_EXPORTS long UnInitDevice(PKDEVICE *pDevice)
{
	Drv_LogMessage(PK_LOGLEVEL_NOTICE, "UnInitDevice(device:%s)", pDevice->szName);
	return 0;
}

/*
反初始化驱动
*/
PKDRIVER_EXPORTS long UnInitDriver(PKDRIVER *pDriver)
{
	Drv_LogMessage(PK_LOGLEVEL_NOTICE, "UnInitDriver(driver:%s)", pDriver->szName);
	return 0;
}


/*
ModbusTCP无校验码，头部6个长度，数据区（pdu）读请求为6个字节，读应答根据请求寄存器长度而定。
从地址1开始读取10个AO寄存器的请求，固定12个字节，蓝色为请求头部，红色为数据负荷pdu：
03 6D 00 00 00 06 01 03 00 00 00 0A
解释：
03 6D两个字节是主机发出的检验信息（事务号）。从机（即设备）只需将这两个字节的内容copy以后再放到response的报文相应位置，头部
00 00 两个字节表示tcp / ip的协议是modbus协议，头部
00 06表示该字节以后的长度。读固定为6个字节，头部
01 站号，缺省为1，头部
03 功能码，读取AO为03，读取AI为04，读取DI为02，读取DO为01，头部
00 起始地址高字节，PDU部分数据
00 起始地址低字节，PDU部分数据
00 读取寄存器个数，高字节，PDU部分数据
0A 读取寄存器个数，低字节，PDU部分数据

设备返回的从地址1开始读取10个AO寄存器的请求的应答（29个字节）。蓝色为头部，红色为pdu数据负荷。
03 6D 00 00 00 17 01 03 14 00 00 00 01 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
03 6D检验信息，事务号，必须和请求相同
00 00 tcp / ip为modbus协议
00 17 表示该字节后的长度
01 设备地址
03 功能码
14 该字节后的字节数（20），10个寄存器共20个字节
*/
/**
*
* 构件一个读请求数据的modbus包, 如果返回0，说明构建包成功
*
*  @param - [in] { PKDEVICE * } pDevice: 驱动对象
*  @param - [in] { DRVGROUP * } pTagGroup: tag组
*  @param - [in] { char * } szRequestBuff: 报文缓存
*  @param - [in] { int }  nPackBufLen: 报文缓存长度
*  @param - [out] { int &} nTotalPackLen:  完整一条读数据 报文字节长度
*  @param - [out] { unsigned short & } uTransID:  modbustcp 事务号
*  @retval{ long }  success : 0, fail: -1
*  @version     12 / 11 / 2008    Initial Version.
*/
long BuildReadRequestPacket(PKDEVICE *pDevice, DRVGROUP *pTagGroup, char *szRequestBuff, int nPackBufLen, int &nTotalPackLen, unsigned short &uTransID)
{
	bool bModbusTCP = IsModbusTCP(pDevice);
	memset(szRequestBuff, 0, nPackBufLen);

	char* pTransmit = szRequestBuff;
	if (pTagGroup->nRegisterNum <= 0)
	{
		Drv_LogMessage(PK_LOGLEVEL_NOTICE, "%s, group:%s, blocktype:%s, registernum:%d <= 0", pDevice->szName, pTagGroup->szAutoGroupName, pTagGroup->szHWBlockName, pTagGroup->nRegisterNum);
		return -1;
	}

	// 起始地址，ModbusTCP才有
	if (bModbusTCP)
	{
		uTransID = (unsigned short) ++pDevice->nUserData[DEVPARAM_TRANSID];

		// 事务号为2个字节
		memcpy(pTransmit, &uTransID, 2);
		pTransmit += 2;

		// 协议标识符，先高字节，后低字节，都是0
		memset(pTransmit, 0, 2);
		pTransmit += 2;

		// 长度字段（后续字节的数量），高字节
		*pTransmit = 0;
		pTransmit++;
		//  长度字段（后续字节的数量），低字节
		*pTransmit = 0x06; // 后续请求头部的长度，固定为6个字节
		pTransmit++;
	}

	// 下面开始时Modbus共有的
	unsigned char nStationNo = GetStationID(pDevice, pTagGroup);
	// 站号
	memcpy(pTransmit, &nStationNo, 1);
	pTransmit++;

	// 下面是Modbus请求头部，5个字节
	// 功能码
	unsigned char nFuncCode = GetReadFuncCodeNo(pTagGroup);
	memcpy(pTransmit, &nFuncCode, 1);// 功能码和类型数值相同;
	pTransmit++;

	//nStartAddress += 1; // 起始地址从0开始.AO:8的nBeginRegister为7
	// 起始地址高字节
	*pTransmit = (pTagGroup->nBeginRegister) >> BITS_PER_BYTE; // 地址是字，寄存器
	pTransmit++;
	// 起始地址低字节
	*pTransmit = (pTagGroup->nBeginRegister) & 0xFF;
	pTransmit++;

	// 读取bit/寄存器个数，高字节
	*pTransmit = pTagGroup->nRegisterNum >> BITS_PER_BYTE;
	pTransmit++;
	// 读取bit/寄存器个数，低字节
	*pTransmit = pTagGroup->nRegisterNum & 0xFF;
	pTransmit++;

	int nDataLen = pTransmit - szRequestBuff;
	if (!bModbusTCP) // ModbusRTU
	{
		// 计算CRC
		unsigned short nCRCValue = CRC16((unsigned char *)szRequestBuff, nDataLen);
		memcpy(pTransmit, &nCRCValue, sizeof(unsigned short));
		pTransmit += 2;
	}
	nTotalPackLen = pTransmit - szRequestBuff;
	return 0;
}

/**
*
* 构件一个控制的modbus包; 如果返回0，说明构建包成功;
*
*  @param - [in] { PKDEVICE * } pDevice: 驱动对象
*  @param - [in] { DRVGROUP * } pTagGroup: tag组
*  @param  -[in] { PKTAG * } pTag :某个Tag参数
*  @param - [in] { char * } szRequestBuff: 报文缓存
*  @param - [in] { int }  nPackBufLen: 报文缓存长度
*  @param - [out] { int &} nTotalPackLen:  完整一条读数据 报文字节长度
*  @param - [out] { unsigned short & } uTransID:  modbustcp 事务号
*  @retval { long } success : 0
*  @version     12 / 11 / 2008    Initial Version.
*/
long BuildWriteRequestPacket(PKDEVICE *pDevice, DRVGROUP *pTagGroup, PKTAG *pTag, char *szRequestBuff, int nPackBufLen, char *szStrValue, int &nTotalPackLen, int &nFunctionCode, unsigned short &uTransID)
{
	char szBinValue[PK_TAGDATA_MAXLEN] = { 0 };
	int nBinValueLen = 0;
	Drv_TagValStr2Bin(pTag, szStrValue, szBinValue, sizeof(szBinValue), &nBinValueLen);

	bool bModbusTCP = IsModbusTCP(pDevice);
	memset(szRequestBuff, 0, nPackBufLen);

	int nBlockType = GetBlockTypeId(pTagGroup->szHWBlockName);

	int nStartBits = pTag->nStartBit;// 相对于块内的起始地址位（如AI/DI)
	int nEndBits = pTag->nEndBit; // 相对于块内的结束地址位（如AI/DI)
	int nLenBits = nEndBits - nStartBits + 1; // 长度（位表示）

	// 起始地址和要控制的地址
	long lStartAddress = nStartBits;	// 以0开始的地址// 要控制的地址长度,是以寄存器长度为单位的
	int  nRegisterNum = nLenBits;
	int  nByteCountToWrite = (int)ceil((nLenBits) / 8.0f);	// 写入数据的字节数,协议中要求写入是以字节数为单位的

	if (nBlockType == BLOCK_TYPE_ID_AO){ // AO
		lStartAddress = (int)ceil(nStartBits / 16.0f); // AO是2个字节为单位
		nRegisterNum = (int)ceil(nRegisterNum / 16.0f); // AO是2个字节为单位
	}
	else // DO
	{
		lStartAddress = nStartBits;	// 以0开始的地址// 要控制的地址长度,是以寄存器长度为单位的
		nRegisterNum = nLenBits;
	}

	if (pDevice->nUserData[DEVPARAM_RWCMD] != 0)
		nFunctionCode = (nBlockType == BLOCK_TYPE_ID_DO ? FUNCCODE_WRITE_MULTIDO : FUNCCODE_WRITE_MULTIAO);
	else
	{
		if (nRegisterNum == 1)
			nFunctionCode = (nBlockType == BLOCK_TYPE_ID_DO ? FUNCCODE_WRITE_SGLDO : FUNCCODE_WRITE_SGLAO);
		else
			nFunctionCode = (nBlockType == BLOCK_TYPE_ID_DO ? FUNCCODE_WRITE_MULTIDO : FUNCCODE_WRITE_MULTIAO);
	}

	char* pTransmit = szRequestBuff;
	uTransID = 0;
	if (bModbusTCP)
	{
		// 事务处理号
		unsigned short uTransID = (unsigned short) ++pDevice->nUserData[DEVPARAM_TRANSID];

		// 按照协议组织写消息
		// 事务处理号，高字节
		*pTransmit = uTransID >> BITS_PER_BYTE;
		pTransmit++;

		// 事务处理号，低字节
		*pTransmit = uTransID & 0xFF;
		pTransmit++;

		// 协议标识符，先高字节，后低字节
		*pTransmit = 0;
		pTransmit++;
		*pTransmit = 0;
		pTransmit++;

		// 长度字段，高字节
		*pTransmit = (7 + nByteCountToWrite) >> BITS_PER_BYTE;
		pTransmit++;

		// 长度字段，低字节
		*pTransmit = (nFunctionCode == FUNCCODE_WRITE_SGLAO || nFunctionCode == FUNCCODE_WRITE_SGLDO) ? MB_WRITEPDU_LENGTH : ((7 + nByteCountToWrite) & 0xFF);   // 后面字节的数量
		pTransmit++;
	} // bModbusTCP

	// 站号
	*pTransmit = GetStationID(pDevice, pTagGroup);
	pTransmit++;

	// 功能码
	*pTransmit = nFunctionCode;
	pTransmit++;

	// 起始地址高字节
	*pTransmit = lStartAddress >> BITS_PER_BYTE;
	pTransmit++;

	// 起始地址低字节
	*pTransmit = lStartAddress & 0xFF;
	pTransmit++;

	// 功能码为06时不需要这几个字段。写多个线圈或者寄存器请求
	if (nFunctionCode != FUNCCODE_WRITE_SGLAO && nFunctionCode != FUNCCODE_WRITE_SGLDO)
	{
		// 写入线圈数/寄存器个数，高字节
		*pTransmit = nRegisterNum >> BITS_PER_BYTE;
		pTransmit++;

		// 写入线圈数/寄存器个数，低字节
		*pTransmit = nRegisterNum & 0xFF;
		pTransmit++;

		// 写入数据的字节数
		*pTransmit = nByteCountToWrite & 0xFF;
		pTransmit++;
	}

	//////////////////////////////////////
	if (nBlockType == BLOCK_TYPE_ID_DO)
	{
		// 一个寄存器即只有1个比特，为1时将所有bit全部置为1
		if (nRegisterNum == 1 && nFunctionCode != FUNCCODE_WRITE_MULTIDO) // 比特值：0或1
		{
			if (szBinValue[0] == 0)
				*pTransmit = 0;
			else
				*pTransmit = (char)0xFF;

			pTransmit++;
			*pTransmit = 0;
			pTransmit++;
		}
		else
		{
			// edit by xingxing,强制写多个线圈时不一定是写8个位的整数倍，因此需要考虑非8的整数倍的情况 
			for (int i = 0; i < nByteCountToWrite; i++)
			{
				*pTransmit = szBinValue[i];
				pTransmit++;
			}
		}
	}
	else // BLOCK_TYPE_AO
	{
		// 先写整个寄存器
		// 可能该次请求要控制的缓冲区的头部、尾部都有不是完整寄存器长度（如4个字节）位，要先判断出来并写入
		if (nLenBits == 1 || (nLenBits % 8 != 0)) // 比特值：0或1
		{
			// 待写入数据
			//int lValueStatus = 0;
			//unsigned short nData = 0;
			//PKTAGDATA tagData;
			//strncpy(tagData.szName, pTag->szName,sizeof(tagData.szName) - 1);
			//int lRet = Drv_GetTagData(pDevice,&tagData); // 获取到上次的数据
			//if(lRet != PK_SUCCESS)
			//	return lRet;

			//if(lValueStatus != TAG_QUALITY_GOOD)
			//	return -1;

			//memcpy(&nData, &tagData, 2); // AO or AI
			//// 设置指定bit处的值
			//int nBitNo = nStartBits % 16; // AO:W5.13, 一定是某个Word的余数
			//nData = nData & ~(1 << nBitNo);
			//nData = nData | (szBinValue[0] << nBitNo);
			//
			//// 待写入数据
			//*pTransmit = nData >> BITS_PER_BYTE;
			//pTransmit++;
			//*pTransmit = (nData & 0xFF);
			//pTransmit++;
		}
		else // TAG_DATATYPE_ANALOG、TAG_DATATYPE_BLOB、TAG_DATATYPE_TEXT
		{
			// 待写入数据
			if (NeedSwap(pTagGroup->szHWBlockName))
				SwapByteOrder(szBinValue, nBinValueLen, 2);

			// 写入字节数刚好等于寄存器字节时
			if (nByteCountToWrite == nBinValueLen)
			{
				for (int i = 0; i < nBinValueLen; i++)
				{
					*pTransmit = szBinValue[i];
					pTransmit++;
				}
			}
			else
			{
				int i = 0;
				// 写入字节数不等于寄存器字节数时
				// 第一部分：已经swap过的值，由高到低
				for (i = 0; i < (int)(nRegisterNum - 1) * 2; i++)
				{
					*pTransmit = szBinValue[i];
					pTransmit++;
				}

				// 第二部分：最后一个寄存器的高字节
				int nToAdd = nByteCountToWrite - nBinValueLen;
				for (int j = 0; j < nToAdd; j++)
				{
					*pTransmit = 0;
					pTransmit++;
				}

				// 第三部分：最后一个寄存器的低字节，由高到低
				for (; i < nBinValueLen; i++)
				{
					*pTransmit = szBinValue[i];
					pTransmit++;
				}
			}
		}
	}

	// 生成的写消息长度
	nTotalPackLen = pTransmit - szRequestBuff;
	if (!bModbusTCP)
	{
		unsigned short nCRCValue = CRC16((unsigned char *)szRequestBuff, nTotalPackLen);
		memcpy(pTransmit, &nCRCValue, sizeof(unsigned short));
		pTransmit += 2;
		nTotalPackLen += sizeof(unsigned short);
	}

	return 0;
}

/**
*
*  得到读取指令的数据区的长度（字节为单位）
*
*  @param - [in] { PKDEVICE * } pDevice: 驱动对象
*  @param - [in] { DRVGROUP * } pTagGroup: tag组
*  @retval  {int}  数据区的字节长度
*  @version     12 / 11 / 2008    Initial Version.
*/
int GetReadTagGroupDataLen(PKDEVICE *pDevice, DRVGROUP *pTagGroup)
{
	int nDataLenShould = 0;
	int nBlockType = GetBlockTypeId(pTagGroup->szHWBlockName);
	if (nBlockType == BLOCK_TYPE_ID_DO || nBlockType == BLOCK_TYPE_ID_DI) // DI、DO
		nDataLenShould = ceil(pTagGroup->nRegisterNum / 8.0f);
	else // AI、AO
		nDataLenShould = pTagGroup->nRegisterNum * 2;
	return nDataLenShould;
}

/**
// 是否是一个有效的包, 如果是则返回包的总长度
// modbustcp:
// 读取请求:3d 00(事务号)	 00 00(协议版本)	 00 06(后面数据长度)	 01(从机地址)	 01(功能号)	 00 00(数据地址)	 00 64(读取数据个数)
// 读取应答:3d 00(事务号)	 00 00(协议版本)	 00 10(后面数据长度)	 01(从机地址)	 01(功能号)	 0d(读取到的字节数） 0c 4c 2a a9 14 22 65 0c 07 00 00 00 01
// 北辰应答:1a 00(事务号)    00 00(协议版本)     00 06(发送的数据长度)   01(从机地址)	 04(功能号)  42 6e 00 38  aa 71 18 4c 75 00 07 00 00 21 e9 60 84 02 0b 00 00 00
// modbusrtu:
// 读取请求：0x01(从机地址)	03(功能号)	00 01(数据地址)		00 01(读取数据个数)	D5 CA(CRC校验)
// 读取应答：0x01(从机地址)	03(功能号)	02(数据字节个数)	00 17(两个字节数据)	F8 4A(CRC校验)
*  @param - [in] { PKDEVICE * } pDevice: 驱动对象
*  @param - [in] { DRVGROUP * } pTagGroup: tag组
*  @param - [in] { char * } pBuff: 报文内容
*  @param - [in] { int} nBufLen: 报文长度
*  @param - [out] { int } nTotalPackLen: 返回包的总长度
*  @param - [in] { int } nReqStationNo: 站号-从机地址
*  @param - [in] { int } nReqFuncCode:  功能号
*  @param - [in] { int } nReqTransId:  modbustcp事务号
*  @retval { bool } success : true , fail: false
*  @version     12 / 11 / 2008    Initial Version.
*/
bool IsValidPackage(PKDEVICE *pDevice, DRVGROUP *pTagGroup, char *pBuff, int nBufLen, int &nTotalPackLen, int nReqStationNo, int nReqFuncCode, int nReqTransId)
{
	char *pBuffBegin = pBuff;
	unsigned char nStationNo = 0xFF;
	unsigned char nFuncCode = 0xFF;

	int nDataLenBytesSend = GetReadTagGroupDataLen(pDevice, pTagGroup);

	bool bModbusTCP = IsModbusTCP(pDevice);
	if (bModbusTCP)
	{
		unsigned short nTransIdInResponse = *(unsigned short *)pBuff;
		if (nTransIdInResponse != nReqTransId) // 事务号必须相等！！！
		{
			return false;
		}

		unsigned short nProtocolNo = *(unsigned short *)(pBuff + 2); // 协议号	// 跳过协1d议号。永远是0
		if (nProtocolNo != 0)
			return false;

		nStationNo = *(unsigned char*)(pBuff + 6);
		nFuncCode = *(unsigned char*)(pBuff + 7);
		// 站号和功能码的检查
		if (nStationNo != nReqStationNo || nFuncCode != nReqFuncCode)
			return false;
		unsigned short nBodyLenBytesSend = nDataLenBytesSend + 3; // 数据长度+站号1+功能码1+数据长度1
		unsigned short nBodyLenBytesResponseHigh = *(unsigned char*)(pBuff + 4);
		unsigned short nBodyLenBytesResponseLow = *(unsigned char*)(pBuff + 5);
		unsigned short nBodyLenBytesResponse = (nBodyLenBytesResponseHigh << BITS_PER_BYTE) + nBodyLenBytesResponseLow; // 后面数据的长度+站号+功能号+数据长度，北辰模块这个长度为0x06
		unsigned char nDataLenBytesResponse = *(unsigned char*)(pBuff + 8); // 后面数据的长度+站号+功能号+数据长度，北辰模块这个长度为0
		if (nBodyLenBytesResponse != nBodyLenBytesSend && nBodyLenBytesResponse != 0x06)
			return false;
		if (nDataLenBytesResponse != nDataLenBytesSend && nBodyLenBytesResponse != 0x00) // 北辰的长度为0
			return false;
		// int uPackBodyLen = (((unsigned char)*pBuff) << BITS_PER_BYTE) + (unsigned char)*(pBuff + 1);
		nTotalPackLen = nBodyLenBytesSend + 6; // 后面长度（字节）+包头6个字节，3d 00(事务号)	 00 00(协议版本)	 00 10(后面数据长度)

		if (nBufLen < nTotalPackLen)
			return false;
	}
	else
	{
		nStationNo = *(unsigned char*)(pBuff + 0);
		nFuncCode = *(unsigned char*)(pBuff + 1);
		unsigned char nDataLenBytesResponse = *(unsigned char*)(pBuff + 2);
		nTotalPackLen = nDataLenBytesResponse + 5; // 连同头部和CRC，0x01(从机地址)	03(功能号)	02(数据字节个数)	00 17(两个字节数据)	F8 4A(CRC校验)
		// 站号和功能码的检查
		if (nStationNo != nReqStationNo || nFuncCode != nReqFuncCode)
			return false;
		//包长度的检查
		if (nBufLen < nTotalPackLen)
			return false;

		int nCRCInResponse = *(unsigned short*)(pBuffBegin + nTotalPackLen - 2); // 包中的CRC值

		// 计算CRC
		unsigned short nCRCValue = CRC16((unsigned char *)pBuffBegin, nTotalPackLen - 2);
		if (nCRCValue != nCRCInResponse)
			return false;
	}

	return true;
}

/*
*  找到有效的满足条件的响应包
*  @param - [in] { PKDEVICE * } pDevice: 驱动对象
*  @param - [in] { DRVGROUP * } pTagGroup: tag组
*  @param - [in] { bool}  bModbusTCP: 是否是ModbusTCP协议
*  @param - [in] { char * } pBuff: 报文内容
*  @param - [in] { int} lCurRecvLen: 当前报文长度
*  @param - [in] { int } nReqStationNo: 站号-从机地址
*  @param - [in] { int } nReqFuncCode:  功能号
*  @param - [in] { int } nReqTransId:  modbustcp事务号
*  @retval { bool }  success : true , fail: false
*  @version     12 / 11 / 2008    Initial Version.
*/
bool FindValidResponsePackage(PKDEVICE *pDevice, DRVGROUP *pTagGroup, bool bModbusTCP, char *pBuff, int lCurRecvLen, int nReqStationNo, int nReqFuncCode, int nReqTransId)
{
	// 头部+子站号+功能码
	if ((bModbusTCP && lCurRecvLen < PROTOCOL_TCP_PACKAGE_HEADER_LEN + 2) ||
		(!bModbusTCP && lCurRecvLen < PROTOCOL_RTU_PACKAGE_HEADER_LEN + 2))
	{
		// Drv_LogMessage(PK_LOGLEVEL_ERROR, "send to device:%s success, but receive %d bytes < header len!!!", pDevice->szName, lCurRecvLen);
		return false;
	}

	bool bFoundPack = false;
	char *pPacksBegin = pBuff;
	char *pPacksEnd = pPacksBegin + lCurRecvLen;
	while (pPacksBegin < pPacksEnd)
	{
		// 假定pPacksBegin是一个包的包头
		int nTotalPackLen = 0;
		bool bValidPack = IsValidPackage(pDevice, pTagGroup, pPacksBegin, lCurRecvLen, nTotalPackLen, nReqStationNo, nReqFuncCode, nReqTransId);
		if (!bValidPack)
		{
			pPacksBegin++; // 未找到，那么找下一个合法的包
			continue;
		}

		bFoundPack = true;
		break;
	}

	return bFoundPack;
}

/**
// 解析一个完整的数据包
// modbustcp:
// 读取请求:3d 00(事务号)	 00 00(协议版本)	 00 06(后面数据长度)	 01(从机地址)	 01(功能号)	 00 00(数据地址)	 00 64(读取数据个数)
// 读取应答:3d 00(事务号)	 00 00(协议版本)	 00 10(后面数据长度)	 01(从机地址)	 01(功能号)	 0d(读取到的字节数） 0c 4c 2a a9 14 22 65 0c 07 00 00 00 01
// modbusrtu:
// 读取请求：0x01(从机地址)	 03(功能号)			 00 01(寄存器地址)		 00 01(读取寄存器个数)	D5 CA(CRC校验)
// 读取应答：0x01(从机地址)	 03(功能号)			 02(数据字节个数=上面的寄存器个数*2)		 00 17(两个字节数据)	F8 4A(CRC校验)
*  @param  -[in] { PKDEVICE * } pDevice : 驱动对象
*  @param  -[in] { DRVGROUP * } pTagGroup : tag组
*  @param  -[in] { char * } pPackBuffer : 报文内容
*  @param  -[in] { int } nTotalPackLen: 报文长度
*  @retval { int }  success : 0 , fail: others
*  @version     12/11/2008    Initial Version.
*/
int ParseOnePackage(PKDEVICE *pDevice, DRVGROUP *pTagGroup, char *pPackBuffer, int nTotalPackLen)
{
	int nStartBytes = pTagGroup->nStartAddrBit / 8; // 从0开始的第N个字节
	int nEndBytes = pTagGroup->nEndAddrBit / 8; // 从0开始的第N个字节
	int nLenBytes = nEndBytes - nStartBytes + 1;
	int nBlockSize = nLenBytes; //(int)ceil((pTagGroup->nEndBit - pTagGroup->nStartAddrBit + 1)/8.0f);
	// 循环解析收到的所有可能数据包
	// 2表示站号（1个字节）和功能码（1个字节）。站号表示从机地址
	// 任何请求的头6个字节都是一样的意义，第7和第8个字节也是一样的意义（站号和功能码）
	long nRecvBytes = nTotalPackLen;
	char *pCurBuf = (char *)pPackBuffer;
	char *pDataBuff = pPackBuffer;
	int nDataBufLen = 0;
	int nFuncCode = 0;

	bool bModbusTCP = IsModbusTCP(pDevice);
	if (bModbusTCP)
	{
		pDataBuff += 9; // 从第9个字节开始
		nDataBufLen = nTotalPackLen - 9;
		nFuncCode = *(unsigned char *)(pPackBuffer + 7);
	}
	else
	{
		pDataBuff += 3; // 从第9个字节开始
		nDataBufLen = nTotalPackLen - 5;
		nFuncCode = *(unsigned char *)(pPackBuffer + 1);
	}

	// 5/6/15/16 是写请求的应答, 读取请求的响应包 1/2/3/4是读请求
	// 写请求应答不需要处理数据
	if (nFuncCode == 5 || nFuncCode == 6 || nFuncCode == 15 || nFuncCode == 16)
	{
		Drv_LogMessage(PK_LOGLEVEL_NOTICE, "success to parse write control response, function code %d, device:%s, block:%s", nFuncCode, pDevice->szName, pTagGroup->szAutoGroupName);
		return TAG_QUALITY_GOOD; // 驱动框架会自动设置控制的状态
	}

	char *szBlockType = pTagGroup->szHWBlockName; // AI/DI/AO/DO
	// 设置数据块的数据
	if (NeedSwap(szBlockType))
	{
		// 每个寄存器的长度（位）
		int nElemBits = 1;

		//先判断下是否有从站号
		string  strBlockType = szBlockType;
		vector<string> vecTemp = PKStringHelper::StriSplit(strBlockType, "#");
		if (vecTemp.size() >= 2)
		{
			strBlockType = vecTemp[1];
		}
		if (PKStringHelper::StriCmp(strBlockType.c_str(), BLOCK_TYPE_AO) == 0 || PKStringHelper::StriCmp(strBlockType.c_str(), BLOCK_TYPE_AI) == 0)
			nElemBits = 16;

		// pCurBuf指向第6个字节，应该再跳过站号、功能码和长度共三个字节
		SwapByteOrder(pDataBuff, nDataBufLen, nElemBits / BITS_PER_BYTE);
	}

	// 更新数据块的数据。当在定时器中读取到数据后，调用该接口更新数据以便过程数据库读取
	long lRet = UpdateGroupData(pDevice, pTagGroup, pDataBuff, nDataBufLen, TAG_QUALITY_GOOD);
	if (lRet == 0)
		Drv_LogMessage(PK_LOGLEVEL_INFO, "device:%s, block:%s, success to update %d tag, databytes:%d", pDevice->szName, pTagGroup->szAutoGroupName, pTagGroup->vecTags.size(), nDataBufLen);
	else
		Drv_LogMessage(PK_LOGLEVEL_ERROR, "device:%s, block:%s, fail to update %d tag, databytes:%d, retcode:%d", pDevice->szName, pTagGroup->szAutoGroupName, pTagGroup->vecTags.size(), nDataBufLen, lRet);
	return 0;
}

/**
*  接收某个事务号（modbustcp）的数据包，modbusrtu的话。应答包中应该包含功能码、子站，modbustcp还包括事务号
*  @param  -[in] { PKDEVICE * } pDevice :  驱动对象
*  @param  -[in] { DRVGROUP * } pTagGroup :tag组
*  @param  -[in] { int } nReqFuncCode : 功能码
*  @param  -[in] { unsigned short } nReqTransId : modbustcp还包括事务号
*  @retval { long }  success : 0 , fail: others
*  @version     12/11/2008    Initial Version.
*/
long RecvAndParseReadPacket(PKDEVICE *pDevice, DRVGROUP *pTagGroup, int nReqFuncCode, unsigned short nReqTransId)
{
	Drv_LogMessage(PK_LOGLEVEL_DEBUG, "--->RecvAndParseReadPacket");
	bool bModbusTCP = IsModbusTCP(pDevice);
	int nReqStationNo = GetStationID(pDevice, pTagGroup); // 不取tag点的站号

	// 从设备接收应答消息
	long lRet = 0;
	long lCurRecvLen = pDevice->nUserData[DEVPARAM_DATA_LEFTBUFFER_LEN]; // 上次剩余的长度
	char *pCurRecvBuff = (char *)pDevice->pUserData[DEVPARAM_DATAPOINTER_RECVBUFF];
	int nBlankBufLen = DEFAULT_RESPONSE_MAXLEN - lCurRecvLen;
	char szResponse[DEFAULT_RESPONSE_MAXLEN] = { 0 };

	Drv_LogMessage(PK_LOGLEVEL_DEBUG, "user data info, lCurRecvLen=%d,pCurRecvBuff=%x", lCurRecvLen, pCurRecvBuff);

	// 先收完所有的数据，直到没有数据可收了（收不到，或者收满了），或者超时了
	ACE_Time_Value tvBegin = ACE_OS::gettimeofday();
	while (true)
	{
		Drv_LogMessage(PK_LOGLEVEL_DEBUG, "recv loop...");
		ACE_Time_Value tvNow = ACE_OS::gettimeofday();
		ACE_Time_Value tvSpan = tvNow - tvBegin; // 已经过得时间

		int nTimeoutMS = 5000 - tvSpan.msec(); // 每次最多等待8秒. PLC-5通过网络转之后，2秒都收不到数据
		if (nTimeoutMS < 0 || nTimeoutMS > 100 * 1000) // 超时用尽还没有满足条件则跳过, 或者修改了系统时间
		{
			Drv_LogMessage(PK_LOGLEVEL_DEBUG, "timeout. stop loop");
			break;
		}

		if (lCurRecvLen > 2048)
		{
			Drv_LogMessage(PK_LOGLEVEL_ERROR, "device:%s, recv reponse len:%d > 2048, abandon these bytes!", pDevice->szName, lCurRecvLen);
			lCurRecvLen = 0;
			pDevice->nUserData[DEVPARAM_DATA_LEFTBUFFER_LEN] = 0;
			break;
		}

		long lRecvBytes = Drv_Recv(pDevice, pCurRecvBuff, nBlankBufLen, nTimeoutMS);
		//Drv_LogMessage(PK_LOGLEVEL_DEBUG, "lRecvBytes=%d", lRecvBytes);
		if (lRecvBytes > 250) continue;
		if (lRecvBytes <= 0) // 收不到数据了, 或者缓冲区收满了
		{
			Drv_LogMessage(PK_LOGLEVEL_DEBUG, "no more data. stop loop");
			break;
		}
		LogHex(true, PK_LOGLEVEL_INFO, pDevice, pCurRecvBuff, lRecvBytes);
		// 仅仅接收1次是不够的
		lCurRecvLen += lRecvBytes; // 已经接收到的长度
		pCurRecvBuff += lRecvBytes; // 接收缓冲区指针
		nBlankBufLen -= lRecvBytes; // 空余的可接收数据的缓冲区长度
		char *pPacksBegin = (char *)pDevice->pUserData[DEVPARAM_DATAPOINTER_RECVBUFF];
		bool bFoundResponse = FindValidResponsePackage(pDevice, pTagGroup, bModbusTCP, pPacksBegin, lCurRecvLen, nReqStationNo, nReqFuncCode, nReqTransId);
		if (bFoundResponse) // 找到了想要的响应包
		{
			Drv_LogMessage(PK_LOGLEVEL_DEBUG, "data found, stop loop");
			break;
		}
	}

	// 下面是超时了，或者是收到合法的包了，都需要处理！
	// 头部+子站号+功能码
	if ((bModbusTCP && lCurRecvLen < PROTOCOL_TCP_PACKAGE_HEADER_LEN + 2) ||
		(!bModbusTCP && lCurRecvLen < PROTOCOL_RTU_PACKAGE_HEADER_LEN + 2))
	{
		Drv_LogMessage(PK_LOGLEVEL_ERROR, "send to device:%s success, but receive %d bytes < header len!!!", pDevice->szName, lCurRecvLen);
		return -2;
	}

	bool bFoundPack = false;
	char *pPacksBegin = (char *)pDevice->pUserData[DEVPARAM_DATAPOINTER_RECVBUFF];
	char *pPacksEnd = pPacksBegin + lCurRecvLen;
	while (pPacksBegin < pPacksEnd)
	{
		// 假定pPacksBegin是一个包的包头
		int nTotalPackLen = 0;
		bool bValidPack = IsValidPackage(pDevice, pTagGroup, pPacksBegin, lCurRecvLen, nTotalPackLen, nReqStationNo, nReqFuncCode, nReqTransId);
		if (!bValidPack)
		{
			pPacksBegin++;
			continue;
		}

		// pPacksBegin和lCurRecvLen是一个完整的包
		ParseOnePackage(pDevice, pTagGroup, pPacksBegin, nTotalPackLen);
		pPacksBegin += nTotalPackLen;
		bFoundPack = true;
	}

	if (bFoundPack)
	{
		// ParseOnePackage已经打印过日志了，这里不需要重复打印！Drv_LogMessage(PK_LOGLEVEL_INFO, "device:%s, now %d bytes with left, found response package[ReqTransId:0x%x]", pDevice->szName, lCurRecvLen, nReqTransId);
		CheckBlockStatus(pDevice, pTagGroup, PK_SUCCESS);
	}
	else
	{
		Drv_LogMessage(PK_LOGLEVEL_ERROR, "device:%s, now %d bytes with left, not found response package[ReqTransId:0x%x]", pDevice->szName, lCurRecvLen, nReqTransId);
		//UpdateGroupQuality(pDevice, pTagGroup, -100, "parsing,packlen:%d < headlen:%d,discard", lCurRecvLen, PROTOCOL_TCP_PACKAGE_HEADER_LEN);
		CheckBlockStatus(pDevice, pTagGroup, -1);
	}

	// 下面是没解析完毕的	
	char *pDevLeftBuff = (char *)pDevice->pUserData[DEVPARAM_DATAPOINTER_RECVBUFF];
	int nLeftLen = pPacksEnd - pPacksBegin;
	pDevice->nUserData[DEVPARAM_DATA_LEFTBUFFER_LEN] = nLeftLen; // 上次剩余的长度
	if (nLeftLen > 0)
		memcpy(pDevLeftBuff, pPacksBegin, nLeftLen);
	/*
	//xhzw-debug
	if (bFoundPack)
	{
	SetClearRecvBufferFlag(pDevice);
	Drv_LogMessage(PK_LOGLEVEL_DEBUG, "SetClearRecvBufferFlag");
	}*/
	Drv_LogMessage(PK_LOGLEVEL_DEBUG, "<---RecvAndParseReadPacket");
	return 0;
}

/**
000014-Tx:03 6D 00 00 00 06 01 03 00 00 00 0A
Client request：
03 6D两个字节是主机发出的检验信息，从机只需将这两个字节的内容copy以后再放到response的报文相应位置。
00 00 两个字节表示tcp/ip的协议是modbus协议
00 06表示该字节以后的长度。

PDU：
01 设备地址
03 功能码
00 00寄存器起始地址
00 0A 寄存器的长度 （10）
000015-Rx:server response：03 6D 00 00 00 17   PDU： 01 03 14 00 00 00 01 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
03 6D检验信息
00 00 tcp/ip为modbus协议
00 17 表示该字节后的长度
01 设备地址
03 功能码
14 该字节后的字节数（20）
*/
/**
*  定时器周期性函数
*  @param  -[in] { PKDEVICE * } pDevice :  驱动对象
*  @param  -[in] {  PKTIMER * } timerInfo :
*  @retval { long }  success : 0 , fail: others
*  @version     12/11/2008    Initial Version.
*/
PKDRIVER_EXPORTS long OnTimer(PKDEVICE *pDevice, PKTIMER *timerInfo)
{
	Drv_LogMessage(PK_LOGLEVEL_DEBUG, "--->OnTimer");
	bool bModbusTCP = IsModbusTCP(pDevice);
	// 组织读取消息
	DRVGROUP *pTagGroup = (DRVGROUP *)timerInfo->pUserData[0];
	unsigned short uReqTransID = 0; // MODBUSTCP的事务号
	char szRequestBuff[DEFAULT_REQUEST_MAXLEN];
	memset(szRequestBuff, 0, sizeof(szRequestBuff));
	int nTotalPackLen = 0;
	int nRet = BuildReadRequestPacket(pDevice, pTagGroup, szRequestBuff, sizeof(szRequestBuff), nTotalPackLen, uReqTransID);
	if (nRet != 0)
		return -1;

	// 先清空接收缓冲区，避免以前发的请求数据没有来得及接收还放在缓冲区，影响本次请求
	if (GetClearRecvBufferFlag(pDevice))
		Drv_ClearRecvBuffer(pDevice);

	// 时间
	time_t tmRequest;
	time(&tmRequest);


	// 生成的读消息长度，应该是固定的12个字节
	long lSentBytes = Drv_Send(pDevice, szRequestBuff, nTotalPackLen, 200);
	if (lSentBytes != nTotalPackLen)
	{
		Drv_LogMessage(PK_LOGLEVEL_ERROR, "device(%s), fail to send request for datablock(%s) (need send:%d, sent:%d), transaction:%d",
			pDevice->szName, pTagGroup->szAutoGroupName, nTotalPackLen, lSentBytes, uReqTransID);
		CheckBlockStatus(pDevice, pTagGroup, -1);
		return -1;
	}
	LogHex(false, PK_LOGLEVEL_INFO, pDevice, szRequestBuff, nTotalPackLen);
	Drv_LogMessage(PK_LOGLEVEL_DEBUG, "device(%s), success to send request for datablock(%s) (sent:%d), transaction:%d",
		pDevice->szName, pTagGroup->szAutoGroupName, lSentBytes, uReqTransID);

	// 功能码
	int nReqFuncCode = GetBlockTypeId(pTagGroup->szHWBlockName); // 功能码和类型数值相同
	nRet = RecvAndParseReadPacket(pDevice, pTagGroup, nReqFuncCode, uReqTransID);
	Drv_LogMessage(PK_LOGLEVEL_DEBUG, "<---OnTimer");
	return nRet;
}

/**
*  当有控制命令时该函数被调用
*  @param  -[in] { PKDEVICE * } pDevice :  驱动对象
*  @param  -[in] { PKTAG * } pTag :某个参数
*  @param  -[in] { const char * } szStrValue:变量值，除blob外都已经已经转换为字符串。blob转换为base64编码
*  @param  -[in] { long } lCmdId :
*  @retval { long }   success : 0 , fail: others
*  @version     12/11/2008    Initial Version.
*/
PKDRIVER_EXPORTS long OnControl(PKDEVICE *pDevice, PKTAG *pTag, const char *szStrValue, long lCmdId)
{
	char *szTagName = pTag->szName;
	char *szAddress = pTag->szAddress;

	bool bModbusTCP = IsModbusTCP(pDevice);

	Drv_LogMessage(PK_LOGLEVEL_DEBUG, "收到控制命令：向设备(%s)的tag(%s)进行控制，地址:%s",
		pDevice->szName, szTagName, szAddress);

	DRVGROUP *pTagGroup = (DRVGROUP *)pTag->pData1;
	char *szBlockType = pTagGroup->szHWBlockName;
	int nBlockType = GetBlockTypeId(szBlockType);

	string strBlockType = szBlockType;
	vector<string> vecTemp = PKStringHelper::StriSplit(szBlockType, "#");
	if (vecTemp.size() >= 2)
	{
		strBlockType = vecTemp[1];
	}

	// AI/DI类型，不能写入
	if (PKStringHelper::StriCmp(strBlockType.c_str(), BLOCK_TYPE_DO) != 0 && PKStringHelper::StriCmp(strBlockType.c_str(), BLOCK_TYPE_AO) != 0)
	{
		Drv_LogMessage(PK_LOGLEVEL_ERROR, "device:%s, block:%s, tag:%s, value:%s, 控制时数据块类型为 %s, 不正确, 不是AO或DO！",
			pDevice->szName, pTagGroup->szAutoGroupName, pTag->szName, szStrValue, szBlockType);
		return EC_ICV_DRIVER_DATABLOCK_UNKNOWNTYPE;
	}

	char szRequestBuff[DEFAULT_REQUEST_MAXLEN];
	memset(szRequestBuff, 0, sizeof(szRequestBuff));
	int nTotalPackLen = 0;
	unsigned short nReqTransId = 0;
	int nReqFunctionCode = 0; // 写入功能码, 5 写单个线圈,6 写单个保持寄存器,15 写多个线圈,16 写多个保持寄存器
	long nRet = BuildWriteRequestPacket(pDevice, pTagGroup, pTag, szRequestBuff, sizeof(szRequestBuff), (char *)szStrValue, nTotalPackLen, nReqFunctionCode, nReqTransId);

	// 先判断是否要清除标志位
	if (GetClearRecvBufferFlag(pDevice))
		Drv_ClearRecvBuffer(pDevice);

	long lSentBytes = Drv_Send(pDevice, szRequestBuff, nTotalPackLen, 200);
	if (lSentBytes != nTotalPackLen)
	{
		Drv_LogMessage(PK_LOGLEVEL_ERROR, "向设备(%s)发送读写tag(%s)请求失败(发送%d个字节，实际发送%d个)，事务号：%d",
			pDevice->szName, szTagName, nTotalPackLen, lSentBytes, nReqTransId);
		UpdateGroupQuality(pDevice, pTagGroup, -201, "control, sentlen(%d)!=reqlen(%d)", lSentBytes, nTotalPackLen);
		return -201;
	}
	LogHex(false, PK_LOGLEVEL_INFO, pDevice, szRequestBuff, nTotalPackLen);

	Drv_LogMessage(PK_LOGLEVEL_NOTICE, "!!!控制命令：向设备(%s)发送写tag(%s)请求成功，功能码:%d，实际发送%d个, 事务号：%d",
		pDevice->szName, pTag->szName, nReqFunctionCode, lSentBytes, nReqTransId);

	// 从设备接收应答消息并解析
	nRet = RecvAndParseReadPacket(pDevice, pTagGroup, nReqFunctionCode, nReqTransId);
	if (nRet != 0)
	{
		Drv_LogMessage(PK_LOGLEVEL_ERROR, "从设备(%s)接收写tag(%s)请求失败，事务号：%d",
			pDevice->szName, pTag->szName, nReqTransId);
		UpdateGroupQuality(pDevice, pTagGroup, -202, "control, recvlen(%d)<=0", -1);
		return -202;
	}

	// 对于设备为AO，但上位配置为DO（取AO的某一个位）；如果上位连续对某个AO（2个字节），
	// 在一个读写周期内，两个不同的位连续控制，会出现后面会覆盖前面的控制值
	int nStartBits = pTag->nStartBit;// 相对于块内的起始地址位（如AI/DI)
	int nEndBits = pTag->nEndBit; // 相对于块内的结束地址位（如AI/DI)
	int nLenBits = nEndBits - nStartBits + 1; // 长度（位表示）

	if (PKStringHelper::StriCmp(szBlockType, BLOCK_TYPE_AO) == 0 && nLenBits == 1)
	{
		char szBinValue[PK_TAGDATA_MAXLEN] = { 0 };
		int nBinValueLen = 0;
		Drv_SetTagData_Text(pTag, szStrValue);

		vector<PKTAG *> vecTags;
		vecTags.push_back(pTag);
		Drv_UpdateTagsData(pDevice, vecTags.data(), vecTags.size());
		vecTags.clear();
	}
	return 0;
}


/**
*  解析连续Tag地址
// 格式支持包括：AO:200.5,AO等同于#200:5
// 支持的地址格式：按位类型包括：DI、DO或者首地址为0、1。按字类型包括：AI、AO或者首地址为3、4
*  @param  -[in] { char * } szAddressInDevice: 变量地址，按字类型：[AI|AO|DI|DO|0|1|2|3][.|#|:]20[.5],AO3,3001,3004,AI20.2,3003.3。按位类型：DI1,DO2,1001,0001。支持以:#分割块和地址
*  @param  -[in] { int }  nTagLenBits:  根据tag类型计算得到的tag值位数
*  @param  -[in] { char * } szBlockName: 数据区域名称
*  @param  -[in] { int }   nBlockNameLen : 数据区域名称的长度
*  @param  -[out] { int * }  pnStartBits: 相对于相对整个物理块（如AI、DO、D、DB1，而不是重组的某个AI内的某个Group）内的起始位、结束位（含结束位，如16则起始位0，结束位15），以位为单位
*  @param  -[out] { int * }  pnEndBits :
*  @retval { int } success : 0 , fail: others
*  @version     12/11/2008    Initial Version.
*/
int AnalyzeTagAddr_Continuous(char *szAddressInDevice, int nLenBits, char *szBlockName, int nBlockNameLen, int *pnStartBits, int *pnEndBits)
{
	*pnStartBits = 0;
	*pnEndBits = 0;

	// 复制地址，并去掉#和：号。该方法可能会被多线程调用，不能用静态变量
	char szAddress[PK_IOADDR_MAXLEN + 1] = { 0 };
	char *pDest = szAddress;
	char *pTmp = szAddressInDevice;
	while (*pTmp != '\0')
	{
		if (*pTmp == ':') // 仅仅复制非: 和非#??
			*pDest = '.';
		else
			*pDest = *pTmp;
		pTmp++;
		pDest++;
	}

	string strNewAddress = szAddress; // 这个地址:号改为了.号

	// 5#AO:1.1,取出5作为站号
	string strStationNo = "";
	vector<string> vecTemp = PKStringHelper::StriSplit(strNewAddress.c_str(), "#"); // 结果可能是:40004/AO:100.2-4/40001.2-4;2段
	if (vecTemp.size() >= 2)
	{
		strStationNo = vecTemp[0]; //  5#AO:1.1--> 5
		strNewAddress = vecTemp[1]; // 5#AO:1.1 --> AO:1.1
	}

	vector<string> vecAddr = PKStringHelper::StriSplit(strNewAddress.c_str(), "."); // 结果可能是:40004/AO:100.2-4/40001.2-4;2段
	if (vecAddr.size() <= 0)
	{
		Drv_LogMessage(PK_LOGLEVEL_ERROR, "parse tag addr:%s, 寄存器地址位配置, 地址为空", szAddressInDevice);
		return -2;
	}

	string strBlockName = vecAddr[0]; // 可能为:AO, 400001
	string strRegisterNo("");
	string strBitNo("");
	char chBlockFlag = strBlockName[0];
	if (isdigit(chBlockFlag)) // 40001,30002,10001,00005
	{
		if (strBlockName.length() <= 4) // 1...9999,一定是DO
		{
			strcpy(szBlockName, "DO");
			strRegisterNo = strBlockName;
			if (vecAddr.size() >= 2) // 寄存器下面的位, 40003.3-5
				strBitNo = vecAddr[1]; // 3-5
		}
		else // 5位或6位
		{
			if (chBlockFlag == '1' || chBlockFlag == '3' || chBlockFlag == '4')
			{
				if (chBlockFlag == '1')
					strcpy(szBlockName, "DI");
				else if (chBlockFlag == '4')
					strcpy(szBlockName, "AO");
				else if (chBlockFlag == '3')
					strcpy(szBlockName, "AI");
				strRegisterNo = strBlockName.substr(1); // 去掉40001的4，得到寄存器的编号
			}
			else // 非3/4/1，包括0，就认为是0开头
			{
				strcpy(szBlockName, "DO");
				strRegisterNo = strBlockName;
				//Drv_LogMessage(PK_LOGLEVEL_ERROR, "parse tag addr:%s, no block type name! must:AI/AO/DI/DO/0/1/2/3", szAddressInDevice);
			}
			if (vecAddr.size() >= 2) // 寄存器下面的位, 40003.3-5
				strBitNo = vecAddr[1]; // 3-5
		}
	}
	else // 字母，必须是AO,AI,DO,DI
	{
		strncpy(szBlockName, strBlockName.c_str(), sizeof(szBlockName)-1);
		if (vecAddr.size() <= 1)
		{
			Drv_LogMessage(PK_LOGLEVEL_ERROR, "parse tag addr:%s, no register number! ", szAddressInDevice);
			return -3;
		}
		strRegisterNo = vecAddr[1];
		if (vecAddr.size() >= 3) // 寄存器下面的位, 40003.3-5
			strBitNo = vecAddr[2]; // 3-5
	}

	bool bBlockByBit = false;
	bool bBlockByWord = false;
	if (PKStringHelper::StriCmp("AI", szBlockName) == 0 || PKStringHelper::StriCmp("AO", szBlockName) == 0)
	{
		bBlockByWord = true;
	}
	else if (PKStringHelper::StriCmp("DI", szBlockName) == 0 || PKStringHelper::StriCmp("DO", szBlockName) == 0)
	{
		bBlockByBit = true;
	}
	else
	{
		Drv_LogMessage(PK_LOGLEVEL_ERROR, "parse tag addr:%s, 数据块类型:%s不支持, 异常", szAddressInDevice, szBlockName);
		return -2;
	}

	// 计算起始地址.Modbus的地址都是从1开始的，因此要减去1
	int	 nStartAddr = ::atoi(strRegisterNo.c_str());
	if (bBlockByBit) // bit,DI, DO
		*pnStartBits = nStartAddr - 1;
	else // AI,AO
		*pnStartBits = (nStartAddr - 1) * 16;

	// 计算起始位，对于字地址有效
	// AO和AI按位取时，数据长度和位长度相等
	if (bBlockByWord && strBitNo.length() > 0) // 40001.4-6
	{
		vector<string> vecBit = PKStringHelper::StriSplit(strBitNo.c_str(), "-"); // 4-8, 或者 4
		string strBitStart = vecBit[0]; // 4
		string strBitEnd("");
		if (vecBit.size() >= 2) // 8
			strBitEnd = vecBit[1];

		int nStartBit = 0, nEndBit = 0;
		nStartBit = ::atoi(strBitStart.c_str()); // 40001.4/40001.4-8/AO:3.4/AO:3.4-8
		if (nStartBit >= 0)
			*pnStartBits += nStartBit;
		if (strBitEnd.length() > 0)
		{
			nEndBit = ::atoi(strBitEnd.c_str()); // 40001.4/40001.4-8/AO:3.4/AO:3.4-8
			*pnEndBits = *pnStartBits + (nEndBit - nStartBit); // 两头都的位包含,AO:3.4-8
		}
		else // AO:3.4
			*pnEndBits = *pnStartBits; // 两头都的位包含，因此不能减1

	}
	else // DI和DO，以及AO和AI不按位时，数据长度和tag数据长度相同
		*pnEndBits = *pnStartBits + nLenBits - 1;

	if (strStationNo.size() > 0) // 如果有站号，则重新命令块为 stationNo#BLockName
	{
		char szTmpBlockName[128] = { 0 };
		PKStringHelper::Snprintf(szTmpBlockName, nBlockNameLen, "%s#%s", strStationNo.c_str(), szBlockName);
		//@version 有bug,所以声明一个临时变量
		//PKStringHelper::Snprintf(szBlockName, nBlockNameLen, "%s#%s", strStationNo.c_str(), szBlockName);
		sprintf(szBlockName, "%s", szTmpBlockName);
	}

	return 0;
}