// Modbus �������豸����1Ϊ0��ʾModbusRTU������Ϊ1��ʾModbusTCP����
// ���ݿ����1��ʾվ�š�վ�ű�����ڵ���1
// ����2����д����Щָ�
// ����3��ÿ���������ֽ���
// ����Ҳ����һ����������ʾվ�ţ����һ���豸�����˶����ͬ���飬ÿ�����վ�ſ����ǲ�ͬ�ģ�
// DRVTAG��nData1����ʼ��ַλ�������AI��0��ַ����nData2�ǽ�����ַλ��nData3���ڸÿ��ڵ���ʼλ��
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

#define DEVPARAM_TRANSID							0	// �����
#define DEVPARAM_STATION_NO							1	// վ��
#define DEVPARAM_RWCMD								2	// ��д��ʲôָ��
#define DEVPARAM_MAXBYTES_ONEBLOCK					3	// ÿ��������ID
#define DEVPARAM_CLEARFLAG							4	// �Ƿ���Ҫ�����������
#define DEVPARAM_DATA_LEFTBUFFER_LEN				5	// �ϴη�������󣬻�ʣ�°�����Ļ���������

#define DEVPARAM_DATAPOINTER_RECVBUFF				0	// �����ϴη������İ��������
#define PK_TAGDATA_MAXLEN							4096
#define PACKAGE_LEN                                 1024  //��ӡ����󳤶�

void LogHex(bool bIsReceive, int nLogLevel, PKDEVICE *pDevice, char *szBuf, int nBufLen)
{
	char szLog[PACKAGE_LEN] = { 0 };
	unsigned int nLogBufLen = sizeof(szLog);
	PKStringHelper::HexDumpBuf(szBuf, nBufLen, szLog, sizeof(szLog), &nLogBufLen);
	if (bIsReceive)
		Drv_LogMessage(nLogLevel, "<<<�豸:%s, ��:%s������:%d", pDevice->szName, szLog, nBufLen);
	else
		Drv_LogMessage(nLogLevel, ">>>�豸:%s, ��:%s������:%d", pDevice->szName, szLog, nBufLen);
	//DWORD thid = GetCurrentThreadId();
	//Drv_LogMessage(PK_LOGLEVEL_NOTICE, "threadid=%ld", thid);  //xhzw-debug
}

/**
*  �Ƿ���Ҫ�ߵ��ֽ�ת��.- little_endian needs
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
	//@version 2019/07/30  �ڴ��ж����Ƿ���# �Դ�վ�ŵĽ���
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

//notice ModbusTCP ���Ԥ������ �����  MODBUSTYPE_TCP
bool IsModbusTCP(PKDEVICE *pDevice)
{
#ifdef MODBUSTYPE_TCP
	return true;
#else
	return false;
#endif
}

// ModbusRTUʱ���豸ID����Ϊ0��������1
// ��ȡtag��ĵ��������������û����ȡ�豸�ĵ����������������û����ȱʡΪ1
// ����ʱ������վ�ţ����Ƕ�дʱδ����վ�ţ�������������������������������������
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

//������ձ�־λ
void SetClearRecvBufferFlag(PKDEVICE *pDevice)
{
	pDevice->nUserData[DEVPARAM_CLEARFLAG] = 1;
}

//��ȡ��ձ�־λ
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
		if (pTagGroup->nFailCountRecent > 10)	// ���ʧ�ܴ���
		{
			char szTip[1024] = { 0 };
			sprintf(szTip, "read failcount:%d", pTagGroup->nFailCountRecent);
			UpdateGroupQuality(pDevice, pTagGroup, TAG_QUALITY_COMMUNICATE_FAILURE, szTip);
			pTagGroup->nFailCountRecent = 0; // �������̫����ѭ��
		}
		else
			pTagGroup->nFailCountRecent += 1;
	}
}

/*
��ʼ������
*/
PKDRIVER_EXPORTS long InitDriver(PKDRIVER *pDriver)
{
	Drv_LogMessage(PK_LOGLEVEL_NOTICE, "InitDriver(driver:%s)", pDriver->szName);
	Drv_LogMessage(PK_LOGLEVEL_NOTICE, "�豸param1:վ��,ȱʡΪ1. ����2:0��ʾ����ָ���д,1��ʾ��Ĵ�����д,ȱʡΪ0.  ����3:ÿ��������ֽ���,ȱʡ������.  ����4:δ����");
	return 0;
}
/*
��ʼ���豸
*/
PKDRIVER_EXPORTS long InitDevice(PKDEVICE *pDevice)
{
	Drv_LogMessage(PK_LOGLEVEL_NOTICE, "InitDevice(device:%s)", pDevice->szName);
	pDevice->nUserData[DEVPARAM_TRANSID] = 0;				// �����
	pDevice->pUserData[DEVPARAM_DATAPOINTER_RECVBUFF] = new char[DEFAULT_RESPONSE_MAXLEN]; // �����ϴ�ʣ��İ�����Ļ�����
	pDevice->nUserData[DEVPARAM_DATA_LEFTBUFFER_LEN] = 0; // �ϴ�ʣ��İ�����ĳ���

	// ����1��վ��
	pDevice->nUserData[DEVPARAM_STATION_NO] = 1;			// վ�ţ�ȱʡΪ1
	if (pDevice->szParam1 != NULL && atoi(pDevice->szParam1) != 0)
		pDevice->nUserData[DEVPARAM_STATION_NO] = atoi(pDevice->szParam1);

	// ����2������ʲôָ��
	pDevice->nUserData[DEVPARAM_RWCMD] = 0;				// ��д��ʲôָ��,0��ʾ������ָ�1��ʾ��Ĵ���ָ��
	if (pDevice->szParam2 != NULL && atoi(pDevice->szParam2) != 0)
		pDevice->nUserData[DEVPARAM_RWCMD] = atoi(pDevice->szParam2);

	// ����3��ÿ��������ֽ���  modbusÿ���豸��������ͬ��Ӧ������Ϊ�����������ȽϺ���
	if (strlen(pDevice->szParam3) > 0)
	{
		int nCfgMaxBytes = ::atoi(pDevice->szParam3);
		if (nCfgMaxBytes > 0)
			pDevice->nUserData[DEVPARAM_MAXBYTES_ONEBLOCK] = nCfgMaxBytes;
	}

	// ��ȡ�����е�tag�㡣��Ҫ��tag��洢����ƫ�ƣ�λ�������ȣ�λ�����������tag������б��Ա���㣩

	// ��������鴦�������е�tag��������BLOCK
	GroupVector vecTagGroup;
	GROUP_OPTION groupOption;
	groupOption.nIsGroupTag = 1;
	groupOption.nMaxBytesOneGroup = pDevice->nUserData[DEVPARAM_MAXBYTES_ONEBLOCK]; // modbusÿ���豸��������ͬ��Ӧ������Ϊ�����������ȽϺ���
	if (groupOption.nMaxBytesOneGroup <= 0) //���û�����룬��ȡȱʡ230���ֽ�
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

		//�����ж�һ���Ƿ���վ��  
		string  strHWBlockName = pTagGrp->szHWBlockName;
		vector<string> vecTemp = PKStringHelper::StriSplit(strHWBlockName, "#");
		if (vecTemp.size() >= 2)
		{
			strHWBlockName = vecTemp[1];
		}

		if (PKStringHelper::StriCmp(strHWBlockName.c_str(), BLOCK_TYPE_AO) == 0 || PKStringHelper::StriCmp(strHWBlockName.c_str(), BLOCK_TYPE_AI) == 0)
			nRegisterLenBits = 16;
		CalcGroupRegisterInfo(pTagGrp, nRegisterLenBits); // AI,AO��������飬DI��DO����λ���

		//��ȡϵͳʱ��
		const time_t t = time(NULL);
		struct tm* systemTime = localtime(&t);
		int intervalSec = (60 - systemTime->tm_sec) * 1000;


		char szBaseTime[32] = { 0 };
		unsigned int nMSec = 0;
		int nSec = PKTimeHelper::GetHighResTime(&nMSec);
		PKTimeHelper::HighResTime2String(szBaseTime, sizeof(szBaseTime), nSec, nMSec);

		string strTime = szBaseTime;
		int curSec = stoi(strTime.substr(17, 2).c_str()); //��ȡ��ǰ��

		Drv_LogMessage(PK_LOGLEVEL_NOTICE, "!!!!!!!!!   %d ---------  %d   !!!!!!!!!!!!!!!!!!", curSec, systemTime->tm_sec);
		//PKTIMER timerInfo;
		//// ��ʱ���ڣ���λ���� TODO
		//timerInfo.nPeriodMS = pTagGrp->nPollRate;

		//// ������λ, ��ʱ������ʼʱ�䣬���Ժ�������������������  TODO
		//timerInfo.nPhaseMS = (60 - curSec) * 1000;
		//

		PKTIMER timerInfo;
		//timerInfo.nPeriodMS = pTagGrp->nPollRate;
		timerInfo.nPeriodMS = 60000;
		//�޸���λ��������
		timerInfo.nPhaseMS = intervalSec;


		timerInfo.pUserData[0] = pTagGrp;
		void *pTimerHandle = Drv_CreateTimer(pDevice, &timerInfo); // �趨��ʱ��
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
����ʼ���豸
*/
PKDRIVER_EXPORTS long UnInitDevice(PKDEVICE *pDevice)
{
	Drv_LogMessage(PK_LOGLEVEL_NOTICE, "UnInitDevice(device:%s)", pDevice->szName);
	return 0;
}

/*
����ʼ������
*/
PKDRIVER_EXPORTS long UnInitDriver(PKDRIVER *pDriver)
{
	Drv_LogMessage(PK_LOGLEVEL_NOTICE, "UnInitDriver(driver:%s)", pDriver->szName);
	return 0;
}


/*
ModbusTCP��У���룬ͷ��6�����ȣ���������pdu��������Ϊ6���ֽڣ���Ӧ���������Ĵ������ȶ�����
�ӵ�ַ1��ʼ��ȡ10��AO�Ĵ��������󣬹̶�12���ֽڣ���ɫΪ����ͷ������ɫΪ���ݸ���pdu��
03 6D 00 00 00 06 01 03 00 00 00 0A
���ͣ�
03 6D�����ֽ������������ļ�����Ϣ������ţ����ӻ������豸��ֻ�轫�������ֽڵ�����copy�Ժ��ٷŵ�response�ı�����Ӧλ�ã�ͷ��
00 00 �����ֽڱ�ʾtcp / ip��Э����modbusЭ�飬ͷ��
00 06��ʾ���ֽ��Ժ�ĳ��ȡ����̶�Ϊ6���ֽڣ�ͷ��
01 վ�ţ�ȱʡΪ1��ͷ��
03 �����룬��ȡAOΪ03����ȡAIΪ04����ȡDIΪ02����ȡDOΪ01��ͷ��
00 ��ʼ��ַ���ֽڣ�PDU��������
00 ��ʼ��ַ���ֽڣ�PDU��������
00 ��ȡ�Ĵ������������ֽڣ�PDU��������
0A ��ȡ�Ĵ������������ֽڣ�PDU��������

�豸���صĴӵ�ַ1��ʼ��ȡ10��AO�Ĵ����������Ӧ��29���ֽڣ�����ɫΪͷ������ɫΪpdu���ݸ��ɡ�
03 6D 00 00 00 17 01 03 14 00 00 00 01 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
03 6D������Ϣ������ţ������������ͬ
00 00 tcp / ipΪmodbusЭ��
00 17 ��ʾ���ֽں�ĳ���
01 �豸��ַ
03 ������
14 ���ֽں���ֽ�����20����10���Ĵ�����20���ֽ�
*/
/**
*
* ����һ�����������ݵ�modbus��, �������0��˵���������ɹ�
*
*  @param - [in] { PKDEVICE * } pDevice: ��������
*  @param - [in] { DRVGROUP * } pTagGroup: tag��
*  @param - [in] { char * } szRequestBuff: ���Ļ���
*  @param - [in] { int }  nPackBufLen: ���Ļ��泤��
*  @param - [out] { int &} nTotalPackLen:  ����һ�������� �����ֽڳ���
*  @param - [out] { unsigned short & } uTransID:  modbustcp �����
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

	// ��ʼ��ַ��ModbusTCP����
	if (bModbusTCP)
	{
		uTransID = (unsigned short) ++pDevice->nUserData[DEVPARAM_TRANSID];

		// �����Ϊ2���ֽ�
		memcpy(pTransmit, &uTransID, 2);
		pTransmit += 2;

		// Э���ʶ�����ȸ��ֽڣ�����ֽڣ�����0
		memset(pTransmit, 0, 2);
		pTransmit += 2;

		// �����ֶΣ������ֽڵ������������ֽ�
		*pTransmit = 0;
		pTransmit++;
		//  �����ֶΣ������ֽڵ������������ֽ�
		*pTransmit = 0x06; // ��������ͷ���ĳ��ȣ��̶�Ϊ6���ֽ�
		pTransmit++;
	}

	// ���濪ʼʱModbus���е�
	unsigned char nStationNo = GetStationID(pDevice, pTagGroup);
	// վ��
	memcpy(pTransmit, &nStationNo, 1);
	pTransmit++;

	// ������Modbus����ͷ����5���ֽ�
	// ������
	unsigned char nFuncCode = GetReadFuncCodeNo(pTagGroup);
	memcpy(pTransmit, &nFuncCode, 1);// �������������ֵ��ͬ;
	pTransmit++;

	//nStartAddress += 1; // ��ʼ��ַ��0��ʼ.AO:8��nBeginRegisterΪ7
	// ��ʼ��ַ���ֽ�
	*pTransmit = (pTagGroup->nBeginRegister) >> BITS_PER_BYTE; // ��ַ���֣��Ĵ���
	pTransmit++;
	// ��ʼ��ַ���ֽ�
	*pTransmit = (pTagGroup->nBeginRegister) & 0xFF;
	pTransmit++;

	// ��ȡbit/�Ĵ������������ֽ�
	*pTransmit = pTagGroup->nRegisterNum >> BITS_PER_BYTE;
	pTransmit++;
	// ��ȡbit/�Ĵ������������ֽ�
	*pTransmit = pTagGroup->nRegisterNum & 0xFF;
	pTransmit++;

	int nDataLen = pTransmit - szRequestBuff;
	if (!bModbusTCP) // ModbusRTU
	{
		// ����CRC
		unsigned short nCRCValue = CRC16((unsigned char *)szRequestBuff, nDataLen);
		memcpy(pTransmit, &nCRCValue, sizeof(unsigned short));
		pTransmit += 2;
	}
	nTotalPackLen = pTransmit - szRequestBuff;
	return 0;
}

/**
*
* ����һ�����Ƶ�modbus��; �������0��˵���������ɹ�;
*
*  @param - [in] { PKDEVICE * } pDevice: ��������
*  @param - [in] { DRVGROUP * } pTagGroup: tag��
*  @param  -[in] { PKTAG * } pTag :ĳ��Tag����
*  @param - [in] { char * } szRequestBuff: ���Ļ���
*  @param - [in] { int }  nPackBufLen: ���Ļ��泤��
*  @param - [out] { int &} nTotalPackLen:  ����һ�������� �����ֽڳ���
*  @param - [out] { unsigned short & } uTransID:  modbustcp �����
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

	int nStartBits = pTag->nStartBit;// ����ڿ��ڵ���ʼ��ַλ����AI/DI)
	int nEndBits = pTag->nEndBit; // ����ڿ��ڵĽ�����ַλ����AI/DI)
	int nLenBits = nEndBits - nStartBits + 1; // ���ȣ�λ��ʾ��

	// ��ʼ��ַ��Ҫ���Ƶĵ�ַ
	long lStartAddress = nStartBits;	// ��0��ʼ�ĵ�ַ// Ҫ���Ƶĵ�ַ����,���ԼĴ�������Ϊ��λ��
	int  nRegisterNum = nLenBits;
	int  nByteCountToWrite = (int)ceil((nLenBits) / 8.0f);	// д�����ݵ��ֽ���,Э����Ҫ��д�������ֽ���Ϊ��λ��

	if (nBlockType == BLOCK_TYPE_ID_AO){ // AO
		lStartAddress = (int)ceil(nStartBits / 16.0f); // AO��2���ֽ�Ϊ��λ
		nRegisterNum = (int)ceil(nRegisterNum / 16.0f); // AO��2���ֽ�Ϊ��λ
	}
	else // DO
	{
		lStartAddress = nStartBits;	// ��0��ʼ�ĵ�ַ// Ҫ���Ƶĵ�ַ����,���ԼĴ�������Ϊ��λ��
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
		// �������
		unsigned short uTransID = (unsigned short) ++pDevice->nUserData[DEVPARAM_TRANSID];

		// ����Э����֯д��Ϣ
		// ������ţ����ֽ�
		*pTransmit = uTransID >> BITS_PER_BYTE;
		pTransmit++;

		// ������ţ����ֽ�
		*pTransmit = uTransID & 0xFF;
		pTransmit++;

		// Э���ʶ�����ȸ��ֽڣ�����ֽ�
		*pTransmit = 0;
		pTransmit++;
		*pTransmit = 0;
		pTransmit++;

		// �����ֶΣ����ֽ�
		*pTransmit = (7 + nByteCountToWrite) >> BITS_PER_BYTE;
		pTransmit++;

		// �����ֶΣ����ֽ�
		*pTransmit = (nFunctionCode == FUNCCODE_WRITE_SGLAO || nFunctionCode == FUNCCODE_WRITE_SGLDO) ? MB_WRITEPDU_LENGTH : ((7 + nByteCountToWrite) & 0xFF);   // �����ֽڵ�����
		pTransmit++;
	} // bModbusTCP

	// վ��
	*pTransmit = GetStationID(pDevice, pTagGroup);
	pTransmit++;

	// ������
	*pTransmit = nFunctionCode;
	pTransmit++;

	// ��ʼ��ַ���ֽ�
	*pTransmit = lStartAddress >> BITS_PER_BYTE;
	pTransmit++;

	// ��ʼ��ַ���ֽ�
	*pTransmit = lStartAddress & 0xFF;
	pTransmit++;

	// ������Ϊ06ʱ����Ҫ�⼸���ֶΡ�д�����Ȧ���߼Ĵ�������
	if (nFunctionCode != FUNCCODE_WRITE_SGLAO && nFunctionCode != FUNCCODE_WRITE_SGLDO)
	{
		// д����Ȧ��/�Ĵ������������ֽ�
		*pTransmit = nRegisterNum >> BITS_PER_BYTE;
		pTransmit++;

		// д����Ȧ��/�Ĵ������������ֽ�
		*pTransmit = nRegisterNum & 0xFF;
		pTransmit++;

		// д�����ݵ��ֽ���
		*pTransmit = nByteCountToWrite & 0xFF;
		pTransmit++;
	}

	//////////////////////////////////////
	if (nBlockType == BLOCK_TYPE_ID_DO)
	{
		// һ���Ĵ�����ֻ��1�����أ�Ϊ1ʱ������bitȫ����Ϊ1
		if (nRegisterNum == 1 && nFunctionCode != FUNCCODE_WRITE_MULTIDO) // ����ֵ��0��1
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
			// edit by xingxing,ǿ��д�����Ȧʱ��һ����д8��λ���������������Ҫ���Ƿ�8������������� 
			for (int i = 0; i < nByteCountToWrite; i++)
			{
				*pTransmit = szBinValue[i];
				pTransmit++;
			}
		}
	}
	else // BLOCK_TYPE_AO
	{
		// ��д�����Ĵ���
		// ���ܸô�����Ҫ���ƵĻ�������ͷ����β�����в��������Ĵ������ȣ���4���ֽڣ�λ��Ҫ���жϳ�����д��
		if (nLenBits == 1 || (nLenBits % 8 != 0)) // ����ֵ��0��1
		{
			// ��д������
			//int lValueStatus = 0;
			//unsigned short nData = 0;
			//PKTAGDATA tagData;
			//strncpy(tagData.szName, pTag->szName,sizeof(tagData.szName) - 1);
			//int lRet = Drv_GetTagData(pDevice,&tagData); // ��ȡ���ϴε�����
			//if(lRet != PK_SUCCESS)
			//	return lRet;

			//if(lValueStatus != TAG_QUALITY_GOOD)
			//	return -1;

			//memcpy(&nData, &tagData, 2); // AO or AI
			//// ����ָ��bit����ֵ
			//int nBitNo = nStartBits % 16; // AO:W5.13, һ����ĳ��Word������
			//nData = nData & ~(1 << nBitNo);
			//nData = nData | (szBinValue[0] << nBitNo);
			//
			//// ��д������
			//*pTransmit = nData >> BITS_PER_BYTE;
			//pTransmit++;
			//*pTransmit = (nData & 0xFF);
			//pTransmit++;
		}
		else // TAG_DATATYPE_ANALOG��TAG_DATATYPE_BLOB��TAG_DATATYPE_TEXT
		{
			// ��д������
			if (NeedSwap(pTagGroup->szHWBlockName))
				SwapByteOrder(szBinValue, nBinValueLen, 2);

			// д���ֽ����պõ��ڼĴ����ֽ�ʱ
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
				// д���ֽ��������ڼĴ����ֽ���ʱ
				// ��һ���֣��Ѿ�swap����ֵ���ɸߵ���
				for (i = 0; i < (int)(nRegisterNum - 1) * 2; i++)
				{
					*pTransmit = szBinValue[i];
					pTransmit++;
				}

				// �ڶ����֣����һ���Ĵ����ĸ��ֽ�
				int nToAdd = nByteCountToWrite - nBinValueLen;
				for (int j = 0; j < nToAdd; j++)
				{
					*pTransmit = 0;
					pTransmit++;
				}

				// �������֣����һ���Ĵ����ĵ��ֽڣ��ɸߵ���
				for (; i < nBinValueLen; i++)
				{
					*pTransmit = szBinValue[i];
					pTransmit++;
				}
			}
		}
	}

	// ���ɵ�д��Ϣ����
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
*  �õ���ȡָ����������ĳ��ȣ��ֽ�Ϊ��λ��
*
*  @param - [in] { PKDEVICE * } pDevice: ��������
*  @param - [in] { DRVGROUP * } pTagGroup: tag��
*  @retval  {int}  ���������ֽڳ���
*  @version     12 / 11 / 2008    Initial Version.
*/
int GetReadTagGroupDataLen(PKDEVICE *pDevice, DRVGROUP *pTagGroup)
{
	int nDataLenShould = 0;
	int nBlockType = GetBlockTypeId(pTagGroup->szHWBlockName);
	if (nBlockType == BLOCK_TYPE_ID_DO || nBlockType == BLOCK_TYPE_ID_DI) // DI��DO
		nDataLenShould = ceil(pTagGroup->nRegisterNum / 8.0f);
	else // AI��AO
		nDataLenShould = pTagGroup->nRegisterNum * 2;
	return nDataLenShould;
}

/**
// �Ƿ���һ����Ч�İ�, ������򷵻ذ����ܳ���
// modbustcp:
// ��ȡ����:3d 00(�����)	 00 00(Э��汾)	 00 06(�������ݳ���)	 01(�ӻ���ַ)	 01(���ܺ�)	 00 00(���ݵ�ַ)	 00 64(��ȡ���ݸ���)
// ��ȡӦ��:3d 00(�����)	 00 00(Э��汾)	 00 10(�������ݳ���)	 01(�ӻ���ַ)	 01(���ܺ�)	 0d(��ȡ�����ֽ����� 0c 4c 2a a9 14 22 65 0c 07 00 00 00 01
// ����Ӧ��:1a 00(�����)    00 00(Э��汾)     00 06(���͵����ݳ���)   01(�ӻ���ַ)	 04(���ܺ�)  42 6e 00 38  aa 71 18 4c 75 00 07 00 00 21 e9 60 84 02 0b 00 00 00
// modbusrtu:
// ��ȡ����0x01(�ӻ���ַ)	03(���ܺ�)	00 01(���ݵ�ַ)		00 01(��ȡ���ݸ���)	D5 CA(CRCУ��)
// ��ȡӦ��0x01(�ӻ���ַ)	03(���ܺ�)	02(�����ֽڸ���)	00 17(�����ֽ�����)	F8 4A(CRCУ��)
*  @param - [in] { PKDEVICE * } pDevice: ��������
*  @param - [in] { DRVGROUP * } pTagGroup: tag��
*  @param - [in] { char * } pBuff: ��������
*  @param - [in] { int} nBufLen: ���ĳ���
*  @param - [out] { int } nTotalPackLen: ���ذ����ܳ���
*  @param - [in] { int } nReqStationNo: վ��-�ӻ���ַ
*  @param - [in] { int } nReqFuncCode:  ���ܺ�
*  @param - [in] { int } nReqTransId:  modbustcp�����
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
		if (nTransIdInResponse != nReqTransId) // ����ű�����ȣ�����
		{
			return false;
		}

		unsigned short nProtocolNo = *(unsigned short *)(pBuff + 2); // Э���	// ����Э1d��š���Զ��0
		if (nProtocolNo != 0)
			return false;

		nStationNo = *(unsigned char*)(pBuff + 6);
		nFuncCode = *(unsigned char*)(pBuff + 7);
		// վ�ź͹�����ļ��
		if (nStationNo != nReqStationNo || nFuncCode != nReqFuncCode)
			return false;
		unsigned short nBodyLenBytesSend = nDataLenBytesSend + 3; // ���ݳ���+վ��1+������1+���ݳ���1
		unsigned short nBodyLenBytesResponseHigh = *(unsigned char*)(pBuff + 4);
		unsigned short nBodyLenBytesResponseLow = *(unsigned char*)(pBuff + 5);
		unsigned short nBodyLenBytesResponse = (nBodyLenBytesResponseHigh << BITS_PER_BYTE) + nBodyLenBytesResponseLow; // �������ݵĳ���+վ��+���ܺ�+���ݳ��ȣ�����ģ���������Ϊ0x06
		unsigned char nDataLenBytesResponse = *(unsigned char*)(pBuff + 8); // �������ݵĳ���+վ��+���ܺ�+���ݳ��ȣ�����ģ���������Ϊ0
		if (nBodyLenBytesResponse != nBodyLenBytesSend && nBodyLenBytesResponse != 0x06)
			return false;
		if (nDataLenBytesResponse != nDataLenBytesSend && nBodyLenBytesResponse != 0x00) // �����ĳ���Ϊ0
			return false;
		// int uPackBodyLen = (((unsigned char)*pBuff) << BITS_PER_BYTE) + (unsigned char)*(pBuff + 1);
		nTotalPackLen = nBodyLenBytesSend + 6; // ���泤�ȣ��ֽڣ�+��ͷ6���ֽڣ�3d 00(�����)	 00 00(Э��汾)	 00 10(�������ݳ���)

		if (nBufLen < nTotalPackLen)
			return false;
	}
	else
	{
		nStationNo = *(unsigned char*)(pBuff + 0);
		nFuncCode = *(unsigned char*)(pBuff + 1);
		unsigned char nDataLenBytesResponse = *(unsigned char*)(pBuff + 2);
		nTotalPackLen = nDataLenBytesResponse + 5; // ��ͬͷ����CRC��0x01(�ӻ���ַ)	03(���ܺ�)	02(�����ֽڸ���)	00 17(�����ֽ�����)	F8 4A(CRCУ��)
		// վ�ź͹�����ļ��
		if (nStationNo != nReqStationNo || nFuncCode != nReqFuncCode)
			return false;
		//�����ȵļ��
		if (nBufLen < nTotalPackLen)
			return false;

		int nCRCInResponse = *(unsigned short*)(pBuffBegin + nTotalPackLen - 2); // ���е�CRCֵ

		// ����CRC
		unsigned short nCRCValue = CRC16((unsigned char *)pBuffBegin, nTotalPackLen - 2);
		if (nCRCValue != nCRCInResponse)
			return false;
	}

	return true;
}

/*
*  �ҵ���Ч��������������Ӧ��
*  @param - [in] { PKDEVICE * } pDevice: ��������
*  @param - [in] { DRVGROUP * } pTagGroup: tag��
*  @param - [in] { bool}  bModbusTCP: �Ƿ���ModbusTCPЭ��
*  @param - [in] { char * } pBuff: ��������
*  @param - [in] { int} lCurRecvLen: ��ǰ���ĳ���
*  @param - [in] { int } nReqStationNo: վ��-�ӻ���ַ
*  @param - [in] { int } nReqFuncCode:  ���ܺ�
*  @param - [in] { int } nReqTransId:  modbustcp�����
*  @retval { bool }  success : true , fail: false
*  @version     12 / 11 / 2008    Initial Version.
*/
bool FindValidResponsePackage(PKDEVICE *pDevice, DRVGROUP *pTagGroup, bool bModbusTCP, char *pBuff, int lCurRecvLen, int nReqStationNo, int nReqFuncCode, int nReqTransId)
{
	// ͷ��+��վ��+������
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
		// �ٶ�pPacksBegin��һ�����İ�ͷ
		int nTotalPackLen = 0;
		bool bValidPack = IsValidPackage(pDevice, pTagGroup, pPacksBegin, lCurRecvLen, nTotalPackLen, nReqStationNo, nReqFuncCode, nReqTransId);
		if (!bValidPack)
		{
			pPacksBegin++; // δ�ҵ�����ô����һ���Ϸ��İ�
			continue;
		}

		bFoundPack = true;
		break;
	}

	return bFoundPack;
}

/**
// ����һ�����������ݰ�
// modbustcp:
// ��ȡ����:3d 00(�����)	 00 00(Э��汾)	 00 06(�������ݳ���)	 01(�ӻ���ַ)	 01(���ܺ�)	 00 00(���ݵ�ַ)	 00 64(��ȡ���ݸ���)
// ��ȡӦ��:3d 00(�����)	 00 00(Э��汾)	 00 10(�������ݳ���)	 01(�ӻ���ַ)	 01(���ܺ�)	 0d(��ȡ�����ֽ����� 0c 4c 2a a9 14 22 65 0c 07 00 00 00 01
// modbusrtu:
// ��ȡ����0x01(�ӻ���ַ)	 03(���ܺ�)			 00 01(�Ĵ�����ַ)		 00 01(��ȡ�Ĵ�������)	D5 CA(CRCУ��)
// ��ȡӦ��0x01(�ӻ���ַ)	 03(���ܺ�)			 02(�����ֽڸ���=����ļĴ�������*2)		 00 17(�����ֽ�����)	F8 4A(CRCУ��)
*  @param  -[in] { PKDEVICE * } pDevice : ��������
*  @param  -[in] { DRVGROUP * } pTagGroup : tag��
*  @param  -[in] { char * } pPackBuffer : ��������
*  @param  -[in] { int } nTotalPackLen: ���ĳ���
*  @retval { int }  success : 0 , fail: others
*  @version     12/11/2008    Initial Version.
*/
int ParseOnePackage(PKDEVICE *pDevice, DRVGROUP *pTagGroup, char *pPackBuffer, int nTotalPackLen)
{
	int nStartBytes = pTagGroup->nStartAddrBit / 8; // ��0��ʼ�ĵ�N���ֽ�
	int nEndBytes = pTagGroup->nEndAddrBit / 8; // ��0��ʼ�ĵ�N���ֽ�
	int nLenBytes = nEndBytes - nStartBytes + 1;
	int nBlockSize = nLenBytes; //(int)ceil((pTagGroup->nEndBit - pTagGroup->nStartAddrBit + 1)/8.0f);
	// ѭ�������յ������п������ݰ�
	// 2��ʾվ�ţ�1���ֽڣ��͹����루1���ֽڣ���վ�ű�ʾ�ӻ���ַ
	// �κ������ͷ6���ֽڶ���һ�������壬��7�͵�8���ֽ�Ҳ��һ�������壨վ�ź͹����룩
	long nRecvBytes = nTotalPackLen;
	char *pCurBuf = (char *)pPackBuffer;
	char *pDataBuff = pPackBuffer;
	int nDataBufLen = 0;
	int nFuncCode = 0;

	bool bModbusTCP = IsModbusTCP(pDevice);
	if (bModbusTCP)
	{
		pDataBuff += 9; // �ӵ�9���ֽڿ�ʼ
		nDataBufLen = nTotalPackLen - 9;
		nFuncCode = *(unsigned char *)(pPackBuffer + 7);
	}
	else
	{
		pDataBuff += 3; // �ӵ�9���ֽڿ�ʼ
		nDataBufLen = nTotalPackLen - 5;
		nFuncCode = *(unsigned char *)(pPackBuffer + 1);
	}

	// 5/6/15/16 ��д�����Ӧ��, ��ȡ�������Ӧ�� 1/2/3/4�Ƕ�����
	// д����Ӧ����Ҫ��������
	if (nFuncCode == 5 || nFuncCode == 6 || nFuncCode == 15 || nFuncCode == 16)
	{
		Drv_LogMessage(PK_LOGLEVEL_NOTICE, "success to parse write control response, function code %d, device:%s, block:%s", nFuncCode, pDevice->szName, pTagGroup->szAutoGroupName);
		return TAG_QUALITY_GOOD; // ������ܻ��Զ����ÿ��Ƶ�״̬
	}

	char *szBlockType = pTagGroup->szHWBlockName; // AI/DI/AO/DO
	// �������ݿ������
	if (NeedSwap(szBlockType))
	{
		// ÿ���Ĵ����ĳ��ȣ�λ��
		int nElemBits = 1;

		//���ж����Ƿ��д�վ��
		string  strBlockType = szBlockType;
		vector<string> vecTemp = PKStringHelper::StriSplit(strBlockType, "#");
		if (vecTemp.size() >= 2)
		{
			strBlockType = vecTemp[1];
		}
		if (PKStringHelper::StriCmp(strBlockType.c_str(), BLOCK_TYPE_AO) == 0 || PKStringHelper::StriCmp(strBlockType.c_str(), BLOCK_TYPE_AI) == 0)
			nElemBits = 16;

		// pCurBufָ���6���ֽڣ�Ӧ��������վ�š�������ͳ��ȹ������ֽ�
		SwapByteOrder(pDataBuff, nDataBufLen, nElemBits / BITS_PER_BYTE);
	}

	// �������ݿ�����ݡ����ڶ�ʱ���ж�ȡ�����ݺ󣬵��øýӿڸ��������Ա�������ݿ��ȡ
	long lRet = UpdateGroupData(pDevice, pTagGroup, pDataBuff, nDataBufLen, TAG_QUALITY_GOOD);
	if (lRet == 0)
		Drv_LogMessage(PK_LOGLEVEL_INFO, "device:%s, block:%s, success to update %d tag, databytes:%d", pDevice->szName, pTagGroup->szAutoGroupName, pTagGroup->vecTags.size(), nDataBufLen);
	else
		Drv_LogMessage(PK_LOGLEVEL_ERROR, "device:%s, block:%s, fail to update %d tag, databytes:%d, retcode:%d", pDevice->szName, pTagGroup->szAutoGroupName, pTagGroup->vecTags.size(), nDataBufLen, lRet);
	return 0;
}

/**
*  ����ĳ������ţ�modbustcp�������ݰ���modbusrtu�Ļ���Ӧ�����Ӧ�ð��������롢��վ��modbustcp�����������
*  @param  -[in] { PKDEVICE * } pDevice :  ��������
*  @param  -[in] { DRVGROUP * } pTagGroup :tag��
*  @param  -[in] { int } nReqFuncCode : ������
*  @param  -[in] { unsigned short } nReqTransId : modbustcp�����������
*  @retval { long }  success : 0 , fail: others
*  @version     12/11/2008    Initial Version.
*/
long RecvAndParseReadPacket(PKDEVICE *pDevice, DRVGROUP *pTagGroup, int nReqFuncCode, unsigned short nReqTransId)
{
	Drv_LogMessage(PK_LOGLEVEL_DEBUG, "--->RecvAndParseReadPacket");
	bool bModbusTCP = IsModbusTCP(pDevice);
	int nReqStationNo = GetStationID(pDevice, pTagGroup); // ��ȡtag���վ��

	// ���豸����Ӧ����Ϣ
	long lRet = 0;
	long lCurRecvLen = pDevice->nUserData[DEVPARAM_DATA_LEFTBUFFER_LEN]; // �ϴ�ʣ��ĳ���
	char *pCurRecvBuff = (char *)pDevice->pUserData[DEVPARAM_DATAPOINTER_RECVBUFF];
	int nBlankBufLen = DEFAULT_RESPONSE_MAXLEN - lCurRecvLen;
	char szResponse[DEFAULT_RESPONSE_MAXLEN] = { 0 };

	Drv_LogMessage(PK_LOGLEVEL_DEBUG, "user data info, lCurRecvLen=%d,pCurRecvBuff=%x", lCurRecvLen, pCurRecvBuff);

	// ���������е����ݣ�ֱ��û�����ݿ����ˣ��ղ��������������ˣ������߳�ʱ��
	ACE_Time_Value tvBegin = ACE_OS::gettimeofday();
	while (true)
	{
		Drv_LogMessage(PK_LOGLEVEL_DEBUG, "recv loop...");
		ACE_Time_Value tvNow = ACE_OS::gettimeofday();
		ACE_Time_Value tvSpan = tvNow - tvBegin; // �Ѿ�����ʱ��

		int nTimeoutMS = 5000 - tvSpan.msec(); // ÿ�����ȴ�8��. PLC-5ͨ������ת֮��2�붼�ղ�������
		if (nTimeoutMS < 0 || nTimeoutMS > 100 * 1000) // ��ʱ�þ���û����������������, �����޸���ϵͳʱ��
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
		if (lRecvBytes <= 0) // �ղ���������, ���߻�����������
		{
			Drv_LogMessage(PK_LOGLEVEL_DEBUG, "no more data. stop loop");
			break;
		}
		LogHex(true, PK_LOGLEVEL_INFO, pDevice, pCurRecvBuff, lRecvBytes);
		// ��������1���ǲ�����
		lCurRecvLen += lRecvBytes; // �Ѿ����յ��ĳ���
		pCurRecvBuff += lRecvBytes; // ���ջ�����ָ��
		nBlankBufLen -= lRecvBytes; // ����Ŀɽ������ݵĻ���������
		char *pPacksBegin = (char *)pDevice->pUserData[DEVPARAM_DATAPOINTER_RECVBUFF];
		bool bFoundResponse = FindValidResponsePackage(pDevice, pTagGroup, bModbusTCP, pPacksBegin, lCurRecvLen, nReqStationNo, nReqFuncCode, nReqTransId);
		if (bFoundResponse) // �ҵ�����Ҫ����Ӧ��
		{
			Drv_LogMessage(PK_LOGLEVEL_DEBUG, "data found, stop loop");
			break;
		}
	}

	// �����ǳ�ʱ�ˣ��������յ��Ϸ��İ��ˣ�����Ҫ����
	// ͷ��+��վ��+������
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
		// �ٶ�pPacksBegin��һ�����İ�ͷ
		int nTotalPackLen = 0;
		bool bValidPack = IsValidPackage(pDevice, pTagGroup, pPacksBegin, lCurRecvLen, nTotalPackLen, nReqStationNo, nReqFuncCode, nReqTransId);
		if (!bValidPack)
		{
			pPacksBegin++;
			continue;
		}

		// pPacksBegin��lCurRecvLen��һ�������İ�
		ParseOnePackage(pDevice, pTagGroup, pPacksBegin, nTotalPackLen);
		pPacksBegin += nTotalPackLen;
		bFoundPack = true;
	}

	if (bFoundPack)
	{
		// ParseOnePackage�Ѿ���ӡ����־�ˣ����ﲻ��Ҫ�ظ���ӡ��Drv_LogMessage(PK_LOGLEVEL_INFO, "device:%s, now %d bytes with left, found response package[ReqTransId:0x%x]", pDevice->szName, lCurRecvLen, nReqTransId);
		CheckBlockStatus(pDevice, pTagGroup, PK_SUCCESS);
	}
	else
	{
		Drv_LogMessage(PK_LOGLEVEL_ERROR, "device:%s, now %d bytes with left, not found response package[ReqTransId:0x%x]", pDevice->szName, lCurRecvLen, nReqTransId);
		//UpdateGroupQuality(pDevice, pTagGroup, -100, "parsing,packlen:%d < headlen:%d,discard", lCurRecvLen, PROTOCOL_TCP_PACKAGE_HEADER_LEN);
		CheckBlockStatus(pDevice, pTagGroup, -1);
	}

	// ������û������ϵ�	
	char *pDevLeftBuff = (char *)pDevice->pUserData[DEVPARAM_DATAPOINTER_RECVBUFF];
	int nLeftLen = pPacksEnd - pPacksBegin;
	pDevice->nUserData[DEVPARAM_DATA_LEFTBUFFER_LEN] = nLeftLen; // �ϴ�ʣ��ĳ���
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
Client request��
03 6D�����ֽ������������ļ�����Ϣ���ӻ�ֻ�轫�������ֽڵ�����copy�Ժ��ٷŵ�response�ı�����Ӧλ�á�
00 00 �����ֽڱ�ʾtcp/ip��Э����modbusЭ��
00 06��ʾ���ֽ��Ժ�ĳ��ȡ�

PDU��
01 �豸��ַ
03 ������
00 00�Ĵ�����ʼ��ַ
00 0A �Ĵ����ĳ��� ��10��
000015-Rx:server response��03 6D 00 00 00 17   PDU�� 01 03 14 00 00 00 01 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
03 6D������Ϣ
00 00 tcp/ipΪmodbusЭ��
00 17 ��ʾ���ֽں�ĳ���
01 �豸��ַ
03 ������
14 ���ֽں���ֽ�����20��
*/
/**
*  ��ʱ�������Ժ���
*  @param  -[in] { PKDEVICE * } pDevice :  ��������
*  @param  -[in] {  PKTIMER * } timerInfo :
*  @retval { long }  success : 0 , fail: others
*  @version     12/11/2008    Initial Version.
*/
PKDRIVER_EXPORTS long OnTimer(PKDEVICE *pDevice, PKTIMER *timerInfo)
{
	Drv_LogMessage(PK_LOGLEVEL_DEBUG, "--->OnTimer");
	bool bModbusTCP = IsModbusTCP(pDevice);
	// ��֯��ȡ��Ϣ
	DRVGROUP *pTagGroup = (DRVGROUP *)timerInfo->pUserData[0];
	unsigned short uReqTransID = 0; // MODBUSTCP�������
	char szRequestBuff[DEFAULT_REQUEST_MAXLEN];
	memset(szRequestBuff, 0, sizeof(szRequestBuff));
	int nTotalPackLen = 0;
	int nRet = BuildReadRequestPacket(pDevice, pTagGroup, szRequestBuff, sizeof(szRequestBuff), nTotalPackLen, uReqTransID);
	if (nRet != 0)
		return -1;

	// ����ս��ջ�������������ǰ������������û�����ü����ջ����ڻ�������Ӱ�챾������
	if (GetClearRecvBufferFlag(pDevice))
		Drv_ClearRecvBuffer(pDevice);

	// ʱ��
	time_t tmRequest;
	time(&tmRequest);


	// ���ɵĶ���Ϣ���ȣ�Ӧ���ǹ̶���12���ֽ�
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

	// ������
	int nReqFuncCode = GetBlockTypeId(pTagGroup->szHWBlockName); // �������������ֵ��ͬ
	nRet = RecvAndParseReadPacket(pDevice, pTagGroup, nReqFuncCode, uReqTransID);
	Drv_LogMessage(PK_LOGLEVEL_DEBUG, "<---OnTimer");
	return nRet;
}

/**
*  ���п�������ʱ�ú���������
*  @param  -[in] { PKDEVICE * } pDevice :  ��������
*  @param  -[in] { PKTAG * } pTag :ĳ������
*  @param  -[in] { const char * } szStrValue:����ֵ����blob�ⶼ�Ѿ��Ѿ�ת��Ϊ�ַ�����blobת��Ϊbase64����
*  @param  -[in] { long } lCmdId :
*  @retval { long }   success : 0 , fail: others
*  @version     12/11/2008    Initial Version.
*/
PKDRIVER_EXPORTS long OnControl(PKDEVICE *pDevice, PKTAG *pTag, const char *szStrValue, long lCmdId)
{
	char *szTagName = pTag->szName;
	char *szAddress = pTag->szAddress;

	bool bModbusTCP = IsModbusTCP(pDevice);

	Drv_LogMessage(PK_LOGLEVEL_DEBUG, "�յ�����������豸(%s)��tag(%s)���п��ƣ���ַ:%s",
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

	// AI/DI���ͣ�����д��
	if (PKStringHelper::StriCmp(strBlockType.c_str(), BLOCK_TYPE_DO) != 0 && PKStringHelper::StriCmp(strBlockType.c_str(), BLOCK_TYPE_AO) != 0)
	{
		Drv_LogMessage(PK_LOGLEVEL_ERROR, "device:%s, block:%s, tag:%s, value:%s, ����ʱ���ݿ�����Ϊ %s, ����ȷ, ����AO��DO��",
			pDevice->szName, pTagGroup->szAutoGroupName, pTag->szName, szStrValue, szBlockType);
		return EC_ICV_DRIVER_DATABLOCK_UNKNOWNTYPE;
	}

	char szRequestBuff[DEFAULT_REQUEST_MAXLEN];
	memset(szRequestBuff, 0, sizeof(szRequestBuff));
	int nTotalPackLen = 0;
	unsigned short nReqTransId = 0;
	int nReqFunctionCode = 0; // д�빦����, 5 д������Ȧ,6 д�������ּĴ���,15 д�����Ȧ,16 д������ּĴ���
	long nRet = BuildWriteRequestPacket(pDevice, pTagGroup, pTag, szRequestBuff, sizeof(szRequestBuff), (char *)szStrValue, nTotalPackLen, nReqFunctionCode, nReqTransId);

	// ���ж��Ƿ�Ҫ�����־λ
	if (GetClearRecvBufferFlag(pDevice))
		Drv_ClearRecvBuffer(pDevice);

	long lSentBytes = Drv_Send(pDevice, szRequestBuff, nTotalPackLen, 200);
	if (lSentBytes != nTotalPackLen)
	{
		Drv_LogMessage(PK_LOGLEVEL_ERROR, "���豸(%s)���Ͷ�дtag(%s)����ʧ��(����%d���ֽڣ�ʵ�ʷ���%d��)������ţ�%d",
			pDevice->szName, szTagName, nTotalPackLen, lSentBytes, nReqTransId);
		UpdateGroupQuality(pDevice, pTagGroup, -201, "control, sentlen(%d)!=reqlen(%d)", lSentBytes, nTotalPackLen);
		return -201;
	}
	LogHex(false, PK_LOGLEVEL_INFO, pDevice, szRequestBuff, nTotalPackLen);

	Drv_LogMessage(PK_LOGLEVEL_NOTICE, "!!!����������豸(%s)����дtag(%s)����ɹ���������:%d��ʵ�ʷ���%d��, ����ţ�%d",
		pDevice->szName, pTag->szName, nReqFunctionCode, lSentBytes, nReqTransId);

	// ���豸����Ӧ����Ϣ������
	nRet = RecvAndParseReadPacket(pDevice, pTagGroup, nReqFunctionCode, nReqTransId);
	if (nRet != 0)
	{
		Drv_LogMessage(PK_LOGLEVEL_ERROR, "���豸(%s)����дtag(%s)����ʧ�ܣ�����ţ�%d",
			pDevice->szName, pTag->szName, nReqTransId);
		UpdateGroupQuality(pDevice, pTagGroup, -202, "control, recvlen(%d)<=0", -1);
		return -202;
	}

	// �����豸ΪAO������λ����ΪDO��ȡAO��ĳһ��λ���������λ������ĳ��AO��2���ֽڣ���
	// ��һ����д�����ڣ�������ͬ��λ�������ƣ�����ֺ���Ḳ��ǰ��Ŀ���ֵ
	int nStartBits = pTag->nStartBit;// ����ڿ��ڵ���ʼ��ַλ����AI/DI)
	int nEndBits = pTag->nEndBit; // ����ڿ��ڵĽ�����ַλ����AI/DI)
	int nLenBits = nEndBits - nStartBits + 1; // ���ȣ�λ��ʾ��

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
*  ��������Tag��ַ
// ��ʽ֧�ְ�����AO:200.5,AO��ͬ��#200:5
// ֧�ֵĵ�ַ��ʽ����λ���Ͱ�����DI��DO�����׵�ַΪ0��1���������Ͱ�����AI��AO�����׵�ַΪ3��4
*  @param  -[in] { char * } szAddressInDevice: ������ַ���������ͣ�[AI|AO|DI|DO|0|1|2|3][.|#|:]20[.5],AO3,3001,3004,AI20.2,3003.3����λ���ͣ�DI1,DO2,1001,0001��֧����:#�ָ��͵�ַ
*  @param  -[in] { int }  nTagLenBits:  ����tag���ͼ���õ���tagֵλ��
*  @param  -[in] { char * } szBlockName: ������������
*  @param  -[in] { int }   nBlockNameLen : �����������Ƶĳ���
*  @param  -[out] { int * }  pnStartBits: ����������������飨��AI��DO��D��DB1�������������ĳ��AI�ڵ�ĳ��Group���ڵ���ʼλ������λ��������λ����16����ʼλ0������λ15������λΪ��λ
*  @param  -[out] { int * }  pnEndBits :
*  @retval { int } success : 0 , fail: others
*  @version     12/11/2008    Initial Version.
*/
int AnalyzeTagAddr_Continuous(char *szAddressInDevice, int nLenBits, char *szBlockName, int nBlockNameLen, int *pnStartBits, int *pnEndBits)
{
	*pnStartBits = 0;
	*pnEndBits = 0;

	// ���Ƶ�ַ����ȥ��#�ͣ��š��÷������ܻᱻ���̵߳��ã������þ�̬����
	char szAddress[PK_IOADDR_MAXLEN + 1] = { 0 };
	char *pDest = szAddress;
	char *pTmp = szAddressInDevice;
	while (*pTmp != '\0')
	{
		if (*pTmp == ':') // �������Ʒ�: �ͷ�#??
			*pDest = '.';
		else
			*pDest = *pTmp;
		pTmp++;
		pDest++;
	}

	string strNewAddress = szAddress; // �����ַ:�Ÿ�Ϊ��.��

	// 5#AO:1.1,ȡ��5��Ϊվ��
	string strStationNo = "";
	vector<string> vecTemp = PKStringHelper::StriSplit(strNewAddress.c_str(), "#"); // ���������:40004/AO:100.2-4/40001.2-4;2��
	if (vecTemp.size() >= 2)
	{
		strStationNo = vecTemp[0]; //  5#AO:1.1--> 5
		strNewAddress = vecTemp[1]; // 5#AO:1.1 --> AO:1.1
	}

	vector<string> vecAddr = PKStringHelper::StriSplit(strNewAddress.c_str(), "."); // ���������:40004/AO:100.2-4/40001.2-4;2��
	if (vecAddr.size() <= 0)
	{
		Drv_LogMessage(PK_LOGLEVEL_ERROR, "parse tag addr:%s, �Ĵ�����ַλ����, ��ַΪ��", szAddressInDevice);
		return -2;
	}

	string strBlockName = vecAddr[0]; // ����Ϊ:AO, 400001
	string strRegisterNo("");
	string strBitNo("");
	char chBlockFlag = strBlockName[0];
	if (isdigit(chBlockFlag)) // 40001,30002,10001,00005
	{
		if (strBlockName.length() <= 4) // 1...9999,һ����DO
		{
			strcpy(szBlockName, "DO");
			strRegisterNo = strBlockName;
			if (vecAddr.size() >= 2) // �Ĵ��������λ, 40003.3-5
				strBitNo = vecAddr[1]; // 3-5
		}
		else // 5λ��6λ
		{
			if (chBlockFlag == '1' || chBlockFlag == '3' || chBlockFlag == '4')
			{
				if (chBlockFlag == '1')
					strcpy(szBlockName, "DI");
				else if (chBlockFlag == '4')
					strcpy(szBlockName, "AO");
				else if (chBlockFlag == '3')
					strcpy(szBlockName, "AI");
				strRegisterNo = strBlockName.substr(1); // ȥ��40001��4���õ��Ĵ����ı��
			}
			else // ��3/4/1������0������Ϊ��0��ͷ
			{
				strcpy(szBlockName, "DO");
				strRegisterNo = strBlockName;
				//Drv_LogMessage(PK_LOGLEVEL_ERROR, "parse tag addr:%s, no block type name! must:AI/AO/DI/DO/0/1/2/3", szAddressInDevice);
			}
			if (vecAddr.size() >= 2) // �Ĵ��������λ, 40003.3-5
				strBitNo = vecAddr[1]; // 3-5
		}
	}
	else // ��ĸ��������AO,AI,DO,DI
	{
		strncpy(szBlockName, strBlockName.c_str(), sizeof(szBlockName)-1);
		if (vecAddr.size() <= 1)
		{
			Drv_LogMessage(PK_LOGLEVEL_ERROR, "parse tag addr:%s, no register number! ", szAddressInDevice);
			return -3;
		}
		strRegisterNo = vecAddr[1];
		if (vecAddr.size() >= 3) // �Ĵ��������λ, 40003.3-5
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
		Drv_LogMessage(PK_LOGLEVEL_ERROR, "parse tag addr:%s, ���ݿ�����:%s��֧��, �쳣", szAddressInDevice, szBlockName);
		return -2;
	}

	// ������ʼ��ַ.Modbus�ĵ�ַ���Ǵ�1��ʼ�ģ����Ҫ��ȥ1
	int	 nStartAddr = ::atoi(strRegisterNo.c_str());
	if (bBlockByBit) // bit,DI, DO
		*pnStartBits = nStartAddr - 1;
	else // AI,AO
		*pnStartBits = (nStartAddr - 1) * 16;

	// ������ʼλ�������ֵ�ַ��Ч
	// AO��AI��λȡʱ�����ݳ��Ⱥ�λ�������
	if (bBlockByWord && strBitNo.length() > 0) // 40001.4-6
	{
		vector<string> vecBit = PKStringHelper::StriSplit(strBitNo.c_str(), "-"); // 4-8, ���� 4
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
			*pnEndBits = *pnStartBits + (nEndBit - nStartBit); // ��ͷ����λ����,AO:3.4-8
		}
		else // AO:3.4
			*pnEndBits = *pnStartBits; // ��ͷ����λ��������˲��ܼ�1

	}
	else // DI��DO���Լ�AO��AI����λʱ�����ݳ��Ⱥ�tag���ݳ�����ͬ
		*pnEndBits = *pnStartBits + nLenBits - 1;

	if (strStationNo.size() > 0) // �����վ�ţ������������Ϊ stationNo#BLockName
	{
		char szTmpBlockName[128] = { 0 };
		PKStringHelper::Snprintf(szTmpBlockName, nBlockNameLen, "%s#%s", strStationNo.c_str(), szBlockName);
		//@version ��bug,��������һ����ʱ����
		//PKStringHelper::Snprintf(szBlockName, nBlockNameLen, "%s#%s", strStationNo.c_str(), szBlockName);
		sprintf(szBlockName, "%s", szTmpBlockName);
	}

	return 0;
}