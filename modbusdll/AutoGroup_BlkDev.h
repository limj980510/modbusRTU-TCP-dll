#ifndef CMODBUS_GROUP_BUILDER_H
#define CMODBUS_GROUP_BUILDER_H

#include "pkdriver/pkdrvcmn.h"
#include <vector>
#include <string>
#include <memory.h>

using namespace std;

#define PK_DATABLOCKNAME_MAXLEN		64
#define PK_DATABLOCKTYPE_MAXLEN		16

typedef struct _GROUP_OPTION
{
	int nIsGroupTag;
	int nMaxBytesOneGroup;
	_GROUP_OPTION()
	{
		nIsGroupTag = 1;
		nMaxBytesOneGroup = 1000;
	}
}GROUP_OPTION;

typedef struct _DRVGROUP
{
	char            szAutoGroupName[PK_DATABLOCKNAME_MAXLEN + 1];
	char			szHWBlockName[PK_DATABLOCKTYPE_MAXLEN + 1]; // AI/DI/AO/DO 或者 5#AO 

	int				nStartAddrBit;	// 起始位数，相对于整个物理块 起始地址
	int				nEndAddrBit;	// 结束位数
	int				nLenBits;		// 位数长度

	// 为了周期内读取计算开始寄存器，控制时不需要用到。按位的寄存器也有可能是按字读取（如三菱的X、Y等位寄存器）
	int				nBeginRegister;	// 起始寄存器号，是第1个tag点地址的寄存器，如第一个tag地址D1就是数字1。通常用nStartAddrBit可以计算出来，要考虑起始寄存器为1还是0开始？，按照nRegisterLenBits为单位计算
	int				nEndRegister;	// 结束寄存器号，是最后1个tag点地址的寄存器
	int				nRegisterNum;		// 要读取的寄存器数量，按照nRegisterLenBits为单位计算，=nEndRegister - nBeginRegister + 1
	int				nRegisterLenBits;	// 要读取的寄存器的长度单位（通常为1或16（位），表示按位还是按字。按位的寄存器也有可能按字读取，如三菱的X/Y是按字读取的）

	int				nPollRate;		// 扫描周期，单位ms
	vector<PKTAG *>		vecTags;		// 本组内的tag点数组指针

	int				nFailCountRecent; // 最近失败次数，运行时需要
	int				nUserData[10]; // 临时数据
	void *			pUserData[10]; // 临时数据
	_DRVGROUP()
	{
		memset(szAutoGroupName, 0, sizeof(szAutoGroupName));
		memset(szHWBlockName, 0, sizeof(szHWBlockName));
		nStartAddrBit = 0;
		nEndAddrBit = 0;
		nLenBits = 0;
		/*nPollRate = 1000;*/
		nPollRate = 60000;
		nFailCountRecent = 0;
		nBeginRegister = nEndRegister = nRegisterNum = nRegisterLenBits = 0;
		for (int i = 0; i < 10; i++){
			nUserData[i] = 0;
			pUserData[i] = NULL;
		}
		vecTags.clear();
	}
	~_DRVGROUP()
	{
		vecTags.clear();
	}
}DRVGROUP;
typedef std::vector<DRVGROUP *> GroupVector;

class CAutoGroupBlkDev
{
public:
	CAutoGroupBlkDev(vector<PKTAG *> &vecTags);
	virtual ~CAutoGroupBlkDev(void);
	void AutoGroupTags(GroupVector &vecTagGrpInfo, GROUP_OPTION *pGroupOption);
	void GroupTagsOfGroupType(string strGrpType, vector<PKTAG *> &vecTagsOfGrpType, GroupVector &vecTagGrpInfo, GROUP_OPTION *pGroupOption);
	void calcTagsStartAndEndBits();
protected:
	const char *GenerateGroupName(const char *szGrpType);

private:
	vector<PKTAG *> m_vecAllDevTags;
	static unsigned int m_nAutoGrpNum;
};

extern long TagsToGroups(vector<PKTAG *> &vecTags, GROUP_OPTION *pGroupOption, GroupVector &vecTagGroup);
extern long CalcGroupRegisterInfo(DRVGROUP *pTagGroup, int nRegisterLenBits); // 计算块内寄存器起始地址、寄存器个数、初始有效位,egisterLenBits 可为1、8、16
extern long UpdateGroupData(PKDEVICE *pDevice, DRVGROUP *pTagGroup, const char *szBuffer, long lBufLen, short nStatus);
extern long UpdateGroupQuality(PKDEVICE *pDevice, DRVGROUP *pTagGroup, int nQuality, const char *szQualityFormat, ...);
#endif