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
	char			szHWBlockName[PK_DATABLOCKTYPE_MAXLEN + 1]; // AI/DI/AO/DO ���� 5#AO 

	int				nStartAddrBit;	// ��ʼλ������������������ ��ʼ��ַ
	int				nEndAddrBit;	// ����λ��
	int				nLenBits;		// λ������

	// Ϊ�������ڶ�ȡ���㿪ʼ�Ĵ���������ʱ����Ҫ�õ�����λ�ļĴ���Ҳ�п����ǰ��ֶ�ȡ���������X��Y��λ�Ĵ�����
	int				nBeginRegister;	// ��ʼ�Ĵ����ţ��ǵ�1��tag���ַ�ļĴ��������һ��tag��ַD1��������1��ͨ����nStartAddrBit���Լ��������Ҫ������ʼ�Ĵ���Ϊ1����0��ʼ��������nRegisterLenBitsΪ��λ����
	int				nEndRegister;	// �����Ĵ����ţ������1��tag���ַ�ļĴ���
	int				nRegisterNum;		// Ҫ��ȡ�ļĴ�������������nRegisterLenBitsΪ��λ���㣬=nEndRegister - nBeginRegister + 1
	int				nRegisterLenBits;	// Ҫ��ȡ�ļĴ����ĳ��ȵ�λ��ͨ��Ϊ1��16��λ������ʾ��λ���ǰ��֡���λ�ļĴ���Ҳ�п��ܰ��ֶ�ȡ���������X/Y�ǰ��ֶ�ȡ�ģ�

	int				nPollRate;		// ɨ�����ڣ���λms
	vector<PKTAG *>		vecTags;		// �����ڵ�tag������ָ��

	int				nFailCountRecent; // ���ʧ�ܴ���������ʱ��Ҫ
	int				nUserData[10]; // ��ʱ����
	void *			pUserData[10]; // ��ʱ����
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
extern long CalcGroupRegisterInfo(DRVGROUP *pTagGroup, int nRegisterLenBits); // ������ڼĴ�����ʼ��ַ���Ĵ�����������ʼ��Чλ,egisterLenBits ��Ϊ1��8��16
extern long UpdateGroupData(PKDEVICE *pDevice, DRVGROUP *pTagGroup, const char *szBuffer, long lBufLen, short nStatus);
extern long UpdateGroupQuality(PKDEVICE *pDevice, DRVGROUP *pTagGroup, int nQuality, const char *szQualityFormat, ...);
#endif