#ifdef EXAMPLEMODULE_EXPORTS
#define LINEMODULE_API __declspec(dllexport)
#else
#define LINEMODULE_API __declspec(dllimport)
#endif
#include "VmModuleBase.h"
#include "VmAlgModuBase.h"
#include "ErrorCodeDefine.h"
#include "VmModuleSharedMemoryBase.h"
#include "LineDetect.h"

// This class is exported from the LineModule.dll
class LINEMODULE_API CAlgorithmModule : public CVmAlgModuleBase, public CModuleSharedMemoryBase
{
public:
	// ����
	explicit CAlgorithmModule();
	
	// ����
	virtual ~CAlgorithmModule();
	LineDetectClass lineDetect;
public:

	// ��ʼ��
	int Init();

	int GetInputParam(const void* hInput);

	// �����㷨
	int Process(IN void* hInput, IN void* hOutput, IN MVDSDK_BASE_MODU_INPUT* modu_input);

	// ��ȡ�㷨����
	int GetParam(IN const char* szParamName, OUT char* pBuff, IN int nBuffSize, OUT int* pDataLen);

	// �����㷨����
	int SetParam(IN const char* szParamName, IN const char* pData, IN int nDataLen);

	int GetImage2(IN void* hInput, Mat& img);

	// ����Ĭ��ͼ���ʽת�� Ϊcv��ʽ
	cv::Mat HKAImageToMat(HKA_IMAGE hik_Image);

	// cv ͼ���ʽת��Ϊ��������ͼ���ʽ
	HKA_IMAGE MatToHKAImage(cv::Mat mat);

	void MatToHKAImageV2(Mat& mat, HKA_IMAGE& Output);


public:
	//void* m_hModule;   // ģ���� - 4.3 �ڻ����ж�����


private:

	int m_nthreshold;
	float m_fminLength;
	float m_fmaxLineLength;
	float m_fmaxLineGap;
	int m_ncannyLow;
	int m_ncannyHigh;
	int m_nroiWidth;
	int m_nroiHeight;
};


/////////////////////////////ģ���뵼���Ľӿڣ�ʵ�ֿ�ʼ��//////////////////////////////////////////
#ifdef __cplusplus
extern "C"
{
#endif
    
    // ����__stdcall����Լ����������.def�ļ������ӽӿ�������
	LINEMODULE_API CAbstractUserModule* __stdcall CreateModule(void* hModule);
	LINEMODULE_API void __stdcall DestroyModule(void* hModule, CAbstractUserModule* pUserModule);

#ifdef __cplusplus
};
#endif
/////////////////////////////ģ���뵼���Ľӿڣ�ʵ�ֽ�����//////////////////////////////////////////
