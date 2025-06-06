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
	// 构造
	explicit CAlgorithmModule();
	
	// 析构
	virtual ~CAlgorithmModule();
	LineDetectClass lineDetect;
public:

	// 初始化
	int Init();

	int GetInputParam(const void* hInput);

	// 进行算法
	int Process(IN void* hInput, IN void* hOutput, IN MVDSDK_BASE_MODU_INPUT* modu_input);

	// 获取算法参数
	int GetParam(IN const char* szParamName, OUT char* pBuff, IN int nBuffSize, OUT int* pDataLen);

	// 设置算法参数
	int SetParam(IN const char* szParamName, IN const char* pData, IN int nDataLen);

	int GetImage2(IN void* hInput, Mat& img);

	// 海康默认图像格式转化 为cv格式
	cv::Mat HKAImageToMat(HKA_IMAGE hik_Image);

	// cv 图像格式转化为海康适配图像格式
	HKA_IMAGE MatToHKAImage(cv::Mat mat);

	void MatToHKAImageV2(Mat& mat, HKA_IMAGE& Output);


public:
	//void* m_hModule;   // 模块句柄 - 4.3 在基类中定义了


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


/////////////////////////////模块须导出的接口（实现开始）//////////////////////////////////////////
#ifdef __cplusplus
extern "C"
{
#endif
    
    // 采用__stdcall调用约定，且须在.def文件中增加接口描述。
	LINEMODULE_API CAbstractUserModule* __stdcall CreateModule(void* hModule);
	LINEMODULE_API void __stdcall DestroyModule(void* hModule, CAbstractUserModule* pUserModule);

#ifdef __cplusplus
};
#endif
/////////////////////////////模块须导出的接口（实现结束）//////////////////////////////////////////
