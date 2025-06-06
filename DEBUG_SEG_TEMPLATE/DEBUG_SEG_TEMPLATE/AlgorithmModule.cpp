#include "stdafx.h"
#include "AlgorithmModule.h"
#include <stdlib.h>
#include <fstream>
#include "ErrorCodeDefine.h"
#include "iMVS-6000PixelFormatDefine.h"
#include <algorithm>


CAlgorithmModule::CAlgorithmModule()
{
	m_fconfThreshold = 0.3;
	m_fiouThreshold = 0.40;
	m_fmaskThreshold = 0.50;
	m_strModelDir = "F:\\goodstudy\\dataset\\trained-model\\y59.onnx";
}

CAlgorithmModule::~CAlgorithmModule()
{

}

int CAlgorithmModule::Init()
{

	int nRet = VM_M_GetModuleId(m_hModule, &m_nModuleId);
	if (IMVS_EC_OK != nRet)
	{
		m_nModuleId = -1;
		return nRet;
	}

	nRet = ResetDefaultParam();
	if (nRet != IMVS_EC_OK)
	{
		OutputDebugStringA("###Call ResetDefaultParam failed.");
	}

	return nRet;
}

Mat CAlgorithmModule::HKAImageToMat(HKA_IMAGE hik_Image)
{
	Mat mat;
	if (hik_Image.format == HKA_IMG_MONO_08)
	{
		mat = Mat(hik_Image.height, hik_Image.width, CV_8UC1, hik_Image.data[0]);
	}
	return mat;
}

HKA_IMAGE CAlgorithmModule::MatToHKAImage(Mat mat)
{
	HKA_IMAGE image = { HKA_IMG_MONO_08, 0 };
	if (mat.channels() == 1)
	{
		image.width = mat.cols;
		image.height = mat.rows;
		image.format = HKA_IMG_MONO_08;
		image.step[0] = mat.cols;
		image.data[0] = mat.data;
	}
	return image;
}

void CAlgorithmModule::MatToHKAImageV2(Mat& mat, HKA_IMAGE& Output)
{
	if (mat.channels() == 1)
	{
		Output.width = mat.cols;
		Output.height = mat.rows;
		Output.format = HKA_IMG_MONO_08;
		Output.step[0] = mat.cols;
		if (Output.data[0] != NULL)
		{
			memset(Output.data[0], 0, Output.width * Output.height);
			memcpy_s(Output.data[0], Output.width * Output.height, mat.data, Output.width * Output.height);
		}
	}
}


int CAlgorithmModule::Process(IN void* hInput, IN void* hOutput, IN MVDSDK_BASE_MODU_INPUT* modu_input)
{
	OutputDebugStringA("###Call CAlgorithmModule::Proces -->begin\n");
	int nErrCode = 0;

	HKA_IMAGE pImage;
	pImage.height = modu_input->pImageInObj->GetHeight();
	pImage.width = modu_input->pImageInObj->GetWidth();
	pImage.format = HKA_IMG_MONO_08;
	int nLen = pImage.height * pImage.width;
	char* pSharedName = NULL;

	nErrCode = AllocateSharedMemory(m_nModuleId, nLen, (char**)(&pImage.data), &pSharedName);
	if (nErrCode != IMVS_EC_OK)
	{
		return IMVS_EC_PARAM;
	}

	memcpy_s(pImage.data[0], nLen, modu_input->pImageInObj->GetImageData(0)->pData, modu_input->pImageInObj->GetImageData(0)->nSize);

	Mat srcImage = HKAImageToMat(pImage);
	//cvtColor(srcImage, srcImage, COLOR_GRAY2BGR);

// 2.算法处理

	clock_t startTime, endTime;
	startTime = clock();
	if (predictor.isInit)
	{
		std::vector<Yolov11Result> result = predictor.predict(srcImage);
		srcImage = YOLOutils::getBinaryMask(srcImage, result);
	}
	endTime = clock();
	float spendTime = (float)(endTime - startTime) / CLOCKS_PER_SEC * 1000;

	// 3.输出结果
		/************************************************/
		//HKA_IMAGE hkaImage = MatToHKAImage(srcImage);
	MatToHKAImageV2(srcImage, pImage);
	//nErrCode = AllocateSharedMemory(m_nModuleId, nLen, (char**)(&hkaImage.data), &pSharedName);
	nErrCode = VmModule_OutputImageByName_8u_C1R(hOutput, 1, "OutImage", "OutImageWidth", "OutImageHeight", "OutImagePixelFormat", &pImage, 0, pSharedName);
	/************************************************/

	if (nErrCode != IMVS_EC_OK)
	{
		return IMVS_EC_PARAM;
	}



	/************************************************/
	//默认算法时间20ms，根据实际时间计算
	MODULE_RUNTIME_INFO struRunInfo = { 0 };
	struRunInfo.fAlgorithmTime = spendTime;
	VM_M_SetModuleRuntimeInfo(m_hModule, &struRunInfo);

	OutputDebugStringA("###Call CAlgorithmModule::Proces end\n");

	return IMVS_EC_OK;
}
std::string GBKToUTF8(const char* gbkStr)
{
	int gbkLen = strlen(gbkStr);
	int utf8Len = gbkLen * 3; // UTF-8最多需要3个字节来表示一个字符  
	wchar_t* wstr = new wchar_t[utf8Len + 1];
	memset(wstr, 0, sizeof(wchar_t) * (utf8Len + 1));

	// 将GBK编码的字符串转换为宽字符（通常是UTF-16）  
	MultiByteToWideChar(CP_ACP, 0, gbkStr, -1, wstr, utf8Len);

	char* utf8Str = new char[utf8Len + 1];
	memset(utf8Str, 0, utf8Len + 1);

	// 将宽字符转换为UTF-8编码的字符串  
	WideCharToMultiByte(CP_UTF8, 0, wstr, -1, utf8Str, utf8Len, NULL, NULL);

	delete[] wstr;
	std::string utf8String(utf8Str);
	delete[] utf8Str;

	return utf8String;
};
std::string UTF8ToGBK(const std::string& utf8Str) {
	int utf8Len = utf8Str.length();
	int gbkLen = MultiByteToWideChar(CP_UTF8, 0, utf8Str.c_str(), utf8Len, NULL, 0);
	wchar_t* wstr = new wchar_t[gbkLen];
	MultiByteToWideChar(CP_UTF8, 0, utf8Str.c_str(), utf8Len, wstr, gbkLen);

	int gbkOutputSize = WideCharToMultiByte(CP_ACP, 0, wstr, gbkLen, NULL, 0, NULL, NULL);
	char* gbkStr = new char[gbkOutputSize];
	WideCharToMultiByte(CP_ACP, 0, wstr, gbkLen, gbkStr, gbkOutputSize, NULL, NULL);

	delete[] wstr;
	std::string result(gbkStr);
	delete[] gbkStr;

	return result;
}

std::string CleanOnnxPath(const std::string& s) {
	// Step 1: 找到最后一个 ".onnx" 的位置
	size_t pos = s.rfind(".onnx");
	if (pos == std::string::npos)
	{
		return ""; // 未找到有效模型路径
	}

	std::string sub = s.substr(0, pos + 5); // 截取到 .onnx 结束

	// Step 2: 处理双反斜杠转单反斜杠
	std::string processed;
	bool prev_backslash = false;
	for (char c : sub) 
	{
		if (c == '\\' && prev_backslash) 
		{
			prev_backslash = false; // 跳过重复的反斜杠
		}
		else 
		{
			processed += c;
			prev_backslash = (c == '\\');
		}
	}

	// Step 3: 正斜杠转反斜杠
	std::replace(processed.begin(), processed.end(), '/', '\\');

	return processed;
}

int CAlgorithmModule::GetParam(IN const char* szParamName, OUT char* pBuff, IN int nBuffSize, OUT int* pDataLen)
{
	OutputDebugStringA("###Call CAlgorithmModule::GetParam");

	int nMsgLen = 0;
	int nErrCode = IMVS_EC_OK;
	if (szParamName == NULL || strlen(szParamName) == 0 || pBuff == NULL || nBuffSize <= 0 || pDataLen == NULL)
	{
		return IMVS_EC_PARAM;
	}
	//memset(pBuff, 0, nBuffSize);

	if (0 == strcmp("confThreshold", szParamName))
	{
		sprintf_s(pBuff, nBuffSize, "%f", m_fconfThreshold);
	}
	else if (0 == strcmp("iouThreshold", szParamName))
	{
		sprintf_s(pBuff, nBuffSize, "%f", m_fiouThreshold);
	}
	else if (0 == strcmp("maskThreshold", szParamName))
	{
		sprintf_s(pBuff, nBuffSize, "%f", m_fmaskThreshold);
	}
	else if (0 == strcmp("ModelDir", szParamName))
	{
		std::string gbkmodeldir = UTF8ToGBK(m_strModelDir);
		int modelfilelen = gbkmodeldir.length();
		if (modelfilelen < nBuffSize)
		{
			strcpy_s(pBuff, nBuffSize, gbkmodeldir.c_str());
			*pDataLen = modelfilelen;
		}
		//sprintf_s(pBuff, nBuffSize, "%s", m_strModelDir);
		else
		{
			return IMVS_EC_PARAM;
		}
	}
	else
	{
		return CVmAlgModuleBase::GetParam(szParamName, pBuff, nBuffSize, pDataLen);
	}
	//if(0 == strcmp(szParamName, "paramA"))
	//{
	//	  sprintf(pBuff, nBuffSize, "", ..);
	//}

	return nErrCode;
}

int CAlgorithmModule::SetParam(IN const char* szParamName, IN const char* pData, IN int nDataLen)
{
	OutputDebugStringA("###Call CAlgorithmModule::SetParam");

	int nErrCode = IMVS_EC_OK;
	if (szParamName == NULL || strlen(szParamName) == 0 || pData == NULL || nDataLen == 0)
	{
		return IMVS_EC_PARAM;
	}

	if (0 == strcmp("confThreshold", szParamName))
	{
		sscanf_s(pData, "%f", &m_fconfThreshold);
	}
	else if (0 == strcmp("iouThreshold", szParamName))
	{
		sscanf_s(pData, "%f", &m_fiouThreshold);
	}
	else if (0 == strcmp("maskThreshold", szParamName))
	{
		sscanf_s(pData, "%f", &m_fmaskThreshold);
	}
	else if (0 == strcmp("ModelDir", szParamName))
	{
		//m_strModelDir=pData;
		lastModelPath = CleanedModelPath;
		m_strModelDir = GBKToUTF8(pData);
		CleanedModelPath = CleanOnnxPath(m_strModelDir);
	}
	else
	{
		return CVmAlgModuleBase::SetParam(szParamName, pData, nDataLen);
	}
	if (!predictor.isInit)
	{
		predictor.YOLOSegInit(CleanedModelPath, true, m_fconfThreshold, m_fiouThreshold, m_fmaskThreshold);
		OutputDebugStringA("Init Success \r\n");
	}
	else
	{
		predictor.getParam(m_fconfThreshold, m_fiouThreshold, m_fmaskThreshold);
		if (lastModelPath != CleanedModelPath)
		{
			OutputDebugStringA("Model Change \r\n");
			predictor.Free();
			predictor.YOLOSegInit(CleanedModelPath, true, m_fconfThreshold, m_fiouThreshold, m_fmaskThreshold);
		}
	}

	return nErrCode;
}


/////////////////////////////模块须导出的接口（实现开始）//////////////////////////////////////////

LINEMODULE_API CAbstractUserModule* __stdcall CreateModule(void* hModule)
{
	assert(hModule != NULL);


	// 创建用户模块，并记录实例。
	CAlgorithmModule* pUserModule = new(nothrow) CAlgorithmModule;

	if (pUserModule == NULL)
	{
		return NULL;
	}

	pUserModule->m_hModule = hModule;

	int nRet = pUserModule->Init();
	// 对于模型初始化

	if (IMVS_EC_OK != nRet)
	{
		delete pUserModule;
		return NULL;
	}

	OutputDebugStringA("###Call CreateModule");

	return pUserModule;
}


LINEMODULE_API void __stdcall DestroyModule(void* hModule, CAbstractUserModule* pUserModule)
{
	assert(hModule != NULL);
	OutputDebugStringA("###Call DestroyModule");

	if (pUserModule != NULL)
	{
		delete pUserModule;
	}
}
/////////////////////////////模块须导出的接口（实现结束）//////////////////////////////////////////
