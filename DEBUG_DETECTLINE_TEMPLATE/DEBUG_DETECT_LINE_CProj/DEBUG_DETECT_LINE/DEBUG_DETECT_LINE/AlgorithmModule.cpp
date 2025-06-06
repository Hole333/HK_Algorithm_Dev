#include "stdafx.h"
#include "AlgorithmModule.h"
#include <stdlib.h>
#include <fstream>
#include "ErrorCodeDefine.h"
#include "iMVS-6000PixelFormatDefine.h"


CAlgorithmModule::CAlgorithmModule()
{
	m_nthreshold = 120;
	m_fminLength = 0.4;
	m_fmaxLineLength = 1.00;
	m_fmaxLineGap = 0.30;
	m_ncannyLow = 100;
	m_ncannyHigh = 300;
	m_nroiWidth = 70;
	m_nroiHeight = 300;
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

int CAlgorithmModule::GetInputParam(const void* hInput)
{
	int nRet = 0;
	HKA_F32 fValue;
	int nArrayCount = 0;
	float c_x, c_y, rIn, rOut, ROI_Width, ROI_Height;
	VM_M_GetFloat(hInput, "cx", 0, &fValue, &nArrayCount);
	if (IMVS_EC_OK == nRet && nArrayCount > 0) c_x = fValue;
	HKA_MODU_CHECK_ERROR(IMVS_EC_OK != nRet, nRet);

	VM_M_GetFloat(hInput, "cy", 0, &fValue, &nArrayCount);
	if (IMVS_EC_OK == nRet && nArrayCount > 0) c_y = fValue;
	HKA_MODU_CHECK_ERROR(IMVS_EC_OK != nRet, nRet);

	VM_M_GetFloat(hInput, "rIn", 0, &fValue, &nArrayCount);
	if (IMVS_EC_OK == nRet && nArrayCount > 0) rIn = fValue;
	HKA_MODU_CHECK_ERROR(IMVS_EC_OK != nRet, nRet);

	VM_M_GetFloat(hInput, "rOut", 0, &fValue, &nArrayCount);
	if (IMVS_EC_OK == nRet && nArrayCount > 0) rOut = fValue;
	HKA_MODU_CHECK_ERROR(IMVS_EC_OK != nRet, nRet);

	lineDetect.updateInputParam(c_x, c_y, rIn, rOut);
}

int CAlgorithmModule::GetImage2(IN void* hInput, Mat &img)
{
	HKA_IMAGE image;
	HKA_S32 nRet = IMVS_EC_UNKNOWN;
	HKA_S32 nImageStatus = 0;
	bool isGetMask = false;
	do
	{
		nRet = VmModule_GetInputImageByName(hInput,
											"InImage2",
											"InImage2Width",
											"InImage2Height",
											"InImage2PixelFormat",
											&image,
											&nImageStatus);
		HKA_CHECK_BREAK(IMVS_EC_OK != nRet);
		isGetMask = true;
	} while (0);

	if (isGetMask)
	{
		img = HKAImageToMat(image);
	}

	return nRet;
}




int CAlgorithmModule::Process(IN void* hInput, IN void* hOutput, IN MVDSDK_BASE_MODU_INPUT* modu_input)
{
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
	Mat resizeImage;
	Mat FilteredImage;
	GetImage2(hInput, FilteredImage);

	Mat dstImage;
	Mat ROIMask;
	
	/// /// /////////////////////////////////////////////算法部分////////////////////////////////////////////////////////////////////////
	int64 start = cv::getTickCount();

	resize(srcImage, resizeImage, Size(612, 512));
	resize(FilteredImage, FilteredImage, Size(612, 512));
	threshold(resizeImage, ROIMask, 0, 255, THRESH_BINARY + THRESH_OTSU);
	bitwise_and(FilteredImage, ROIMask, dstImage);
	GetInputParam(hInput);
	RotatedRect ROI = lineDetect.detect(dstImage);
	float t = (cv::getTickCount() - start) / static_cast<float>(cv::getTickFrequency()) * 1000;
	resize(dstImage, dstImage, Size(2448, 2048));
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	MatToHKAImageV2(dstImage, pImage);
	if (nErrCode != IMVS_EC_OK)
	{
		return IMVS_EC_PARAM;
	}
	else
	{ 
		VmModule_OutputImageByName_8u_C1R(hOutput, 1, "OutImage", "OutImageWidth", "OutImageHeight", "OutImagePixelFormat", &pImage, 0, pSharedName);
		VM_M_SetFloat(hOutput, "rectRoiCenterX", 0, ROI.center.x);
		VM_M_SetFloat(hOutput, "rectRoiCenterY", 0, ROI.center.y);
		VM_M_SetFloat(hOutput, "rectRoiWidth",   0, ROI.size.width);
		VM_M_SetFloat(hOutput, "rectRoiHeight",  0, ROI.size.height);
		VM_M_SetFloat(hOutput, "rectRoiAngle",   0, ROI.angle);
	}
  
	/************************************************/
	//默认算法时间20ms，根据实际时间计算
	MODULE_RUNTIME_INFO struRunInfo = { 0 };
	struRunInfo.fAlgorithmTime = t;
	VM_M_SetModuleRuntimeInfo(m_hModule, &struRunInfo);

	return IMVS_EC_OK;
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

	if (0 == strcmp("threshold", szParamName))
	{
		sprintf_s(pBuff, nBuffSize, "%d", m_nthreshold);
	}
	else if (0 == strcmp("minLength", szParamName))
	{
		sprintf_s(pBuff, nBuffSize, "%f", m_fminLength);
	}
	else if (0 == strcmp("maxLineLength", szParamName))
	{
		sprintf_s(pBuff, nBuffSize, "%f", m_fmaxLineLength);
	}
	else if (0 == strcmp("maxLineGap", szParamName))
	{
		sprintf_s(pBuff, nBuffSize, "%f", m_fmaxLineGap);
	}
	else if (0 == strcmp("cannyLow", szParamName))
	{
		sprintf_s(pBuff, nBuffSize, "%d", m_ncannyLow);
	}
	else if (0 == strcmp("cannyHigh", szParamName))
	{
		sprintf_s(pBuff, nBuffSize, "%d", m_ncannyHigh);
	}
	else if (0 == strcmp("roiWidth", szParamName))
	{
		sprintf_s(pBuff, nBuffSize, "%d", m_nroiWidth);
	}
	else if (0 == strcmp("roiHeight", szParamName))
	{
		sprintf_s(pBuff, nBuffSize, "%d", m_nroiHeight);
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

	if (0 == strcmp("threshold", szParamName))
	{
		sscanf_s(pData, "%d", &m_nthreshold);
	}
	else if (0 == strcmp("minLength", szParamName))
	{
		sscanf_s(pData, "%f", &m_fminLength);
	}
	else if (0 == strcmp("maxLineLength", szParamName))
	{
		sscanf_s(pData, "%f", &m_fmaxLineLength);
	}
	else if (0 == strcmp("maxLineGap", szParamName))
	{
		sscanf_s(pData, "%f", &m_fmaxLineGap);
	}
	else if (0 == strcmp("cannyLow", szParamName))
	{
		sscanf_s(pData, "%d", &m_ncannyLow);
	}
	else if (0 == strcmp("cannyHigh", szParamName))
	{
		sscanf_s(pData, "%d", &m_ncannyHigh);
	}
	else if (0 == strcmp("roiWidth", szParamName))
	{
		sscanf_s(pData, "%d", &m_nroiWidth);
	}
	else if (0 == strcmp("roiHeight", szParamName))
	{
		sscanf_s(pData, "%d", &m_nroiHeight);
	}
	else
	{
		return CVmAlgModuleBase::SetParam(szParamName, pData, nDataLen);
	}
	lineDetect.updateRunningParam(m_nthreshold, m_fminLength, m_fmaxLineLength, m_fmaxLineGap, m_nroiWidth, m_nroiHeight, m_ncannyLow, m_ncannyHigh);
		
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
