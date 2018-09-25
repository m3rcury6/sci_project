//
// Created by stan on 10.04.18.
//

#ifndef CONEDETECTION_CDARKNET_H
#define CONEDETECTION_CDARKNET_H


extern "C" {
    #include <darknet.h>
    #include <image.h>
}

#include <opencv2/opencv.hpp>
#include <ai_darknet/bbox.h>
#include <ai_darknet/bbox_array.h>




class CDarknet
{
public:
    CDarknet(char* pDataFile, char* pCfgFile, char* pWeightFile);
    ~CDarknet();

    //Set

    void        SetNMS(const float& fNMS);
    void        SetThreshold(const float& fThreshold);
    void        SetInput(const cv::Mat & oMat);
    void        SetNClasses(const int& nClasses);   //TODO: read from .data file
    ai_darknet::bbox_array        Forward();


    //get

    cv::Mat     GetDetections();


private:
    list*       m_pOptions;
    char*       m_pNameList;
    char**      m_ppNames;
    network*    m_pNet;
    float       m_fNMS;
    float       m_fHierThreshold;
    float       m_fThreshold;
    int         m_nWidth;
    int         m_nHeight;
    int         m_nChannels;
    detection*  m_pDetections;
    int         m_nDetectionNum;
    int         m_nClasses;


    bool        m_bFirstImage;



    //buffer
    image       m_oImageBuffer;
    image       m_oImageSized;
    cv::Mat     m_oMat;
    cv::Mat     m_oResized;
    cv::Mat     m_oOriginal;
	ai_darknet::bbox_array	m_oBBoxArray;


    //Conversion
    void        UpdateImageBuffer(const cv::Mat& oMat);
    void        AllocBuffer(const cv::Mat& oMat);
    cv::Mat     resizeKeepAspectRatio(const cv::Mat &input, const cv::Size &dstSize, const cv::Scalar &bgcolor);
    std::vector<cv::Mat> floatMatChannels;




};


#endif //CONEDETECTION_CDARKNET_H
