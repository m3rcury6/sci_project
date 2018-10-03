//
// Created by stan on 10.04.18.
//


#include "CDarknet.h"
#include <ros/console.h>

CDarknet::CDarknet(char* pDataFile, char* pCfgFile, char* pWeightFile) {
    m_pOptions = read_data_cfg(pDataFile);
    m_pNameList = option_find_str(m_pOptions, "names", "data/names.list");
    m_ppNames = get_labels(m_pNameList);

    m_pNet = load_network(pCfgFile,pWeightFile,0);
    set_batch_network(m_pNet,1);
    srand(2222222);
    m_fNMS = 0.45;
    m_fThreshold = 0.20;
    m_fHierThreshold = 0.5;

    m_bFirstImage = false;
    m_pDetections = nullptr;
    m_nDetectionNum = 0;
    m_nClasses = 3;

    m_oBBoxArray.bboxes.clear();
}

CDarknet::~CDarknet() {
	delete m_pNet;
	m_pNet = NULL;
}

void CDarknet::SetThreshold(const float &fThreshold) {
    this->m_fThreshold = fThreshold;
}

void CDarknet::SetNMS(const float &fNMS) {
    this->m_fNMS = fNMS;
}

void CDarknet::SetInput(const cv::Mat &oMat) {
    if(!m_bFirstImage)
    {
        this->AllocBuffer(oMat);
        if(oMat.cols > 0 && oMat.rows > 0) m_bFirstImage = true;
    }
    this->UpdateImageBuffer(oMat);
}

void CDarknet::UpdateImageBuffer(const cv::Mat &oMat) {

	cv::rectangle(oMat,cv::Point(0,oMat.rows*0.7),cv::Point(oMat.cols,oMat.rows),cv::Scalar(0,0,0),-1);
    oMat.copyTo(m_oOriginal);
    //oMat.copyTo(m_oResized);
    m_oResized = resizeKeepAspectRatio(m_oOriginal,cv::Size(m_pNet->w,m_pNet->h),cv::Scalar(0,0,0));

    m_oResized.convertTo(m_oResized,CV_32FC3, 1.0/255.0);


    floatMatChannels.clear();
    cv::split(m_oResized, floatMatChannels);
    cv::vconcat(floatMatChannels, m_oResized);

}

void CDarknet::AllocBuffer(const cv::Mat &oMat) {
    m_nWidth = oMat.cols;
    m_nHeight = oMat.rows;
    m_nChannels = oMat.channels();

    m_oImageBuffer.h = m_nHeight;
    m_oImageBuffer.w = m_nWidth;
    m_oImageBuffer.c = m_nChannels;
    m_oImageBuffer.data = (float*)malloc(sizeof(float)*m_nHeight*m_nWidth*m_nChannels);
    ROS_INFO("Image: %d %d %d",m_nWidth,m_nHeight,m_nChannels);
	floatMatChannels.resize(3);
}

ai_darknet::bbox_array CDarknet::Forward() {

    network_predict(m_pNet,(float*)m_oResized.data);
    m_pDetections = get_network_boxes(m_pNet,m_nWidth,m_nHeight,m_fThreshold,m_fHierThreshold,0,1,&m_nDetectionNum);
    do_nms_sort(m_pDetections,m_nDetectionNum,m_nClasses,m_fNMS);

   // ROS_INFO("Boxes:%d",m_nDetectionNum);

    m_oBBoxArray.header.frame_id = "map";

    m_oBBoxArray.bboxes.clear();

    int nMaxIndex = 0;
    float fMaxProb = 0.0;

    for(int i = 0; i < m_nDetectionNum;i++)
    {

        for(int j=0; j < m_nClasses;j++)
        {
         if(m_pDetections[i].prob[j] > fMaxProb)
         {
             fMaxProb = m_pDetections[i].prob[j];
             nMaxIndex = j;
         }
        }

        if(m_pDetections[i].prob[nMaxIndex] > m_fThreshold)
        {
            //can be added
            ai_darknet::bbox oBox;
            box b = m_pDetections[i].bbox;
            int left  = (b.x-b.w/2.)*m_nWidth;
            int right = (b.x+b.w/2.)*m_nWidth;
            int top   = (b.y-b.h/2.)*m_nHeight;
            int bot   = (b.y+b.h/2.)*m_nHeight;
            switch(nMaxIndex)
            {
                case 0:  //yellow

                    oBox.Class = "yellow";
                    oBox.prob = m_pDetections[i].prob[0];
                    oBox.x = b.x;
                    oBox.y = b.y;
                    oBox.w = b.w;
                    oBox.h = b.h;
                    m_oBBoxArray.bboxes.push_back(oBox);
                    cv::rectangle(m_oOriginal,cv::Point(left,top),cv::Point(right,bot),cv::Scalar(255,255,0),1);
                    break;
                case 1:  //blue

                    oBox.Class = "blue";
                    oBox.prob = m_pDetections[i].prob[1];
                    oBox.x = b.x;
                    oBox.y = b.y;
                    oBox.w = b.w;
                    oBox.h = b.h;
                    m_oBBoxArray.bboxes.push_back(oBox);
                    cv::rectangle(m_oOriginal,cv::Point(left,top),cv::Point(right,bot),cv::Scalar(0,0,255),1);
                    break;
                case 2:  //orange

                    oBox.Class = "orange";
                    oBox.prob = m_pDetections[i].prob[2];
                    oBox.x = b.x;
                    oBox.y = b.y;
                    oBox.w = b.w;
                    oBox.h = b.h;
                    m_oBBoxArray.bboxes.push_back(oBox);
                    cv::rectangle(m_oOriginal,cv::Point(left,top),cv::Point(right,bot),cv::Scalar(255,160,0),1);
                    break;
            };





        }

    }

    return m_oBBoxArray;
    //fetch detections pointer now
}

void CDarknet::SetNClasses(const int &nClasses) {
    this->m_nClasses = nClasses;
}

cv::Mat CDarknet::resizeKeepAspectRatio(const cv::Mat &input, const cv::Size &dstSize, const cv::Scalar &bgcolor)
{
    cv::Mat output;

    double h1 = dstSize.width * (input.rows/(double)input.cols);
    double w2 = dstSize.height * (input.cols/(double)input.rows);
    if( h1 <= dstSize.height) {
        cv::resize( input, output, cv::Size(dstSize.width, h1));
    } else {
        cv::resize( input, output, cv::Size(w2, dstSize.height));
    }

    int top = (dstSize.height-output.rows) / 2;
    int down = (dstSize.height-output.rows+1) / 2;
    int left = (dstSize.width - output.cols) / 2;
    int right = (dstSize.width - output.cols+1) / 2;

    cv::copyMakeBorder(output, output, top, down, left, right, cv::BORDER_CONSTANT, bgcolor );

    return output;
}

cv::Mat CDarknet::GetDetections() {
    return m_oOriginal;
}
