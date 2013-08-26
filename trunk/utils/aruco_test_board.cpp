/*****************************
Copyright 2011 Rafael Muñoz Salinas. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are
permitted provided that the following conditions are met:

   1. Redistributions of source code must retain the above copyright notice, this list of
      conditions and the following disclaimer.

   2. Redistributions in binary form must reproduce the above copyright notice, this list
      of conditions and the following disclaimer in the documentation and/or other materials
      provided with the distribution.

THIS SOFTWARE IS PROVIDED BY Rafael Muñoz Salinas ''AS IS'' AND ANY EXPRESS OR IMPLIED
WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL Rafael Muñoz Salinas OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those of the
authors and should not be interpreted as representing official policies, either expressed
or implied, of Rafael Muñoz Salinas.
********************************/
#include <iostream>
#include <fstream>
#include <sstream>
#include <opencv2/opencv.hpp>
#include <stdlib.h>
#include "aruco.h"
using namespace cv;
using namespace aruco;

string TheInputVideo;
string TheIntrinsicFile;
string boardConfigFile;
bool The3DInfoAvailable=false;
float TheMarkerSize=-1;
VideoCapture videoCap;
Mat inputImg,inputImgCopy;
CameraParameters camParams;
BoardConfiguration TheBoardConfig;
BoardDetector boardDetect;

string TheOutVideoFilePath;
cv::VideoWriter vidWriter;

void cvTackBarEvents(int pos,void*);
pair<double,double> avgTime(0,0) ;//determines the average time required for detection
double ThresParam1,ThresParam2;
int iThresParam1,iThresParam2;
int waitTime= 1;




/************************************
 *
 *
 *
 *
 ************************************/

static void getAbsoluteFilePath(const char* input, std::string& output)
{
    char fullPath [PATH_MAX+1];
    
    //getcwd(fullPath,sizeof(fullPath));
    //cout << "getcwd: " << fullPath << endl;
    
    realpath(input, fullPath);
    output = fullPath;
    
    
    cout << "getAbsoluteFilePath: " << input << " : " << output << endl;

}

bool readArguments ( int argc,char **argv )
{

    if (argc<3) {
        cerr<<"Invalid number of arguments"<<endl;
        cerr<<"Usage: (in.avi|live) boardConfig.yml [intrinsics.yml] [size] [out]"<<endl;
        return false;
    }

    getAbsoluteFilePath(argv[1],TheInputVideo);
    getAbsoluteFilePath(argv[2],boardConfigFile);

    if (argc>=4)
        getAbsoluteFilePath(argv[3],TheIntrinsicFile);
    if (argc>=5)
        TheMarkerSize=atof(argv[4]);
    if (argc>=6)
        getAbsoluteFilePath(argv[5],TheOutVideoFilePath);


    if (argc==4)
        cerr<<"NOTE: You need makersize to see 3d info!!!!"<<endl;

    return true;
}

void processKey(char k) {
    switch (k) {
    case 's':
        if (waitTime==0) waitTime=10;
        else waitTime=0;
        break;

/*    case 'p':
        if (MDetector.getCornerRefinementMethod()==MarkerDetector::SUBPIX)
            MDetector.setCornerRefinementMethod(MarkerDetector::NONE);
        else
            MDetector.setCornerRefinementMethod(MarkerDetector::SUBPIX);
        break;*/
    }
}

/************************************
 *
 *
 *
 *
 ************************************/
int main(int argc,char **argv)
{
    try
    {
        if (  readArguments (argc,argv)==false) return 0;
//parse arguments
        TheBoardConfig.readFromFile(boardConfigFile);
        //read from camera or from  file
        if (TheInputVideo=="live") {
            videoCap.open(0);
            waitTime=10;
        }
        else videoCap.open(TheInputVideo);
        //check video is open
        if (!videoCap.isOpened()) {
            cerr<<"Could not open video"<<endl;
            return -1;

        }

        //read first image to get the dimensions
        videoCap>>inputImg;

        //Open outputvideo
        if (!TheOutVideoFilePath.empty()) {
            std::cout << "Opening video output: " << TheOutVideoFilePath << endl;
            vidWriter.open(TheOutVideoFilePath,
                       videoCap.get(CV_CAP_PROP_FOURCC),
                       videoCap.get(CV_CAP_PROP_FPS),
                           cv::Size(videoCap.get(CV_CAP_PROP_FRAME_WIDTH), videoCap.get(CV_CAP_PROP_FRAME_HEIGHT)));
            
            //vidWriter.open(TheOutVideoFilePath,CV_FOURCC('M','J','P','G'),15,inputImg.size());
            if (!vidWriter.isOpened())  {
                std::cout << "!!! Output video could not be opened" << std::endl;
                return -1;
            }
        }

        //read camera parameters if passed
        if (TheIntrinsicFile!="") {
            camParams.readFromXMLFile(TheIntrinsicFile);
            camParams.resize(inputImg.size());
        }

        //Create gui

        cv::namedWindow("thres",1);
        cv::namedWindow("in",1);
        boardDetect.setParams(TheBoardConfig,camParams,TheMarkerSize);
        boardDetect.getMarkerDetector().getThresholdParams( ThresParam1,ThresParam2);
    // 	boardDetect.getMarkerDetector().enableErosion(true);//for chessboards
        iThresParam1=ThresParam1;
        iThresParam2=ThresParam2;
        cv::createTrackbar("ThresParam1", "in",&iThresParam1, 13, cvTackBarEvents);
        cv::createTrackbar("ThresParam2", "in",&iThresParam2, 13, cvTackBarEvents);
        char key=0;
        int index=0;
                
        //capture until press ESC or until the end of the video
        do
        {
            videoCap.retrieve(inputImg);
            inputImg.copyTo(inputImgCopy);
            index++; //number of images captured
            double tick = (double)getTickCount();//for checking the speed
            //Detection of the board
            float probDetect=boardDetect.detect(inputImg);
            //chekc the speed by calculating the mean speed of all iterations
            avgTime.first+=((double)getTickCount()-tick)/getTickFrequency();
            avgTime.second++;

            cout << ".";
            
            //print marker borders
            for (unsigned int i=0;i<boardDetect.getDetectedMarkers().size();i++) {
                boardDetect.getDetectedMarkers()[i].draw(inputImgCopy,Scalar(0,0,255),1);
            }

            //print board
             if (camParams.isValid()) {
                if ( probDetect>0.2)   {
                    CvDrawingUtils::draw3dAxis( inputImgCopy,boardDetect.getDetectedBoard(),camParams);
                    //draw3dBoardCube( inputImgCopy,TheBoardDetected,TheIntriscCameraMatrix,TheDistorsionCameraParams);
                }
            }
            //DONE! Easy, right?

            //show input with augmented information and  the thresholded image
            cv::imshow("in",inputImgCopy);
            cv::imshow("thres",boardDetect.getMarkerDetector().getThresholdedImage());
            //write to video if required
            if (  !TheOutVideoFilePath.empty()) {
                //create a beautiful compiosed image showing the thresholded
                //first create a small version of the thresholded image
                cv::Mat smallThres;
                cv::resize( boardDetect.getMarkerDetector().getThresholdedImage(),smallThres,cvSize(inputImgCopy.cols/3,inputImgCopy.rows/3));
                cv::Mat small3C;
                cv::cvtColor(smallThres,small3C,CV_GRAY2BGR);
                cv::Mat roi=inputImgCopy(cv::Rect(0,0,inputImgCopy.cols/3,inputImgCopy.rows/3));
                small3C.copyTo(roi);
                vidWriter << inputImgCopy;
// 			 cv::imshow("inputImgCopy",inputImgCopy);

            }

            key=cv::waitKey(waitTime);//wait for key to be pressed
            processKey(key);
        }
        while ( key!=27 && videoCap.grab());
        cout<< endl << "Average detect time: ="<<1000*avgTime.first/avgTime.second<<"ms"<<endl;


        vidWriter.release();
        
    } catch (std::exception &ex)

    {
        cout<<"Exception :"<<ex.what()<<endl;
    }

}
/************************************
 *
 *
 *
 *
 ************************************/

void cvTackBarEvents(int pos,void*)
{
    if (iThresParam1<3) iThresParam1=3;
    if (iThresParam1%2!=1) iThresParam1++;
    if (ThresParam2<1) ThresParam2=1;
    ThresParam1=iThresParam1;
    ThresParam2=iThresParam2;
     boardDetect.getMarkerDetector().setThresholdParams(ThresParam1,ThresParam2);
//recompute
//Detection of the board
    float probDetect=boardDetect.detect( inputImg);
    inputImg.copyTo(inputImgCopy);
    if (camParams.isValid() && probDetect>0.2)
        aruco::CvDrawingUtils::draw3dAxis(inputImgCopy,boardDetect.getDetectedBoard(),camParams);

    
    cv::imshow("in",inputImgCopy);
    cv::imshow("thres",boardDetect.getMarkerDetector().getThresholdedImage());
}



