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
#include <opencv2/highgui/highgui.hpp>
//#include<highgui.h>

#include <stdlib.h>
#include "aruco.h"
using namespace cv;
using namespace aruco;

string inputVideoFilename;
string intrinsicFilename;
string boardConfigFile;
bool The3DInfoAvailable=false;
float markerSize=-1;
VideoCapture videoCap;
Mat inputImg,outImg;
CameraParameters camParams;
BoardConfiguration boardCfg;
BoardDetector boardDetect;

string vidOutPath;

void cvTackBarEvents(int pos,void*);
pair<double,double> avgTime(0,0) ;//determines the average time required for detection
double thresh1,thresh2;
int iThresh1,iThresh2;
int waitTime= 1;




static void dumpIntItem(int n) {
    cout << n << "," ;
}
    
/************************************
 *
 *
 *
 *
 ************************************/

static void getAbsoluteFilePath(const char* input, std::string& output)
{
    char fullPath [PATH_MAX+1];
    
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

    getAbsoluteFilePath(argv[1],inputVideoFilename);
    getAbsoluteFilePath(argv[2],boardConfigFile);

    if (argc>=4)
        getAbsoluteFilePath(argv[3],intrinsicFilename);
    if (argc>=5)
        markerSize=atof(argv[4]);
    if (argc>=6)
        getAbsoluteFilePath(argv[5],vidOutPath);


    if (argc==4)
        cerr<<"NOTE: You need makersize to see 3d info!!!!"<<endl;

    return true;
}

bool processKey(char k) {
    bool terminate = false;
    
    switch (k) {
        case 's':
            if (waitTime==0)
                waitTime=10;
            else
                waitTime=0;
            break;
            
        case 'q':
        case 27:
            terminate = true;
            break;
            
    }
    
    return terminate;
    
}

void drawMarkers(const vector<int>& searchIds, Mat& outImg )
{
    char labelBuf[40];
    Scalar markerBorderColor = CV_RGB(100,255,0);    
    cv::Size imgSize = outImg.size();    
    int xOffset = imgSize.width / 2;
    int yOffset = imgSize.height / 2;
    
    for (int j = 0; j < searchIds.size(); ++j) {
        MarkerInfo minfo = boardCfg.getMarkerInfo(searchIds[j]);

        vector<cv::Point3f> markerPts = minfo;
        Point2d pt0 = cvPoint(markerPts[0].x + xOffset,markerPts[0].y + yOffset);
        Point2d pt1 = cvPoint(markerPts[1].x + xOffset,markerPts[1].y + yOffset);
        Point2d pt2 = cvPoint(markerPts[2].x + xOffset,markerPts[2].y + yOffset);
        //Point2d pt3 = cvPoint(markerPts[3].x + xOffset,markerPts[3].y + yOffset);
        cv::Rect markerBorder = cvRect(pt0.x, pt0.y, pt1.x - pt0.x, pt2.y - pt1.y);
        cv::rectangle(outImg, markerBorder, markerBorderColor);
        //label
        Point2d textPt = cvPoint(markerBorder.tl().x + markerBorder.width/2, markerBorder.tl().y + markerBorder.height/2);
        sprintf(labelBuf,"%d",minfo.id);
        cv::putText(outImg, labelBuf, textPt, FONT_HERSHEY_PLAIN, 0.8, cvScalar(200,200,250), 1, CV_AA);

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
        if (  readArguments (argc,argv)==false)
            return 0;
        
//parse arguments
        boardCfg.readFromFile(boardConfigFile);
        //read from camera or from  file
        if (inputVideoFilename=="live") {
            videoCap.open(0);
            waitTime=10;
        }
        else {
            videoCap.open(inputVideoFilename);
        }
        
        //check video is open
        if (!videoCap.isOpened()) {
            cerr<<"Could not open video"<<endl;
            return -1;

        }

        //read first image to get the dimensions
        videoCap >> inputImg;


        //read camera parameters if passed
        if (intrinsicFilename!="") {
            camParams.readFromXMLFile(intrinsicFilename);
            camParams.resize(inputImg.size());
        }

        //Create gui
        //cv::namedWindow("thres",1);
        cv::namedWindow("in",1);
        
        boardDetect.setParams(boardCfg,camParams,markerSize);

        boardDetect.getMarkerDetector().getThresholdParams(thresh1,thresh2);
        //boardDetect.getMarkerDetector().setCornerRefinementMethod(MarkerDetector::SUBPIX);
        //boardDetect.getMarkerDetector().enableErosion(true);//for chessboards
        
        iThresh1=thresh1;
        iThresh2=thresh2;
        cv::createTrackbar("thresh1", "in", &iThresh1, 13, cvTackBarEvents);
        cv::createTrackbar("thresh2", "in", &iThresh2, 13, cvTackBarEvents);
        char key=0;
        int index=0;
        double missTicks = 0;
        double totalTicks = 0;
        
        vector<int> searchIds;
        boardCfg.getIdList(searchIds);
   
        std::set<int> foundSet;
        bool keyTerminate = false;
        
        //capture until press ESC or until the end of the video
        do {
            videoCap.retrieve(inputImg);
            inputImg.copyTo(outImg);
            index++; //number of images captured
            double tick = (double)getTickCount();//for checking the speed
            //Detection of the board
            float probDetect = boardDetect.detect(inputImg);
            //calc mean speed of all iterations
            double elapsedTicks = ((double)getTickCount()-tick)/getTickFrequency();
            totalTicks += elapsedTicks;
            avgTime.first += elapsedTicks;
            avgTime.second++;

            cout << ".";
            
            
            //drawMarkers(searchIds,outImg);
            
            cv::Scalar borderColor = CV_RGB(128,255,0);
            vector<Marker>& detectedMarkers = boardDetect.getDetectedMarkers();
            //print marker borders
            for (unsigned int i=0;i < detectedMarkers.size();i++) {
                Marker curMarker = detectedMarkers[i];
                if (curMarker.isValid()) {
                    foundSet.insert(curMarker.id);
                    cout << curMarker.id << endl;
                    curMarker.draw(outImg,borderColor,1);
                }
            }

            //print board
             if (camParams.isValid()) {
                if ( probDetect > 0.1)   {
                    Board dboard = boardDetect.getDetectedBoard();
                    
                    CvDrawingUtils::draw3dAxis(outImg, dboard, camParams);
                    
                   
                }
                else {
                    missTicks += elapsedTicks; //miss!
                }
            }

            //show input with augmented information and  the thresholded image
            cv::imshow("in",outImg);
            //cv::imshow("thres",boardDetect.getMarkerDetector().getThresholdedImage());
            

            key = cv::waitKey(waitTime);//wait for key to be pressed
            if (key != '\xff')
                keyTerminate = processKey(key);
        }
        while ( !keyTerminate && videoCap.grab());
        
        cout << endl << "Miss percent: " << (missTicks / totalTicks) << endl;
        //cout<< endl << "Average detect time: "<<1000*avgTime.first/avgTime.second<<"ms"<<endl;
        
        cout << "Found IDs: " ;
        std::for_each(foundSet.begin(), foundSet.end(),dumpIntItem);
        cout << endl;
        
        
    } catch (std::exception &ex)

    {
        cout<<"Exception :"<< ex.what() << endl;
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
    if (iThresh1 < 3)
        iThresh1 = 3;
    
    if (iThresh1 % 2 != 1)
        iThresh1++;
    
    if (iThresh2 < 1)
        iThresh2 = 1;
    
    thresh1 = (float)iThresh1;
    thresh2 = (float)iThresh2;
    
    boardDetect.getMarkerDetector().setThresholdParams(thresh1,thresh2);
    

//    float probDetect = boardDetect.detect(inputImg);
//    inputImg.copyTo(outImg);
//    if (camParams.isValid() && probDetect>0.2)
//        aruco::CvDrawingUtils::draw3dAxis(outImg,boardDetect.getDetectedBoard(),camParams);

    
    cv::imshow("in",outImg);
    //cv::imshow("in",boardDetect.getMarkerDetector().getThresholdedImage());
}



