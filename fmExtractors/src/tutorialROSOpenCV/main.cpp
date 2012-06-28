#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include "sensor_msgs/Image.h"
#include "cv_bridge/CvBridge.h"
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <unistd.h>
#include "math.h"
#include "opencv2/highgui/highgui.hpp"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include "std_msgs/Int8.h"

//Store all constants for image encodings in the enc namespace to be used later.
namespace enc = sensor_msgs::image_encodings;
using namespace cv;
using namespace std;


//Use method of ImageTransport to create image publisher
image_transport::Publisher pub;
ros::Publisher position_pub;
ros::Publisher marker_pub;
int count1 = 0;
int count2 = 0;

IplImage* HSVToTerraCotta( IplImage* imgSrc, IplImage* imgDst)
{
	// Allocate variables
	int tmp_pix;
	int y,x;
	uchar * _SrcPtr, * _DstPtr;

	// Iterate over the image line by line
	for(y = /*imgSrc->height*0.8*/ 0 ; y < imgSrc->height ; y++ )
	  {
	    // Locate pointers to the first data element in the current line
	    _SrcPtr = ( uchar* )( imgSrc->imageData + y * imgSrc->widthStep );
	    _DstPtr = ( uchar* )( imgDst->imageData + y * imgDst->widthStep );

	    // Iterate over the elements in the current line
	    for(x = 0 ; x < imgSrc->width ; x++ )
	    	{

	    				int bB =  _SrcPtr[3*x+0];	// Blue component
	    				int bG =  _SrcPtr[3*x+1];	// Green component
	    				int bR =  _SrcPtr[3*x+2];	// Red component

	    				int iMax, iMin;
	    				if (bB < bG)
	    				{
	    					if (bB < bR)
	    					{
	    						iMin = bB;
	    						if (bR > bG)
	    						{
	    							iMax = bR;
	    						}
	    						else
	    						{
	    							iMax = bG;
	    						}
	    					}
	    					else
	    					{
	    						iMin = bR;
	    						iMax = bG;
	    					}
	    				}
	    				else
	    				{
	    					if (bG < bR)
	    					{
	    						iMin = bG;
	    						if (bB > bR)
	    						{
	    							iMax = bB;
	    						}
	    						else
	    						{
	    							iMax = bR;
	    						}
	    					}
	    					else
	    					{
	    						iMin = bR;
	    						iMax = bB;
	    					}
	    				}

	    				float iDelta = iMax - iMin;
	    							int bS;
	    							int	bV = iMax;
	    							float fH;
	    							if (iMax != 0) {			// Make sure its not pure black.
	    								bS = (int)(0.5f + (iDelta / iMax) * 255.0f);	// Saturation.
	    									float ANGLE_TO_UNIT = 1.0f / (6.0f * iDelta);	// Make the Hues between 0.0 to 1.0 instead of 6.0
	    									if (iMax == bR) {		// between yellow and magenta.
	    										fH = (bG - bB) * ANGLE_TO_UNIT;
	    									}
	    									else if (iMax == bG) {		// between cyan and yellow.
	    										fH = (2.0f/6.0f) + ( bB - bR ) * ANGLE_TO_UNIT;
	    									}
	    									else {				// between magenta and cyan.
	    										fH = (4.0f/6.0f) + ( bR - bG ) * ANGLE_TO_UNIT;
	    									}
	    									// Wrap outlier Hues around the circle.
	    									if (fH < 0.0f)
	    										fH += 1.0f;
	    									if (fH >= 1.0f)
	    										fH -= 1.0f;
	    								}
	    								else
	    								{
	    									fH = 0;
	    								}

	    							int bH = (int)(0.5f + fH * 255.0f);

	    										if (bH > 255)
	    											bH = 255;
	    										if (bH < 0)
	    											bH = 0;
	    										if (bS > 255)
	    											bS = 255;
	    										if (bS < 0)
	    											bS = 0;
	    										if (bV > 255)
	    											bV = 255;
	    										if (bV < 0)
	    											bV = 0;




	    	    if (bS > 40 && bS < 256 && bH > 140 && bH < 190 && bV > 60 && bV < 256 )
	            {
	            	tmp_pix = 255;
			count1++;
	            }
		    else if (bS > 20 && bS < 130 && bH > 30 && bH < 70 && bV > 150 && bV < 256 )
	            {
	  	    	tmp_pix = 255;
			count2++;
		    }
	            else
	            {
	            	tmp_pix = 0;
	            }
	    		    _DstPtr[x] = tmp_pix;

	        }
	  }
		  	return imgDst;
}


//This function is called everytime a new image is published
void imageCallback(const sensor_msgs::ImageConstPtr& original_image)
{

	sensor_msgs::CvBridge bridge;

    	IplImage* cv_image;
  	     try
  	     {
  	        cv_image = bridge.imgMsgToCv(original_image, "bgr8");
  	     }
  	     catch (sensor_msgs::CvBridgeException& error)
  	     {
  	       ROS_ERROR("error: %s", error.what());
  	       return;
  	     }

	     IplImage* cv_image_filtered = NULL;
	 
	    
	    cv_image_filtered = cvCreateImage(cvGetSize(cv_image), 8, 1);
	    

	
	HSVToTerraCotta(cv_image, cv_image_filtered);

	ROS_INFO("Count 1 er %d", count1);
	ROS_INFO("Count 2 er %d", count2);

	if (count1 > 2000 && count2 > 2000){
		std_msgs::Int8 msg;
		msg.data = 1;
		marker_pub.publish(msg);
	}

	count1 = 0;
	count2 = 0;
	
	
/*	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	
	CvMemStorage* 	g_storage = NULL;

	if( g_storage == NULL ){
		g_storage = cvCreateMemStorage(0);
	} else {
		cvClearMemStorage( g_storage );
	}

	//CvSeq* contours = 0;
	
	cvFindContours(threshold_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0,0));
//	int n = cvFindContours( cv_image_filtered, g_storage, &contours, sizeof(CvContour),CV_RETR_EXTERNAL),CV_CHAIN_APPROX_SIMPLE;
//	cvZero( g_gray );
	
	vector<vector<Point> > contours.poly(contours.size());
	vector<Rect> boundRect(contours.size());
	vector<Point2f>center(contours.size());
	vector<float>radius (contours.size());

	for(int i = 0; i < contours.size(); i++)
	{
	boundRect[i] = boundingRect (Mat(contours_poly[i]));
	minEnclosingCircle(contours_poly[i], center[i],radius[i]);
	ROS_INFO("width er: %d og height er: %d", boundRect[i].width,boundRect[i].height);
		
	}	
	
	Mat drawing = Mat::zeros(threshold_output.size(), CV_8UC3 );
	
	for(int i = 0; i < contours.size(); i++)
	{
	circle(drawing, center[i], (int)radius[i], color,2,8,0);	
	}
	if( contours ){
		cvDrawContours(
			cv_image_filtered,
			contours,
			cvScalarAll(255),
			cvScalarAll(255),
			1 );
	}*/
//	ROS_INFO("Number of contours: %d", n);
//	cvShowImage("Image", cv_image_filtered);

//	cvShowImage("Image window", cv_image);
	

	
cvWaitKey(33);

/*	try
	     {
	       pub.publish(bridge.cvToImgMsg(cv_image, "bgr8"));
	     }
	     catch (sensor_msgs::CvBridgeException error)
	     {
	       ROS_ERROR("error");
	     }
*/

}


int main(int argc, char **argv)
{
	
        ros::init(argc, argv, "image_processor");

       
        ros::NodeHandle nh;

	
        image_transport::ImageTransport it(nh);
	
//    cv::namedWindow("Image", CV_WINDOW_AUTOSIZE);
 
//	cv::namedWindow("Image window", CV_WINDOW_AUTOSIZE);



//	  position_pub=nh.advertise<geometry_msgs::TwistStamped>("TwistStamped",1);


      image_transport::Subscriber sub = it.subscribe("camera/image_raw", 1, imageCallback);

      pub = it.advertise("camera/image_processed", 1);
      marker_pub = nh.advertise<std_msgs::Int8>("/marker_pub", 1);
	
//	 cv::destroyWindow("Image");
//         cv::destroyWindow("Image window");
         ros::spin();

 

}
