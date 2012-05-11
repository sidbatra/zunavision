/*****************************************************************************
** ZunaVision
** Copyright (c) 2008 
**
** FILENAME:    contentTracker.h
** AUTHOR(S):   Sid Batra <sidbatra@cs.stanford.edu>
** DESCRIPTION:
** Computes homographies and tracks content regions between frames

*****************************************************************************/

#pragma once

#include "base.h"
#include "video.h"
#include "sift.h"

class contentTracker
{
private:
    sift _siftObject;           //Handles all SIFT feature related operations
    int _currentParticle;       //Holds the index of the particle currently being evaluated

public:
    contentTracker();           //Default constructor
    
    //Tracks the given content in the given video volume
    void trackContent(video &cVideo , unsigned volumeIndex , int contentIndex);      

    //Computes the best initial estimate of the homography using various particles
    CvMat* generateHomographyEstimate(vector<vector<vector<svlPoint2d> > > correspondences , unsigned volumeIndex , contentTracks &currentTracks , video &cVideo, int contentIndex); 

    //Run Newton's method to improve the estimation of the transform
    CvMat* estimateProjectiveTransform(video &cVideo , unsigned volumeIndex, CvMat *A, content &cContent , contentTracks &currentTracks, double &J_Reference , int constrainIterations);

    //Does required checks for the STILL_CAMERA tracking type
    CvMat* estimateIdentityTransform(video &cVideo , unsigned volumeIndex, CvMat *A, content &cContent , contentTracks &currentTracks, double &J_Reference , int constrainIterations);

    //Wraps up the optimization and updates the models and other data structures
    void finalizeTracking(video &cVideo , unsigned volumeIndex, contentTracks &currentTracks,contentTracks &nextTracks,content &cContent);

    //Computes the value of the objective function
    double J(IplImage *IDash , IplImage *IDashColor , CvMat *H , unsigned index  , content &cContent , contentTracks &cTracks); 

    //Computes the value of the differential objective function
    double delJ(IplImage *IDash, IplImage *IDashColor, CvMat *H, int y , int x ,unsigned index , content &cContent , contentTracks &cTracks);

    //Computes the normalized cross correlation of the given gray image with the model
    double NCC(IplImage *IDash, IplImage* IDashColor, CvMat *H, unsigned index,content &cContent);   

    //Estimates which pixels may be occluded in the model based upon initial transform estimates
    bool estimateOccludedPixels(content &cContent, CvMat *A , IplImage *IDashColor);

    //Runs a restricted optimization to evaluate the given particle
    double evaluateParticle(video &cVideo , unsigned volumeIndex, CvMat *A, contentTracks &currentTracks, int contentIndex);    

    //Populates the pixel model for the given content
    void populateModel(vector<frame> &cFrames,content &cContent , contentTracks &currentTracks, int visualize); 


    //******* Faster inline versions of some visionUtilities functions

    //Projects the given coordinate based on the given transform
    inline void projectCoordinate(CvMat *homography , double &x , double &y);
    
    //Computes subpixel values in the RGB image
    inline CvScalar subPixelRGB( IplImage *I , double &PiX , double &PiY);
    
    //Computes subpixel values in the image
    inline double subPixel( IplImage *I , double &PiX , double &PiY);   
    
};


//*************************** Faster inline versions of some visionUtilites functions to be used only in NCC ****************



inline double contentTracker::subPixel( IplImage *I , double &PiX , double &PiY)
{
    int intPx = (int)PiX;
    int intPy = (int)PiY;
	
	double x1 = CV_IMAGE_ELEM(I,unsigned char,intPy,intPx) 
		+  (PiX - intPx) * ( CV_IMAGE_ELEM(I,unsigned char,intPy ,intPx + 1) - CV_IMAGE_ELEM(I,unsigned char,intPy,intPx) );

	double x2 = CV_IMAGE_ELEM(I,unsigned char,intPy+1 ,intPx) 
		+ (PiX - intPx) * ( CV_IMAGE_ELEM(I,unsigned char,intPy+1 ,intPx + 1) - CV_IMAGE_ELEM(I,unsigned char,intPy+1,intPx) );

	return x1 + (PiY - intPy) * (x2 - x1);
}



//Computes subpixel values in the RGB image
inline CvScalar contentTracker::subPixelRGB( IplImage *I , double &PiX , double &PiY)
{
   
    int intPx = (int)PiX;
    int intPy = (int)PiY;
    CvScalar result;

    uchar* left = &((uchar*)(I->imageData + I->widthStep*(intPy)))[(intPx)*3];
    uchar* right = &((uchar*)(I->imageData + I->widthStep*(intPy)))[(intPx+1)*3];
    uchar* bottomLeft = &((uchar*)(I->imageData + I->widthStep*(intPy+1)))[(intPx)*3];
    uchar* bottomRight = &((uchar*)(I->imageData + I->widthStep*(intPy+1)))[(intPx+1)*3];

	//Floating point of the coordinates
	double delX = PiX - intPx;
	double delY = PiY - intPy;

    double x1 = left[0] +  delX * ( right[0] - left[0] );
    double x2 = bottomLeft[0] + delX * ( bottomRight[0] - bottomLeft[0] );
    result.val[0] = x1 + delY * (x2 - x1);

    x1 = left[1] +  delX * ( right[1] - left[1] );
    x2 = bottomLeft[1] + delX * ( bottomRight[1] - bottomLeft[1] );
    result.val[1] = x1 + delY * (x2 - x1);
 
    x1 = left[2] +  delX * ( right[2] - left[2] );
    x2 = bottomLeft[2] + delX * ( bottomRight[2] - bottomLeft[2] );
    result.val[2] = x1 + delY * (x2 - x1);

	return result;
}


//Performs the opeation H * X .. where H is the homography matrix and X = [ x y z ] the coordinate vector
//where z is assumed to be 1 and the nprojects the new x y coordinates via xNew = xNew / zNew &  yNew = yNew / zNew
inline void contentTracker::projectCoordinate(CvMat *homography , double &x , double &y)
{
	double xOld = x;
	double yOld = y;
	double zOld = 1.0;
	double z = 0.0;
    	
    x = xOld * CV_MAT_ELEM(*homography,double,0,0) + yOld * CV_MAT_ELEM(*homography,double,0,1) + zOld * CV_MAT_ELEM(*homography,double,0,2);
	y = xOld * CV_MAT_ELEM(*homography,double,1,0) + yOld * CV_MAT_ELEM(*homography,double,1,1) + zOld * CV_MAT_ELEM(*homography,double,1,2);
	z = xOld * CV_MAT_ELEM(*homography,double,2,0) + yOld * CV_MAT_ELEM(*homography,double,2,1) + zOld * CV_MAT_ELEM(*homography,double,2,2);
    
	x = x / z;
	y = y / z;	
}
