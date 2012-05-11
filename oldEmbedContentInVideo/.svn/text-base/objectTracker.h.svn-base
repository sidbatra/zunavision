/*****************************************************************************
** MOVIE PROJECT
** Copyright (c) 2008 
**
** FILENAME:    objectTracker.h
** AUTHOR(S):   Sid Batra <sidbatra@cs.stanford.edu>
** DESCRIPTION:
** Computes estimates of the location of the region being tracked based on a temporal model of its appearance
*****************************************************************************/

#pragma once

#include <stdio.h>
#include <cstdlib>
#include <cassert>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <stdlib.h>
#include <math.h>
#include <assert.h>
#include <string>
#include <vector>

#include "cv.h"
#include "cxcore.h"
#include "highgui.h"

#include <sys/types.h>
#if defined(_WIN32)||defined(WIN32)||defined(__WIN32__)
#include "dirent.h"
#else
#include <dirent.h>
#endif


#include "svlSIFT.h"
#include "utilities.h"
#include "visionUtilities.h"
#include "constants.h"


class objectTracker
{

//Data members
private:

svlSIFT _siftObject;
vector<svlPoint2d> _regionPixels;
vector<vector<double> > _regionPixelMeans;
vector<vector<double> > _regionPixelVariances;

IplImage *_IDashOccColor;

vector<CvScalar> _colorPixelMeans;
vector<CvScalar> _colorPixelVariances;

int _currentFrame;
int _currentParticle;
int _NCCOcclusionMode;


IplImage *_preCutSceneFrame;
IplImage *_preCutSceneFrameColor;

vector<int> _isOccluded;
vector<svlPoint2d> _occludedPoints;
IplImage* _cleanImage;
CvMat* _cleanTransform;
int _occlusionMode;
int _previousOcclusionMode;
int _unOccludedPixels;
CvScalar _tempModelVariances;

CvMat* _previousH;

double _pixelModelSigmaNCC ;
vector<double> _pixelModelSigmasNCC;


//Constructor Logic
public:
objectTracker();

static int _selectedParticle;

//#DEBUG#
//static string _tempOut;
//#DEBUG#

//Methods
public:

int trackObjectsWithHomography( IplImage* imageOne, IplImage* imageTwo ,  vector<svlPoint2d> &objects , vector<svlPoint2d> &affineObjects ,IplImage* previousFrame, IplImage* currentFrame);
svlPoint2d findMappingInPolygon(vector<svlPoint2d> polygon ,vector<svlPoint2d> targetPolygon ,svlPoint2d p);
int inPolygon(vector<svlPoint2d> E , svlPoint2d p);
double shortestDistanceToEdge(vector<svlPoint2d> polygon, svlPoint2d p);
void findMinMax(vector<svlPoint2d> objects,int margin,double &minX,double &minY ,double &maxX ,double &maxY);
int isOccluded(IplImage * occlusionModel ,int row , int col);

void freeMemory();

double subPixelValueFloat( IplImage *I , svlPoint2d Pi);
inline double subPixelValue( IplImage *I , double &PiX , double &PiY);
inline CvScalar subPixelValueRGB( IplImage *I , double &PiX , double &PiY);

IplImage* finalizeOccludedPixels(CvMat *H,IplImage *IDashColor , IplImage *cleanImage , CvScalar modelVariances, vector<svlPoint2d> polygon , double margin 
                                           , double subSampleRatio , double stepSizeX , double stepSizeY);
void computeOcclusionMetrics(CvMat *H);

int estimateCutScene(IplImage *two);
void recoverToPreCutScene(IplImage *previousImage , IplImage *previousFrame);
void savePreCutScene(IplImage *previousFrame , IplImage *previousFrameColor);

//Data retrieve functioins
CvMat *getCleanTransform();
IplImage* getCleanImage();
CvScalar getModelVariances();
IplImage *getPreCutColor();
IplImage *getPreCut();

private:

int isOutlier(CvScalar pixelMean , CvScalar pixelVariance , CvScalar pixelValue);
inline int isOutlier(int pixelIndex , CvScalar pixelValue);


CvMat* formAffineMatrix(vector<CvMat*> matches, int totalMatches);
CvMat* estimateProjectiveTransform(CvMat *H ,  IplImage *one , IplImage *two , vector<svlPoint2d> objects , IplImage *previousFrame , IplImage *currentFrame, int isFirstParticle , double *J_referenc , int constrainIterations);
void finalizeOptimizationStats(CvMat *H , IplImage *imageTwo , IplImage *previousFrame ,IplImage *currentFrame);
double J(IplImage *IDash, CvMat *H,int index);
double delJ(IplImage *IDash, CvMat *H, int y , int x , int index);

int estimateOccludedPixels(CvMat *A,IplImage *IDashColor);

void computeEdgePixels(IplImage *I , IplImage * IColor, vector<svlPoint2d> polygon , int visualize);

void populateModelNCCStats(int index);
double NCC(IplImage *IDash , CvMat *H  , int index);
void updatePixelModel(IplImage *IDash ,IplImage *IDashColor, int index , int update );
inline void projectCoordinate(CvMat *homography , double *x , double *y);
};


//Checks whether the given pixel value is an outlier based on the gaussian model of that pixel
inline int objectTracker::isOutlier(int pixelIndex , CvScalar pixelValue)
{

    double result = pow( pixelValue.val[0] - _colorPixelMeans[pixelIndex].val[0] , 2 ) / ( _colorPixelVariances[pixelIndex].val[0] + constants::VARIANCE_REGULARIZATION[0] ) +
        pow( pixelValue.val[1] - _colorPixelMeans[pixelIndex].val[1] , 2 )/ ( _colorPixelVariances[pixelIndex].val[1] + constants::VARIANCE_REGULARIZATION[1] ) +
        pow( pixelValue.val[2] - _colorPixelMeans[pixelIndex].val[2] , 2 ) / ( _colorPixelVariances[pixelIndex].val[2] + constants::VARIANCE_REGULARIZATION[2] );
    
    return  (result >= constants::OUTLIER_THRESHOLD);
}


//Computes subpixel values in the image
inline double objectTracker::subPixelValue( IplImage *I , double &PiX , double &PiY)
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
inline CvScalar objectTracker::subPixelValueRGB( IplImage *I , double &PiX , double &PiY)
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
inline void objectTracker::projectCoordinate(CvMat *homography , double *x , double *y)
{
	double xOld = *x;
	double yOld = *y;
	double zOld = 1.0;
	double z = 0.0;
    	
    *x = xOld * CV_MAT_ELEM(*homography,double,0,0) + yOld * CV_MAT_ELEM(*homography,double,0,1) + zOld * CV_MAT_ELEM(*homography,double,0,2);
	*y = xOld * CV_MAT_ELEM(*homography,double,1,0) + yOld * CV_MAT_ELEM(*homography,double,1,1) + zOld * CV_MAT_ELEM(*homography,double,1,2);
	z = xOld * CV_MAT_ELEM(*homography,double,2,0) + yOld * CV_MAT_ELEM(*homography,double,2,1) + zOld * CV_MAT_ELEM(*homography,double,2,2);
    
	(*x) = (*x) / (z);
	(*y) = (*y) / (z);	
}

