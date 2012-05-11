/*****************************************************************************
** STAIR VISION LIBRARY
** Copyright (c) 2007, Stanford University
**
** FILENAME:    svlSIFT.h
** AUTHOR(S):   Siddharth Batra <sidbatra@stanford.edu>
** DESCRIPTION: Encapsulates the functionality of David Lowe's SIFT descriptor
**
*****************************************************************************/

//Include only once
#pragma once

#include <stdio.h>
#include <cstdlib>
#include <cassert>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <stdlib.h>
#include <math.h>
#include <algorithm>
#include <assert.h>
#include <string>
#include <limits>
#include <vector>


//Import OpenCV headers
#include "cv.h"
#include "cxcore.h"
#include "highgui.h"

//Import custom headers
#include "constants.h"
#include "svlPoint2d.h"
#include "lowe_defs.h"
#include "utilities.h"
#include "visionUtilities.h"

using namespace std;

class svlSIFT
{

//Private data members
private:
IplImage *_imageOne;
IplImage *_imageTwo;
IplImage *_combinedImage;
Keypoint _imageOneFeatures;
Keypoint _imageTwoFeatures;
vector<vector<svlPoint2d> > _matchPairs;
vector<double> _actualDistances;
double _actualDistanceMedian;
vector<double> _imageDistances;
vector<double> _featureSpaceRatios;


//static members
public:
static int _norepeats;

//Constructor Logic
public:
svlSIFT();



//Public methods
public:

int matchImagesFast(IplImage* imageOne,IplImage* imageTwo , double threshold,CvMat *previousH
                    , svlPoint2d centreOfRegion);
vector<CvMat*> sampleSIFTFeaturesForAffineEstimate(int *totalMatches ,CvMat *previousH , int typeIndex, int ratioIndex );
int getTotalSIFTFeatures(IplImage *image);

Image convertToImageObject(IplImage *inputImage);
IplImage* convertToIplImage(Image imageObject, IplImage *inputImage);

int distSquared(Keypoint k1, Keypoint k2);
Keypoint checkForMatch(Keypoint key, Keypoint klist , double threshold , double *ratio );
vector<vector<svlPoint2d> > findMatches(Keypoint keys1, Keypoint keys2 , double threshold,vector<double> &ratios);

//Private methods
private:
void  combineImagesVertically(IplImage* imageOne , IplImage* imageTwo);
void  combineImagesHorizontally(IplImage* imageOne , IplImage* imageTwo);

};

