/*****************************************************************************
** MOVIE PROJECT
** Copyright (c) 2008 
**
** FILENAME:    visionUtilities.h
** AUTHOR(S):   Sid Batra <sidbatra@cs.stanford.edu>
** DESCRIPTION:
** Combination of various misc vision utilities to be removed from objectTracker.h

*****************************************************************************/
#pragma once

#include <cstdlib>
#include <cassert>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <math.h>
#include <assert.h>
#include <string>
#include <vector>
#include <sstream>
#include <map>

#include "cv.h"
#include "cxcore.h"
#include "highgui.h"

#include "utilities.h"
#include "svlPoint2d.h"
#include "constants.h"

using namespace std;

class visionUtilities
{
	public:

        //Checks whether the given polygon lies within the image or not
        static int checkOutofBounds(IplImage *image , vector<svlPoint2d> polygon);
        static int checkOutofBounds(IplImage *image , vector<svlPoint2d> polygon , int margin);
        static int checkOutofBounds(IplImage *image , vector<svlPoint2d> polygon , CvMat *H);

		static int estimateCutScene(IplImage *one , IplImage *two);
        
        //Computes Mahalnobis distance
        static double computeMahalnobis(CvScalar pixelMean , CvScalar pixelVariance , CvScalar pixelValue);
        
        //Performs the opeation H * X .. where H is the homography matrix and X = [ x y z ] the coordinate vector
        //where z is assumed to be 1 and the nprojects the new x y coordinates via xNew = xNew / zNew &  yNew = yNew / zNew
        static void projectCoordinate(CvMat *homography , double *x , double *y);
        
        //Fast method of retriving the values of a color image
        static CvScalar getRGB( IplImage *image , int *x , int *y );
        
        //Computes the connected components in an image and removes the ones below a certain size 
        static void removeSmallComponents( IplImage *occlusionModel , double areaThershold, bool invert , bool keepEdgeComponents);

        static vector<svlPoint2d> findPointsForArrow( vector<svlPoint2d> polygon , IplImage *image , int direction);
        static int findDirectionForArrow( vector<svlPoint2d> polygon , IplImage *image );
        static void drawArrow(IplImage *image, vector<svlPoint2d> points);

};
