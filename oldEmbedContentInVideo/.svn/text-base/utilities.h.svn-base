/*****************************************************************************
** MOVIE PROJECT
** Copyright (c) 2008 
**
** FILENAME:    utilities.h
** AUTHOR(S):   Sid Batra <sidbatra@cs.stanford.edu>
** DESCRIPTION:
** A set of static methods which are used throughout the code as utilities
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
#include <algorithm>
#include <vector>
#include <sstream>
#include <map>

#include <sys/types.h>
#if defined(_WIN32)||defined(WIN32)||defined(__WIN32__)
#include "dirent.h"
#else
#include <dirent.h>
#endif

#include "cv.h"
#include "cxcore.h"
#include "highgui.h"

#include "svlPoint2d.h"

using namespace std;

class utilities
{
	public:
		static std::string toString(int v);
		static int roundOff(double d); //Rounds of floating point to integer
		static void display(IplImage* image,std::string text = "Default" );
        static void projectCoordinate(CvMat *homography , double *x , double *y);
        static IplImage* edgeMap(IplImage *I);
        static void loadAllFilenames(string folderName , string extension , vector<string> &frameNames,bool isReverse = false);
        static void resizeInPlace(IplImage **image, double scale);
        static void resizeInPlace(IplImage **image, int height, int width);
        static double squareDistance( svlPoint2d a , svlPoint2d b);
};

