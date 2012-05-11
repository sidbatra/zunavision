/*****************************************************************************
** MOVIE PROJECT
** Copyright (c) 2008 
**
** FILENAME:    frameInstance.h
** AUTHOR(S):   Sid Batra <sidbatra@cs.stanford.edu>
** DESCRIPTION:
** Saves the noisy location estimates and the frame images at a current time slice
*****************************************************************************/

#pragma once

#include <stdio.h>
//#include <tchar.h>
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

#include "svlPoint2d.h"
#include "utilities.h"

#include <sys/types.h>
#if defined(_WIN32)||defined(WIN32)||defined(__WIN32__)
#include "dirent.h"
#else
#include <dirent.h>
#endif

#include "objectTracker.h"


class frameInstance
{

public:
    IplImage *colorFrame;
    IplImage *_cleanImage;
    
    CvMat *_cleanTransform;
    CvScalar _modelVariances;
    vector<svlPoint2d> polygons;
    vector<svlPoint2d> affinePolygons;
    vector< int > startingFrameNumbers;
    string outputFilename;
    int contentFrameIndex;


public:
    frameInstance(IplImage *icolorFrame ,   IplImage *icleanImage ,  CvMat *icleanTransform
        ,CvScalar imodelVariances , vector<svlPoint2d>  ipolygons ,vector<svlPoint2d> iaffinePolygons
        , vector<int> istartingFrameNumbers , string ioutputFilename , int icontentFrameIndex );
    void clean();

private:
};
