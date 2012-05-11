/*****************************************************************************
** MOVIE PROJECT
** Copyright (c) 2008 
**
** FILENAME:    frameInstance.cpp
** AUTHOR(S):   Sid Batra <sidbatra@cs.stanford.edu>
** DESCRIPTION:
** Saves the noisy location estimates and the frame images at a current time slice
*****************************************************************************/

#include "frameInstance.h"


//****************** Constructor Logic ***********************

frameInstance:: frameInstance(IplImage *icolorFrame ,   IplImage *icleanImage ,  CvMat *icleanTransform
        ,CvScalar imodelVariances , vector<svlPoint2d> ipolygons ,vector<svlPoint2d> iaffinePolygons
        , vector<int> istartingFrameNumbers , string ioutputFilename , int icontentFrameIndex)
{
    colorFrame = cvCloneImage(icolorFrame);

    if( icleanImage != NULL )
        _cleanImage = cvCloneImage(icleanImage);    
    else
        _cleanImage = NULL;

    if( icleanTransform != NULL )
        _cleanTransform = cvCloneMat(icleanTransform);
    else
        _cleanTransform = NULL;

    _modelVariances = imodelVariances;

    polygons = ipolygons;
    affinePolygons = iaffinePolygons;
    startingFrameNumbers = istartingFrameNumbers;
    outputFilename = ioutputFilename;
    contentFrameIndex =  icontentFrameIndex;
   
}



//****************** Methods ***********************

//Cleans up resources which are not freed when a vector entry is deleted
void frameInstance::clean()
{    
    cvReleaseImage(&colorFrame);

    if( _cleanImage != NULL )
        cvReleaseImage(&_cleanImage);

    if( _cleanTransform != NULL )
        cvReleaseMat(&_cleanTransform);

    polygons.clear();
    affinePolygons.clear();
    startingFrameNumbers.clear();
    
}

