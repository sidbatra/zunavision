/*****************************************************************************
** MOVIE PROJECT
** Copyright (c) 2008 
**
** FILENAME:    embedCentral.h
** AUTHOR(S):   Sid Batra <sidbatra@cs.stanford.edu>
** DESCRIPTION:
** Handles the outline of the code for tracking and has the code for the various blending
** techniques used
*****************************************************************************/


//C++ headers

#include <stdio.h>
#include <cstdlib>
#include <cassert>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <string>
#include <vector>


//OpenCV headers

#include "cv.h"
#include "cxcore.h"
#include "highgui.h"

//External headers

#include "svlPoint2d.h"

#include <sys/types.h>
#if defined(_WIN32)||defined(WIN32)||defined(__WIN32__)
#include "dirent.h"
#else
#include <dirent.h>
#endif

//Custom headers

#include "utilities.h"

#include "objectTracker.h"
#include "frameInstance.h"


#define COLOR_MODEL_SIZE 3

//Enumerations to make selection of command line options cleaner
enum ContentType { LOGO , REAL_LIFE , REAL_LIFE_WIDE , SPECIAL_LOGO , ON_TEXTURE , LOGO_WITH_EFFECT};
enum OutputType { CONTENT , RECTANGLE , RECT_AFFINE };


class embedCentral
{

//Data members
vector<objectTracker> _objectTrackers;
int _currentTrackerIndex;
vector<int> _framesPerTracker;
vector<int> _arrowDirection;

bool _isReverseTracking;

double _videoMeans[COLOR_MODEL_SIZE];
double _videoVariances[COLOR_MODEL_SIZE];

vector< frameInstance > _frameInstances;
vector< vector< string > > _contentFilenames;
vector< int > _trackLength;

//Constructor Logic
public:
embedCentral();

//Methods
public:
void embedContentAtLocations(string framesPath , string extension,vector<vector<svlPoint2d> > polygon 
                             , vector<int> startingFrameNumbers ,vector<int> endFrameNumbers ,
                             vector<vector<string> > content 
                             ,string outputFolderName , int skipFrames , int outputMode , vector<int> contentType
                             ,int firstFrameDebug , int smooth , int norepeats, bool isReverse);

private:


void estimateLocationFromNoisyEstimates(int videoFrameIndex, int smoothingSide, int outputMode , int contentType);
void smoothenEstimates( int index , int smoothingSide );

void populateSubPixelMatrix( vector<vector< svlPoint2d > > &pMatrix , unsigned x , unsigned y , vector< CvScalar > &contentPoints , CvScalar **subPixelMatrix , double **subPixelWeight , int startX , int startY);
void populateVideoColorStats(IplImage *frame);
void renderContent(IplImage *currentFrame, IplImage *cleanImage , CvMat *cleanTransform 
                                        , CvScalar modelVariances ,vector<int> startingFrameNumbers,int videoFrameIndex,  int contentFrameIndex , int outputMode , vector< svlPoint2d> polygons , vector< svlPoint2d> affinePoly , vector< int > startingFrameIndex  , string outputFilename , int contentType);
void renderContentToFrame(IplImage *frame , IplImage *content , IplImage *cleanImage , CvMat *cleanTransform 
                                        , CvScalar modelVariances ,vector<svlPoint2d> targetPolygon  , int contentType);


};
