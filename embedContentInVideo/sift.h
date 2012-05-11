/*****************************************************************************
** ZunaVision
** Copyright (c) 2008 
**
** FILENAME:    sift.h
** AUTHOR(S):   Sid Batra <sidbatra@cs.stanford.edu>
** DESCRIPTION:
** Manages computation matching and sampling of SIFT features
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

#include "video.h"
#include "SIFT/lowe_defs.h"

using namespace std;

class sift
{
private:
    vector<double> _actualDistances;        //Distances between SIFT features in the image plane
    vector<double> _imageDistances;         //Distances of SIFT features from the centre of the region being tracked
    vector<double> _featureSpaceRatios;     //Metric for quality of the SIFT features
    double _actualDistanceMedian;           //Median of the _actualDistances

public:
    sift();         //Default constructor

    int computeMetricsForSampling( vector< vector< vector< svlPoint2d > > > &correspondences , contentTracks &tracker);   //Populates various metrics for sampling sift features
    CvMat* sampleSIFTFeatures(vector<vector<vector<svlPoint2d> > > &correspondences, contentTracks &currentTracks ,int siftIndex , int sampleIndex);//Samples SIFT features based upon different correspondence metrics

    void computeSIFTFeatures(frame &cFrame);      //Computes the SIFT features for the given frame
    vector< vector<vector<svlPoint2d> > > computeCorrespondences( vector<frame> &frames); //Computes correspondences of the central frame to all the others
    vector<vector<svlPoint2d> > findMatches(Keypoint keys1, Keypoint keys2 , double threshold , vector<double> &ratios); //Computes the correspondences between two frames
    Keypoint checkForMatch(Keypoint key, Keypoint klist , double threshold , double *ratio); //Checks a euclidean distance based metric to find the best match for the given keypoint
    int distSquared(Keypoint k1, Keypoint k2);    //Return squared distance in feature space between two keypoint descriptors

    Image convertToImageObject(IplImage *inputImage);     //Convert the image from an openCV image object to the Image object required by Lowe's implementation
};

