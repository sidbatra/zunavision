/*****************************************************************************
** MOVIE PROJECT
** Copyright (c) 2008 
**
** FILENAME:    constants.h
** AUTHOR(S):   Sid Batra <sidbatra@cs.stanford.edu>
** DESCRIPTION:
** Holds the various constants and parameters used in the application

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


using namespace std;

class constants
{

    public:
		static double CUT_SCENE_THRESHOLD; //Threshold for labeling a frame as a cut scene
        static int SMOOTHING_MASK_SIZE; //Mask size of the gaussian blur used for smoothing
        static int MARGIN; //Margin for checking for points that lie outside the image
        static double TRACTABLE_OCCLUSION_THRESHOLD; //Threshold for % of occlusion points 
                                                                 //after which optimization is skipped
        
        static double VARIANCE_REGULARIZATION[3]; //Regularizations for the variance in outlier detection for occlusion
        static double OUTLIER_THRESHOLD;

        //OPTIMIZATION
        static double OPTIMIZATION_REGULARIZATION;
        static int MAX_ITERATIONS;
        static int ITERATIONS_FOR_PF; //Iteration for which optimization is run at each level while particle filtering is being done
        static double DELTA_HJ;
        static double TOLERANCE;
        static double INITIAL_ALPHA;

        //PARTICLES
        static int TOTAL_PARTICLES;
        static int SIFT_PARTICLES;
        static int SIFT_SAMPLE_RATIOS;
        static int TEST_SIFT_INDEX;
        static int TEST_SAMPLE_INDEX;
        static double IDENTITY[9];
        static int IDENTITY_PARTICLE;
        static double OCC_PERCENTAGE_FOR_IDENTITY_PARTICLE;
        static double OCC_PERCENTAGE_FOR_PARTICLE;

        //SIFT
        static double THRESHOLD_FOR_REPEAT_FRAME ;
        static int MINIMUM_SIFT_FEATURES ;

        //CONNECTED COMPONENTS
        static double CC_AREA_THRESHOLD ;

        //ARROW DRAWING
        static int ARROW_GAP_FROM_POLYGON;
        static int ARROW_LENGTH;
        static CvScalar ARROW_COLOR;
        static int ARROW_THICKNESS;
        static double RATIO_OF_ARROW_AHEADS ;
        static double ANGLE_OF_ARROW_AHEADS ;
        static int ARROW_DISPLAY_LENGTH;

        static double PI;

        //RENDERING
        static int EFFECT_LENGTH;

        
};
