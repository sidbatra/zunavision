/*****************************************************************************
** MOVIE PROJECT
** Copyright (c) 2008 
**
** FILENAME:    constants.cpp
** AUTHOR(S):   Sid Batra <sidbatra@cs.stanford.edu>
** DESCRIPTION:
** Values of the constants and parameters in constants.h

*****************************************************************************/

#include "constants.h"


double constants::CUT_SCENE_THRESHOLD = 0.09;//640; //Threshold for labeling a frame as a cut scene

int constants::SMOOTHING_MASK_SIZE = 3; //Mask size of the gaussian blur used for smoothing

int constants::MARGIN = 2; //Margin for checking for points that lie outside the image



double constants::OPTIMIZATION_REGULARIZATION = 1.0e-6; //Regulaization weight for velocity for the H parameters matrix

double constants::PI = 3.14159265358979323846;

//*********** Rendering ************
int constants::EFFECT_LENGTH = 20;

//*********** Optimization *********
int constants::MAX_ITERATIONS = 500; //Maximum iterations for convergence
int constants::ITERATIONS_FOR_PF = 2; //Iteration for which optimization is run at each level while particle filtering is being done
double constants::DELTA_HJ = 1.0e-6;//3.9e-5;//1.0e-6;//1.0e-6;//1.0e-9 ; //Increment of H_j while computing numerical differentiation of J1
double constants::TOLERANCE = 0;//1.0e-12;//1.0e-6;//1.0e-15;//0.0 ;//Threshold for change in value of the objective function between iterations
double constants::INITIAL_ALPHA = 1; //Intial value of the learning rate


//************* Occlusion outlier detection **************

double constants::VARIANCE_REGULARIZATION[3] = {55,4.5,4.5}; //Regularizations for the variance in outlier detection for occlusion
double constants::OUTLIER_THRESHOLD = 9; //Base threshold for outlier detection based on mahalnobis distance
double constants::OCC_PERCENTAGE_FOR_IDENTITY_PARTICLE = 0.09; //Index of the identity particle 
double constants::OCC_PERCENTAGE_FOR_PARTICLE = 0.00000005; //Index of the identity particle 
double constants::TRACTABLE_OCCLUSION_THRESHOLD = .01; //Threshold for % of occlusion points 
                                                                 //after which optimization is skipped

//************* Particles *********
int constants::TOTAL_PARTICLES = 15;
int constants::TEST_SIFT_INDEX = 0;
int constants::TEST_SAMPLE_INDEX = 0;
double constants::IDENTITY[9] = {1,0,0, 0,1,0, 0,0,1};
int constants::SIFT_PARTICLES = 6;
//int constants::SIFT_SAMPLE_RATIOS = 2;
int constants::IDENTITY_PARTICLE = 13; //Index of the identity particle 

//************ SIFT **********
double constants::THRESHOLD_FOR_REPEAT_FRAME = 0.09;
int constants::MINIMUM_SIFT_FEATURES = 11;

//************ Connection component *************
double constants::CC_AREA_THRESHOLD = 0.03;


//************* Arrow Drawing ******************

int constants::ARROW_GAP_FROM_POLYGON = 10;
int constants::ARROW_LENGTH = 75;
CvScalar constants::ARROW_COLOR = cvScalar(50,255,0);
int constants::ARROW_THICKNESS = 2;
double constants::RATIO_OF_ARROW_AHEADS = 3;
double constants::ANGLE_OF_ARROW_AHEADS = 1.2;
int constants::ARROW_DISPLAY_LENGTH = 20;
