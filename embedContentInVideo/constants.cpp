/*****************************************************************************
** Zunavision
** Copyright (c) 2008 
**
** FILENAME:    constants.cpp
** AUTHOR(S):   Sid Batra <sidbatra@cs.stanford.edu>
** DESCRIPTION:
** Values of the constants and parameters in constants.h

*****************************************************************************/

#include "constants.h"

//***************** Misc **********************

double constants::PI = 3.14159265358979323846;
double constants::THRESHOLD_FOR_REPEAT_FRAME = 1.2;
double constants::THRESHOLD_FOR_BLANK_FRAME = 20;
double constants::THRESHOLD_FOR_CUT_FRAME = 100000;
double constants::THRESHOLD_FOR_SIMILAR_FRAME = 70000;
bool constants::VERBOSE = false;
CvSize constants::SPATIAL_IMAGE_SIZE = cvSize(10,10);

//************* Tracking ***************
unsigned constants::VOLUME_MARGIN = 1;   //Margin of frames in time to be used for tracking
int constants::MARGIN = 1; //Margin for checking for points that lie outside the image

//*********** Rendering ************
int constants::EFFECT_LENGTH = 20;
double constants::PREPROCESS_CONTENT_SCALEUP = 1.8;
double constants::CONTENT_MARGIN = 2;  
double constants::SUB_SAMPLE_STEPSIZE = 0.4;//0.2;  
double constants::CC_AREA_THRESHOLD = 0.03;
double constants::SUB_OCC_THRESHOLD = 253;       //Threshold for calling a sub pixel occluded in occlusion mask
string constants::FRAME_FILENAME = "..//images//frame.bmp";      //Filname of the frame template image
CvRect constants::FRAME_ROI = cvRect(14,14,349,262);           //ROI where the image is placed within a frame
CvSize constants::FRAME_BORDER = cvSize(13,13);       //Size of the border to be excluded from blending
int constants::OCC_EDGE_SMOOTH_SIDE = 5;   //Side of the gaussian kernel used for smoothing the edge map of the occ mask 
CvPoint constants::OCC_EDGE_CANNY_THRESHOLDS = cvPoint(50,250); // Thresholds to be used for computing the edge map of the occ mask
string constants::EXT_FOR_TRANSPARENT_IMAGES = ".png"; //Extension for transparent images
string constants::NAME_FOR_TRANSPARENT_IMAGES = "transparentdata.txt"; //Extension for transparent images
int constants::FADE_DURATION = 15 ; //Duration in frames for which the fade in-out period should last
unsigned int constants::FADE_PIXEL_THRESHOLD = 240; //Threshold on occlusion mask below which pixels fade in or out

unsigned char constants::FRAME_ALPHA[3] = {0, 0,0 };//102     //Values of the alpha masks for each of the three color channels HSV
//unsigned char constants::FRAME_ALPHA[3] = {0, 0,102 };//102     //Values of the alpha masks for each of the three color channels HSV
//double constants::S_VALUE_CONTENT[3] = {.01, .02, 0.4};//{.01, .05, 0.4}; //Mean parameters for color alignment
//double constants::D_VALUE_CONTENT[3] = {.1, 1, 1};      //Variance parameters for color alignment
double constants::S_VALUE_CONTENT[3] = {0, 0, 0.0};//{.01, .05, 0.4}; //Mean parameters for color alignment
double constants::D_VALUE_CONTENT[3] = {0, 0, 0.0};      //Variance parameters for color alignment

//*********** Optimization *********
double constants::OPTIMIZATION_REGULARIZATION = 1.0e-6; //Regulaization weight for velocity for the H parameters matrix
unsigned constants::MAX_ITERATIONS = 500; //Maximum iterations for convergence
unsigned constants::ITERATIONS_FOR_PF = 2; //Iteration for which optimization is run at each level while particle filtering is being done
double constants::DELTA_HJ = 1.0e-6;//3.9e-5;//1.0e-6;//1.0e-6;//1.0e-9 ; //Increment of H_j while computing numerical differentiation of J1
double constants::TOLERANCE = 0;//1.0e-12;//1.0e-6;//1.0e-15;//0.0 ;//Threshold for change in value of the objective function between iterations
double constants::INITIAL_ALPHA = 1; //Intial value of the learning rate
double constants::BACK_TRACKING_FACTOR = 2; //Amount by which the learning rate is divided at each iteration of line search
double constants::DEFAULT_TRACK_SCALE = 1.0;  //The default scale at which tracking is performed

//********** Optimization - Annealing ********
unsigned constants::TOTAL_SMOOTHING_ITERATIONS = 4; //Total level of smoothing
unsigned constants::SMOOTHING_ITERATIONS[4] = {4,3,2,1};  //Values for each level of smoothing
double constants::SMOOTHING_WEIGHTS[4] = {0.05,0.05,0.2,0.7};//{0.25,0.25,0.25,0.25}; //Signifies how important each level of blurring is
int constants::OCC_SMOOTHING_INDEX = 3; //Index in SMOOTHING_ITERATIONS for 
CvScalar constants::INITIAL_MODEL_VARIANCE = cvScalar(10,3,3); //Initial variances for each pixel, over each channel
int constants::SMOOTHING_MASK_SIZE = 3; //Mask size of the gaussian blur used for smoothing
double constants::LAMBDA_GRAY_WEIGHT = 0.0009;
double constants::LAMBDA_COLOR_WEIGHT = -0.09;

//************* Occlusion outlier detection **************

double constants::VARIANCE_REGULARIZATION[3] = {60,6,6}; //Regularizations for the variance in outlier detection for occlusion
double constants::OUTLIER_THRESHOLD = 9; //Base threshold for outlier detection based on mahalnobis distance
double constants::OCC_PERCENTAGE_FOR_IDENTITY_PARTICLE = 0.09; //Index of the identity particle 
double constants::OCC_PERCENTAGE_FOR_PARTICLE = 0.00000005; //Index of the identity particle 
double constants::TRACTABLE_OCCLUSION_THRESHOLD = 0.7; //Threshold for % of occlusion points 
                                                                 //after which optimization is skipped
string constants::EXT_FOR_OCC_MASK = ".bmp";    //Image extension for when occlusion masks are saved as meta data
string constants::EXT_FOR_FLASH_OCC_MASK = ".jpg";    //Image extension for when occlusion masks are saved in flash mode
string constants::OLD_EXT_FOR_OCC_MASK = ".jpg";    //Previous image extension for when occlusion masks are saved as meta data

//************* Particles *********
int constants::TOTAL_PARTICLES = 15;
double constants::IDENTITY[9] = {1,0,0, 0,1,0, 0,0,1};
int constants::SIFT_PARTICLE_TYPES = 6;
int constants::SAMPLE_PARTICLE_TYPES = 2;
int constants::IDENTITY_PARTICLE = 13; //Index of the identity particle 

//************ SIFT **********
unsigned constants::MINIMUM_SIFT_FEATURES = 4;
int constants::SIFT_FV_SIZE = 128;
double constants::SIFT_THRESHOLD = 6.0;
int constants::SIFT_PREFIX_FOR_KEY_POOL = 10;

//************* Arrow Drawing ******************

int constants::ARROW_GAP_FROM_POLYGON = 10;
int constants::ARROW_LENGTH = 75;
CvScalar constants::ARROW_COLOR = cvScalar(50,255,0);
int constants::ARROW_THICKNESS = 2;
double constants::RATIO_OF_ARROW_AHEADS = 3;
double constants::ANGLE_OF_ARROW_AHEADS = 1.2;
int constants::ARROW_DISPLAY_LENGTH = 20;

