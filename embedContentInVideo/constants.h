/*****************************************************************************
** ZunaVision
** Copyright (c) 2008 
**
** FILENAME:    constants.h
** AUTHOR(S):   Sid Batra <sidbatra@cs.stanford.edu>
** DESCRIPTION:
** Holds the various constants and parameters used in the application

*****************************************************************************/

#pragma once

#include "base.h"


using namespace std;

enum renderingType { FULL_BLENDED , NO_LIGHTING , NO_BLENDING , ALPHA_ONLY};
enum contentType { NORMAL , POISSON , FRAME };
enum trackingType { STILL_CAMERA , EASIER_MOTION , COMPLEX_MOTION };
enum fadeType { NO_FADE , FADE_IN , FADE_OUT , FADE_IN_OUT };

class constants
{

    public:
	static double CUT_SCENE_THRESHOLD; //Threshold for labeling a frame as a cut scene
        static int SMOOTHING_MASK_SIZE; //Mask size of the gaussian blur used for smoothing
        static double TRACTABLE_OCCLUSION_THRESHOLD; //Threshold for % of occlusion points 
                                                                 //after which optimization is skipped
        static string EXT_FOR_OCC_MASK;
        static string EXT_FOR_FLASH_OCC_MASK;
        static string OLD_EXT_FOR_OCC_MASK;    //Previous image extension for when occlusion masks are saved as meta data

        //MISC
        static double THRESHOLD_FOR_REPEAT_FRAME ;
        static double THRESHOLD_FOR_CUT_FRAME;
        static double THRESHOLD_FOR_SIMILAR_FRAME;
        static double THRESHOLD_FOR_BLANK_FRAME ;
        static int MARGIN; //Margin for checking for points that lie outside the image
        static bool VERBOSE; //Controls the level of output text
	static CvSize SPATIAL_IMAGE_SIZE ;
        
        static double VARIANCE_REGULARIZATION[3]; //Regularizations for the variance in outlier detection for occlusion
        static double OUTLIER_THRESHOLD;

        //TRACKING
        static unsigned VOLUME_MARGIN;   //Margin of frames in time to be used for tracking
        
        //RENDERING
        static int EFFECT_LENGTH;
        static double PREPROCESS_CONTENT_SCALEUP;   //Factor by which the content image size is increased in proporation to target polygon
                                                    //to enable smoothing for subsampling
        static double CONTENT_MARGIN;           //Margin in pixels taken at the edges of the rendering region for blending with the 
                                                //background (the actual frame)
        static double SUB_SAMPLE_STEPSIZE;       //Factor by which the region being rendered is theoretically scaled up
        static double CC_AREA_THRESHOLD ;
        static double SUB_OCC_THRESHOLD ;       //Threshold for calling a sub pixel occluded in occlusion mask

        static string FRAME_FILENAME;      //Filname of the frame template image
        static CvRect FRAME_ROI;           //ROI where the image is placed within a frame
        static CvSize FRAME_BORDER;       //Size of the border to be excluded from blending

        static unsigned char  FRAME_ALPHA[3];          //Values of the alpha masks for each of the three color channels HSV
        static double S_VALUE_CONTENT[3];       //Mean parameters for color alignment
        static double D_VALUE_CONTENT[3];      //Variance parameters for color alignment

        static int OCC_EDGE_SMOOTH_SIDE ;   //Side of the gaussian kernel used for smoothing the edge map of the occ mask 
        static CvPoint OCC_EDGE_CANNY_THRESHOLDS ; // Thresholds to be used for computing the edge map of the occ mask		
	static string EXT_FOR_TRANSPARENT_IMAGES; //Extension for transparent images
	static string NAME_FOR_TRANSPARENT_IMAGES; //Name for transparent images
	static int FADE_DURATION ; //Duration in frames for which the fade in-out period should last
	static unsigned int FADE_PIXEL_THRESHOLD; //Threshold on occlusion mask below which pixels fade in or out
        

        //OPTIMIZATION
        static double OPTIMIZATION_REGULARIZATION;
        static unsigned MAX_ITERATIONS;
        static unsigned ITERATIONS_FOR_PF; //Iteration for which optimization is run at each level while particle filtering is being done
        static double DELTA_HJ;
        static double TOLERANCE;
        static double INITIAL_ALPHA;
        static double BACK_TRACKING_FACTOR;
        static double DEFAULT_TRACK_SCALE ;  //The default scale at which tracking is performed

        //OPTIMIZATION - ANNELAING
        static unsigned TOTAL_SMOOTHING_ITERATIONS; //Total level of smoothing
        static unsigned SMOOTHING_ITERATIONS[4];  //Values for each level of smoothing
        static double SMOOTHING_WEIGHTS[4];//Signifies how important each level of blurring is
        static int OCC_SMOOTHING_INDEX ; //Index in SMOOTHING_ITERATIONS for 
        static CvScalar INITIAL_MODEL_VARIANCE; //Initial variances for each pixel, over each channel
        static double LAMBDA_GRAY_WEIGHT ;     //Lambda value of the cumulative time weighting of the grayscale model
        static double LAMBDA_COLOR_WEIGHT ;    //Lambda value of the cumulative time weighting of the color model

        //PARTICLES
        static int TOTAL_PARTICLES;
        static int SIFT_PARTICLE_TYPES;
        static int SAMPLE_PARTICLE_TYPES;
        //static int TEST_SIFT_INDEX;
        //static int TEST_SAMPLE_INDEX;
        static double IDENTITY[9];
        static int IDENTITY_PARTICLE;
        static double OCC_PERCENTAGE_FOR_IDENTITY_PARTICLE;
        static double OCC_PERCENTAGE_FOR_PARTICLE;

        //SIFT
        static unsigned MINIMUM_SIFT_FEATURES ;
        static int SIFT_FV_SIZE ;
        static double SIFT_THRESHOLD ;
        static int SIFT_PREFIX_FOR_KEY_POOL;

        
        //ARROW DRAWING
        static int ARROW_GAP_FROM_POLYGON;
        static int ARROW_LENGTH;
        static CvScalar ARROW_COLOR;
        static int ARROW_THICKNESS;
        static double RATIO_OF_ARROW_AHEADS ;
        static double ANGLE_OF_ARROW_AHEADS ;
        static int ARROW_DISPLAY_LENGTH;

        static double PI;
        
};
