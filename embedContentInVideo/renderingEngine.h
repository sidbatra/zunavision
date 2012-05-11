/*****************************************************************************
** ZunaVision
** Copyright (c) 2008 
**
** FILENAME:    renderingEngine.h
** AUTHOR(S):   Sid Batra <sidbatra@cs.stanford.edu>
** DESCRIPTION:
** Renders content onto the images based on meta data information
*****************************************************************************/

#pragma once

#include "base.h"
#include "video.h"
#include "blendingManager.h"

using namespace std;


class renderingEngine
{
private:
  
public:
    renderingEngine();
    
    void renderFrame(video &cVideo,unsigned volumeIndex , int frameIndex);     //Renders the given frame index from the volume
    void renderContent(IplImage *frame , IplImage *occlusionMask,vector<svlPoint2d> targetPolygon , content &cContent);      //Subpixel renders the given content in the given frame at the given location
    void blendPixel(CvScalar &contentValue , CvScalar &frameValue , content &cContent , CvScalar &alphaValue);            //Blends the given pixel given the content type
    
    void setupContentStatistics(video &cVideo , unsigned volumeIndex , int frameIndex , content &cContent, int frameTrackIndex); //Sets up the content image statistics to be used for rendering
    void computeImageMeansVariances(IplImage *image, vector<double> &means , vector<double> &variances); //Computes the vectors of means and variances of the given image
    void customBlurContent(IplImage *content, vector<svlPoint2d> targetPolygon);  //Custom blurs the content image based upon a custom kernel
    IplImage* createAlphaMasks(IplImage *occlusionMask , IplImage *poissonImage ,  IplImage **edgeMap  , 
                                                    CvMat *H , int &discreteX , int &discreteY ,
                                                     int &discreteHeight , int &discreteWidth , vector<svlPoint2d> &targetPolygon,CvSize frameSize , content &cContent);//Creates the final alpha mask combining the occlusion mask the poisson image and the tranparency

};

