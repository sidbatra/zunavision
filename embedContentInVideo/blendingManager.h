/*****************************************************************************
** ZunaVision
** Copyright (c) 2008 
**
** FILENAME:    blendingManager.h
** AUTHOR(S):   Ashutosh Saxena <asaxena@cs.stanford.edu>
** DESCRIPTION:
** Algorithms for blending the ad into the scene.

*****************************************************************************/



#pragma once

#include "base.h"

using namespace std;

//******************************** CLASS content ****************************************
  
const double BLENDING_ALPHA[3] = {0.01, 0.1, 0.9};   // H-S-V
const double BLENDING_LIMIT[3] = {10, 30, 100};    // H-S-V
const double BLENDING_DELTAGAMMA_LIMIT[3] = {.05, .05, .2};  // H-S-V

class BlendingManager
{

private:
  
  double _low[3], _median[3], _high[3], _gamma[3];    // stores 5, 50 and 95 percentile of the data

  double _alpha[3], _limit[3], _deltaGammaLimit[3];

  int _minX, _minY, _maxX, _maxY;


  bool _localMode;      // estimate correction from a local neighborhood only
  bool _intensityMode;  // correct only V channel in H-S-V space
  bool _gammaCorrectionMode;  // this corrects for kurtosis.

public:
 
  BlendingManager(bool intensityMode = false, bool localMode = true, bool gammaCorrectionMode = true)
  { 
    _intensityMode = intensityMode;
    _localMode = localMode;  
    _gammaCorrectionMode = gammaCorrectionMode;
    setParameters();
  }

  void setParameters();

  void computeLocalNeighborhood(const IplImage *img, unsigned minX, unsigned minY, unsigned maxX, unsigned maxY);

  // input below must be in H-S-V
  void calculateHistogramStats(const IplImage *img, double minX=0, double minY=0, double maxX=INT_MAX, double maxY=INT_MAX);

  // input below must be in H-S-V
  void adjustCreative(IplImage *img);
  
  inline
  void shiftPixel(CvScalar * s, const double *);

  vector<double> makeVector(const IplImage *img, unsigned channel, double, double, double, double);


};

