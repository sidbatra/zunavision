/*****************************************************************************
** ZunaVision
** Copyright (c) 2008
**
** FILENAME:    blendingManager.cpp
** AUTHOR(S):   Ashutosh Saxena <asaxena@cs.stanford.edu>
** DESCRIPTION:
** Algorithms for blending the ad into the scene.

*****************************************************************************/

#include "blendingManager.h"


void BlendingManager::setParameters()
{
  for(int c=0; c<3; c++) {
    _alpha[c] = BLENDING_ALPHA[c];
    _limit[c] = BLENDING_LIMIT[c];
    _deltaGammaLimit[c] = _gammaCorrectionMode ? BLENDING_DELTAGAMMA_LIMIT[c] : 0;
  }
  if( _intensityMode ) {
    // in intensity mode, only the value of H-S-V should be changed
    _alpha[0] = _alpha[1] = 0;
    _deltaGammaLimit[0] = _deltaGammaLimit[1] = 0;
  }

}


// This function just returns 'channel' of 'img' as a sorted vector
vector<double> BlendingManager::makeVector(const IplImage* img, unsigned channel=2, double minX=0, double minY=0, double maxX=INT_MAX, double maxY=INT_MAX)
{
  computeLocalNeighborhood(img, unsigned(minX), unsigned(minY), unsigned(maxX), unsigned(maxY) );
  
  vector<double> values;

  // iterate over all the values
  for(int y=_minY; y<_maxY; y++) {
    for(int x=_minX; x<_maxX; x++) {
      CvScalar s = cvGet2D(img, y, x);
      values.push_back( s.val[channel] );
    }
  }
  
  std::sort(values.begin(), values.end() );

  return values;

}

void BlendingManager::computeLocalNeighborhood(const IplImage *img, unsigned minX, unsigned minY, unsigned maxX, unsigned maxY)
{

  _minX = minX;   _maxX = maxX;
  _minY = minY;   _maxY = maxY;

  // only look into the local neighborbood if localMode flag is set
  if(_localMode) {
    _minY -= img->height/3;
    if(_minY<0)  _minY = 0;
    _minX -= img->width/3;
    if(_minX<0)  _minX = 0;

    _maxY += img->height/3;
    if(_maxY>img->height) _maxY = img->height;
    _maxX += img->width/3;
    if(_maxX>img->width) _maxX = img->width;
  }
  else {
    _minY = _minX = 0;
    _maxY = img->height;
    _maxX = img->width;
  }
}

void BlendingManager::calculateHistogramStats(const IplImage* img, double minX, double minY, double maxX, double maxY)
{

  vector< vector<double> > values;
  values.push_back( makeVector(img,0,minX,minY,maxX,maxY) );
  values.push_back( makeVector(img,1,minX,minY,maxX,maxY) );
  values.push_back( makeVector(img,2,minX,minY,maxX,maxY) );

  // compute 5%, 50% and 95% percentiles
  for(int c=0; c<3; c++) {
    _low[c] = values[c][ int( 0.05*values[c].size() ) ];
    _median[c] = values[c][ int( 0.5*values[c].size() ) ];
    _high[c] = values[c][ int( 0.95*values[c].size() ) ];

    _gamma[c] = _median[c] / (0.5 * (_low[c] + _high[c]) );

    if (_gamma[c]-1 > _deltaGammaLimit[c])
      _gamma[c] = 1 + _deltaGammaLimit[c];
    else if (_gamma[c]-1 > -_deltaGammaLimit[c])
      _gamma[c] = 1 - _deltaGammaLimit[c];

    
    //cout << "LowV: " << _low[c] << " medianV:" << _median[c] << " HighV: " << _high[c] << endl;
  }
                                      

}


// adjusts the creative according to the image in question
void BlendingManager::adjustCreative(IplImage* img)
{
  double maxValue[3] = {255, 255, 255};

  // first compute the range of the creative
  for(int c=0; c<3; c++) {
    vector<double> v = makeVector(img,c,0,0,img->width,img->height);
    maxValue[c] = *max_element( v.begin(), v.end() );
  }

  for(int y=0; y<img->height; y++) {
    for(int x=0; x<img->width; x++) {
      CvScalar s = cvGet2D(img, y, x);
      shiftPixel( &s, maxValue );
      cvSet2D(img, y, x, s);
    }
  }
}

// shifts the pixel to equalize the histogram
inline
void BlendingManager::shiftPixel(CvScalar * s, const double * maxValue)
{
  for(int c=0; c<3; c++) {
    // there is no gamma (~kurtosis) correction currently
    double scaledV = s->val[c] / maxValue[c];
    scaledV = pow(scaledV, _gamma[c]);

    double delta = scaledV * (_high[c] - _low[c])  + _low[c] - s->val[c];   // rescale histogram
    if( delta > BLENDING_LIMIT[c])   
      delta = BLENDING_LIMIT[c];
    else if( delta < -BLENDING_LIMIT[c])
      delta = -BLENDING_LIMIT[c];   // cap it at the max

    s->val[c] += delta * BLENDING_ALPHA[c];  // shift it only to some extent
  }
}
