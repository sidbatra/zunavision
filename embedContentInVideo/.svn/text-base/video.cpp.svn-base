/*****************************************************************************
** ZunaVision
** Copyright (c) 2008 
**
** FILENAME:    video.cpp
** AUTHOR(S):   Sid Batra <sidbatra@cs.stanford.edu>
** DESCRIPTION:
** Holds classes for all meta-data / tracking / rendering information needed by a video
*****************************************************************************/

#include "video.h"




//************************************* CLASS content ****************************************************

//****************** Constructor Logic ***********************

//Default constructor
content::content()
{
    _currentIndex = 0;      //Set the index of the currently rendered content image to zero
    _currentImage = NULL;
    _currentPoissonImage = NULL;
    _renderingType = 0;
    _trackingType = 0;
    _timeStep = 2;
    _previousGrayWeight = exp(-(double)constants::LAMBDA_GRAY_WEIGHT * 1); //Fwd;
    _previousColorWeight = exp(-(double)constants::LAMBDA_COLOR_WEIGHT * 1); //Fwd
    _previousOcclusionMode = 0;
    _occlusionType = 1;
    _frameBorder = cvSize(-1,-1);
    _fadePositionBwd = 0;
    _fadePositionFwd = 0;
    _fadeType = NO_FADE;
    	
}


//****************** Methods ***********************

//Frees all the memory being used
void content::freeMemory()
{       

    _filenames.clear();
    _frameMeans.clear();
    _frameVariances.clear();
    _contentMeans.clear();
    _contentVariances.clear();

    if( _currentPoissonImage != NULL )
        cvReleaseImage(&_currentPoissonImage);
        
    if( _currentImage != NULL )
        cvReleaseImage(&_currentImage);
}

//Cleans the model of the region
void content::cleanRegionModel()
{   
    //Cleans all data structures associated with the region model
    _pixelLocations.clear();
    _colorPixelMeans.clear();
    _colorPixelVariances.clear();
    _grayPixelMeans.clear();
    _grayPixelVariances.clear();
    
}

//Cleans the occlusion related variables 
void content::cleanOcclusionModel()
{
    _unOccludedPixels = 0;
	_occlusionMode = 0;   
    _NCCOcclusionMode = 0;

    _isOccluded = vector<bool>(_pixelLocations.size());
}

//Copies content of given object into data members
void content::copy(content &cContent)
{
    _renderingType = cContent._renderingType;    
    _contentType =  cContent._contentType;
    _endFrameBackward =  cContent._endFrameBackward;
    _trackingType =  cContent._trackingType;
    _fadeType =  cContent._fadeType;
    _startingFrame = cContent._startingFrame;
    _name = cContent._name;
    _filenames = cContent._filenames ;
    _extension = cContent._extension;
    _endFrameForward =  cContent._endFrameForward;
    
    _frameMeans = cContent._frameMeans;
    _frameVariances = cContent._frameVariances;

    _contentMeans = cContent._contentMeans;
    _contentVariances = cContent._contentVariances;

    _currentIndex = cContent._currentIndex;

    if( cContent._currentImage != NULL)
        _currentImage = cvCloneImage(cContent._currentImage);

    if( cContent._currentPoissonImage != NULL)
        _currentPoissonImage = cvCloneImage(cContent._currentPoissonImage);

    _frameMeans = cContent._frameMeans;             
    _frameVariances = cContent._frameVariances;         

    _contentMeans = cContent._contentMeans;           
    _contentVariances = cContent._contentVariances;       
    _grayPixelMeans = cContent._grayPixelMeans;         
    _grayPixelVariances = cContent._grayPixelVariances;     
    _pixelLocations = cContent._pixelLocations;         

    _colorPixelMeans = cContent._colorPixelMeans;
    _colorPixelVariances = cContent._colorPixelVariances;

    _unOccludedPixels = cContent._unOccludedPixels;
    _occlusionMode  = cContent._occlusionMode;
    _occlusionType = cContent._occlusionType;
    _NCCOcclusionMode = cContent._NCCOcclusionMode;

    _NCCDeviations = cContent._NCCDeviations;

    _isOccluded = cContent._isOccluded; 

    _previousGrayWeight = cContent._previousGrayWeight;
    _previousColorWeight = cContent._previousColorWeight;

    _timeStep = cContent._timeStep;

    _fadePositionBwd = cContent._fadePositionBwd;
    _fadePositionFwd = cContent._fadePositionFwd;
    
}


//Returns the first frame in the video where the content begins to appear
void content::setFadePositions(int frameIndex , unsigned int totalFrames)
{
	_fadePositionBwd =  abs(( _endFrameBackward == -1 ?  (int)_startingFrame : _endFrameBackward) - frameIndex);
	_fadePositionFwd =  abs(( _endFrameForward == INT_MAX ?  (int)totalFrames : _endFrameForward) - frameIndex);
}


//Returns the alpha value to be used for the content on the fading spectrum
int content::getFade()
{

	double alpha = 0.0;

	if( _fadePositionBwd < constants::FADE_DURATION && (_fadeType == FADE_IN || _fadeType == FADE_IN_OUT) )
		alpha = 255.0 * ( constants::FADE_DURATION - _fadePositionBwd)  / constants::FADE_DURATION;
	else if( _fadePositionFwd < constants::FADE_DURATION && (_fadeType == FADE_OUT || _fadeType == FADE_IN_OUT) )
		alpha = 255.0 *  ( constants::FADE_DURATION - _fadePositionFwd)  /  constants::FADE_DURATION;

	return (int)alpha;

}


//Caches model related stats for NCC
void content::populateNCCStats(unsigned index)
{
   //****************** Compute Means *************************
   double IBar = 0.0;
  		
    for( unsigned i=0 ; i<_pixelLocations.size() ; i++)
	{
		if( _isOccluded[i] )
            continue;
        
        IBar += _grayPixelMeans[index][i];        
	}
    
    IBar /= _unOccludedPixels;
        

	//****************** Compute Variances & Deviations*************************

    _NCCVariance = 0.0;
    _NCCDeviations = vector<double>(_pixelLocations.size());


	for( unsigned i=0 ; i<_pixelLocations.size() ; i++)
	{
		if(_isOccluded[i] )
        {
            _NCCDeviations[i] = 0;
			continue;
        }
	
        double IValue = _grayPixelMeans[index][i] - IBar;

        _NCCDeviations[i] = IValue;
        _NCCVariance += pow( IValue , 2);
		
	}

}


//Populates the locations of the pixels in the region
void content::populateModelLocations(contentTracks &currentTracks, IplImage *visualizeImage)
{
    cout<<"\n Populating pixel locations \n";

    //Variables for holding the possibly larger rectangle surronding the polygon
	double minX = 0.0 , minY = 0.0 , maxX = 0.0 , maxY = 0.0;

    visionUtilities::findEnclosingRectangle(currentTracks._polygon,minX,minY,maxX,maxY);

    //Populate every single pixel location
    for( double y=minY ; y<=maxY ; y++)
    {
        for( double x=minX ; x<=maxX ; x++)
        {
            if( visionUtilities::inPolygon(currentTracks._polygon,x,y))
            {
                _pixelLocations.push_back( svlPoint2d(x,y) );

                if( visualizeImage != NULL )
                     cvSet2D(visualizeImage,(int)y,(int)x,cvScalar(0,0,0));
            }
        }
    }

    //Initialize all the model data structures
    _grayPixelMeans = vector<vector<double> >(constants::TOTAL_SMOOTHING_ITERATIONS);
    _grayPixelVariances = vector<vector<double> >(constants::TOTAL_SMOOTHING_ITERATIONS);
    _colorPixelMeans = vector<CvScalar>(_pixelLocations.size());
    _colorPixelVariances = vector<CvScalar>(_pixelLocations.size());
    
}

//Projects each pixel in the model by the given transform
void content::projectModelPixels(CvMat *H)
{
    for( unsigned i=0 ; i<_pixelLocations.size() ; i++)
        visionUtilities::projectCoordinate(H,_pixelLocations[i].x,_pixelLocations[i].y);
}

//Creates a distribution by updating the means of each pixel
void content::updateGrayModelValue(IplImage *grayImage,unsigned index)
{
    //Compute weight for current timestep
    double currentWeight = exp(-(double)constants::LAMBDA_GRAY_WEIGHT * (_timeStep)); //Fwd

    for( unsigned i=0 ; i<_pixelLocations.size() ; i++)
    {        				
        if( _isOccluded[i] )
            continue;

        //Compute new value of pixel
        double currentPixelValue = visionUtilities::subPixel(grayImage,_pixelLocations[i].x,_pixelLocations[i].y);
        
        //Update mean of the distribution
        _grayPixelMeans[index][i] = _grayPixelMeans[index][i] * _previousGrayWeight + currentPixelValue * currentWeight;
		_grayPixelMeans[index][i] /= (_previousGrayWeight + currentWeight);

        //Update variance of the distribution
        _grayPixelVariances[index][i] = _grayPixelVariances[index][i] * _previousGrayWeight 
                                            + pow( currentPixelValue - _grayPixelMeans[index][i] , 2) * currentWeight;
        _grayPixelVariances[index][i] /= (_previousGrayWeight + currentWeight);
    }
}

//Populates the initial values of the pixel locations
void content::populateGrayModelValues(IplImage *grayImage , unsigned index)
{
    //Initialize the given level of the model
    _grayPixelMeans[index] = vector<double>(_pixelLocations.size());
    _grayPixelVariances[index] = vector<double>(_pixelLocations.size());

    //Compute and store initial value of each pixel
    for( unsigned j=0 ; j<_pixelLocations.size() ; j++)
	{
        _grayPixelMeans[index][j] = visionUtilities::subPixel(grayImage,_pixelLocations[j].x,_pixelLocations[j].y);
        _grayPixelVariances[index][j] = constants::INITIAL_MODEL_VARIANCE.val[0];
	
	}
}

//Creates a distribution by updating the means of each pixel
void content::updateColorModelValue(IplImage *colorImage)
{    

    //Compute weight for current timestep
    double currentColorWeight = exp(-(double)constants::LAMBDA_COLOR_WEIGHT * (_timeStep)); //Fwd

    for( unsigned i=0 ; i<_pixelLocations.size() ; i++)
    {
          if( _isOccluded[i] )
		    continue;

          CvScalar currentColorValue = visionUtilities::subPixelRGB(colorImage,_pixelLocations[i].x,_pixelLocations[i].y);
  
            //Update all three channels
            for( int k=0 ; k<3 ; k++)
            {            
                //Update distribution mean
                _colorPixelMeans[i].val[k] = _colorPixelMeans[i].val[k] * _previousColorWeight + currentColorValue.val[k] * currentColorWeight;
                _colorPixelMeans[i].val[k] /= (_previousColorWeight + currentColorWeight);

                //Update distribution variance
                _colorPixelVariances[i].val[k] = _colorPixelVariances[i].val[k] * _previousColorWeight 
                    +  pow( _colorPixelMeans[i].val[k] - currentColorValue.val[k] , 2) * currentColorWeight;

                _colorPixelVariances[i].val[k] /= (_previousColorWeight + currentColorWeight);
            
            }

        }//pixels i
}

//Populates the initial color values of the pixel locations
void content::populateColorModelValues(IplImage *colorImage)
{
     
    //Compute and store initial value of each pixel
    for( unsigned j=0 ; j<_pixelLocations.size() ; j++)
	{
        _colorPixelMeans[j] = visionUtilities::subPixelRGB(colorImage,_pixelLocations[j].x,_pixelLocations[j].y);
        _colorPixelVariances[j] = constants::INITIAL_MODEL_VARIANCE;
	}
}

//Writes all the content data to disk
void content::write(ofstream &out , int index , bool isReverse , unsigned totalFrames)
{
     if( _endFrameForward == INT_MAX )
        _endFrameForward = -1;

    out<<"    <content id=\""<<index<<"\" name=\""<<_name.c_str()<<"\" ext=\""<<_extension.c_str()<<"\" rType=\""<<_renderingType;
    out<<"\" cType=\""<<_contentType<<"\" tType=\""<<_trackingType<<"\" fType=\""<<_fadeType<<"\" startF=\""<<_startingFrame;
    out<<"\" endFF=\""<<_endFrameForward<<"\" endFB=\""<<_endFrameBackward<<"\" occ=\""<<_occlusionType<<"\""" />"<< endl;

      if( _endFrameForward == -1 )
         _endFrameForward = INT_MAX;

}

//Reads the data from the content node and returns a content object
void content::read(XMLNode contentNode , bool isReverse , unsigned totalFrames)
{       
     //Populate content attributes
     _name = string(contentNode.getAttribute("name"));
     _extension = string(contentNode.getAttribute("ext"));
     _renderingType = atoi(contentNode.getAttribute("rType"));
     _contentType = atoi(contentNode.getAttribute("cType"));
     _trackingType = atoi(contentNode.getAttribute("tType"));
     _fadeType = atoi(contentNode.getAttribute("fType"));
     _startingFrame = atoi(contentNode.getAttribute("startF"));
     _endFrameForward = atoi(contentNode.getAttribute("endFF"));
     _endFrameBackward = atoi(contentNode.getAttribute("endFB"));

     //Ensure backward compatibility of meta data versions 
     //even when new attributes are added
     if( contentNode.isAttributeSet("occ") )
         _occlusionType = atoi(contentNode.getAttribute("occ"));
     else
         _occlusionType = 1;

     if( _endFrameForward == -1 )
        _endFrameForward = INT_MAX;
         
}


//Populates all the content names
void content::populateFilenames(string mainDirectory)
{	    
    //Populate data members
    string contentFolder = mainDirectory + "//" + _name;
   
    if( isTransparent() )
	_maskPath = contentFolder + "//" + constants::NAME_FOR_TRANSPARENT_IMAGES;
	 
    _filenames = utilities::loadFilenames(contentFolder,_extension , true , false);
}

//Based on the type of content sets index to next content image
void content::setIndex(int frameIndex)
{
    //If content type is a video clean content image stats so that they are refreshed with every frame
    if( isVideo() || frameIndex == -1)
    {
        _contentMeans.clear();
        _contentVariances.clear();

        _currentIndex = (unsigned)((frameIndex + 1) % _filenames.size()); //Increment index
    }
    else
        _currentIndex = 0;


}


//Returns whether the current content image is transparent or not
bool content::isTransparent()
{
	return _extension == constants::EXT_FOR_TRANSPARENT_IMAGES;
}


//Preprocesses the content image to prepare it for sub sampling
void content::preprocessCurrentImage(vector<svlPoint2d> targetPolygon )
 {     
     if( _contentType == POISSON )
     {
         //Make grayscale copy of current imaeg for poisson processing
        IplImage *grayScale = cvCreateImage(cvGetSize(_currentImage),_currentImage->depth,1);
        cvCvtColor(_currentImage,grayScale,CV_RGB2GRAY);

        //Compute poissons edge map
        _currentPoissonImage =  visionUtilities::poissonEdgeMap(grayScale);

        cvReleaseImage(&grayScale);
     }
     else if( _contentType == FRAME )
     {
         //Create frame image with room for frame border
         IplImage* frame = cvLoadImage(constants::FRAME_FILENAME.c_str(),CV_LOAD_IMAGE_COLOR);
            
         vector<svlPoint2d> framePolygon =  visionUtilities::formPolygonFromRect(constants::FRAME_ROI);
         vector<svlPoint2d> contentPolygon =  visionUtilities::formPolygonFromImage(_currentImage);

         CvMat *H = visionUtilities::findMappingBetweenPolygons(framePolygon,contentPolygon);

         double tempX = -1 , tempY = -1;

         //Place the image into the frame
         for( int y=constants::FRAME_ROI.y ; y<constants::FRAME_ROI.y + constants::FRAME_ROI.height ; y++)
         {
             for( int x=constants::FRAME_ROI.x ; x<constants::FRAME_ROI.x + constants::FRAME_ROI.width ; x++)
            {  
                tempX = x ; tempY = y;
                visionUtilities::projectCoordinate(H,tempX,tempY);
                cvSet2D(frame,y,x,cvGet2D(_currentImage,(int)tempY,(int)tempX));
            }         
         }         

         _frameBorder.width = (constants::FRAME_BORDER.width * _currentImage->width) / frame->width;
         _frameBorder.height = (constants::FRAME_BORDER.height * _currentImage->height) / frame->height;

         utilities::resizeInPlace(&frame,_currentImage->height,_currentImage->width);
         
         cvReleaseMat(&H);
         cvReleaseImage(&_currentImage);
         _currentImage = cvCloneImage(frame);
         cvReleaseImage(&frame);
  
     }//frame

     double minX = 0.0 , minY = 0.0 , maxX = 0.0 , maxY = 0.0;//Variables for holding the possibly larger rectangle surronding the polygon

     visionUtilities::findEnclosingRectangle(targetPolygon,minX,minY,maxX,maxY);

    //Compute factor by which content image is now larger than the actual region for embeding
	//this is used to blur the content image by the same factor
    int scaleXFactor = (int)(utilities::roundOff(_currentImage->width  / (maxX-minX)) * constants::PREPROCESS_CONTENT_SCALEUP);
	int scaleYFactor = (int)(utilities::roundOff(_currentImage->height / (maxY-minY)) * constants::PREPROCESS_CONTENT_SCALEUP);
	
	//Round off scale factor to an odd number
	scaleXFactor += 1 - (scaleXFactor) % 2;
	scaleYFactor += 1 - (scaleYFactor) % 2;
	
	//Blur the content image to enable sub  sampling
    cvSmooth(_currentImage,_currentImage,CV_GAUSSIAN,scaleXFactor,scaleYFactor);
 }

//Precomputes dynamic parameters about blending
void content::precomputeDynamicParameters()
{
    if( _renderingType == FULL_BLENDED || _renderingType == NO_LIGHTING )
    {
        _S = vector<double>(3);
        _D = vector<double>(3);
    
        //Compute dynamic blending parameters
        for( int i=0 ; i<3 ; i++)
         _S[i] = 1 - constants::S_VALUE_CONTENT[i] + constants::S_VALUE_CONTENT[i] * (_frameVariances[i] / _contentVariances[i]);

	    for( int i=0 ; i<3 ; i++)
         _D[i] = constants::D_VALUE_CONTENT[i] * (_frameMeans[i] - _contentMeans[i]);
    }
}

//Loads the current image from disk 
void content::loadCurrentImage()
{
    if( _currentImage != NULL)
        cvReleaseImage(&_currentImage);

    if( _currentPoissonImage != NULL )
        cvReleaseImage(&_currentPoissonImage);

    if( !isTransparent() )
    {
   	 //Loads and returns the current content image
   	 _currentImage = cvLoadImage(_filenames[_currentIndex].c_str(),CV_LOAD_IMAGE_COLOR);
    }
    else 
    {
	//pngReader reader;
	//reader.mainFunction(_filenames[_currentIndex]);

	//_currentImage = cvCloneImage(reader.sourceImage);
	//_currentPoissonImage = cvCloneImage(reader.maskImage);
    }

 

/*IplImage *temp = cvCreateImage(cvSize(_currentImage->width,_currentImage->height - 103 *2),_currentImage->depth,3);

for( int y=103 ; y<_currentImage->height-103 ; y++)
for( int x=0;x<_currentImage->width;x++)
cvSet2D(temp,y-103,x,cvGet2D(_currentImage,y,x));

//utilities::display(_currentImage);
cvReleaseImage(&_currentImage);
_currentImage = cvCloneImage(temp);
cvReleaseImage(&temp);
//utilities::display(_currentImage);*/
 
}

//Returns whether the current content is a video or not
bool content::isVideo()
{
    return _filenames.size() > 1 ? true : false;
}



//********************************************************************************************************

//************************************* CLASS contentTracks ****************************************************

//****************** Constructor Logic ***********************

//Default constructor
contentTracks::contentTracks()
{
    //Initialize data members to NULL
    _H = NULL;
    _cleanTransform = NULL;
    _occlusionMask = NULL;
    _cleanColorImage = NULL;
}


//****************** Methods ***********************

//Populates tracks for the next frame based on previous tracks and transformation matrix
void contentTracks::populateForNextFrame(CvMat *H , CvMat *A , contentTracks &previousTracks, int contentIndex)
{
    _H = cvCloneMat(H);
    _contentID = contentIndex;
    _affinePolygon = previousTracks._affinePolygon;
    _polygon = previousTracks._polygon;
            
    projectPolygon(H);
    projectAffinePolygon(A);
}

//If current frame is occlulded, occlusion metrics are computed
void contentTracks::computeOcclusionMetrics(IplImage *IDashColor,content &cContent,contentTracks &currentTracks)
{
    if( constants::VERBOSE )
        cout<<"\n\n Computing occlusion metrics";

	//*********** Setup transform matrix ************
    
    _cleanTransform = cvCreateMat(_H->rows,_H->cols,_H->type);
	     
     //If there was no occlusion previously
	if( cContent._previousOcclusionMode == 0 )
	{     
        cvmInvert( _H , _cleanTransform);
	}
	else
	{       
		CvMat *inv = cvCreateMat(_H->rows,_H->cols,_H->type);
        cvmInvert(_H,inv);
        cvmMul(currentTracks._cleanTransform,inv,_cleanTransform);
		cvReleaseMat(&inv);
	}

     //Average out all the individual gaussian variances to estimate a variance for each channel
    _meanModelVariances = cvScalar(0.0 , 0.0 , 0.0);
    
    for( unsigned i=0 ; i<cContent._pixelLocations.size() ; i++)
        for( int k=0 ; k<3 ; k++)
            _meanModelVariances.val[k] += cContent._colorPixelVariances[i].val[k];

    for( int k=0 ; k<3 ; k++)
        _meanModelVariances.val[k] /= cContent._pixelLocations.size();

    //TESTING AVERAGING ESTIMATE ACCURACY
   /* CvScalar testVariances = cvScalar(0.0 , 0.0 , 0.0);

    for( int i=0 ; i<_regionPixels.size() ; i++)
        for( int k=0 ; k<3 ; k++)
            testVariances.val[k] += pow( modelVariances.val[k] - _colorPixelVariances[i].val[k] , 2 );

    for( int k=0 ; k<3 ; k++)
        testVariances.val[k] /= _regionPixels.size();

    cout<<"\n "<<testVariances.val[0]<<" "<<testVariances.val[1]<<" "<<testVariances.val[2];*/

    if( constants::VERBOSE )
        cout<<"\n Computing occlusion mask \n";


    //Compute the occlusion mask
    _occlusionMask = cvCreateImage(cvSize(IDashColor->width,IDashColor->height),IPL_DEPTH_8U,1);
    
    cvSetZero(_occlusionMask);
    ////Initialize to default gray value
    //for( int y=0 ; y<_occlusionMask->height ; y++)
    //    for( int x=0 ; x<_occlusionMask->width ;x++)
    //        CV_IMAGE_ELEM(_occlusionMask,unsigned char,y,x) = 125;

    //Compute bounding rectangle to the content
    double minX , maxX , minY , maxY;
    visionUtilities::findEnclosingRectangle(_polygon,minX,minY,maxX,maxY);
    minX -= constants::CONTENT_MARGIN ; minY -= constants::CONTENT_MARGIN;
    maxX += constants::CONTENT_MARGIN ; maxY += constants::CONTENT_MARGIN;

    //Compute discrete height width of the region in the image which is to be replaced with the content
    int discreteWidth  = (int)(ceill(maxX) - floor(minX) + 1);
    int discreteHeight = (int)(ceill(maxY) - floor(minY) + 1);
    int discreteX = (int)floor(minX);
    int discreteY = (int)floor(minY);

    //The actual rectangle enclosing the occlusion window within the actual occlusion mask
    IplImage *occWindow = cvCreateImage(cvSize(discreteWidth,discreteHeight),IPL_DEPTH_8U,1);
    
    
    //Check if each pixel is occluded or not
    for( int y=discreteY; y<discreteY + discreteHeight ; y++ )
        for( int x=discreteX ; x<discreteX + discreteWidth ; x++ )
        {
             //Filter points in the occlusion model if they lie outside the image 
             if( x - constants::MARGIN < 0 || x + constants::MARGIN - IDashColor->width  > 0 ||
                 y - constants::MARGIN < 0 || y + constants::MARGIN - IDashColor->height  > 0 )
                    continue;
                                

            CvScalar currentValue = visionUtilities::getRGB(IDashColor,x,y);

            //Find value of current pixel in clean image
            double Hx = x , Hy = y;
            visionUtilities::projectCoordinate(_cleanTransform,Hx,Hy);
            CvScalar cleanValue = visionUtilities::subPixelRGB(_cleanColorImage,Hx,Hy);

            if ( visionUtilities::isOutlier(cleanValue,currentValue,_meanModelVariances) )
                CV_IMAGE_ELEM(occWindow,unsigned char,y-discreteY,x-discreteX) = 255;
            else
                CV_IMAGE_ELEM(occWindow,unsigned char,y-discreteY,x-discreteX) = 0;
        }

        
        //Perform morphological operations to fill small holes and erode small noise occlusion patches

        //Remove false occlusions
        visionUtilities::removeSmallComponents(occWindow,constants::CC_AREA_THRESHOLD,0,1);

        //Remove holes in occlusions
        visionUtilities::removeSmallComponents(occWindow,constants::CC_AREA_THRESHOLD,1,1);

        
        //cvDilate(occWindow,occWindow,NULL,4);
        //cvErode(occWindow,occWindow,NULL,5);
        cvDilate(occWindow,occWindow,NULL,1);
        cvErode(occWindow,occWindow,NULL,1);
       
       
       

       //Push the occlusion window back into the final occlusion mask
        for( int y=discreteY; y<discreteY + discreteHeight ; y++ )
            for( int x=discreteX ; x<discreteX + discreteWidth ; x++ )
                if( !(x < 0 || x >= _occlusionMask->width   ||
                      y < 0 || y >= _occlusionMask->height  ) )
                    CV_IMAGE_ELEM(_occlusionMask,unsigned char,y,x) = CV_IMAGE_ELEM(occWindow,unsigned char,y-discreteY,x-discreteX);

        cvReleaseImage(&occWindow);
}


//Projects the polygon with the given transformation matrix
void contentTracks::projectPolygon(CvMat *H)
{
    for( unsigned i=0 ; i<_polygon.size() ; i++)
        visionUtilities::projectCoordinate(H,_polygon[i].x,_polygon[i].y);             
}

//Projects the affine polygon with the given transformation matrix
void contentTracks::projectAffinePolygon(CvMat *A)
{
    for( unsigned i=0 ; i<_affinePolygon.size() ; i++)
        visionUtilities::projectCoordinate(A,_affinePolygon[i].x,_affinePolygon[i].y); 
}

//Copies the contents from the given object to the data members
void contentTracks::copy(contentTracks &cTracks)
{
    _affinePolygon = cTracks._affinePolygon;
    _polygon = cTracks._polygon;    

    if( cTracks._occlusionMask != NULL )
        _occlusionMask = cvCloneImage(cTracks._occlusionMask);

    if( cTracks._H != NULL )
        _H = cvCloneMat(cTracks._H );

    _contentID = cTracks._contentID ;

    if( cTracks._cleanTransform != NULL )
        _cleanTransform = cvCloneMat(cTracks._cleanTransform);
    
    if ( cTracks._cleanColorImage != NULL )
        _cleanColorImage = cvCloneImage(cTracks._cleanColorImage);
    

}

//Frees all the memory being used
void contentTracks::freeMemory()
{       
    if( _H != NULL )
        cvReleaseMat(&_H);

    if( _cleanTransform != NULL )
        cvReleaseMat(&_cleanTransform);

    if( _occlusionMask != NULL )
        cvReleaseImage(&_occlusionMask);

    if( _cleanColorImage != NULL )
        cvReleaseImage(&_cleanColorImage);

    _polygon.clear();
    _affinePolygon.clear();
        
}

//Writes all the content track data to disk
void contentTracks::write(ofstream &out , int index , double scale)
{
    visionUtilities::scalePolygon(_polygon,1.0 / scale);

    out<<endl<<"        <contentT id=\""<<index<<"\" cid=\""<<_contentID<<"\">"<<endl;
    
    xmlUtilities::writePoly(out , _polygon);
    
    out<< "       </contentT>"<< endl;

    visionUtilities::scalePolygon(_polygon,scale);
}

//Reads all the content track meta data from the xml node
void contentTracks::read(XMLNode trackNode , double scale)
{
    _contentID = atoi(trackNode.getAttribute("cid"));
    _polygon = xmlUtilities::readPoly(trackNode.getChildNode("poly"));

    visionUtilities::scalePolygon(_polygon,scale);
       

    _affinePolygon = _polygon;
}



//********************************************************************************************************


//************************************* CLASS frame **********************************************************


//****************** Constructor Logic ***********************

frame::frame()
{
    //Initialize all image objects to NULL
    _colorImage = NULL; 
    _grayScaleImage = NULL;  
    _siftFeatures = NULL;
}


//****************** Methods ***********************

//Retrieves the contentTracks associated with the current content ID
int frame::getContentTracks(int contentIndex , contentTracks &tracks)
{
    int index = -1;

    //Find the content tracks in the given frame corresponding to the given content ID
    for( unsigned i=0 ; i<_contentTracks.size() ; i++)
    {
        if( _contentTracks[i]._contentID == contentIndex )
        {
            tracks = _contentTracks[i];
            index = i;
            break;
        }
    }

    
    return index;
}

//Sets the contentTracks associated with the current content ID
int frame::setContentTracks(int contentIndex , contentTracks &tracks)
{
    _contentTracks.push_back(tracks);
    
    return 1;
}


//Saves the color image to disk
void frame::save(string filename)
{
    cvSaveImage(filename.c_str(), _colorImage);
}

//Loads the color and grayscale images from disk
void frame::load(string filename , double scale)
{
   _grayScaleImage = cvLoadImage(filename.c_str() , CV_LOAD_IMAGE_GRAYSCALE);
   _colorImage = cvLoadImage(filename.c_str() , CV_LOAD_IMAGE_COLOR);

   if( scale != constants::DEFAULT_TRACK_SCALE )
   {
       utilities::resizeInPlace(&_grayScaleImage,scale);
       utilities::resizeInPlace(&_colorImage,scale);
   }
}

//Copies the contents of the given frame into the data members
void frame::copy(frame &cFrame)
{
    if( cFrame._colorImage != NULL)
        _colorImage = cvCloneImage( cFrame._colorImage );

    if( cFrame._grayScaleImage != NULL )
        _grayScaleImage = cvCloneImage( cFrame._grayScaleImage);

    _contentTracks = vector<contentTracks>(cFrame._contentTracks.size());
    for( unsigned i=0 ; i<_contentTracks.size() ; i++)
        _contentTracks[i].copy(cFrame._contentTracks[i]);
    
    _index = cFrame._index;
    
}

//Cleans up resources which are not freed when a vector entry is deleted
void frame::freeMemory()
{    
    if( constants::VERBOSE )
        cout<<"\n Deleting memory for frame "<<_index;

    //Do not free memory for stub frames
    if( _index != -1 )
    {     
        if( _colorImage != NULL )
             cvReleaseImage(&_colorImage);

        if( _grayScaleImage != NULL )
             cvReleaseImage(&_grayScaleImage);

        //Free memory used by all the content on the current frame
        for( unsigned i=0 ; i<_contentTracks.size() ; i++)
            _contentTracks[i].freeMemory();
	
	if( _siftFeatures != NULL )
         FreeStoragePool((_index % (constants::VOLUME_MARGIN * 7) ) +  constants::SIFT_PREFIX_FOR_KEY_POOL);
    }
    
}


//Writes all the frame data to disk
void frame::write(ofstream &out , string metaDataFolder , double scale , vector<content> &cContent , bool flashMode)
{
    
    //Do not write stub frames
    if( _index != -1 && _contentTracks.size() > 0)
    {

	CvMat *maskTransform = NULL; //Holds the transform from the ad-space quad to a unit size image
    
        //If supporting meta data has to be stored
        if( metaDataFolder != "" )
        {
	    string occExt = flashMode ? constants::EXT_FOR_FLASH_OCC_MASK : constants::EXT_FOR_OCC_MASK;
	    
            //Write occlusion mask for each content being trakced within this frame
            for( unsigned i=0 ; i<_contentTracks.size() ; i++)
            {
                string filename = metaDataFolder + "//" + utilities::toString(_index) + "_" + utilities::toString(i) + occExt;
                
                if( _contentTracks[i]._occlusionMask != NULL && cContent[ _contentTracks[i]._contentID ]._occlusionType == 1)
                {
                    if( scale != constants::DEFAULT_TRACK_SCALE )
                    {
                        IplImage *temp = cvCloneImage(_contentTracks[i]._occlusionMask);

                        utilities::resizeInPlace(&temp,1.0 / scale);

                        cvSaveImage(filename.c_str() , temp );
                        cvReleaseImage(&temp);
                    }
                    else
		    {
			if( !flashMode )
	                        cvSaveImage(filename.c_str() , _contentTracks[i]._occlusionMask );
		    	else
			{
			    //Compute bounding rectangle to the content
			    double minX , maxX , minY , maxY;
			    visionUtilities::findEnclosingRectangle(_contentTracks[i]._polygon,minX,minY,maxX,maxY);
			    //minX -= constants::CONTENT_MARGIN ; minY -= constants::CONTENT_MARGIN;
			    //maxX += constants::CONTENT_MARGIN ; maxY += constants::CONTENT_MARGIN;

			    //Compute discrete height width of the region in the image which is to be replaced with the content
			    int discreteWidth  = (int)(ceill(maxX) - floor(minX) + 1);
			    int discreteHeight = (int)(ceill(maxY) - floor(minY) + 1);
			    int discreteX = (int)floor(minX);
			    int discreteY = (int)floor(minY);

			   //Extract rectangle around the occlusion maks
			   IplImage *maskArea = cvCreateImage( cvSize( discreteWidth , discreteHeight), IPL_DEPTH_8U,1);
			   IplImage *colorMaskArea = cvCreateImage( cvSize( discreteWidth , discreteHeight), IPL_DEPTH_8U,3);
			   cvSetImageROI(_contentTracks[i]._occlusionMask,cvRect(discreteX,discreteY,discreteWidth,discreteHeight));
			   cvCopy(_contentTracks[i]._occlusionMask,maskArea);
			   cvResetImageROI(_contentTracks[i]._occlusionMask);

			   cvCvtColor(maskArea,colorMaskArea,CV_GRAY2RGB);
        		   cvSmooth(colorMaskArea,colorMaskArea,CV_GAUSSIAN,constants::OCC_EDGE_SMOOTH_SIDE,constants::OCC_EDGE_SMOOTH_SIDE);        

			   //Save mask for use by the flash player
			   cvSaveImage(filename.c_str(),colorMaskArea);
			   cvReleaseImage(&maskArea);
			   cvReleaseImage(&colorMaskArea);

			   //vector<svlPoint2d> unitPolygon = visionUtilities::formPolygonFromRect(cvRect(0,0,360,450));
		 	   //Compute transform mapping region within new mask coordinate system to a unit content
			   //maskTransform = visionUtilities::findMappingBetweenPolygons(unitPolygon , _contentTracks[i]._polygon , discreteX , discreteY);
			
				
		        }	
		    }
                }//if valid
            }//for i
        }

        out<<"    <frame id=\""<<_index<<"\">";
        
        for( unsigned i=0 ; i<_contentTracks.size() ; i++)
            _contentTracks[i].write(out,i,scale);
        
        out<< "    </frame>"<< endl;

	if( maskTransform != NULL )
		cvReleaseMat(&maskTransform);
    }
}

//Reads all the frame meta data from xml node
void frame::read(XMLNode &frameNode , string metaDataFolder , double scale)
{
    _index = atoi(frameNode.getAttribute("id"));
    
    //Read in all the content information
    for (int i = 0; i < frameNode.nChildNode("contentT"); i++) 
    {
        XMLNode trackNode = frameNode.getChildNode("contentT", i);
        contentTracks currentTrack;
        currentTrack.read(trackNode,scale);
        
        _contentTracks.push_back(currentTrack);
    }

    if( metaDataFolder != "" )
    {
        //Check for occlusion mask images of each content tracks 
        for( unsigned i=0 ; i<_contentTracks.size() ; i++)
        {
            string filename = metaDataFolder + "//" + utilities::toString(_index) + "_" + utilities::toString(i) + constants::EXT_FOR_OCC_MASK;
            string oldFilename = metaDataFolder + "//" + utilities::toString(_index) + "_" + utilities::toString(i) + constants::OLD_EXT_FOR_OCC_MASK;
                
            if( utilities::doesFileExist(filename) )
                _contentTracks[i]._occlusionMask = cvLoadImage(filename.c_str() , CV_LOAD_IMAGE_GRAYSCALE);
            else
                _contentTracks[i]._occlusionMask = cvLoadImage(oldFilename.c_str() , CV_LOAD_IMAGE_GRAYSCALE);
            
            if( scale != constants::DEFAULT_TRACK_SCALE && _contentTracks[i]._occlusionMask != NULL)
                utilities::resizeInPlace( &_contentTracks[i]._occlusionMask , scale);
        }
    }
    

}

//Replaces the occlusion mask with the new one
void frame::replaceOccMask(string metaDataFolder , unsigned i)
{
    string filename = metaDataFolder + "//" + utilities::toString(_index) + "_" + utilities::toString(i) + constants::EXT_FOR_OCC_MASK;
    string oldFilename = metaDataFolder + "//" + utilities::toString(_index) + "_" + utilities::toString(i) + constants::OLD_EXT_FOR_OCC_MASK;
    
    if( utilities::doesFileExist(filename) )
        cvSaveImage(filename.c_str(),_contentTracks[i]._occlusionMask);
    else
        cvSaveImage(oldFilename.c_str(),_contentTracks[i]._occlusionMask);            
}



//********************************************************************************************************




//************************************* CLASS videoVolume *************************************************

//****************** Constructor Logic ***********************
videoVolume::videoVolume()
{
    _currentPart = 0;    
    _currentIndexInPart = 0;
    _stubCounter = 0;
    _totalFrames = 0;
    _isFinished = false;
}

//****************** Methods ***********************

//Frees any memory being used by the volume
void videoVolume::freeMemory()
{
    _volumeParts.clear();
}

//Checks if the given frame index is within the volume
bool videoVolume::isWithinVolume(unsigned index)
{
    bool isWithin = false;

    //Check each part of vaidity
    for( unsigned p=0 ; p<_volumeParts.size() ; p++)
    {
        if( _volumeParts[p].x <= (int)index && (int)index <= _volumeParts[p].y )
        {
            isWithin = true;
            break;
        }
    }
    
    return isWithin;
}

//Initializes the part indicies after the volume has been read
void videoVolume::initializeIndices(bool isReverse)
{
    //if volume is not empty
    if( _volumeParts.size() > 0 )
    {
        if( !isReverse )
        {
            _currentPart = 0;
            _currentIndexInPart = _volumeParts[_currentPart].x;
        }
        else
        {
            _currentPart = (int)(_volumeParts.size() - 1);
            _currentIndexInPart = _volumeParts[_currentPart].y;
        }
    }
}

//Computes the index of the next frame to be read
void videoVolume::computeNextFrameIndex(bool isReverse)
 {
     //If end of volume has not been reached
     if( !_isFinished )
     {
         if( !isReverse )
         {
             //If current index is not within current volume part
             if( (int)_currentIndexInPart >= _volumeParts[_currentPart].y )
             {
                 _currentPart++;

                 if( _currentPart >= (int)_volumeParts.size() )
                    _isFinished = true;
                 else
                     _currentIndexInPart = _volumeParts[_currentPart].x;
             }
             else
                 _currentIndexInPart++;
         }
         else// is reverse
         {
             if( (int)_currentIndexInPart <= _volumeParts[_currentPart].x )
             {
                 _currentPart--;

                 if( _currentPart < 0 )
                    _isFinished = true;
                 else
                     _currentIndexInPart = _volumeParts[_currentPart].y;
             }
             else
                 _currentIndexInPart--;
         }

         _totalFrames++;

     }// != -1
     else
     {
         _stubCounter++;
     }


 }

//********************************************************************************************************




//************************************* CLASS video **********************************************************


//****************** Constructor Logic ***********************

video::video()
{
    //Does nothing as of now
    _scale = constants::DEFAULT_TRACK_SCALE;
    _render = true;
    _reverse = false;
    _flashMode = false;
}


//****************** Methods ***********************

//Returns the total frames within the video
unsigned int video::totalFrames()
{
	return (unsigned int)_filenames.size();
}

//Returns the full output filename for the frame "index"
string video::fullOutputFilename(int index)
{
    return _outputFolder + string("//") + _filenames[index];    
}

//Returns the full input filename for the frame "index"
string video::fullInputFilename(int index)
{
    return _sourceFolder + string("//") + _filenames[index];    
}

//Populates all the content names
void video::populateContentFilenames(string mainDirectory)
{
      for( unsigned i=0 ; i<_content.size() ; i++)
        _content[i].populateFilenames(mainDirectory);   
}

//Populates all the frame names
void video::populateFilenames(string mainDirectory)
{	    
    //Populate data members
    _sourceFolder = mainDirectory + "//" + _name;
    _outputFolder = _sourceFolder + "_out";

    _filenames = utilities::loadFilenames(_sourceFolder,_extension);
}


//Copies the contents onto the given video object
void video::copy(video &cVideo)
{
    _content = vector<content>(cVideo._content.size());
        
    for( unsigned i=0 ; i<_content.size() ; i++)
        _content[i].copy(cVideo._content[i]);
   

    _extension = cVideo._extension;
    _filenames = cVideo._filenames;
    _format = cVideo._format;
    _name = cVideo._name;
    _outputFilename = cVideo._outputFilename;
    _outputFolder = cVideo._outputFolder;
    _sourceFolder = cVideo._sourceFolder;
    _render = cVideo._render;
    _videoVolumes = cVideo._videoVolumes;
}

//Cleans up resources 
void video::freeMemory()
{         
    //Free memory used by all the frames

    for( unsigned i=0 ; i<_frames.size() ; i++)
        for( unsigned j=0 ; j<_frames[i].size() ; j++)
            _frames[i][j].freeMemory();

    //Free memory used by all the content
    for( unsigned i=0 ; i<_content.size() ; i++)
        _content[i].freeMemory();
    
    for( unsigned v=0 ; v<_videoVolumes.size() ; v++)
        _videoVolumes[v].freeMemory();
}



//Reads all the video metadata into the object
void video::read(string filename , string mainDirectory , string contentDirectory , string metaDataDirectory , bool readFrames)
{

    cout<<"\nReading video metadata from filename - "<<filename.c_str()<<"\n";

    _metaDataFolder = metaDataDirectory;

    //Open link to the main video tag
    _videoNode = XMLNode::parseFile(filename.c_str(), "video");

    readInitial(mainDirectory);

    if( contentDirectory != "")
    {
        cout<<"\n Populating content filenames";

        //Populating filenames for ads for all content region
        populateContentFilenames(contentDirectory);
    }

   if( readFrames )
   {
        cout<<"\n Reading limited frame metadata\n";

        //Clean existing data structures
        _frames.clear();

        //Fill stub frames for each of the volumes
        for( unsigned v=0 ; v<_videoVolumes.size() ; v++ )
        {
            _frames.push_back(vector<frame>());

            //For the first few frames the past is not availible, so insert stub
            for( unsigned i=0 ; i<constants::VOLUME_MARGIN * 2; i++)
            {
                frame tempFrame;
                tempFrame._index = -1;
                _frames[v].push_back(tempFrame);
            }
        }
        
        //Populate actual frames for each of the video volumes
        for( unsigned v=0 ; v<_videoVolumes.size() ; v++ )
        {
            cout<<"\n volume id "<<v;
            
            //Read the max number of frames into the memory including the images
            for( unsigned i=0 ; _frames[v].size() < 3 * constants::VOLUME_MARGIN + 1 ; i++)
            {
                cout<<"\n frame number "<<_videoVolumes[v]._currentIndexInPart;

                //If volume exceeds total frames in video
                if( _videoVolumes[v]._currentIndexInPart >= _filenames.size() || _videoVolumes[v]._isFinished )
                {
                    frame tempFrame;
                    tempFrame._index = -1;
                    _frames[v].push_back(tempFrame);
                }
                else
                {                
                    _frames[v].push_back(readFrame(_videoVolumes[v]._currentIndexInPart));
                    _videoVolumes[v].computeNextFrameIndex(_reverse);
                }
            }
        }

   }//readFrames

    
    cout<<"\nDone reading video metadata\n";

}

//Checks if the current frame data is availible in the xml node
char video::isFrameAvailible(unsigned index)
{
    
    return !_videoNode.getChildNodeWithAttribute("frame","id",utilities::toString(index).c_str()).isEmpty();     
}

//Reads the given frame index from disk as well as the metadata file
frame video::readFrame(unsigned index , bool load)
{
     frame currentFrame;
               
     XMLNode frameNode = _videoNode.getChildNodeWithAttribute("frame","id",utilities::toString(index).c_str());

     if( !frameNode.isEmpty())
        currentFrame.read(frameNode,_metaDataFolder,_scale);

              
    currentFrame._index = index;

    if( load )
        currentFrame.load( fullInputFilename(index) , _scale);

    return currentFrame;
     
}

//Reads the frame metadata based on <index> in sequence of metadata availbile not frame index
frame video::readFrameSeq(unsigned index)
{
    frame currentFrame;
               
    XMLNode frameNode = _videoNode.getChildNode("frame",index);//("frame","id",utilities::toString(index).c_str());

     if( !frameNode.isEmpty())
        currentFrame.read(frameNode,_metaDataFolder,_scale);
     else
         currentFrame._index = -1; //Signifies end of meta data

    return currentFrame;
}


//Reads all the initial video and content related video metadata into the object
void video::readInitial(string mainDirectory)
{
   
    //Read video attributes
    _name = string(_videoNode.getAttribute("name"));
    _extension = string(_videoNode.getAttribute("ext"));
    _format = string(_videoNode.getAttribute("format"));
    
    int volumeSize = -1;
    
    if( _videoNode.isAttributeSet("vol" ) )
       volumeSize = atoi(_videoNode.getAttribute("vol"));
    
    if( mainDirectory != "" )
    {
        cout<<"\n Populating filenames for each frame\n";
        populateFilenames(mainDirectory); //Populate all the video filenames
    }

     unsigned totalFrames = (unsigned)_filenames.size();

    cout<<"\n Reading content metadata\n";

    //Clean existing data structures
    _content.clear();

    //Read in all the content information
    for (int i = 0; i < _videoNode.nChildNode("content"); i++) 
    {
        XMLNode contentNode = _videoNode.getChildNode("content", i);
        content currentContent;
        currentContent.read(contentNode , _reverse , totalFrames);
        
        _content.push_back(currentContent);
    }

    //Read volume data only if availible
    if( volumeSize != -1 )
    {
        cout<<"\n Reading volume metadata\n";

        //Read in all the video volume information
        for( int v=0 ; v<volumeSize ; v++)
        {
            XMLNode volumeNode = _videoNode.getChildNodeWithAttribute("volume","id",utilities::toString(v).c_str());

            if( !volumeNode.isEmpty() )
            {
                 videoVolume temp;
                 temp._volumeParts = xmlUtilities::readVolume(volumeNode); //Read the volume indices
                 temp.initializeIndices(_reverse);   
                 _videoVolumes.push_back(temp);

            }//if non empty xml node
                
        }
    }//if volume is present in metadata
    else
    {
        videoVolume temp;
        temp._volumeParts.push_back(cvPoint(0,(int)(_filenames.size()-1)));
        temp.initializeIndices(_reverse);
        _videoVolumes.push_back(temp);
    }

}

//Reads all the video metadata into the object
void video::write(string filename)
{

    if( constants::VERBOSE )
        cout<<"\n Writing video meta data to file - "<<filename.c_str()<<"\n";

    //Open filestream to target file
    ofstream out(filename.c_str() , ios::out);

    out<<"<video name=\""<<_name.c_str()<<"\" ext=\""<<_extension.c_str()<<"\" format=\""<<_format.c_str()<<"\">"<<endl;
    
    out<<endl;

    if( constants::VERBOSE )
        cout<<"\n Writing content metadata";

    //Write each of the content entries
    for( unsigned i=0 ; i<_content.size() ; i++)
        _content[i].write(out,i);

    out<<endl;

    cout<<"\n Writing frame metadata";

    //Write each of the frame entires
    for( unsigned i=0 ; i<_frames.size() ; i++)
        for( unsigned j=0 ; j<_frames[i].size() ; j++)
        _frames[i][j].write(out , string(""),_scale,_content,_flashMode);

    out<<endl<<"</video>";

    out.close();

    cout<<"Done writing video metadata";
}

//Writes all the initial video and content related video metadata into the metadata file
void video::writeInitial()
{
    if( _outputFilename != "" )
    {
        unsigned totalFrames = (unsigned)_filenames.size();

        _out.open(_outputFilename.c_str() , ios::out);

        cout<<"\n Writing video meta data to file \n";
        
        _out<<"<video name=\""<<_name.c_str()<<"\" ext=\""<<_extension.c_str()<<"\" format=\""<<_format.c_str()<<"\" vol=\""<<_videoVolumes.size();
        _out<<"\">"<<endl;
        
        _out<<endl;

        cout<<"\n Writing content metadata";

        //Write each of the content entries
        for( unsigned i=0 ; i<_content.size() ; i++)
            _content[i].write(_out,i,_reverse,totalFrames);

        _out<<endl;

        for( unsigned v=0 ; v<_videoVolumes.size() ; v++)
            xmlUtilities::writeVolume(_out,_videoVolumes[v]._volumeParts,v);
           
        _out<<endl;
    }

}

//Writes the given frame index to the metadata file
void video::writeFrame(unsigned volume, unsigned index)
{
    if( _outputFilename != "" )
        _frames[volume][index].write(_out , _metaDataFolder,_scale,_content,_flashMode);
}

//Wraps up the staged meta data writing process
void video::writeFinal()
{
    if( _outputFilename != "" )
    {
        _out<<endl<<"</video>";
        _out.close();
    }
}

//Loads the given frame index
void video::loadFrame(unsigned volume , unsigned index)
{
    _frames[volume][index].load(fullInputFilename(_frames[volume][index]._index) , _scale);
}

//Saves the color image of the given frame index to disk
void video::saveFrame(unsigned volume , unsigned index)
{
    _frames[volume][index].save(fullOutputFilename(_frames[volume][index]._index));    
}


//********************************************************************************************************


