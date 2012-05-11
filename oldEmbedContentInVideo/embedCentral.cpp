/*****************************************************************************
** MOVIE PROJECT
** Copyright (c) 2008 
**
** FILENAME:    embedCentral.h
** AUTHOR(S):   Sid Batra <sidbatra@cs.stanford.edu>
** DESCRIPTION:
** Implementations of the functions in embedCentral.h
*****************************************************************************/


#include "embedCentral.h"



//****************************** Constructor Logic ********************
embedCentral::embedCentral()
{
	//Set current tracker index to -1
    _currentTrackerIndex = -1;
}


//******************************* Methods *********************************

void embedCentral::embedContentAtLocations(std::string folderName, string extension
                                           , std::vector<vector<svlPoint2d> > polygons, std::vector<int> startingFrameNumbers 
                                           , vector<int> endFrameNumbers , vector<vector<string> > icontentFilenames 
                                           ,string outputFolderName, int skipFrames
                                           , int outputMode , vector<int> contentType 
                                           , int firstFrameDebug , int smooth , int norepeats , bool isReverse)
{	



    //************* Init local variables *****************
    _contentFilenames = icontentFilenames;
     vector<string> frameNames; //Holds all the filenames of the video
     vector<int> frameCountsForTrackers;
     vector<vector<svlPoint2d> > affinePoly;
    int isProcessing = 0; //Is a region currently being processed
    int cutScene = 0;
    int totalRegions = (int)startingFrameNumbers.size(); //holds the total number of regions to be tracked
    int totalFrames = -1;
     int contentFrameIndex = 0;//Holds current content frame index
    int smoothingSide = smooth; //Number of frames before and after with whom values of coordinates are to be averaged
    IplImage* previousImage = NULL;//Holds image of the previous frame
	IplImage* previousFrame = NULL;//Holds image of the previous frame in color
    IplImage *previousTemp = NULL; //Used for cut scene detection        
    _isReverseTracking = isReverse;  //Inits style of tracking
    string metaDataFilename = outputFolderName + string("_sift//metaData.txt");
    //***************************************************************************

        //Store all video frames
     cout<<"\n Storing video frame names\n";
     utilities::loadAllFilenames(folderName,extension,frameNames,_isReverseTracking);
     totalFrames = (int)frameNames.size();

     //******** Populate effect lengths *********
     _trackLength = vector<int>(totalRegions);
     for( int i=0 ; i<totalRegions ; i++)
     {
         if( endFrameNumbers[i] != INT_MAX && contentType[i] == LOGO_WITH_EFFECT)
             _trackLength[i] = endFrameNumbers[i] - startingFrameNumbers[i];
         else
             _trackLength[i] = -1;
     }
    
     //******************************************

     if( _isReverseTracking )
     {
         //Take the compliment of the starting & ending frame numbers
         for( int i=0 ; i<totalRegions ; i++)
         {
             startingFrameNumbers[i] = totalFrames - 1 - startingFrameNumbers[i];

             if( endFrameNumbers[i] != INT_MAX )
                endFrameNumbers[i] = totalFrames - 1 - endFrameNumbers[i];
         }

             reverse(startingFrameNumbers.begin(), startingFrameNumbers.end());
             reverse(_trackLength.begin(), _trackLength.end());
             reverse(endFrameNumbers.begin(), endFrameNumbers.end());
             reverse(_arrowDirection.begin(), _arrowDirection.end());
             reverse(_contentFilenames.begin(),_contentFilenames.end());
             reverse(polygons.begin() , polygons.end());
             reverse(contentType.begin() , contentType.end());
     }
          
 

    //Holds total number of frames rendered per tracker
    _framesPerTracker = vector<int>(totalRegions);
    for( int i=0 ; i<totalRegions ; i++)
        _framesPerTracker[i] = 0;

    //Reset or preserve data members for the reverse tracking iteration
    if( !_isReverseTracking )
        _arrowDirection = vector<int>(totalRegions);
    
    
	svlSIFT::_norepeats = norepeats;
		
	//Make a copy of the points to be tracked, these points
	//are tracked via affine transforms only
	for( unsigned i=0 ; i<polygons.size() ; i++)
		affinePoly.push_back( polygons[i] );


    cout<<"\n Processing video frames \n";

	//Iterate through and process each of the frames
    for( int videoFrameIndex=0 ; videoFrameIndex< (int)frameNames.size() ; videoFrameIndex++ )
    {           
	    //Make decision on whether current frame has renderable content or not
        if( 
            (videoFrameIndex < skipFrames)
            || 
            (videoFrameIndex !=
            startingFrameNumbers[_currentTrackerIndex < totalRegions-1 ? _currentTrackerIndex+1 : _currentTrackerIndex]
            && !isProcessing)
            ||
            ( isProcessing && videoFrameIndex >= endFrameNumbers[_currentTrackerIndex])
            )
			
            {

                isProcessing = 0;
			
                cout<<"\n Testing for cut scenes and camera switches";

                string imageFilename  = folderName + string("/") + frameNames[videoFrameIndex];
                string outputFilename = outputFolderName + string("/") + frameNames[videoFrameIndex];
                int isFound = 0;//Whether the current cut scene belongs to one of the previous region images
                

                

                //Load grayscale version of current frame for cut scene estimation
                IplImage *currentTemp = cvLoadImage(imageFilename.c_str() , CV_LOAD_IMAGE_GRAYSCALE); 

                //if( previousTemp != NULL && visionUtilities::estimateCutScene(previousTemp,currentTemp) )
                //    cutScene = 1;
                

                if( previousTemp != NULL )
                   cvReleaseImage(&previousTemp);

                previousTemp = cvCloneImage(currentTemp);


                /* //Compare with all the trackers to check camera switch
                if( cutScene == 1 && !_isReverseTracking)
                {                    
                    cutScene = 0 ;
                    
                    for( int j=0 ; j<(int)_objectTrackers.size() ; j++)
                    {                       
                        if(  videoFrameIndex >= endFrameNumbers[j] )
                            continue;

                            //If current scene is similar to precut scene of region j
                            if( !_objectTrackers[j].estimateCutScene(currentTemp) )
                            {
                                cout<<"\n Found - matches tracker "<<j;

                                if( previousImage != NULL)
                                    cvReleaseImage(&previousImage);

                                if( previousFrame != NULL)
                                    cvReleaseImage(&previousFrame);

                                previousImage = cvCloneImage( _objectTrackers[j].getPreCut() );
                                previousFrame = cvCloneImage( _objectTrackers[j].getPreCutColor() );
                                                                
                                populateVideoColorStats(previousFrame);
                                _currentTrackerIndex = j;
                                isFound = 1;
                                break;
                            }
                        
                    }

                    
            } //if cut scene */

            cvReleaseImage(&currentTemp);

            if( !isFound  )
             {
                cout<<"\nSkipping and saving frame "<<frameNames[videoFrameIndex];

                if (!_isReverseTracking )
                {
                    //Load save and release the frame to be skipped
                    IplImage *temp = cvLoadImage(imageFilename.c_str() , CV_LOAD_IMAGE_COLOR);
                    cvSaveImage(outputFilename.c_str() , temp);
                    cvReleaseImage(&temp);
                }

			    continue;
             }

        }//if 
        else
        {
            //Holds whether a new region needs to be started
            int isFreshStart = videoFrameIndex ==
            startingFrameNumbers[_currentTrackerIndex < totalRegions-1 ? _currentTrackerIndex+1 : _currentTrackerIndex];

            //If a cut scene lead to this shift make a precut copy
            if( isFreshStart  && isProcessing  )
                _objectTrackers[_currentTrackerIndex].savePreCutScene(previousImage,previousFrame);
            
            
            //Free previous temp image as tracking has started
            if( previousTemp != NULL )
                cvReleaseImage(&previousTemp);
            previousTemp = NULL;

            if( isFreshStart  )
            {
                _currentTrackerIndex++; //Start looking for next region to track
                _objectTrackers.push_back(objectTracker());
            }

        }
        

            isProcessing = 1; //A tracker is now being started or resumed
            			
			//Form input filenames for the current frame
			string imageFilename ;

			//Choose frame name based on first frame debug option
			if( !firstFrameDebug)
				imageFilename = folderName + string("/") + frameNames[videoFrameIndex];
			else
				imageFilename = folderName + string("/") + frameNames[0];

			string outputFilename = outputFolderName + string("/") + frameNames[videoFrameIndex];
            //string outputFilename = outputFolderName + string("/");
            
            //#DEBUG
            //objectTracker::_tempOut = outputFolderName + string("_sift/") + frameNames[videoFrameIndex] + ".txt";
            //#DEBUG

			//Read current frame from file in color and grayscale
			IplImage *currentImage = cvLoadImage(imageFilename.c_str() , CV_LOAD_IMAGE_GRAYSCALE);
			IplImage *currentFrame = cvLoadImage(imageFilename.c_str() , CV_LOAD_IMAGE_COLOR);
						
        		
			cout<<"\n\nProcessing image "<<imageFilename.c_str();

            
		    //Tracking can only begin when a minimum of 2 frames have been loaded
            if( videoFrameIndex != skipFrames && videoFrameIndex != startingFrameNumbers[_currentTrackerIndex] )
		    {
                
                //Swtich to LAB color model
                cvCvtColor(currentFrame,currentFrame,CV_RGB2Lab);
                cvCvtColor(previousFrame,previousFrame,CV_RGB2Lab);

			    //Call to object tracker, maps given polygon to its best estimated location 
                //in the next frame
                int isFinished = _objectTrackers[_currentTrackerIndex].trackObjectsWithHomography(previousImage, currentImage
                    , polygons[_currentTrackerIndex] ,affinePoly[_currentTrackerIndex], previousFrame, currentFrame);

                //If the region is no longer to be tracked in the video (for now)
                if( isFinished == 1)
                {   
                    isProcessing = 0;

                    if( !_isReverseTracking )
                        cutScene = 1;

                    videoFrameIndex--; //Look at the current frame again from perspective of next region

                    cvReleaseImage(&currentFrame);
                    cvReleaseImage(&currentImage);

                    continue;
                                        
                }
                                   
                //Convert current frame back to RGB
                cvCvtColor(currentFrame,currentFrame,CV_Lab2RGB);
               
		    }
		    else
            {
                //Populate video statistics needed for the blending process
			    populateVideoColorStats(currentFrame);
            }

            
            //#DEBUG#
            //Form debug filename to append the best particle ID to it
            //string test = frameNames[videoFrameIndex];            
            //test.replace(test.length() - 4 , 6, string("_") + utilities::toString(objectTracker::_selectedParticle) + extension );
            //outputFilename += test;
            //#DEBUG#
      
            
            //Free up previous frames
		    if( previousFrame != NULL )
			    cvReleaseImage(&previousFrame);

            if(previousImage != NULL)
				    cvReleaseImage(&previousImage);
     
           			
		    //Make color & bw copies of the previous frame
		    previousFrame = cvCloneImage(currentFrame);
		    previousImage = cvCloneImage(currentImage);
   
	ofstream outStream(metaDataFilename.c_str(),ios::app);

		//Save meta data 
		for(int m=0;m<polygons[_currentTrackerIndex].size() ;m++)
		{
			outStream<<polygons[_currentTrackerIndex][m].x<<" "<<polygons[_currentTrackerIndex][m].y;
			if( m != polygons[_currentTrackerIndex].size() - 1 )
				outStream<<" ";
			else
				outStream<<"\n";
		}
	outStream.close();

            //Save the current state of the frame at this time slice
            _frameInstances.push_back( frameInstance(currentFrame , _objectTrackers[_currentTrackerIndex].getCleanImage()
                , _objectTrackers[_currentTrackerIndex].getCleanTransform() , _objectTrackers[_currentTrackerIndex].getModelVariances()
                , polygons[_currentTrackerIndex] , affinePoly[_currentTrackerIndex] , startingFrameNumbers 
                , outputFilename , contentFrameIndex ) );


            //#DEBUG#
            //Restore affine coordinates to
           affinePoly.clear();

			for( unsigned i=0 ; i<polygons.size();i++)
				affinePoly.push_back(polygons[i]);
            //#DEBUG#

            //***** Start rendering content once enough frames are ready for kalman / other types of filtering *****
           if( videoFrameIndex >= smoothingSide * 2 )
               estimateLocationFromNoisyEstimates(videoFrameIndex,smoothingSide,outputMode,contentType[_currentTrackerIndex]);
    						

           //Increment indices
           contentFrameIndex = (contentFrameIndex + 1) % (int)_contentFilenames[_currentTrackerIndex].size();
		
          
		    //Free memory
			cvReleaseImage(&currentFrame);
			cvReleaseImage(&currentImage);

       }
		
	
    //HANDLE THE REMAINING LEFT OVER FRAMES HERE, THOSE LEFT DUE TO SMOOTHING

    //Free memory used by the various trackers

    for( int i=0 ; i<totalRegions ; i++)
        _objectTrackers[i].freeMemory();

    _objectTrackers.clear();
    _currentTrackerIndex = -1;
    _framesPerTracker.clear();
    _frameInstances.clear();
    _contentFilenames.clear();

	//Free the previous image being kept
	cvReleaseImage(&previousImage);
	cvReleaseImage(&previousFrame);
}


//Uses the noise estiamtes from the object tracker over various frames and 
//estimates a smoothened trajectory
void embedCentral::estimateLocationFromNoisyEstimates(int videoFrameIndex, int smoothingSide, int outputMode , int contentType)
{
    
    int frameIndex = (int)_frameInstances.size() / 2; //Index within the buffer of nosy estimates to be processed
    int actualFrameIndex = videoFrameIndex - smoothingSide; //actual index within the actual video of frame to be processed

    smoothenEstimates(frameIndex,smoothingSide);

    //If its the first instance of rendering all video frames from index 0 to index smoothingSide -1 have 
    // to be processed
    if( videoFrameIndex == smoothingSide * 2 )
    {
        for( int i=0 ; i<frameIndex ; i++)
        {
            smoothenEstimates(i,smoothingSide);

            renderContent(_frameInstances[i].colorFrame, _frameInstances[i]._cleanImage,_frameInstances[i]._cleanTransform
                ,_frameInstances[i]._modelVariances
                ,_frameInstances[i].startingFrameNumbers,i,_frameInstances[i].contentFrameIndex
                ,outputMode,_frameInstances[i].polygons,_frameInstances[i].affinePolygons,_frameInstances[i].startingFrameNumbers
                , _frameInstances[i].outputFilename , contentType);
        }
    }

    //##DEBUG## ##Saves smoothened locations to file to plot graphs##
    //ofstream out("locations.txt",ios::app);

    //for( int i=0 ; i<_frameInstances[frameIndex].polygons[0].size() ; i++)
    //{
    //    //out<<utilities::roundOff(_frameInstances[frameIndex].polygons[0][i].x)<<" "<<utilities::roundOff(_frameInstances[frameIndex].polygons[0][i].y);
    //    out<<utilities::_frameInstances[frameIndex].polygons[0][i].x<<" "<<_frameInstances[frameIndex].polygons[0][i].y;

    //    if( i != _frameInstances[frameIndex].polygons[0].size() -1 )
    //        out<<" ";
    //    else
    //        out<<"\n";
    //}
    //
    //out.close();
    //##DEBUG##
    

    renderContent(_frameInstances[frameIndex].colorFrame,_frameInstances[frameIndex]._cleanImage 
        , _frameInstances[frameIndex]._cleanTransform,_frameInstances[frameIndex]._modelVariances
        ,_frameInstances[frameIndex].startingFrameNumbers,actualFrameIndex ,_frameInstances[frameIndex].contentFrameIndex
        ,outputMode,_frameInstances[frameIndex].polygons,_frameInstances[frameIndex].affinePolygons,_frameInstances[frameIndex].startingFrameNumbers, _frameInstances[frameIndex].outputFilename ,contentType);

   //Delete the fist buffered frame instance
   _frameInstances[0].clean();
   _frameInstances.erase(_frameInstances.begin());
						
}


//Smooths the noisy estimates to estimate region position
void embedCentral::smoothenEstimates( int index , int smoothingSide )
{
    //Total vertices of the polygon to be smoothened
    int totalPoints = (int)_frameInstances[index].polygons.size();

    vector< svlPoint2d > average(totalPoints);
    
    int outliers = 0;

    for( int i = index - smoothingSide ; i<= index + smoothingSide ; i++)
    {        
        if( i >= 0  )
        {
            for( int j=0 ; j<totalPoints ; j++)
                average[j] += _frameInstances[i].polygons[j];
        }
        else
            outliers++;
    }

    for( int j=0 ; j<totalPoints ; j++)
        average[j] /= smoothingSide * 2 + 1 - outliers;

    //Replace smoothened version
    _frameInstances[index].polygons = average;

}


//Launches the correct rendering mechanism based on the output mode 
void embedCentral::renderContent(IplImage *currentFrame, IplImage *cleanImage , CvMat *cleanTransform 
                                        , CvScalar modelVariances ,vector<int> startingFrameNumbers
                                        , int videoFrameIndex,  int contentFrameIndex , int outputMode
                                        , vector< svlPoint2d> polygons 
                                        , vector< svlPoint2d> affinePoly , vector< int > startingFrameIndex
                                        , string outputFilename , int contentType )
{



    if( outputMode == CONTENT )
	{				
		//Places the content into quad being tracked
		//Think about the 2 input option, one for location, one for projection
        IplImage *temp = cvLoadImage(_contentFilenames[_currentTrackerIndex][contentFrameIndex].c_str(),CV_LOAD_IMAGE_COLOR);
		renderContentToFrame(currentFrame,temp,cleanImage,cleanTransform,modelVariances,polygons,contentType);
				
		//blurEdges(frame,polygons[0]);
	}
    else  if( outputMode == RECTANGLE || outputMode == RECT_AFFINE )//If outputMode is a form of debugging
    {
			float scaleUpFactor = 1;//11;
			int lineThickness = 1;//10;

			//Scale up content image based on the scale factor
			//IplImage *temp = cvCreateImage(cvSize(currentFrame->width * scaleUpFactor,currentFrame->height * scaleUpFactor),currentFrame->depth,currentFrame->nChannels);
			//cvResize(currentFrame,temp,1);
			//cvReleaseImage(&currentFrame);
			//currentFrame = cvCloneImage(temp);
			//cvReleaseImage(&temp);

					//Draw polygon on image and save
					for( unsigned j=1 ; j<polygons.size() ; j++)
					{				
                        
						cvDrawLine(currentFrame, cvPoint(utilities::roundOff(polygons[j-1].x * scaleUpFactor) ,utilities::roundOff(polygons[j-1].y * scaleUpFactor)),
							cvPoint(utilities::roundOff(polygons[j].x * scaleUpFactor),utilities::roundOff(polygons[j].y * scaleUpFactor)),cvScalar(0,255,0),lineThickness);							

						//If affine quad should also be displayed
						if( outputMode == RECT_AFFINE )
						{
							cvDrawLine(currentFrame, cvPoint(utilities::roundOff(affinePoly[j-1].x* scaleUpFactor) ,utilities::roundOff(affinePoly[j-1].y* scaleUpFactor)),
							cvPoint(utilities::roundOff(affinePoly[j].x* scaleUpFactor),utilities::roundOff(affinePoly[j].y* scaleUpFactor)),cvScalar(0,0,255),lineThickness);		
						}
					}

					cvDrawLine(currentFrame, cvPoint(utilities::roundOff(polygons[0].x * scaleUpFactor) ,utilities::roundOff(polygons[0].y * scaleUpFactor)),
						cvPoint(utilities::roundOff(polygons[polygons.size()-1].x * scaleUpFactor),utilities::roundOff(polygons[polygons.size()-1].y * scaleUpFactor)),cvScalar(0,255,0),lineThickness);

					//If affine quad should also be displayed
					if( outputMode == RECT_AFFINE )
					{
						cvDrawLine(currentFrame, cvPoint(utilities::roundOff(affinePoly[0].x* scaleUpFactor) ,utilities::roundOff(affinePoly[0].y* scaleUpFactor)),
							cvPoint(utilities::roundOff(affinePoly[affinePoly.size()-1].x* scaleUpFactor),utilities::roundOff(affinePoly[affinePoly.size()-1].y* scaleUpFactor)),cvScalar(0,0,255),lineThickness);
					}
		

			//temp = cvCreateImage(cvSize(currentFrame->width / scaleUpFactor,currentFrame->height / scaleUpFactor),currentFrame->depth,currentFrame->nChannels);
			//cvResize(currentFrame,temp,1);
			//cvReleaseImage(&currentFrame);
			//currentFrame = cvCloneImage(temp);
			//cvReleaseImage(&temp);

    }//else if debug mode

	//Write result of rendering / tracking
	cvSaveImage(outputFilename.c_str(),currentFrame);
}


//Populates means and variances in the different channels of the image
void embedCentral::populateVideoColorStats(IplImage *frame)
{
	//Convert to HSV color mode
	cvCvtColor(frame,frame,CV_RGB2HSV);

	int N = frame->height * frame->width;

	cout<<"\n TOTAL PIXELS IN FRAME "<<N;

	//Setup stat variables
	for( int i=0 ; i<3 ; i++)
	{
		_videoMeans[i] = 0.0;
		_videoVariances[i] = 0.0;
	}

	//***************** Compute means*******************

	for( int y=0 ; y<frame->height ; y++)
	{
		for( int x=0 ; x<frame->width ; x++)
		{
			CvScalar frameValue = visionUtilities::getRGB(frame,&x,&y);

			for( int i=0 ; i<3 ; i++)
				_videoMeans[i] += frameValue.val[i];
						
		}
	}

	for( int i=0 ; i<3 ; i++)
		_videoMeans[i] /= N;


	//****************** Compute variances *************

	for( int y=0 ; y<frame->height ; y++)
	{
		for( int x=0 ; x<frame->width ; x++)
		{
			CvScalar frameValue = visionUtilities::getRGB(frame,&x,&y);

			for( int i=0 ; i<3 ; i++)
				_videoVariances[i] += pow(frameValue.val[i] - _videoMeans[i] , 2);
		}
	}

	for( int i=0 ; i<3 ; i++)
		_videoVariances[i] /= N;

	//Convert backto RGB color model
	cvCvtColor(frame,frame,CV_HSV2RGB);	
	
}



////Renders the content frame into the polygon being tracked in the current video
void embedCentral::renderContentToFrame(IplImage *frame , IplImage *content , IplImage *cleanImage , CvMat *cleanTransform 
                                        , CvScalar modelVariances ,vector<svlPoint2d> targetPolygon  , int contentType)

{    

    cout<<"\n\n Rendering ad content to video\n\n";

    //No need for rendering if the polygon is out of bounds
    if( visionUtilities::checkOutofBounds(frame, targetPolygon ) || (contentType == LOGO_WITH_EFFECT && _isReverseTracking) )
        return;


    _framesPerTracker[_currentTrackerIndex]++;

     //if( _framesPerTracker[_currentTrackerIndex] == 1 && !_isReverseTracking)
    //     _arrowDirection[_currentTrackerIndex] = visionUtilities::findDirectionForArrow( targetPolygon , frame);
     
    //Display the arrow for only a fixed number of initial frames
    //if( _framesPerTracker[_currentTrackerIndex] <= constants::ARROW_DISPLAY_LENGTH || _isReverseTracking)
    //{
   //     cout<<"\n Drawing arrows";
   //     visionUtilities::drawArrow(frame,visionUtilities::findPointsForArrow( targetPolygon ,frame,_arrowDirection[_currentTrackerIndex]));
   // }
    
   

	IplImage *temp = NULL;//Image object used for resizing images

	////Trim the widescreen border if content is movie
	if( contentType ==  REAL_LIFE_WIDE )
	{
		int borderSize = 106;
		temp = cvCreateImage(cvSize(content->width,content->height-2*borderSize),content->depth,content->nChannels);
		cvSetImageROI(content,cvRect(0,borderSize,content->width,content->height-2*borderSize));
        cvCopyImage(content,temp);
		cvResetImageROI(content);
		cvReleaseImage(&content);
		content = cvCloneImage(temp);
		cvReleaseImage(&temp);	
	}


    //Used to render the special "Poission" blended logo
    IplImage *contentTemp = NULL;
    IplImage *contentEdgeMap  = NULL;
    
    	
	double minX = 0.0 , minY = 0.0 , maxX = 0.0 , maxY = 0.0;//Variables for holding the possibly larger rectangle surronding the polygon


	_objectTrackers[_currentTrackerIndex].findMinMax(targetPolygon,0,minX,minY,maxX,maxY);//Compute enclosing rectangle for the target polygon

    ////Compute factor by which content image is now larger than the actual region for embeding
	////this is used to blur the content image by the same factor
    double alpha = 1.8;
	int scaleXFactor = (int)(utilities::roundOff(content->width / (maxX-minX)) * alpha);
	int scaleYFactor = (int)(utilities::roundOff(content->height / (maxY-minY)) * alpha);
	
	//Round off scale factor to an odd number
	scaleXFactor += 1 - (scaleXFactor) % 2;
	scaleYFactor += 1 - (scaleYFactor) % 2;
	
	//Blur the content image to enable flawless sampling
    cvSmooth(content,content,CV_GAUSSIAN,scaleXFactor,scaleYFactor);


    //#IN TEST MODE# #Poisson blending for the logo within the content image#
    if( contentType == SPECIAL_LOGO )
    {
        contentTemp =  cvCreateImage(cvGetSize(content),content->depth,1);
        cvCvtColor(content,contentTemp,CV_BGR2GRAY);
        contentEdgeMap = utilities::edgeMap(contentTemp);
        cvReleaseImage(&contentTemp);
        utilities::display(contentEdgeMap);
    }
    //#IN TEST MODE#
    
		
	//Generate polygon points for the source polygon
	vector<svlPoint2d> sourcePolygon;
	sourcePolygon.push_back(svlPoint2d(0,0));
	sourcePolygon.push_back(svlPoint2d(content->width-1,0));
	sourcePolygon.push_back(svlPoint2d(content->width-1,content->height-1));
	sourcePolygon.push_back(svlPoint2d(0,content->height-1));


	//Optimize by saving the frame and corresponding content points after one operation, to make the others faster
	vector<CvScalar> framePoints;
    vector<svlPoint2d> frameCoordinates;
	vector<CvScalar> contentPoints;
    vector<int> isFramePointInContent;
    vector< vector<svlPoint2d> > pMatrix;

    vector<CvScalar> parameterContentPoints;

	//************************************* Setup content stat variables *********************
	double contentMeans[3];
	double contentVariances[3];

	for( int i=0 ; i<3 ; i++)
	{
		contentMeans[i] = 0.0;
		contentVariances[i] = 0.0;
	}

	
    
    //****************** Prepare to populate the giant sub pixel matrix *************8    
      
    const double SIZE_MARGIN = 2;//1;
    const double DISTANCE_FROM_EDGE = 0.3;//0.8;
    const double SUBSAMPLE_RATIO = 5;//7;//5;// 7;//1.25;
    double stepSizeX = ( (maxX + SIZE_MARGIN) - (minX - SIZE_MARGIN) ) / 
        ceil( ((maxX + SIZE_MARGIN) - (minX - SIZE_MARGIN)) * SUBSAMPLE_RATIO );
    double stepSizeY = ( (maxY + SIZE_MARGIN) - (minY - SIZE_MARGIN) ) / 
        ceil( ((maxY + SIZE_MARGIN) - (minY - SIZE_MARGIN)) * SUBSAMPLE_RATIO );
    int row = 0 , col =0 ; //Indices for the giant sub pixel matrix
    
    //Compute dimensions of sub pixel matrix
    int startX = (int)floor(minX-SIZE_MARGIN);
    int startY = (int)floor(minY-SIZE_MARGIN);
    int height = (int)(ceil( maxY + SIZE_MARGIN) - startY + 1);
    int width  = (int)(ceil( maxX + SIZE_MARGIN) - startX + 1);

    cout<<"\n Computing the occlusion model";    

    //Compute the occlusion model for the current frame
    IplImage *occlusionModel  = _objectTrackers[_currentTrackerIndex].finalizeOccludedPixels(cleanTransform,cvCloneImage(frame),cleanImage
        ,modelVariances,targetPolygon,SIZE_MARGIN,SUBSAMPLE_RATIO,stepSizeX,stepSizeY);
    

        
    //Change color model
	cvCvtColor(frame,frame,CV_RGB2HSV);
	cvCvtColor(content,content,CV_RGB2HSV);

    printf("\n\n Populating the giant sub pixel matrix \n\n");

	//Iterate over the enclosing rectangle
	for( double y=minY - SIZE_MARGIN ; y<=maxY + SIZE_MARGIN; y+=stepSizeY , row++)
	{    
        vector< svlPoint2d > pRow;  //One row of the giant pub pixel matrix
        col = 0;    //Reset column variable to zero
        
		for( double x=minX - SIZE_MARGIN ; x<=maxX + SIZE_MARGIN; x+=stepSizeX , col++)
		{   

			svlPoint2d p(x,y); 
            double distance = _objectTrackers[_currentTrackerIndex].shortestDistanceToEdge(targetPolygon,p);
            int occStatus = _objectTrackers[_currentTrackerIndex].isOccluded(occlusionModel, row,col);
            int isInside = 0;

            //Filter subpixels which lie outside the image
            if( !(p.x - constants::MARGIN < 0 || p.x + constants::MARGIN - frame->width  > 0 ||
                p.y - constants::MARGIN < 0 || p.y + constants::MARGIN - frame->height  > 0) )
            {
                             
            
                if( occStatus == 0 && ( !_objectTrackers[_currentTrackerIndex].inPolygon(targetPolygon,p) || distance < DISTANCE_FROM_EDGE ) || occStatus == 2 )
                {               
                    if( occStatus == 2 || contentType != SPECIAL_LOGO )
                    {                
                        isFramePointInContent.push_back(1);
                        isInside = 1;

                        pRow.push_back(p);
                        frameCoordinates.push_back(p);
                                
                                               
                        //Donot attempt subpixel interpolation at the edges
                         if( p.x == frame->width -1 || p.y == frame->height -1 )
                            framePoints.push_back( cvGet2D(frame,(int)p.y,(int)p.x) );
                         else
                             framePoints.push_back( _objectTrackers[_currentTrackerIndex].subPixelValueRGB(frame,p.x,p.y) );
              
                               
                        if( p.x == frame->width -1 || p.y == frame->height -1 )
                            contentPoints.push_back( cvGet2D(frame,(int)p.y,(int)p.x));
                         else
                             contentPoints.push_back( _objectTrackers[_currentTrackerIndex].subPixelValueRGB(frame,p.x,p.y) );  
                  
                    }

                }                 
			    else if( p.x >=0 && p.y >=0 && p.x <= frame->width-1 && p.y <= frame->height-1 )
			    {
                   
				    //If the current point lies within the target polygon
				    if( _objectTrackers[_currentTrackerIndex].inPolygon(targetPolygon,p) && !occStatus )
				    {                    
					    //Find mapping of the distorted target polygon into the content image
					    svlPoint2d c = _objectTrackers[_currentTrackerIndex].findMappingInPolygon(targetPolygon,sourcePolygon,p);	

                        
					    if( c.x >=0 && c.y >=0 && c.x <= content->width-1 && c.y <= content->height-1)
					    {
                            if( contentType != SPECIAL_LOGO ||_objectTrackers[_currentTrackerIndex].subPixelValueFloat(contentEdgeMap,c) > 0 )
                            {
                                isInside = 1;
                                isFramePointInContent.push_back(0);

                                pRow.push_back(p);
                                frameCoordinates.push_back(p);
                                
                                //Donot attempt subpixel interpolation at the edges
                                if( p.x == frame->width -1 || p.y == frame->height -1 )
                                    framePoints.push_back( cvGet2D(frame,(int)p.y,(int)p.x) );
                                else
                                    framePoints.push_back( _objectTrackers[_currentTrackerIndex].subPixelValueRGB(frame,p.x,p.y) );

                                
                                    
                                if( c.x == content->width -1 || c.y == content->height - 1 )
                                    contentPoints.push_back( cvGet2D(content,(int)c.y,(int)c.x));
                                else
                                    contentPoints.push_back( _objectTrackers[_currentTrackerIndex].subPixelValueRGB(content,c.x,c.y) );

                            }//Special handling of content type SPECIAL_LOGO
                                    
                        }//if projected content point lies within the content image
                        	
                       
                    }//if in polygon

                   
                }//if valid

            }

            //If point is not to be placed within the subpixel matrix
            if( !isInside )
            {
                pRow.push_back(svlPoint2d(-1,-1));
                frameCoordinates.push_back( svlPoint2d(-1,-1));
                framePoints.push_back( cvScalar(-1) );
                contentPoints.push_back( cvScalar(-1) );
                isFramePointInContent.push_back(-1);
            }

        }//x
         pMatrix.push_back(pRow);
	}//y


    cout<<"\n Computing blending parameters";

   //**************************** Compute content means ***************

    for( int y=0 ; y<content->height ; y++)
        for( int x=0 ; x<content->width ; x++)
        {
            CvScalar value = visionUtilities::getRGB(content,&x,&y);

            for( int k=0 ; k<3 ; k++)
                contentMeans[k] += value.val[k];
                
                parameterContentPoints.push_back(value);
        }
    

	for( int i=0 ; i<3 ; i++)
    {
        contentMeans[i] /= parameterContentPoints.size();
        cout<<"\n MEANS "<<contentMeans[i]<<" "<<_videoMeans[i];
    }

 
	//**************************** Compute content variances ***************

	for( unsigned i=0 ; i<parameterContentPoints.size() ; i++ )
	{
        CvScalar contentValue = parameterContentPoints[i];

		for( int i=0 ; i<3 ; i++)
			contentVariances[i] += pow( contentValue.val[i] - contentMeans[i] , 2);
	}

	for( int i=0 ; i<3 ; i++)
    {
		contentVariances[i] /= parameterContentPoints.size();
        cout<<"\n VARIANCES "<<contentVariances[i]<<" "<<_videoVariances[i];
    }


	//*********************************************** Compute parameters ***********************

 
	double S[3];
	double D[3];

	//Constants defining the ratio/weight between the content and the frame while alinging the means & variances
	double sRatioForLogo[3] = {.01, .01, .4};
    double dRatioForLogo[3] = {0.1, 1, 1};
	double weightForAlphaEdgeLogo[3] = {0, 0, 0.65};
	double weightForAlphaNormalLogo[3] = {0, 0, 0.4};

    double sRatioForMovie[3] = {0.01, 0.01, .1};
    double dRatioForMovie[3] = {0.1, 0.1, 0.3};
	double weightForAlphaNormalMovie[3] ={0, 0, 0.30};
    double weightForAlphaEdgeMovie[3] = {0, 0, 0.10};
	

	//Hold the current weights for the mean variance alignment
	double currents[3];	
	double currentd[3];
	double origFrameWeight[3];
	double frameWeight[3];

	//Feed current value based upon input parameter
	for( int i=0 ; i<3 ; i++)
	{
		if( contentType == LOGO || contentType == ON_TEXTURE || contentType == LOGO_WITH_EFFECT)
		{
			currents[i] = sRatioForLogo[i];
			currentd[i] = dRatioForLogo[i];
			origFrameWeight[i] = weightForAlphaEdgeLogo[i];
			frameWeight[i] = weightForAlphaNormalLogo[i];
           
		}
		else if( contentType == REAL_LIFE || contentType == REAL_LIFE_WIDE )
		{
			currents[i] = sRatioForMovie[i];
			currentd[i] = dRatioForMovie[i];	
			origFrameWeight[i] = weightForAlphaEdgeMovie[i];
			frameWeight[i] = weightForAlphaNormalMovie[i];
		}
	}
   

	//The maximum distance of the pixels within the region from the boundary of the region which will have a
	//transitional alpha blending
	//int outlineForLinearBlend = 0;
    
for( unsigned c=0 ; c<frameCoordinates.size() ; c++)
	{
        if( contentPoints[c].val[0] == -1 )
            continue;
	
     
		//The minimum distance of the frame point to an edge
		//double minDistance = _objectTrackers.shortestDistanceToEdge(targetPolygon,p);
	    
		//Retrieve original values from content and frame images
        CvScalar contentValue = contentPoints[c]; 
		CvScalar frameValue = framePoints[c]; 
        
		for( int i=0 ; i<3 ; i++)
			S[i] = 1 - currents[i] + currents[i] * (_videoVariances[i] / contentVariances[i]);
		
		for( int i=0 ; i<3 ; i++)
			D[i] = currentd[i] * (_videoMeans[i] - contentMeans[i]);
 
		
		for( int i=0 ; i<3 ; i++)
		{
			//if( minDistance <= outlineForLinearBlend )
			//{
				//frameWeight[i] = ( ( origFrameWeight[i] - frameWeight[i]) / outlineForLinearBlend) * minDistance;
											
				//frameWeight[i] = origFrameWeight[i] - frameWeight[i];
			//}
          			
             if( isFramePointInContent[c] != 1)
			    contentValue.val[i] = S[i]  *  (contentValue.val[i] - contentMeans[i]) +  contentMeans[i] ;//+ D[i];	

             if( contentValue.val[i] < 0 )
                contentValue.val[i] = 0;
             
                       
             if( contentType != ON_TEXTURE || (contentType == ON_TEXTURE && i != 2) )
                contentValue.val[i] = pow( frameValue.val[i], frameWeight[i]) *  pow(contentValue.val[i], 1 - frameWeight[i]) ;
                    
             
 		}
		
        
        contentPoints[c] = contentValue;
	}


 
    //Create sub pixel matrix
    CvScalar **subPixelMatrix = new CvScalar*[height];
    double **subPixelWeight = new double*[height];

    for( int i=0 ; i<height ; i++)
    {
        subPixelMatrix[i] = new CvScalar[width];
        subPixelWeight[i] = new double[width];

        for( int j=0 ; j<width ; j++)
        {
            subPixelWeight[i][j] = -1;
            subPixelMatrix[i][j] = cvScalar(0,0,0);
        }
    }

    cout<<"\n\n Populating influences of sub-pixels on discrete grid";

    
    //Populate sub pixel matrix
    for( unsigned y=0  ; y<pMatrix.size() ; y++)
    {
        for( unsigned x=0 ; x<pMatrix[y].size() ; x++ )
        {           
            if( pMatrix[y][x].x != -1 )
            {         
               populateSubPixelMatrix(pMatrix ,x ,y ,contentPoints ,subPixelMatrix ,subPixelWeight ,startX ,startY);
             
            }
        }//x
    }//y


    cout<<"\n\n Final rendering\n\n";

    double gap = height / (constants::EFFECT_LENGTH + 0.0);
    bool appearIn = 0;
    bool appearOut = 0;
    
    if( contentType == LOGO_WITH_EFFECT && _framesPerTracker[_currentTrackerIndex] -1 < constants::EFFECT_LENGTH )
        appearIn = 1;
    else if( contentType == LOGO_WITH_EFFECT && _trackLength[_currentTrackerIndex] - _framesPerTracker[_currentTrackerIndex] < constants::EFFECT_LENGTH )
        appearOut = 1;

    
    for( int y=0 ; y<height ; y++)
    {
        for( int x=0 ; x<width ; x++)
        {
            if( appearIn && y > gap * _framesPerTracker[_currentTrackerIndex] )
                break;
            else if( appearOut && y > gap * (_trackLength[_currentTrackerIndex] - _framesPerTracker[_currentTrackerIndex]) )
                break;

            if( subPixelWeight[y][x] != -1 )
            { 
                for( int i=0 ; i<3 ; i++)
                {
                    subPixelMatrix[y][x].val[i] /= subPixelWeight[y][x];              
                }

                cvSet2D(frame,y+startY,x+startX,subPixelMatrix[y][x]);
            }
        }

    }
	
    cout<<"\n Deleting memory\n";
    
    //*************Free lots of memory****************
    cvReleaseImage(&contentEdgeMap);
    cvReleaseImage(&content);
    cvReleaseImage(&occlusionModel);


    for( unsigned i=0 ; i<pMatrix.size() ; i++)
        pMatrix[i].clear();

    pMatrix.clear();
    frameCoordinates.clear();
    framePoints.clear();
    contentPoints.clear();

    for( int i=0 ; i<height ; i++)
    {        
        delete []subPixelMatrix[i];
        delete []subPixelWeight[i];
    }

    delete []subPixelMatrix;
    delete []subPixelWeight;


	//Convert image back to RGB model for saving
	cvCvtColor(frame,frame,CV_HSV2RGB);
}


//Populates the sub pixel matrix to be used to rendering
void embedCentral::populateSubPixelMatrix( vector<vector< svlPoint2d > > &pMatrix , unsigned x , unsigned y , vector< CvScalar > &contentPoints , CvScalar **subPixelMatrix , double **subPixelWeight , int startX , int startY)
{
    int width = (int)(pMatrix[0].size());

    CvScalar mainContent = contentPoints[ y * width + x ];

    int mainX = (int)pMatrix[y][x].x;
    int mainY = (int)pMatrix[y][x].y;

    //Compute coordinates within the acutal sub pixel matrix
    int relX = mainX - startX;
    int relY = mainY - startY;

    double delX = pMatrix[y][x].x - mainX;
    double delY = pMatrix[y][x].y - mainY;


    //If left subpixel neighbour has a value
    if( x > 0 && pMatrix[y][x-1].x != -1 )
    {
        CvScalar leftContent = contentPoints[ y * width + x -1 ];
        CvScalar newContentValue;
        double leftDelX = pMatrix[y][x-1].x - (int)pMatrix[y][x-1].x;

        //Interpolate each of R G & B
        for( int i=0 ; i<3 ; i++)
            newContentValue.val[i] = (mainContent.val[i] * (1-delX) + leftContent.val[i] * leftDelX ) / (1 - delX + leftDelX);
        
        //Set influence of compute intermediate pixel to top final pixel
        if( subPixelWeight[relY][relX] == - 1)
        {
           subPixelWeight[relY][relX] = 1 - delY;

            for( int i=0 ; i<3 ; i++)
                subPixelMatrix[relY][relX].val[i] = newContentValue.val[i] * ( 1 - delY);
        }
        else
        {
            subPixelWeight[relY][relX] += 1 - delY;

            for( int i=0 ; i<3 ; i++)
                subPixelMatrix[relY][relX].val[i] += newContentValue.val[i] * (1 - delY);
        }


        if(  y+1 < pMatrix.size() && pMatrix[y+1][x].x != -1 )
        {
            //Set influence of compute intermediate pixel to bottom final pixel
            if( subPixelWeight[relY+1][relX] == - 1)
            {
                subPixelWeight[relY+1][relX] = delY;

                for( int i=0 ; i<3 ; i++)
                    subPixelMatrix[relY+1][relX].val[i] = newContentValue.val[i] * (delY);
            }
            else
            {
                subPixelWeight[relY+1][relX] += delY;

                for( int i=0 ; i<3 ; i++)
                    subPixelMatrix[relY+1][relX].val[i] += newContentValue.val[i] * (delY);
            }
        }

    }

    //If top subpixel neighbour has a value
    if( y > 0 && pMatrix[y-1][x].x != -1 )
    {
        CvScalar topContent = contentPoints[ (y-1) * width + x ];
        CvScalar newContentValue;
        double topDelY = pMatrix[y-1][x].y - (int)pMatrix[y-1][x].y;

        //Interpolate each of R G & B
        for( int i=0 ; i<3 ; i++)
            newContentValue.val[i] = (mainContent.val[i] * (1 - delY) + topContent.val[i] * topDelY ) / (1 - delY + topDelY );
        

        //Set influence of compute intermediate pixel to left final pixel
        if( subPixelWeight[relY][relX] == - 1)
        {
            subPixelWeight[relY][relX] = 1 - delX;

            for( int i=0 ; i<3 ; i++)
                subPixelMatrix[relY][relX].val[i] = newContentValue.val[i] * ( 1 - delX );
        }
        else
        {
            subPixelWeight[relY][relX] += 1  - delX;

            for( int i=0 ; i<3 ; i++)
                subPixelMatrix[relY][relX].val[i] += newContentValue.val[i] * ( 1 - delX );
        }

        if( x+1 < pMatrix[0].size() && pMatrix[y][x+1].x != -1 )
        {
            //Set influence of compute intermediate pixel to bottom final pixel
            if( subPixelWeight[relY][relX+1] == - 1)
            {
                subPixelWeight[relY][relX+1] = delX;

                for( int i=0 ; i<3 ; i++)
                    subPixelMatrix[relY][relX+1].val[i] = newContentValue.val[i] * delX;
            }
            else
            {
                subPixelWeight[relY][relX+1] += delX;

                for( int i=0 ; i<3 ; i++)
                    subPixelMatrix[relY][relX+1].val[i] += newContentValue.val[i] * delX;
            }
        }


    }
}


