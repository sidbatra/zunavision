/*****************************************************************************
** ZunaVision
** Copyright (c) 2008 
**
** FILENAME:    renderingEngine.h
** AUTHOR(S):   Sid Batra <sidbatra@cs.stanford.edu>
** DESCRIPTION:
** Renders content onto the images based on meta data information
*****************************************************************************/

#include "renderingEngine.h"

//****************************** Constructor Logic *****************************


renderingEngine::renderingEngine()
{
    //Does nothing as of now
}



//***************************** Methods ************************************


//Renders the given frame index from the volume
void renderingEngine::renderFrame(video &cVideo, unsigned volumeIndex , int frameIndex)
{
     if( constants::VERBOSE )
        cout<<"\n Rendering frame "<<cVideo._frames[volumeIndex][frameIndex]._index;

     //Render all the content tracks in the frame
     for( unsigned i=0 ; i<cVideo._frames[volumeIndex][frameIndex]._contentTracks.size() ; i++)
     {   
         int contentIndex =  cVideo._frames[volumeIndex][frameIndex]._contentTracks[i]._contentID;

         cVideo._content[contentIndex].setIndex(cVideo._frames[volumeIndex][frameIndex]._index); //Setup index of the current content
	
         
	cVideo._content[contentIndex].setFadePositions(cVideo._frames[volumeIndex][frameIndex]._index , cVideo.totalFrames() - 1);
	setupContentStatistics(cVideo , volumeIndex, frameIndex , cVideo._content[contentIndex] , i);
         
         if( cVideo._content[ cVideo._frames[volumeIndex][frameIndex]._contentTracks[i]._contentID ]._occlusionType == 1 )
         {
            renderContent(cVideo._frames[volumeIndex][frameIndex]._colorImage,cVideo._frames[volumeIndex][frameIndex]._contentTracks[i]._occlusionMask,
              cVideo._frames[volumeIndex][frameIndex]._contentTracks[i]._polygon,cVideo._content[contentIndex]);
         }
         else
         {
            renderContent(cVideo._frames[volumeIndex][frameIndex]._colorImage,NULL,
              cVideo._frames[volumeIndex][frameIndex]._contentTracks[i]._polygon,cVideo._content[contentIndex]);
         }
     }

   
     
}


//Subpixel renders the given content in the given frame at the given location
void renderingEngine::renderContent(IplImage *frame , IplImage *occlusionMask,vector<svlPoint2d> targetPolygon , content &cContent)
{    
    
    if( constants::VERBOSE )
        cout<<"\n Rendering content";

    IplImage *content = cvCloneImage(cContent._currentImage);         //Pointer to the current content image
    IplImage *poissonMask = cContent._currentPoissonImage;  //Pointer to the poisson blending mask
    
   
    if( !cContent.isTransparent() )
    { 
    	//Convert to HSV color space
   	 cvCvtColor(frame,frame,CV_RGB2HSV);
   	 cvCvtColor(content,content,CV_RGB2HSV);
    }

    IplImage *originalFrame = cvCloneImage(frame);      //Copy of the original frame for rendering


            
    //Compute bounding rectangle to the content
    double minX , maxX , minY , maxY;
    visionUtilities::findEnclosingRectangle(targetPolygon,minX,minY,maxX,maxY);
    minX -= constants::CONTENT_MARGIN ; minY -= constants::CONTENT_MARGIN;
    maxX += constants::CONTENT_MARGIN ; maxY += constants::CONTENT_MARGIN;
    
    
    //BlendingManager blend;
    //blend.calculateHistogramStats(originalFrame, minX, minY, maxX, maxY);
    //blend.adjustCreative(content);

    //Compute discrete height width of the region in the image which is to be replaced with the content
    int discreteWidth  = (int)(ceill(maxX) - floor(minX) + 1);
    int discreteHeight = (int)(ceill(maxY) - floor(minY) + 1);
    int discreteX = (int)floor(minX);
    int discreteY = (int)floor(minY);
    double stepSizeX = constants::SUB_SAMPLE_STEPSIZE;
    double stepSizeY = constants::SUB_SAMPLE_STEPSIZE;
    
    

    //Compute mapping to map a pixel in the actual frame back to the content image
    CvMat *H_F2C = visionUtilities::findMappingBetweenPolygons(targetPolygon,visionUtilities::formPolygonFromImage(content));
    IplImage* edgeMap = NULL;
    
    IplImage* alphaMask = createAlphaMasks(occlusionMask , poissonMask , &edgeMap,
        H_F2C , discreteX , discreteY , discreteHeight ,
        discreteWidth , targetPolygon , cvGetSize(frame) , cContent);


    double outlineValue = -1;
                
    for( double y=discreteY ; y<discreteY + discreteHeight ; y++)
    {
        for( double x=discreteX ; x<discreteX + discreteWidth ; x++)
        {   
            outlineValue = -1;

            //Filter subpixels which lie outside the image
            if( x < constants::MARGIN || x > frame->width - constants::MARGIN ||
                y < constants::MARGIN || y > frame->height - constants::MARGIN )
                    continue;
            
            if( edgeMap != NULL)
                outlineValue = visionUtilities::subPixel(edgeMap,x,y) ;
            
        
            if( visionUtilities::shortestDistanceToEdge(targetPolygon,x,y) <= constants::CONTENT_MARGIN || (outlineValue < 255 && outlineValue > 0 ))
            {
            
                    CvScalar pixelValue = cvScalar(0,0,0);
                    double pixelWeight = 0.0;            

            
                    //Compute sub pixel influences in a sub pixel grid around the discrete point
                    for(double delY =-1 + stepSizeY ; delY < 1 ; delY +=stepSizeY )
                    {                
                        for( double delX =-1 + stepSizeX ; delX < 1 ; delX +=stepSizeX )
                        {      
         
                            CvScalar temp;
                            CvScalar frameTemp;
                            double tempX = x + delX;
                            double tempY = y + delY;
                            double distance = sqrt(delX * delX + delY * delY);
                        
                            if( distance > 1 )
                                continue;

                            //Filter subpixels which lie outside the image
                            if( tempX < constants::MARGIN || tempX > frame->width - constants::MARGIN ||
                                    tempY < constants::MARGIN || tempY > frame->height - constants::MARGIN )
                                        continue;
                      
                            //If pixel lies within the target polygon
                            if( visionUtilities::inPolygon(targetPolygon,tempX,tempY) )
                            {
                                frameTemp = visionUtilities::subPixelRGB(originalFrame,tempX,tempY);                                
                                CvScalar alphaValue = visionUtilities::subPixelRGB(alphaMask,tempX,tempY);

                                //Retrieve subpixel value from content                    
                                visionUtilities::projectCoordinate(H_F2C,tempX,tempY);
                            
                                temp =  visionUtilities::subPixelRGB(content,tempX,tempY);
                                blendPixel(temp, frameTemp , cContent,alphaValue);
				// ASHUTOSH: Mean shift and alpha blend still going on!
				// ERROR
                                
                            }
                            else
                            {
                                //Else retrieve subpixel value from frame
                                temp = visionUtilities::subPixelRGB(originalFrame,tempX,tempY);
                            }                
                       
                            pixelWeight += 1 - distance;

                            for( int j=0 ; j<3 ; j++)
                                pixelValue.val[j] += temp.val[j] * (1 -distance);                        
                        
                    }//delX

                }//delY
                
           
                if( pixelWeight != 0 )
                {
                    //Normalize final pixel value
                    for( int j=0 ; j<3 ; j++)
                        pixelValue.val[j] /= pixelWeight;

                    cvSet2D(frame,(int)y,(int)x,pixelValue);
                }
           
                
                
            }
            else if( visionUtilities::inPolygon(targetPolygon,x,y) )
            {

                    double tempX = x , tempY = y;
                    visionUtilities::projectCoordinate(H_F2C,tempX,tempY);

                    CvScalar contentPixel = visionUtilities::subPixelRGB(content,tempX,tempY);
                    CvScalar framePixel = visionUtilities::subPixelRGB(originalFrame,x,y);
                    CvScalar alphaValue = visionUtilities::subPixelRGB(alphaMask,x,y);
                    
                    blendPixel(contentPixel,framePixel,cContent,alphaValue);
                    cvSet2D(frame,(int)y,(int)x,contentPixel);                      
              
             }
        }
    }



    cvReleaseMat(&H_F2C);
    cvReleaseImage(&originalFrame);
    cvReleaseImage(&edgeMap);
    cvReleaseImage(&alphaMask);
    cvReleaseImage(&content);
    
    //Convert back to RGB color space
    if( !cContent.isTransparent() )    
    	cvCvtColor(frame,frame,CV_HSV2RGB);
}


//Creates the final alpha mask combining the occlusion mask the poisson image and the tranparency
IplImage * renderingEngine::createAlphaMasks(IplImage *occlusionMask , IplImage *poissonImage , IplImage **edgeMap  ,
                                                    CvMat *H , int &discreteX , int &discreteY ,
                                                     int &discreteHeight , int &discreteWidth , vector<svlPoint2d> &targetPolygon , CvSize frameSize , content &cContent)
{

    //Allocate pointers for an alpha mask for each channel
    vector<IplImage *> alphaMask = vector<IplImage *>(3);
    IplImage *finalAlphaMask = cvCreateImage(frameSize,IPL_DEPTH_8U,3);
    

    //Allocate memory based upon occluionMask availiblity
    if( occlusionMask == NULL )
    {
        alphaMask[0] = cvCreateImage(frameSize,IPL_DEPTH_8U,1);
        cvSetZero(alphaMask[0]);
    }
    else
        alphaMask[0] = cvCloneImage(occlusionMask);
    
    
    //Map the poisson image back to the occlusion mask
    if( poissonImage != NULL )
    {   
        //Map each pixel in the occlusion mask back to the poisson image to combine the masks
        for( double y=discreteY ; y<discreteY + discreteHeight ; y++)
        {
            for( double x=discreteX ; x<discreteX + discreteWidth ; x++)
            {    
 
                if( x < constants::MARGIN || x > alphaMask[0]->width - constants::MARGIN ||
               		 y < constants::MARGIN || y > alphaMask[0]->height - constants::MARGIN )
                                        continue;

                if( visionUtilities::inPolygon(targetPolygon,x,y) )
                {                    
                    double tempX = x , tempY = y;
                    visionUtilities::projectCoordinate(H,tempX,tempY);
        
                    //double poissonValue = visionUtilities::subPixelFloat(poissonImage,tempX,tempY);   
                    double poissonValue = visionUtilities::subPixel(poissonImage,tempX,tempY);   
                    double occValue = occlusionMask == NULL ? -1 : CV_IMAGE_ELEM(occlusionMask,unsigned char,(int)y,(int)x);   

                    double maxValue = max(poissonValue,occValue);

                    if( maxValue > -1 )
                        CV_IMAGE_ELEM(alphaMask[0],unsigned char,(int)y,(int)x) = (unsigned char)maxValue;
                }
                else
                    CV_IMAGE_ELEM(alphaMask[0],unsigned char,(int)y,(int)x) = 255;
                
            }
        }
        
    }

    //Prepare edgemap such that pixels on the edges of the masks can be used for inter source rendering
    if( occlusionMask != NULL || poissonImage != NULL )
    {           
        *edgeMap = cvCloneImage(alphaMask[0]);
        cvCanny(*edgeMap,*edgeMap,constants::OCC_EDGE_CANNY_THRESHOLDS.x,constants::OCC_EDGE_CANNY_THRESHOLDS.y);
        cvSmooth(*edgeMap,*edgeMap,CV_GAUSSIAN,constants::OCC_EDGE_SMOOTH_SIDE,constants::OCC_EDGE_SMOOTH_SIDE);        
    }


    
    for( unsigned i=1 ; i<alphaMask.size() ; i++)
            alphaMask[i] = cvCloneImage(alphaMask[0]);
    
    int contentFade = cContent.getFade();

    //Adjust alpha masks for final transparency in each of the channels
    for( unsigned i=0 ; i<alphaMask.size() ; i++)
    {        
        //Map each pixel in the occlusion mask back to the poisson image to combine the masks
        for( int y=discreteY ; y<discreteY + discreteHeight ; y++)
        {
            for( int x=discreteX ; x<discreteX + discreteWidth ; x++)
            {   
                if( x < constants::MARGIN || x > alphaMask[0]->width - constants::MARGIN ||
               		 y < constants::MARGIN || y > alphaMask[0]->height - constants::MARGIN )
                                        continue;
                     unsigned int pixelValue = CV_IMAGE_ELEM(alphaMask[i],unsigned char,y,x);
 
                if( pixelValue  == 0  && cContent._renderingType != NO_BLENDING && cContent._renderingType != NO_LIGHTING)
                	CV_IMAGE_ELEM(alphaMask[i],unsigned char,y,x) = constants::FRAME_ALPHA[i];
		else if( pixelValue  < constants::FADE_PIXEL_THRESHOLD )
			CV_IMAGE_ELEM(alphaMask[i],unsigned char,y,x) =  contentFade;
			
            }
        }
    }

    
    //Merge HSV alpha masks into one final alpha mask
    cvMerge(alphaMask[0],alphaMask[1],alphaMask[2],0,finalAlphaMask);
    
    //Free memory
    for(unsigned i=0 ; i<alphaMask.size() ; i++)
        cvReleaseImage(&alphaMask[i]);
    
    return finalAlphaMask;

}

//Blurs the content image based upon a custom blurring kernel
void renderingEngine::customBlurContent(IplImage *content , vector<svlPoint2d> targetPolygon)
{
    //Compute bounding rectangle to the content
    double minX , maxX , minY , maxY;
    visionUtilities::findEnclosingRectangle(targetPolygon,minX,minY,maxX,maxY);
    minX -= constants::CONTENT_MARGIN ; minY -= constants::CONTENT_MARGIN;
    maxX += constants::CONTENT_MARGIN ; maxY += constants::CONTENT_MARGIN;

    //Compute discrete height width of the region in the image which is to be replaced with the content
    int discreteWidth  = (int)(ceill(maxX) - floor(minX) + 1);
    int discreteHeight = (int)(ceill(maxY) - floor(minY) + 1);


    //Compute variance in both dimensions
    double sigmaX = 1 * content->width / (discreteWidth + 0.0);
    double sigmaY = 1 * content->height / (discreteHeight + 0.0);
    
    //Compute width of the kernel in both dimensions
    int widthX = (int)(2 * sigmaX);
    int widthY = (int)(2 * sigmaY);

   if ( widthX < 1 && widthY < 1 )
	return;	

    //Sum of the kernel values pre-normalization
    double sum = 0.0;

    //Give memory to kernel
    CvMat *kernel = cvCreateMat(2 * widthY + 1 ,2 * widthX + 1,CV_32F);

    //Populate un-normalized kernel
    for( int y=-widthY ; y<= widthY ; y++)
    {
        for( int x=-widthX ; x<=widthX ; x++)
        {
            double value = (float)exp( -0.5 * (x*x / sigmaX*sigmaX + y*y / sigmaY*sigmaY) );
            
            CV_MAT_ELEM(*kernel , float , y+widthY, x+widthX) = (float)value;                            
            sum += value;
        }
    }


    //Convert to normalized kernel
    for( int y=-widthY ; y<= widthY ; y++)
    {
        for( int x=-widthX ; x<=widthX ; x++)
        {
            CV_MAT_ELEM(*kernel,float,y+widthY,x+widthX) = (float)(CV_MAT_ELEM(*kernel,float,y+widthY,x+widthX) / sum);            
        }
    }
    
    //Create larger image with border to enable proper convolution
    IplImage *largerContent = cvCreateImage(cvSize(content->width + 2 * widthX + 1 , content->height + 2 * widthY + 1),content->depth,content->nChannels);
    cvCopyMakeBorder(content, largerContent, cvPoint(widthX, widthY),IPL_BORDER_REPLICATE);
    
    //Convovle
    cvFilter2D(largerContent,largerContent,kernel,cvPoint(kernel->width/2,kernel->height/2));
    
    //Copy back contents to original image
    cvSetImageROI(largerContent,cvRect(widthX,widthY,content->width,content->height));
    cvCopyImage(largerContent,content);
    
    //Free memory
    cvReleaseImage(&largerContent);
    cvReleaseMat(&kernel);
	
}


//Blends the given pixel given the content type
void renderingEngine::blendPixel(CvScalar &contentValue , CvScalar &frameValue , content &cContent , CvScalar &alphaValue)
{
    
    //Blening the pixel with mean & variance alignment and multiplicative alha blending
    for( unsigned i=0 ; i<3 ; i++)
    {
        //if( cContent._renderingType == FULL_BLENDED || cContent._renderingType == NO_LIGHTING )
        //{
        //    contentValue.val[i] = cContent._S[i] * (contentValue.val[i] - cContent._contentMeans[i]) + cContent._contentMeans[i] + cContent._D[i];	

         //   if( contentValue.val[i] < 0 )
         //       contentValue.val[i] = 0;
        //}
             
        //Do multiplicative blending in the lighting channel only if needed
        contentValue.val[i] = pow( frameValue.val[i],alphaValue.val[i] / 255.0) *  pow(contentValue.val[i], 1 - alphaValue.val[i] / 255.0) ;            
               
    }
    
}



//Sets up the content image statistics to be used for rendering
void renderingEngine::setupContentStatistics(video &cVideo , unsigned volumeIndex, int frameIndex , content &cContent, int frameTrackIndex)
{    
    
    //If statistics for the video frame for the given content have not been computed
    if( cContent._frameMeans.size() == 0 && cContent._frameVariances.size() == 0 )
        visionUtilities::computeImageMeansVariances(cVideo._frames[volumeIndex][frameIndex]._colorImage , cContent._frameMeans,cContent._frameVariances);
    
    //If statistics for the content image have not been computed or have to be updated since content is a video
    if( cContent._contentMeans.size() == 0 && cContent._contentVariances.size() == 0 )
    {
        cContent.loadCurrentImage();
        cContent.preprocessCurrentImage( cVideo._frames[volumeIndex][frameIndex]._contentTracks[frameTrackIndex]._polygon);
        customBlurContent(cContent._currentImage,cVideo._frames[volumeIndex][frameIndex]._contentTracks[frameTrackIndex]._polygon);
        visionUtilities::computeImageMeansVariances(cContent._currentImage, cContent._contentMeans,cContent._contentVariances);
        cContent.precomputeDynamicParameters();

	// ASHUTOSH
	// cContent._currentImage -> e.g. coke logo
	// cVideo._frames[volumeIndex][frameIndex]._colorImage
    	// cContent._renderingType == FULL_BLENDED || cContent._renderingType == NO_LIGHTING
	// call the above 3 lines for sure 

    }
}



