/*****************************************************************************
** ZunaVision
** Copyright (c) 2008 
**
** FILENAME:    video.h
** AUTHOR(S):   Sid Batra <sidbatra@cs.stanford.edu>
** DESCRIPTION:
** Holds classes for all meta-data / tracking / rendering information needed by a video
*****************************************************************************/

#pragma once

#include "base.h"
#include "SIFT/lowe_defs.h"

using namespace std;

//************************************* CLASS content *****************************************************
class contentTracks;

class content
{

public:
    
    string _name;                           //Name of the actual content
    string _extension;                      //Extension of the content image
    string _maskPath;                       //Path to the transparent content image

    int _renderingType;                     //Rendering type for the content, fullblending , no lighting etc
    int _contentType;                       //Type of content to be rendering, poisson blended , billboard etc
    int _trackingType;                      //Method to be used for tracking
    int _occlusionType;                     //Whether occlusions have to be handled or not for this surface
    int _fadeType;			    //The type of fading to be applied if any
    unsigned _startingFrame;                //Starting (central) frame index in the video stream
    int _endFrameForward;              	    //End frame while tracking forward
    int _endFrameBackward;                  //End frame while tracking backward
    int _fadePositionFwd;		    //Abs diff of current frame from start display frame
    int _fadePositionBwd;		    //Ads diff of current frame from end display frame

    CvSize _frameBorder;                    //Portion of the content image to be excluded from blending

    vector<string> _filenames;              //All the filenames of the content image(s) (multiple images incase of video)

    unsigned _currentIndex;                 //Index of the current content image being rendered (0 for logos , i for videos)
    IplImage *_currentImage;                //Current content image
    IplImage *_currentPoissonImage;         //Poisson mask for the current image
        
    vector<double> _frameMeans;             //Color means of different channels of a representative frame on which the content will be rendered
    vector<double> _frameVariances;         //Color variances of different channels of a representative frame on which the content will be rendered

    vector<double> _contentMeans;           //Color means of different channels of the contents current (or only) image
    vector<double> _contentVariances;       //Color variances of different channels of the contents current (or only) image

    vector<vector<double> > _grayPixelMeans;         //Grayscale means of the distributions for each pixel
    vector<vector<double> > _grayPixelVariances;     //Grayscale variances of the distributions for each pixel
    vector<svlPoint2d> _pixelLocations;             //Locations of the pixels in the current frame

    vector<CvScalar> _colorPixelMeans;        //Color means of the distributions for each pixel   
    vector<CvScalar> _colorPixelVariances;    //Color variances of the distributions for each pixel

    double _previousGrayWeight;             //Cumulative weight of the previous grayscale timesteps
    double _previousColorWeight;            //Cumulative weight of the previous color timesteps

    int _unOccludedPixels ;              //Total number of unoccluded pixels in the model
	int _occlusionMode ;                 //Status of occluding objects
    int _previousOcclusionMode ;         //Status of occluding objects in previous frame
    unsigned _timeStep;                 //Holds the current time step for which the frames of this content have been tracked
    
    int _NCCOcclusionMode ;              //Status of occluding objects from the NCC point of view
    double _NCCVariance ;            //Cached value of the variance of the gray model in the NCC
    vector<double> _NCCDeviations;      //Cached value of the deviations of the pixels in the gray model in the NCC

    vector<bool> _isOccluded;           //Indicates whether each pixel in the model is occluded or not
    vector<double> _S;                  //Dynamic mean parameters for blending
    vector<double> _D;                  //Dynamic variance parameters for blending

    
    

public:
    content();                              //Default constructor
    void freeMemory();                      //Frees all the memory being used
    void copy(content &cContent);           //Copies content of given object into data members

    int getFade(); 	//Returns the amount of fade to be applied to the content
    void setFadePositions(int frameIndex , unsigned int totalFrames ); //Sets the fade positions based on the start and end frames of the current content
    void write(ofstream &out, int index, bool isReverse =false, unsigned totalFrames = 0);        //Writes all the content data to disk
    void read(XMLNode contentNode , bool isReverse = false, unsigned totalFrames = 0); //Reads the data from the content node 
    void populateFilenames(string mainDirectory);   //Populates all the content names

    void loadCurrentImage();     //Loads the current image from disk and returns it
    void preprocessCurrentImage(vector<svlPoint2d> targetPolygon);  //Preprocesses the content image to prepare it for sub sampling
    void precomputeDynamicParameters();     //Precomputes dynamic parameters about blending
    void setIndex(int frameIndex);          //Based on the type of content increments index to next content image
    bool isVideo();                 //Returns whether the current content is a video or not
    void cleanRegionModel();              //Cleans the model of the region
    void cleanOcclusionModel();        //Cleans the occlusion related variables 
    
    bool isTransparent();		//Returns whether the current content image is a transparent one
    inline bool isOutlier(unsigned pixelIndex , CvScalar pixelValue);

    void populateNCCStats(unsigned index);  //Caches model related stats for NCC
    
    void projectModelPixels(CvMat *H);         //Projects each pixel in the model by the given transform
    void updateGrayModelValue(IplImage *grayImage,unsigned index);      //Creates a distribution by updating the means of each pixel
    void populateModelLocations(contentTracks &currentTracks, IplImage *visualizeImage);  //Populates the locations of the pixels in the region
    void populateGrayModelValues(IplImage *grayImage , unsigned index);      //Populates the initial gray values of the pixel locations
    void updateColorModelValue(IplImage *colorImage);      //Creates a distribution by updating the means of each pixel
    void populateColorModelValues(IplImage *colorImage);      //Populates the initial color values of the pixel locations

};

//Checks whether the given pixel value is an outlier based on the gaussian model of that pixel
inline bool content::isOutlier(unsigned pixelIndex , CvScalar pixelValue)
{

    double result = pow( pixelValue.val[0] - _colorPixelMeans[pixelIndex].val[0] , 2 ) / ( _colorPixelVariances[pixelIndex].val[0] + constants::VARIANCE_REGULARIZATION[0] ) +
        pow( pixelValue.val[1] - _colorPixelMeans[pixelIndex].val[1] , 2 ) / ( _colorPixelVariances[pixelIndex].val[1] + constants::VARIANCE_REGULARIZATION[1] ) +
        pow( pixelValue.val[2] - _colorPixelMeans[pixelIndex].val[2] , 2 ) / ( _colorPixelVariances[pixelIndex].val[2] + constants::VARIANCE_REGULARIZATION[2] );
    
    return  (result >= constants::OUTLIER_THRESHOLD);
}

//**********************************************************************************************************

//************************************* CLASS contentTracks ************************************************

class contentTracks
{

public:
        
    vector<svlPoint2d> _polygon;            //Location of the polygon within the frame
    vector<svlPoint2d> _affinePolygon;      //Location of the affine projected polygon within the frame #DEBUG#
    
    CvMat *_H;                              //Projection from previous frame to the current frame
    CvMat *_cleanTransform;                 //Projection from the previous unoccluded frame to current frame

    IplImage *_occlusionMask;               //Occlusion mask of the region at the current frame
    IplImage *_cleanColorImage;             //Color copy of the last unoccluded frame

    int _contentID;                          //Index representing the instance of the content class for rendering

   CvScalar _meanModelVariances;     //Mean of the variances of all the color pixels in the model 

public:
    contentTracks();                        //Default constructor
    void freeMemory();                      //Frees all the memory being used
    void copy(contentTracks &cTracks);       //Copies the contents from the given object to the data members

    void write(ofstream &out , int index,double scale);        //Writes all the content track data to disk
    void read(XMLNode trackNode , double scale);     //Reads all the content track meta data from the xml node
    void projectPolygon(CvMat *H);    //Projects the polygon with the given transformation matrix
    void projectAffinePolygon(CvMat *A);    //Projects the affine polygon with the given transformation matrix
    void populateForNextFrame(CvMat *H , CvMat *A , contentTracks &previousTracks, int contentIndex); //Populates tracks for the next frame based on previous tracks and transformation matrix

    void computeOcclusionMetrics(IplImage *IDashColor,content &cContent,contentTracks &currentTracks);         //If current frame is occlulded, occlusion metrics are computed

};

//**********************************************************************************************************

//************************************* CLASS frame **********************************************************

class frame
{

public:
    IplImage *_colorImage;              //Color copy of the current frame
    IplImage *_grayScaleImage;          //Grayscale copy of the current frame
    Keypoint _siftFeatures;             //SIFT features for the current frame
    
    int _index;                         //Actual frame in the video being represented by the object

    vector<contentTracks> _contentTracks;           //Vector of the contents to be rendered in the current frame
    

public:
    frame();
    void freeMemory();
    void copy(frame &cFrame);           //Copies the contents of the given frame into the data members

    void write(ofstream &out , string metaDataFolder , double scale , vector<content> &cContent , bool flashMode);        //Writes all the frame data to disk
    void read(XMLNode &frameNode , string metaDataFolder,double scale);     //Reads all the frame meta data from xml node
    void load(string filename , double scale);       //Loads the color and grayscale images from disk
    void save(string filename);       //Saves the color image to disk

    void replaceOccMask(string metaDataFolder , unsigned i);    //Replaces the occlusion mask for the given content track ID

    int getContentTracks(int contentIndex , contentTracks &tracks);     //Retrieves the contentTracks associated with the current content ID
    int setContentTracks(int contentIndex , contentTracks &tracks);     //Sets the contentTracks associated with the current content ID

};

//********************************************************************************************************


//************************************* CLASS videoVolume ******************************************************

class videoVolume
{
public:
    vector<CvPoint> _volumeParts;   //Holds all the different parts of the volume
    int _currentPart;               //Current part being read
    unsigned _currentIndexInPart;        //Current frame index in part being read
    unsigned _stubCounter;          //Counts the number of stubs added after the volume has been finished
    bool _isFinished;               //Signifies whether the current volume has ended or not
    unsigned _totalFrames;          //Total frames processed currently in the volume

    videoVolume();                  //Default constructor
    void freeMemory();              //Frees any memory being used by the volume
    void computeNextFrameIndex(bool isReverse); //Computes the index of the next frame in the volume
    void initializeIndices(bool isReverse); //Initializes the part indicies after the volume has been read
    bool isWithinVolume(unsigned index);    //Checks if the given frame index is within the volume
    

};

//********************************************************************************************************



//************************************* CLASS video ******************************************************

class video
{
public:
    vector<vector<frame> > _frames;  //Each vector of frame objects represents the various volumes in the video
    vector<content> _content;       //Vector of all the content regions in the video
    string _extension;              //Extension of the images of each frame
    string _format;                 //Compression format of the original video file
    string _name;                   //Video name
    bool _render;                   //Controls if the frames are to be rendered or not
    bool _reverse;                  //Starts the tracking process in reverse mode
    bool _flashMode;		    //If set meta data is produced in a format that is needed by the dynamic flash player
    double _scale;                  //Scale at which the tracking ensues
    

    ofstream _out;                  //Output stream for writing the generated meta data
    string _outputFilename;         //Output filename for the writing the generated meta data
    XMLNode _videoNode;              //XML node to the root of the video
    string _sourceFolder;           //Source folder where the images of the video frames are kept
    string _outputFolder;           //Output folder where the processed images of the video frames are kept
    string _metaDataFolder;         //Folder that holds supporting meta data, like occlusion mask images
    vector<string> _filenames;      //Filenames for all the videoframes
    vector< videoVolume > _videoVolumes; //Holds the various video volumes
    

    video();                        //Default constructor
    void freeMemory();              //Frees the memory being used by the object
    void copy(video &cVideo);       //Copies the contents of the given video onto the data members

    unsigned int totalFrames();	    //Returns the total frames within the video
    void read(string filename , string mainDirectory , string contentDirectory , string metaDataDirectory ,bool readFrames=true); //Reads all the video metadata into the object and loads a limited num of frames
    void readInitial(string mainDirectory);   //Reads all the initial video and content related video metadata into the object
    frame readFrame(unsigned index , bool load = true);   //Reads the given frame index from disk as well as the metadata file
    frame readFrameSeq(unsigned index);   //Reads the frame metadata based on <index> in sequence of metadata availbile not frame index
    char isFrameAvailible(unsigned index);      //Checks if the current frame data is availible in the xml node
        
    void writeInitial();   //Writes all the initial video and content related video metadata into the metadata file
    void write(string filename);    //Reads all the video metadata into the object
    void writeFinal();   //Wraps up the staged meta data writing process
    void loadFrame(unsigned volume,unsigned index);   //Loads the given frame index 
    void writeFrame(unsigned volume,unsigned index);   //Writes the given frame index to the metadata file
    void saveFrame(unsigned volume,unsigned index);   //Saves the color image of the given frame index to disk
        
    void populateFilenames(string mainDirectory);      //Populates all the frame names
    void populateContentFilenames(string mainDirectory);      //Populates all the content names
    
    string fullInputFilename(int index);       //Returns the full input filename for the frame "index"
    string fullOutputFilename(int index);       //Returns the full output filename for the frame "index"
   
};
//********************************************************************************************************
