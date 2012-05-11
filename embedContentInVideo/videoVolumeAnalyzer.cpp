/*****************************************************************************
** ZunaVision
** Copyright (c) 2008 
**
** FILENAME:    videoVolumeAnalyzer.cpp
** AUTHOR(S):   Sid Batra <sidbatra@cs.stanford.edu>
** DESCRIPTION:
** Finds and groups the frames that possibly belong to a single camera
*****************************************************************************/

#include "base.h"
#include "video.h"

using namespace std;


void usage()
{
    cerr << "USAGE: ./videoVolumeAnalyzer [OPTIONS] <base folder> <meta data file>" << endl;
    cerr << "OPTIONS:" << endl
        << " -v                :: verbose "<<endl
        << " -o (string)       :: redirect volume enhanced meta data to given file "<<endl
	<< " -d (path)	       :: path for the debug folder where sub volume folders are made "<<endl
	 
	 << endl;
}


int main(int argc, char *argv[])
{
    //Read commandline parameters
    const int NUM_REQUIRED_PARAMETERS = 2;

    //Setup argument holding variables
    const char *outputFilename = "";
    const char *debugFolder = "";
    
	//Read command line options
    char **args = argv + 1;
    while (--argc > NUM_REQUIRED_PARAMETERS) 
	{				
        if(!strcmp(*args, "-v")) 
        {constants::VERBOSE = true;}
        else if(!strcmp(*args, "-o")) 
		{outputFilename = *(++args); argc--;}
        else if(!strcmp(*args, "-d")) 
		{debugFolder = *(++args); argc--;}
    	else
        {cerr << "ERROR: unrecognized option " << *args << endl;return -1;}						
		args++;	    
    }

    if (argc != NUM_REQUIRED_PARAMETERS) 
    {usage();return -1;}

    //Setup local variables 
    string mainDirectory = string(args[0]); //Directory where all the videos and results are kept
    string metaDataFilename = string(args[1]); //Filename for the video metadata file
    
    vector< CvHistogram * > videoVolumeEdges; //Holds the last frame before cut scenes, to check for return
                                                //of the same cut scene
    video cVideo;

    cVideo.read(metaDataFilename , mainDirectory, "" , "",false); //Read all the meta data information


    //Setup outputfilename
    if( (string)(outputFilename) == "" )
        cVideo._outputFilename = metaDataFilename;
    else
        cVideo._outputFilename = outputFilename;
   
     cVideo._videoVolumes.clear();

    
    cout << cVideo._content[0]._endFrameBackward << endl;
    cout << cVideo._content[0]._startingFrame << endl;
    cout << cVideo._content[0]._endFrameForward << endl;

    //Compute histogram for first frame
    IplImage *previousImage = cvLoadImage(cVideo.fullInputFilename(0).c_str(),CV_LOAD_IMAGE_COLOR);
    
    IplImage *previousGray = visionUtilities::toGrayscale(previousImage);
    IplImage *tinyPrevious = visionUtilities::computeSpatialImage(previousGray);

    vector<float> previous = visionUtilities::computeHistogram(previousImage,false);
    
    cvReleaseImage(&previousGray);
    cvReleaseImage(&previousImage);

    
    vector<float> current ;
    
    //Initiate first volume
    cVideo._videoVolumes.push_back( videoVolume());
    cVideo._videoVolumes[0]._volumeParts.push_back( cvPoint(0,-1) );

    if( (string)debugFolder != "" )
	    utilities::createDirectory( string( debugFolder ) + "//0" );

    
    unsigned currentVolume = 0;

    string filename = string(debugFolder) + "//data.txt";
    ofstream out(filename.c_str());
    
    //Iterate over all pairs of filenames to populate videoVolumes
    for(unsigned f=1 ; f<cVideo._filenames.size() ; f++)
    {
    	IplImage *currentImage = cvLoadImage(cVideo.fullInputFilename(f).c_str(),CV_LOAD_IMAGE_COLOR);

    	IplImage *currentGray = visionUtilities::toGrayscale(currentImage);
    	IplImage *tinyCurrent = visionUtilities::computeSpatialImage(currentGray);
	
      current = visionUtilities::computeHistogram(currentImage,false);

      //double difference = -1;//visionUtilities::compareHistograms(previous,current);
	    int l1norm = visionUtilities::compareSpatialImages(tinyPrevious,tinyCurrent);

        //if( constants::VERBOSE )
            cout<<"\n Diff between "<<f-1<<" and "<<f<<" - "<< l1norm <<" | "<<currentVolume;
       for( unsigned i=0 ; i<current.size() ; i++)
	out<<current[i]<<" ";
	for( int y=0 ; y<tinyCurrent->height ; y++)
		for( int x=0 ; x<tinyCurrent->width ; x++)
			out<<(int)CV_IMAGE_ELEM(tinyCurrent,unsigned char,y,x)<<" ";
   	out<<"\n";
	//out<<difference<<" "<<l1norm<<"\n";


	//cout<<"\n"<<current->flags<<"\n";

/*        if( difference > constants::THRESHOLD_FOR_CUT_FRAME)
        {
            if( constants::VERBOSE )
                cout<<"    Cut Scene";
            
            //Close current segment of volume
            cVideo._videoVolumes[currentVolume]._volumeParts[cVideo._videoVolumes[currentVolume]._volumeParts.size()-1].y = f-1;

            //Maintain edge histograms for current volume
            if( currentVolume >= videoVolumeEdges.size() )
                videoVolumeEdges.push_back( (CvHistogram*)cvClone(previous) );
            else
            {
                cvReleaseHist( &videoVolumeEdges[currentVolume] );
                videoVolumeEdges[currentVolume] = (CvHistogram*)cvClone(previous);
            }
                        

            int volumeIndex = -1;

	    cout<<"\n Sizes "<<cVideo._videoVolumes.size()<<" "<<videoVolumeEdges.size()<<"\n";
            //Search for possible continuation of volume
            for( unsigned v=0 ; v<cVideo._videoVolumes.size() ; v++)
                if( v!= currentVolume )
                {
                    if( visionUtilities::compareHistograms(current,videoVolumeEdges[v]) < constants::THRESHOLD_FOR_SIMILAR_FRAME )
                    {
                        if( constants::VERBOSE )
                            cout<<" resuming volume "<<v;
                        volumeIndex = v;
                        break;
                    }

                }
            
            //Maintain volume 
            if( volumeIndex == -1 )
            {
                //Initiate fresh volume
                currentVolume = cVideo._videoVolumes.size();
                cVideo._videoVolumes.push_back( videoVolume() );
                cVideo._videoVolumes[currentVolume]._volumeParts.push_back(cvPoint(f,-1));
		
		
    		if( (string)debugFolder != "" )
		    utilities::createDirectory( string( debugFolder ) + "//" + utilities::toString(currentVolume) );
            }
            else
            {   
                //Add to older volume
                cVideo._videoVolumes[volumeIndex]._volumeParts.push_back(cvPoint(f,-1));
                currentVolume = volumeIndex;
            }
            
        }

	if( (string)debugFolder != "" )
	{
		string tempPath = string(debugFolder) + "//" + utilities::toString(currentVolume) + "//" + utilities::toString(f) + ".jpg" ;
		cvSaveImage( tempPath.c_str() , currentImage); 
	}
*/
	cvReleaseImage(&currentImage);
	cvReleaseImage(&currentGray);
        //cvReleaseHist(&previous);
	cvReleaseImage(&tinyPrevious);
        //previous = (CvHistogram*)cvClone(current);
	tinyPrevious = cvCloneImage(tinyCurrent);
        //cvReleaseHist(&current);
	cvReleaseImage(&tinyCurrent);
        
    }

    out.close();

    ////Close the last volume
    cVideo._videoVolumes[currentVolume]._volumeParts[cVideo._videoVolumes[currentVolume]._volumeParts.size()-1].y = (int)(cVideo._filenames.size() -1);


    //******************** Write meta data *********************
   /* cVideo.writeInitial();

    //Write the frame meta info
    for( unsigned f=0 ; f<cVideo._filenames.size() ; f++)
    {
        cVideo._frames.push_back (vector<frame>());
        cVideo._frames[0].push_back ( cVideo.readFrame(f,false) ); 
        cVideo.writeFrame(0,0);
        cVideo._frames.clear();
    }

    cVideo.writeFinal();*/

    //********************************************************

    //************* Clean up memory ****************
    //cvReleaseHist(&previous);

    //Free volume edge histograms
    for( unsigned i=0 ; i<videoVolumeEdges.size() ; i++)
        cvReleaseHist(&videoVolumeEdges[i]);

    videoVolumeEdges.clear();
    //*************************************************
    
    
    if( constants::VERBOSE )
    {
        cout<<"\n\n";

        for( unsigned i=0 ; i<cVideo._videoVolumes.size() ; i++)
        {
            for( unsigned j=0 ; j<cVideo._videoVolumes[i]._volumeParts.size() ; j++)
                cout<<cVideo._videoVolumes[i]._volumeParts[j].x<<" "<<cVideo._videoVolumes[i]._volumeParts[j].y<<" , ";

            cout<<"\n";
        }
    }
   
    
    
        

    return 0;
}


