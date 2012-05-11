/*****************************************************************************
** ZunaVision
** Copyright (c) 2008 
**
** FILENAME:    utilities.cpp
** AUTHOR(S):   Sid Batra <sidbatra@cs.stanford.edu>
** DESCRIPTION:
** A set of static methods which are used throughout the code as utilities
*****************************************************************************/

#include "utilities.h"


//Populates the given array into the matrix
void utilities::populateMatrix(CvMat **A  , double *data , int width , int height)
{
      *A = cvCreateMat(height,width,CV_64FC1);

            for( int y=0 ; y<height ; y++)
                for( int x=0 ; x<width ; x++)
                    CV_MAT_ELEM(*(*A),double,y,x) = data[y*height+x];      
}


//Tests the existance of the given file
bool utilities::doesFileExist(string path)
{
    bool result = false; //Default result

    #if defined(_WIN32)||defined(WIN32)||defined(__WIN32__)
        FILE *fp;
        fopen_s(&fp,path.c_str(),"r");
    #else
        FILE *fp = fopen(path.c_str(),"r");
    #endif
    

    if( fp )
    {
        result = true;
        fclose(fp);
    }

    return result;
}

//Creates a directory based on the current OS
void utilities::createDirectory(string path)
{
    #if defined(_WIN32)||defined(WIN32)||defined(__WIN32__)
        CreateDirectory(path.c_str(),NULL);
    #else
        mkdir(path.c_str(),0777);
    #endif
}

//Moves a file based upon the current OS
void utilities::moveFile(string source , string destination)
{
    #if defined(_WIN32)||defined(WIN32)||defined(__WIN32__)
        MoveFile(source.c_str(),destination.c_str());
    #else
        string command = "mv " + source + " " + destination;
        system(command.c_str());
        //move(source.c_str(),destination.c_str());
    #endif
}

//Copies a file based upon the current OS
void utilities::copyFile(string source , string destination)
{
    #if defined(_WIN32)||defined(WIN32)||defined(__WIN32__)
        CopyFile(source.c_str(),destination.c_str(),true);
    #else
        string command = "cp " + source + " " + destination;
        system(command.c_str());
        //copy(source.c_str(),destination.c_str());
    #endif
}

//Computes the square distance between the two points
double utilities::squareDistance( svlPoint2d a , svlPoint2d b)
{
    return pow(a.x - b.x ,2 ) * pow(a.y - b.y ,2);
}

//Resize image in place based on new height width
void utilities::resizeInPlace(IplImage **image, int height, int width)
{
    IplImage *tmpImage = cvCreateImage(cvSize(width, height),(*image)->depth, (*image)->nChannels);
    cvResize(*image, tmpImage);
    cvReleaseImage(image);
    *image = cvCloneImage(tmpImage);
    cvReleaseImage(&tmpImage);	
}

//Resize image in place based on scale factor
void utilities::resizeInPlace(IplImage **image, double scale)
{
    int height = (int)(scale * (*image)->height);
    int width = (int)(scale * (*image)->width);
    IplImage *tmpImage = cvCreateImage(cvSize(width, height),(*image)->depth, (*image)->nChannels);
    cvResize(*image, tmpImage);
    cvReleaseImage(image);
    *image = cvCloneImage(tmpImage);
    cvReleaseImage(&tmpImage);
}

//loads all the filenames with the given extension from the given folder into the given vector
vector<string> utilities::loadFilenames(string folderName , string extension , bool appendFoldername , bool isReverse )
{    
	//Open link to image directory
	DIR *dir = opendir(folderName.c_str());
    if (dir == NULL) 
	{
		cerr << "ERROR: could not open image folder " << folderName.c_str() << endl;
		exit(-1);
    }	
	struct dirent *e = readdir(dir);//Init object to read image names

    vector<string> filenames;
    
	//Iterate over each file in the folder and store its filename
    while (e != NULL) 
	{
		//Load next images with needed extension
		if (strstr(e->d_name, extension.c_str() ) != NULL) 
        {
            if( !appendFoldername )
                filenames.push_back(e->d_name);
            else
                filenames.push_back(folderName + "//" + e->d_name);
		    
        }
          
		//Read next file
		e = readdir(dir);
	
	}//While images are being read

    sort(filenames.begin() , filenames.end());

    //Reverse the order of the names if needed
    if( isReverse )
        reverse(filenames.begin() , filenames.end());    

    return filenames;
}


void utilities::display(IplImage *image , string text )
{
	cvNamedWindow(text.c_str(), 1);
	cvShowImage(text.c_str() , image);
	cvWaitKey(-1);
	cvDestroyAllWindows();
}

//Converts the given integer to string
string utilities::toString(int v)
{
	
    std::stringstream s;
    s << v;
    return s.str();
}

//Pads zeroes as prefix to the given string
string utilities::padZeroes(string text , unsigned length )
{
    string result = text;    

    while( result.size() < length )
        result = "0" + result;

    return result;
}

//Rounds off a floating point to an integer
int utilities::roundOff(double d)
{
	//Truncated value
	int result = (int)d;

	//Fractional value is floating point minus truncated value
	double fraction = d - (int)d;

	//If fraction is >= 0.5 round it off
	if( fraction >= 0.5 )
		result = (int)(d + 1);

	return result;
}
	

void utilities::split(string& str, vector<string>& tokens, string& delimiters)
{
    //Skip delimiters at beginning.
    string::size_type lastPos = str.find_first_not_of(delimiters, 0);

    //Find first "non-delimiter".
    string::size_type pos  = str.find_first_of(delimiters, lastPos);

    while (string::npos != pos || string::npos != lastPos)
    {
        //Found a token, add it to the vector.
        tokens.push_back(str.substr(lastPos, pos - lastPos));

        //Skip delimiters.  Note the "not_of"
        lastPos = str.find_first_not_of(delimiters, pos);

        //Find next "non-delimiter"
        pos = str.find_first_of(delimiters, lastPos);
    }
}

