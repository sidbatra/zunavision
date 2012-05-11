/*****************************************************************************
** MOVIE PROJECT
** Copyright (c) 2008 
**
** FILENAME:    contentData.h
** AUTHOR(S):   Sid Batra <sidbatra@cs.stanford.edu>
** DESCRIPTION:
** Implementations of functions in contentData.cpp
*****************************************************************************/


#include "contentData.h"

//****** Constructor Logic *******

contentData::contentData()
{
    //Does nothing as now
    endingFrame = -1;
    reverseEndingFrame = -1;
}

//******** Methods ***********

//Checks whether there is a content region at the current and returns its index if found
int contentData::isContentAtIndex(vector<contentData> data , int currentFrame)
{
    //Default error value for the result index
    int index = -1;

    if( currentFrame != -1 )
    {
        //Look for content closest to and before current frame
        for( unsigned i=0 ; i<data.size() ; i++)
        {
            if( data[i].startingFrame == currentFrame )
            {
                index = i;
                break;
            }
        }
    }

    return index;
}

int contentData::checkClosestContentForward(vector<contentData> data , int currentFrame)
{
    //Default error value for the result index
    int index = -1;

    if( currentFrame != -1 )
    {
        //Look for content closest to and before current frame
        for( unsigned i=0 ; i<data.size() ; i++)
        {
            if( data[i].startingFrame > currentFrame && (index == -1 || data[i].startingFrame < data[index].startingFrame) )
            {
                index = i;
            }
        }
    }

    return index;
}

int contentData::checkClosestContent(vector<contentData> data , int currentFrame)
{
    //Default error value for the result index
    int index = -1;

    if( currentFrame != -1 )
    {
        //Look for content closest to and before current frame
        for( unsigned i=0 ; i<data.size() ; i++)
        {
            if( data[i].startingFrame < currentFrame && (index == -1 || data[i].startingFrame > data[index].startingFrame) )
            {
                index = i;
            }
        }
    }

    return index;
}

//Loads the given tracks from file
void contentData::loadContentData(vector<contentData> &data,string filename,string &extension)
{
    //Remove earlier regions if any
    data.clear();

    //Open stream to file
    ifstream in(filename.c_str(),ios::in);

    int totalRegions = 0 ;
    char ext[10];

    //Read global stats about regions
    in>>totalRegions>>ext;
    extension = string(ext);

    //Give memory to content holder
    data = vector<contentData>(totalRegions);

    //Load stats about each region
    for( int i=0 ; i<totalRegions ; i++)
    {
        contentData temp;
        int totalPoints;
        char tempChar[100];

        in>>totalPoints;
        
        temp.points = vector<wxPoint>(totalPoints);

        in>>temp.reverseEndingFrame>>temp.startingFrame>>temp.endingFrame;
        //in>>temp.startingFrame>>temp.endingFrame; //Compatibiity
        
        in>>tempChar;
        temp.contentName = string(tempChar);
        in>>tempChar;
        temp.contentExtension = string(tempChar);
        
    	    
        in>>temp.contentType>>temp.renderType>>temp.trackType>>temp.occType;
        //in>>temp.contentType;

        for( int j=0 ; j<totalPoints ; j++)
        {
            in>>temp.points[j].x>>temp.points[j].y;
        }

        data[i] = temp;
    }
    
    in.close();
}

//Saves the given tracks to file
void contentData::saveContentData(vector<contentData> &data,string filename,string extension)
{

    //*********** Sort content data based on starting frame indices ************
    for( int i=0 ; i <(int)data.size() ; i++)
    {
        for( int j=i+1 ; j<(int)data.size() ; j++)
        {
            if( data[i].startingFrame > data[j].startingFrame )
            {   
                contentData temp = data[i];
                data[i] = data[j];
                data[j] = temp;
            }
        }
    }

    //**********************************************************************


    //Open filestream
    ofstream out(filename.c_str(),ios::out);

    if( out == NULL )
        return;


    //Write global video stats
    out<<data.size()<<" "<<extension.c_str()<<"\n";

    //Iterate over the region stats to be saved
    for( unsigned i=0 ; i < data.size() ; i++)
    {
        out<<data[i].points.size()<<" "<<data[i].reverseEndingFrame<<" "<<data[i].startingFrame<<" "<<data[i].endingFrame<<" "<<data[i].contentName
            <<" "<<data[i].contentExtension<<" "<<data[i].contentType<<" "<<data[i].renderType<<" "<<data[i].trackType<<" "<<data[i].occType<<"\n";

        for( unsigned j=0 ; j< data[i].points.size() ; j++)
        {
            out<<data[i].points[j].x<<" "<<data[i].points[j].y;

            if( j != data[i].points.size() -1 )
                out<<"\n";
        }

        if( i != data.size() -1 )
                out<<"\n";
    }

    out.close();
    
}

//Retrieves the content data if any for that frame
contentData contentData::getContentDataForFrame(int frameIndex , vector<contentData> data , int &isFound)
{
    contentData returnData;
    isFound = 0;

    for( int i=0 ; i<(int)data.size() ; i++)
    {
        if ( data[i].startingFrame == frameIndex )
        {
            isFound = 1;
            returnData = data[i];
            break;
        }
    }

    return returnData;
}


