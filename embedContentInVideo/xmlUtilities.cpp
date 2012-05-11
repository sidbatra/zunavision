/*****************************************************************************
** ZunaVision
** Copyright (c) 2008 
**
** FILENAME:    xmlUtilities.cpp
** AUTHOR(S):   Sid Batra <sidbatra@cs.stanford.edu>
** DESCRIPTION:
** Various utility functions needed to interface to reading and writing data structures to XML tags

*****************************************************************************/

#include "xmlUtilities.h"

//Writes the volume as an XML tag
void xmlUtilities::writeVolume(ofstream &out , vector<CvPoint> &volume , unsigned index)
{
    out<<"    <volume id=\""<<index<<"\" n=\""<<volume.size()<<"\" ";
        
    for( unsigned v=0 ; v<volume.size() ; v++)  
    {
        out<<" s"<<utilities::toString(v).c_str()<<"=\""<<volume[v].x<<"\" e"<<utilities::toString(v).c_str()<<"=\""<<volume[v].y<<"\"";        
    }
    
    out<<" />"<<endl;

}

//Reads the volume from an XML node
vector<CvPoint> xmlUtilities::readVolume(XMLNode &volumeNode)
{
    //Read the total sub volumes
    unsigned size = atoi(volumeNode.getAttribute("n"));

    //Init volume to hold all sub volumes
    vector<CvPoint> volume(size);
    
    //Read all the sub volumes from the XML node
    for( unsigned i=0 ; i<size ; i++)
    {
        string attribute = "s" + utilities::toString(i);
        volume[i].x = atoi(volumeNode.getAttribute(attribute.c_str()));
        attribute = "e" + utilities::toString(i);
        volume[i].y = atoi(volumeNode.getAttribute(attribute.c_str()));
    }    

    return volume;
}

//Writes the matrix as an XML tag
void xmlUtilities::writeMatrix(ofstream &out , CvMat *matrix)
{

    if( matrix != NULL )
    {
	 out<<"                <matrix ";
	
  	  //Output each coordinate of the polygon
   	 for( int y=0 ; y<matrix->rows; y++)
   	 	for( int x=0 ; x<matrix->cols; x++)
   	 {
    	    string index = utilities::toString(y*matrix->cols + x);
    	    out<<"a"<<index.c_str()<<"=\""<<CV_MAT_ELEM(*(matrix),double,y,x)<<"\" ";
    	 }

    	 out<<" />"<<endl;
    }
}

//Writes the polygon as an XML tag
void xmlUtilities::writePoly(ofstream &out , vector<svlPoint2d> &polygon)
{
    out<<"                <poly n=\""<<polygon.size()<<"\" ";

    //Output each coordinate of the polygon
    for( unsigned i=0 ; i<polygon.size(); i++)
    {
        string index = utilities::toString(i);
        out<<"x"<<index.c_str()<<"=\""<<polygon[i].x<<"\" y"<<index.c_str()<<"=\""<<polygon[i].y<<"\" ";
    }

    out<<" />"<<endl;
}

//Reads the polygon from an XML node
vector<svlPoint2d> xmlUtilities::readPoly(XMLNode polyNode)
{
    //Read the total points in the polygon
    unsigned totalPoints = atoi(polyNode.getAttribute("n"));

    //Init polygon to hold all the points
    vector<svlPoint2d> polygon(totalPoints);

    //Read all the points from the XML node
    for( unsigned i=0 ; i<totalPoints ; i++)
    {
        string attribute = "x" + utilities::toString(i);
        polygon[i].x = atof(polyNode.getAttribute(attribute.c_str()));
        attribute = "y" + utilities::toString(i);
        polygon[i].y = atof(polyNode.getAttribute(attribute.c_str()));
    }    

    return polygon;
}
