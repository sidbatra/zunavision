#!/bin/bash
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/afs/cs.stanford.edu/group/stair/libs32/opencv/lib

#********Setup variables************
#args=("$@")
#${args[10]} 

videoName=$1
extension=$2
size=720
    
     sourceDir="/afs/cs/group/reconstruction3d/scratch/MovieProject/data"
	exeDir="/afs/cs/u/sidbatra/MovieProject/embedContentInVideo/bin"
        video=$sourceDir"/sourceVideos/"$videoName$extension
    outputDir=$sourceDir"/"$videoName

#Create output directories
mkdir $outputDir

printf "\n Source dir - $sourceDir"
printf "\n Video name - $video"


printf "\n\n\n"

$exeDir/video2images $video $outputDir 

