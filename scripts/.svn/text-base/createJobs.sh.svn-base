#!/bin/bash

args=("$@")
mainDir="/afs/cs.stanford.edu/group/movie/scratch/data"
contentDir=$mainDir"/content"
videoDir=$mainDir"/sourceVideos"
outputVideoDir=$mainDir"/outputVideos"
audioDir=$mainDir"/audioFiles"
tempVideoDir=$mainDir"/tempVideos"
metaDataDir="/afs/cs.stanford.edu/group/movie/zunavisionData/metaData"
exeDir="/afs/cs.stanford.edu/group/movie/zunavisionData/codebase/embedContentInVideo/bin"
scriptDir="/afs/cs.stanford.edu/group/movie/scratch/data/scripts"
jobScriptDir="/afs/cs.stanford.edu/group/movie/scratch/data/jobScripts"

ffmpegMachines=("zetes" "dj3")
jobMachines=("dj1" "dj2" "dj3")

totalFFMachines=${#ffmpegMachines[@]}
totalJobMachines=${#jobMachines[@]}

jobList=$mainDir"/jobList.txt"
jobIndex=0

#Test arguments
if [ $# -eq 1 ] ; then jobList=$1 ; fi


#read jobList line by line
while read LINE
do


if [ $jobIndex -ne 0 ] ; then

split=(`echo $LINE | tr ' ' ' '`)

jobID=${split[0]}; videoName=$videoDir"/"${split[1]} ; outputVideoName=$outputVideoDir"/"${split[2]} ; 
flashVideoName=$outputVideoDir"/"${split[3]} ; contentIDs=${split[4]} ; contentTypes=${split[5]} ; contentNames=${split[6]} ; contentExts=${split[7]} ; jobType=${split[8]}

let ffMach=($jobIndex % $totalFFMachines)
let jobMach=($jobIndex % $totalJobMachines)

if [ $jobType == "track" ] ; then

$scriptDir"/"trackVideo.sh $mainDir $videoDir $contentDir $metaDataDir"/"$jobID $exeDir $scriptDir $jobScriptDir $tempVideoDir $audioDir $jobID $videoName $outputVideoName $flashVideoName $contentTypes $contentIDs $contentNames $contentExts ${ffmpegMachines[$ffMach]} ${jobMachines[$jobMach]}

elif [ $jobType == "render" ] ; then

$scriptDir"/"renderVideo.sh $mainDir $videoDir $contentDir $metaDataDir"/"$jobID $exeDir $scriptDir $jobScriptDir $tempVideoDir $audioDir $jobID $videoName $outputVideoName $flashVideoName $contentTypes $contentIDs $contentNames $contentExts ${ffmpegMachines[$ffMach]} ${jobMachines[$jobMach]}

fi

fi


echo "***************************************************************************************************"

let jobIndex=$jobIndex+1
done < $jobList



