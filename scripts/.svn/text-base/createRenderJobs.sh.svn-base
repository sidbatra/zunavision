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

jobList=$mainDir"/renderList.txt"
jobIndex=0

#Test arguments
if [ $# -eq 1 ] ; then joblist=$1 ; exit ; fi

#read jobList line by line
while read LINE
do


if [ $jobIndex -ne 0 ] ; then

split=(`echo $LINE | tr ' ' ' '`)

jobID=${split[0]}; videoName=$videoDir"/"${split[1]} ; outputVideoName=$outputVideoDir"/"${split[2]} ; 
flashVideoName=$outputVideoDir"/"${split[3]} ; contentIDs=${split[4]} ; contentTypes=${split[5]} ; contentNames=${split[6]}

let ffMach=($jobIndex % $totalFFMachines)
let jobMach=($jobIndex % $totalJobMachines)

$scriptDir"/"renderVideo.sh $mainDir $videoDir $contentDir $metaDataDir"/"$jobID $exeDir $scriptDir $jobScriptDir $tempVideoDir $audioDir $jobID $videoName $outputVideoName $flashVideoName $contentTypes $contentIDs $contentNames ${ffmpegMachines[$ffMach]} ${jobMachines[$jobMach]}

echo "***************************************************************************************************"

fi

let jobIndex=$jobIndex+1
done < $jobList







