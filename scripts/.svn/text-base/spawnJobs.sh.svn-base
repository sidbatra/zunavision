#!/bin/bash

args=("$@")

mainDir="/afs/cs.stanford.edu/group/movie/scratch/data"
jobScriptDir="/afs/cs.stanford.edu/group/movie/scratch/data/jobScripts"

jobList=$mainDir"/jobList.txt"
jobIndex=0

#Test arguments
if [ $# -eq 1 ] ; then jobList=$1 ; fi

#read jobList line by line
while read LINE
do

if [ $jobIndex -ne 0 ] ; then

split=(`echo $LINE | tr ' ' ' '`)

jobID=${split[0]};  

$jobScriptDir"/"$jobID".sh" 1>$jobScriptDir"/"$jobID".sh.o" 2>$jobScriptDir"/"$jobID".sh.e" &

echo "Spawning job $jobID"
fi

let jobIndex=$jobIndex+1
done < $jobList







