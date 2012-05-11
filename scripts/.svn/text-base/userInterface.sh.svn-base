#!/bin/bash
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/afs/cs.stanford.edu/group/stair/libs32/opencv/lib

jobID=$1
videoName=$2
videoDir=$3
exeDir=$4
contentNames=$5
contentExts=$6
metaDataText=$7
metaDataXML=$8

videoExt=${videoName:${#videoName}-4:4}

$exeDir/userInterface -videodir $videoDir -content $contentNames -ext $contentExts -outputmd $metaDataText
$exeDir/convertMetadata $jobID $videoExt $metaDataText $metaDataXML
