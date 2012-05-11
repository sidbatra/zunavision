#!/bin/bash

videoName=$1
videoDir=$2
exeDir=$3
tempLoc=$4
bitRate=$5
audioBitRate=256
audioRate=22050

if [ $# -lt 5 ] ; then
echo "Invalid number of arguments"
exit
elif [ $# -eq 6 ] ; then
audioBitRate=$6
elif [ $# -eq 7 ] ; then
audioBitRate=$6
audioRate=$7
fi

videoExt=${videoName:${#videoName}-4:4}


if [ $videoExt == ".flv" ] ; then

echo "Changing video format for compatibility"

ffmpeg -i $videoName -ab $audioBitRate -ar $audioRate -b $bitRate $tempLoc < /dev/null
$exeDir/video2images $tempLoc $videoDir

rm $tempLoc

else

$exeDir/video2images $videoName $videoDir

fi



