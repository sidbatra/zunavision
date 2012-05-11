#!/bin/bash

videoFolder=$1
audioFile=$2
videoName=$3
fps=$4

bitRate=3000000
videoExt=${videoName:${#videoName}-4:4}
ext=".jpg"

if [ $# -lt 4 ] ; then
echo "Invalid number of arguments"
elif [ $# -eq 5 ] ; then
bitRate=$5
elif [ $# -eq 6 ] ; then
bitRate=$5
ext=$6
fi


audioCode="-i $audioFile"

if [ $audioFile == "NULL" ] ; then audioCode="" ; fi

if [ $videoExt == ".wmv" ] ; then 
ffmpeg -r $fps -i $videoFolder"/%07d"$ext $audioCode -acodec wmav2 -vcodec wmv2 -b $bitRate -r $fps $videoName < /dev/null
elif [[ $videoExt == ".mpg" || $videoExt == ".mp4" ]] ; then 
ffmpeg -r $fps -i $videoFolder"/%07d"$ext $audioCode -vcodec mpeg4 -b $bitRate -r $fps $videoName < /dev/null
else
ffmpeg -r $fps -i $videoFolder"/%07d"$ext $audioCode -b $bitRate -r $fps $videoName < /dev/null
fi
