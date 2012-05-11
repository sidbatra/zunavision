#!/bin/bash

videoName=$1
flashVideoName=$2
imageName=$3
ss="NULL"
size="320x240"

#Test arguments
if [ $# -lt 3 ] ; then
echo "Invalid number of arguments"
echo "./preprocessVideo.sh videoName flashVideoname imageName <ss> <size>"
exit
elif [ $# -eq 4 ] ; then
ss=$4
elif [ $# -eq 5 ] ; then
ss=$4
size=$5
fi


split=(`ffmpeg -i $videoName 2>&1 | tr '\n' ' '`)

for (( i=0; i<${#split[@]}; i++ ))
do
	if [ ${split[$i]} == "bitrate:" ]
	then
		let bitRate=${split[$i+1]}*1000
	elif [ ${split[$i]} == "Duration:" ]
	then
		splitDur=(`echo ${split[$i+1]} | tr ':' ' '`)

		if [ ${splitDur[0]:0:1} -eq 0 ] ; then
			hours=${splitDur[0]:1:1} 
		else
			hours=${splitDur[0]}
		fi
					
		if [ ${splitDur[1]:0:1} -eq 0 ] ; then
			mins=${splitDur[1]:1:1} 
		else
			mins=${splitDur[1]}
		fi
			
		if [ ${splitDur[2]:0:1}  -eq 0 ] ; then
			secs=${splitDur[2]:1:1} 
		else
			secs=${splitDur[2]:0:2}
		fi

		let duration=$hours*3600+$mins*60+$secs
		
	elif [ ${split[$i]} == "tb(r)" ]
	then
		fps=${split[$i-1]};	
	
	fi
done


echo Bitrate : $bitRate
echo Duration : $duration
echo FPS : $fps

if [ $ss == "NULL" ] ; then
let ss=($duration/4)+1
fi

ffmpeg -i $videoName -b $bitRate $flashVideoName

ffmpeg -ss $ss -i $videoName -vcodec mjpeg -vframes 1 -an -f rawvideo -s $size $imageName

