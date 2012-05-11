#!/bin/bash

args=("$@")
mainDir=${args[0]}
mainVideoDir=${args[1]}
contentDir=${args[2]}
metaDataDir=${args[3]}
exeDir=${args[4]}
scriptDir=${args[5]}
jobScriptDir=${args[6]}
tempVideoDir=${args[7]}
audioDir=${args[8]}
jobID=${args[9]}
videoName=${args[10]}
outputVideoName=${args[11]}
flashVideoName=${args[12]}
contentTypes=(`echo ${args[13]} | tr ';' ' '`)
contentIDs=(`echo ${args[14]} | tr ';' ' '`)
contentNames=(`echo ${args[15]} | tr ';' ' '`)
ffmpegMachine=${args[16]} 
jobMachine=${args[17]} 

#Test arguments
if [ $# -lt 17 ] ; then echo "Invalid number of arguments" ; exit ; fi


outputVideoExt=${outputVideoName:${#outputVideoName}-4:4}
videoDir=$mainDir"/"$jobID
outputVideoDir=$mainDir"/"$jobID"_out"
tempLoc=$tempVideoDir"/"$jobID".avi"
audioName=$audioDir"/"$jobID".wav"
metaData0=$metaDataDir"/initial.txt"
metaData1=$metaDataDir"/initial.xml"
metaData2=$metaDataDir"/1.xml"
metaData3=$metaDataDir"/2.xml"
jobScript=$jobScriptDir"/"$jobID".sh"
frameExt=".jpg"
defaultBitRate=3000000


################## Make Directories #####################
mkdir $videoDir
mkdir $outputVideoDir

for (( i=0; i<${#contentTypes}; i++ ))
do
	if [ "${contentTypes[$i]}" == "true" ] ; then
		mkdir $contentDir"/"${contentIDs[$i]} 
	fi
done

########################################################	

################# Parse video information via ffmpeg #################
split=(`ffmpeg -i $videoName 2>&1 | tr '\n' ' '`)

for (( i=0; i<${#split[@]}; i++ ))
do
	if [ ${split[$i]} == "bitrate:" ] ; then 
		if [ ${split[$i+1]} == "N/A" ] ; then bitRate=$defaultBitRate ;
		else let bitRate=${split[$i+1]}*1000 ; 
		fi
	elif [ ${split[$i]} == "Duration:" ] ;	then splitDur=(`echo ${split[$i+1]} | tr ':' ' '`) ;
		if [ ${splitDur[0]:0:1} -eq 0 ] ; then  hours=${splitDur[0]:1:1} ; else hours=${splitDur[0]}    ; fi					
		if [ ${splitDur[1]:0:1} -eq 0 ] ; then  mins=${splitDur[1]:1:1}  ; else mins=${splitDur[1]}     ; fi
		if [ ${splitDur[2]:0:1}  -eq 0 ] ; then	secs=${splitDur[2]:1:1}  ; else secs=${splitDur[2]:0:2} ; fi
	
	let duration=$hours*3600+$mins*60+$secs
		
	elif [ ${split[$i]} == "tb(r)" ] ; then fps=${split[$i-1]};	

	fi
done
######################################################################


################# Convert videos to frames ##############################
echo ssh $ffmpegMachine $scriptDir"/decodeVideo.sh" $videoName $videoDir $exeDir $tempLoc $bitRate  > $jobScript

for (( i=0; i<${#contentTypes}; i++ ))
do
	if [ "${contentTypes[$i]}" == "true" ] ; then
		     echo ssh $ffmpegMachine $scriptDir"/decodeVideo.sh" $mainVideoDir/${contentNames[$i]} $contentDir/${contentIDs[$i]} $exeDir $tempLoc $bitRate >> $jobScript
	fi
done
#########################################################################

$scriptDir"/ripAudio.sh" $videoName $audioName

if [ ! -f $metaData1 ] ; then 
mkdir $metaDataDir
echo ssh $ffmpegMachine $scriptDir"/userInterface.sh" $jobID $videoName $videoDir $exeDir $contentNames $metaData0 $metaData1 >> $jobScript
fi

if [ $jobMachine == "robo" ] ; then
jobScriptRobo=$jobScriptDir"/"$jobID"_robo.sh"
echo ssh $jobMachine $exeDir"/trackContent" -md $metaDataDir -o $metaData2 $mainDir $contentDir $metaData1 > $jobScriptRobo
echo ssh $jobMachine $exeDir"/trackContent" -render -reverse -md $metaDataDir -o $metaData3 $mainDir $contentDir $metaData2 >> $jobScriptRobo
chmod +x $jobScriptRobo
echo ssh $jobMachine qsub -q quicksail $jobScriptRobo > $jobScript
else
echo ssh $jobMachine $exeDir"/trackContent" -md $metaDataDir -o $metaData2 $mainDir $contentDir $metaData1 >> $jobScript
echo ssh $jobMachine $exeDir"/trackContent" -render -reverse -md $metaDataDir -o $metaData3 $mainDir $contentDir $metaData2 >> $jobScript
fi


################# Parse audio information via ffmpeg #################
split=(`ffmpeg -i $audioName 2>&1 | tr '\n' ' '`)

for (( i=0; i<${#split[@]}; i++ ))
do
        if [[ ${split[$i]} == "Unknown" && ${split[$i+1]} == "format" ]]; then
		rm $audioName
		audioName="NULL"
        fi
done
######################################################################


echo ssh $ffmpegMachine $scriptDir"/encodeVideo.sh" $outputVideoDir $audioName $outputVideoName $fps >> $jobScript
echo ssh $ffmpegMachine $scriptDir"/encodeVideo.sh" $outputVideoDir $audioName $flashVideoName $fps >> $jobScript

######################## Cleanup #####################################

echo ssh $ffmpegMachine "rm $videoDir/*$frameExt" >> $jobScript
echo ssh $ffmpegMachine "rm $outputVideoDir/*$frameExt" >> $jobScript

if [ "$audioName" != "NULL" ] ; then echo ssh $ffmpegMachine rm $audioName >> $jobScript ; fi

echo ssh $ffmpegMachine rmdir $videoDir >> $jobScript
echo ssh $ffmpegMachine rmdir $outputVideoDir >> $jobScript

for (( i=0; i<"${#contentTypes}"; i++ ))
do
if [ "${contentTypes[$i]}" == "true" ] ; then
echo	ssh $ffmpegMachine "rm -r $contentDir/${contentIDs[$i]}/*$frameExt" >> $jobScript
echo	ssh $ffmpegMachine  "rmdir $contentDir/${contentIDs[$i]}" >> $jobScript
fi
done
########################################################################

echo "echo Done $videoName $jobID | mail -s $jobID sidbatra@cs.stanford.edu" >> $jobScript
chmod +x $jobScript
