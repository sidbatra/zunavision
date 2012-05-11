#!/bin/bash
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/afs/cs.stanford.edu/group/stair/libs32/opencv/lib


#********Setup variables************
#args=("$@")

videoName=$1

#    sourceDir="/afs/cs/group/movie/scratch/data"
     sourceDir="/afs/cs/group/reconstruction3d/scratch/MovieProject/data"
	exeDir="/afs/cs.stanford.edu/u/sidbatra/MovieProject/embedContentInVideo/bin"
        outDir=$sourceDir"/"$videoName"_out"
    metaData=$sourceDir"/tracks/"$videoName".xml"
   outMetaDataFolder=$outDir"_md"
    outMetaData=$outMetaDataFolder"//"$videoName".xml"
    outMetaData2=$outMetaDataFolder"//"$videoName"_2.xml"
    
###Create output directories
mkdir $outDir
mkdir $outMetaDataFolder

printf "\n Source dir - $sourceDir"
printf "\n Video name - $videoName"

printf "\n\n\n"


#$exeDir/renderContent -blanks -md $outMetaDataFolder $sourceDir $sourceDir"/content" $outMetaData 
$exeDir/renderContent -blanks -md $outMetaDataFolder $sourceDir $sourceDir"/content" $outMetaData2 



