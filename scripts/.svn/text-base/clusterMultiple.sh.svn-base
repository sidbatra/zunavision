#!/bin/bash

#********Setup variables************
singleScript="/afs/cs.stanford.edu/u/sidbatra/MovieProject/scripts/single.sh"
videoNames=("2_occ" "2f2f" "casino_2" "field_1" "good_2" "league_2" "league_5" "legend_1" "legend_3" "legend_4" "legend_5" "legend_6" "we_3" "we_4" "we_6" "we_7" "we_8")
videoExts=(".jpg" ".bmp" ".bmp" ".bmp" ".bmp" ".bmp" ".bmp" ".bmp" ".bmp" ".bmp" ".bmp" ".bmp" ".bmp" ".bmp" ".bmp" ".bmp" ".bmp")
contentNames=("h" "adidas" "shrek" "pepsi" "adidas" "coke" "coke" "adidas" "nike" "coke" "pepsi" "nike" "pepsi" "coke" "pepsi" "coke" "h")
contentExts=(".jpg" ".jpg" ".jpg" ".jpg" ".jpg" ".jpg" ".jpg" ".jpg" ".jpg" ".jpg" ".jpg" ".jpg" ".jpg"  ".jpg" ".jpg" ".jpg" ".jpg")

#videoNames=("we_7" "we_6" "2" "league_5" "league_2")
#videoExts=(".bmp" ".bmp" ".jpg" ".bmp" ".bmp")
#contentNames=("coke" "pepsi" "h" "coke" "coke")
#contentExts=(".jpg" ".jpg" ".jpg" ".jpg" ".jpg")

totalVideos=17

for (( i=0 ; i<$totalVideos ; i++ ))
do
name="${videoNames[$i]}.sh"
echo $name
echo "$singleScript ${videoNames[$i]} ${videoExts[$i]} ${contentNames[$i]} ${contentExts[$i]}" > $name
echo "" >> $name

chmod 777 $name
qsub -q quicksail $name

done


