function [output, rawOutput] = insertOnce(content, ad, loc, adSize)

result = matchHistogram(content, ad, loc);
% resizedAd = imresize(result, [2*adSize+1, 2*adSize+1], 'bicubic');
% resizedOriginalAd = imresize(ad, [2*adSize+1, 2*adSize+1], 'bicubic');

output = content;
rawOutput = content;

indR=loc(1)-50:loc(1)+50;
indC=loc(2)-50:loc(2)+50;
 
output(indR,indC,:) = result(indR-loc(1)+adSize+1, indC-loc(2)+adSize+1,:);
rawOutput(indR,indC,:) = ad( indR-loc(1)+adSize+1, indC-loc(2)+adSize+1, :);


