function [outImg] = matchHistogram(inputImg, adImg, loc)

if nargin < 3
  loc = round(size(inputImg)/2 )
end

range = round( 0.4* size(inputImg) );

xmin = max(1, loc(1)-range(1));
ymin = max(1, loc(2)-range(2));
xmax = min( size(inputImg,1), loc(1)+range(1) );
ymax = min( size(inputImg,2), loc(2)+range(2) );


for c=1:3
  colorHist = inputImg( xmin:xmax, ymin:ymax, c);
  lowP = ceil( 0.05 * length(colorHist(:) ) );
  topP = floor( 0.95 * length(colorHist(:) ) );
  vec = sort( colorHist(:) );

%  figure(100+c); plot(vec);

  minV(c) = vec( lowP );
  maxV(c) = vec( topP );

  meanV(c) = mean( vec( lowP:topP ) );

  gammaV(c) = meanV(c) / double((0.5*vec(lowP) + 0.5*vec(topP) ) );

end

% add some regularization
minV = 0.3*minV + 0.7*mean(minV);
maxV = 0.3*maxV + 0.7*mean(maxV);

minV = double(minV);
maxV = double(maxV);
gammaV = double( gammaV );

adImg = double(adImg);
adImg = adImg / max( adImg(:) );

for c=1:3
  adImg(:,:,c) = ( adImg(:,:,c).^gammaV(c) )  .* ( maxV(c) - minV(c) ) + minV(c);
end

outImg = adImg;

% imshow( adImg );
