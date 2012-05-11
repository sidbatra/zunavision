for adNum = 1:4
  for contentNum = 3:5

adNames = {'pepsi.jpg', 'coke.jpg', 'ikea.jpg', 'stanford.gif'};
contentNames = {'simpsons-house', 'mansion', 'apartment', 'darkmusic', 'grayhouse' };

ad = imread(['data/' adNames{adNum} ] );
content = imread(['data/' contentNames{contentNum} '.jpg'] );

adSize = 50;

ad = imresize(ad, [2*adSize+1, 2*adSize+1], 'bicubic');
if size(ad,3) == 1
  ad(:,:,2) = ad(:,:,1);
  ad(:,:,3) = ad(:,:,1);
end

loc = round(size(content)/2 -100);
[output, rawOutput] = insertOnce(content, ad, loc, adSize);

loc = round(size(content)/2 + 150);
[output, tmp] = insertOnce(output, ad, loc, adSize);
[tmp, rawOutput] = insertOnce(rawOutput, ad, loc, adSize);

% figure(100);
% imshow(content);

figure(200);
imshow( output); title('blended histogram');
imwrite(output, [contentNames{contentNum} '_' adNames{adNum} 'blend.jpg'] );

figure(300);
imshow(rawOutput); title('basic');
imwrite(rawOutput, [contentNames{contentNum} '_' adNames{adNum} 'raw.jpg'] );

% figure(400);
% imshow(ad);
end
end
