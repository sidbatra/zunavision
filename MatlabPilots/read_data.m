clear all
close all

% names = { '300' , 'dcp_1' , 'dcp_2' ,  'boyce_1' , 'boyce_2' , 'holly_1' , 'tv1', 'tv3' , 'tv4' , 'tv5' , 'tv6' , 'tv7' , 'tv8' , 'tv9'};
names = { '300' , 'dcp_1' , 'dcp_2' ,  'boyce_1' , 'boyce_2' , 'holly_1' , 'tv1', 'tv3' , 'tv4' , 'tv7' , 'tv8' , 'tv9'};
baseDir = '/afs/cs/group/reconstruction3d/scratch/MovieProject/data';

array_dd = [];
video_pos = [];

fprintf('Loading data: ');
for n= 1:2  % length(names)   % 1:9   %length(names)
  video= names{n};
  fprintf(' (%d, %s),', n, video);
  data{n} = dlmread(strcat(baseDir, '/test', video, '/data.txt'));
  grounddata{n} = dlmread( strcat(baseDir, '/CutSceneGroundLabeledData/ground', video, '.txt') );

  diffdata = filter2([-1 -1; -2 -2; 2 2; 1 1], data{n}, 'same');

  video_pos = [video_pos; size(array_dd,1)+1];
  array_dd = [array_dd; diffdata];

end
fprintf('\n');

% data file format = 74 Histogram bins (10+32+32) for l+a+b  and 100 image values

% viewing raw image folders (video names in cutscene.m) = ./ui.sh <video name> NULL NULL  (run from above location)


figure(100);
imagesc(array_dd);


% color = ['r','g','b','y','m','c','k','r','g','b','k'];






% for i=1 : 1 : 2
%    plot([1:size(data,1)],data(:,i),color(i));
% end
% 
% mn(1) = 18000;
% stan(1) = 0;
% 
% shot_flag = zeros(size(data(:,1),1),1);
% 
% for n=2:size(data,1)
%     % detect if shot change
%     if data(n,1) > mn(n-1) + 2* sqrt( stan(n-1))
%         shot_flag(n) = 1;
%         mn(n) = mn(n-1);
%         stan(n) = stan(n-1);
%         continue;
%     end
% 
%     mn(n) = mn(n-1) * (n-1)/n + data(n,1) / n;
%     
%     
%     stan(n) = stan(n-1) * (n-1)/n + (data(n,1)-mn(n)).^2 / n;
% end
% 
% stan = sqrt(stan);
% 
% close all;
% plot([data(:,1), mn', stan'])
% 
% figure(100);
% plot(shot_flag);
