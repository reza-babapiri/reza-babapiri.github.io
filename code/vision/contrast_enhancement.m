%% calculate contrast indices
clear all; close all; clc;
load('data_vid2.mat');
implay(vid_lc);
Nframe=size(vid,3);
figure('WindowStyle','docked');
% imshow(vid(:,:,1));
imshow(vid_lc(:,:,1));

%%
pause();
% we need to calculate the contrast indices before and after the contrast
% improvement procedures and compare the results
% at the end of the loop CI1, CI2, CI3 are vectors of length Nframe
% you may preallocate CI1-CI3
CI1=zeros(1,Nframe);
% when the dimension of array is too big (more than 1 GB) it is suggested to preallocate
for k = 1 : Nframe
    k
[CI1(k),CI2(k),CI3(k)]=my_contrast_index(vid_lc(:,:,k));
mu(k)=mean2(vid_lc(:,:,k));
% std_I(k)=std2(vid_lc(:,:,k));
subplot(4,1,1);
plot(CI1(1:k));
ylabel('CI1');
subplot(4,1,2);
plot(CI2(1:k));
ylabel('CI2');
subplot(4,1,3);
plot(CI3(1:k));
ylabel('CI3');
subplot(4,1,4);
imhist(vid_lc(:,:,k)); hold on;
% % % % % % % % % % %  important imhist(vid_lc(:,:,k)); hold on;
line([mu(k) mu(k)],[0 1000],'color','r','linewidth',2);
drawnow;
if k==1
    pause();
else
pause(0.1);
end
end
%% imadjust 
close all;
% for homework you can extend to all the video sequence with a further for
% loop

I=vid_lc(:,:,1); % only to the first frame
% for exercise for you you can change the frame index and run
% for exercise you can change the exterme values fr gammav
gammav=linspace(0.1,10,50); % 0.5 can become 0.1, and 2 can become 5, by default gamma=1
a=quantile(I(:),0.05);% corresponds to the inout value with ECDF=0.05, close to minimum but more robust
b=quantile(I(:),0.95);% corresponds to the inout value with ECDF=0.95 % close to maximum but more robust
figure('Windowstyle','dock');
[CI1_o,CI2_o,CI3_o]=my_contrast_index(I);
for i = 1 : length(gammav) % it is an exhaustive search for the optimal value
    warning off;
  [Ie]=imadjust(I,[a b],[0 1],gammav(i));
  [CI1_e(i),CI2_e(i),CI3_e(i)]=my_contrast_index(Ie);
  
  % gammav is the parameter that allwos you to apply nonlinear stretching
  % operator
  % this code allows you to plot the operator T for histogram stretching
  subplot(3,2,1); imshow(I); subplot(3,2,2); imhist(I);
  subplot(3,2,3); imshow(Ie);subplot(3,2,4); imhist(Ie); 
  lum=linspace(0,1,100);
  map=(lum-a).^gammav(i)/(b-a).^gammav(i);
  map(lum<a)=0;
  map(lum>b)=1;
  subplot(3,2,6); plot(lum,map); hold off;
  title(num2str(gammav(i)));
  pause(1); %1.5 means wait for 1 sec and half and then start automatically
end
%%
% CI2_o=0.0184;
% CI1_o=0.2506;
id_opt=find(CI2_e==max(CI2_e),1,'first');
gamma_opt=gammav(id_opt);
[Ie_opt]=imadjust(I,[a b],[0 1],gamma_opt);
figure('Windowstyle','dock');
subplot(1,2,1); imshow(I);
subplot(1,2,2); imshow(Ie_opt);


%% histogram equalization finally changes the histogram in order to have a target histogram 
% that is similar to a uniform one
close all;
figure('windowstyle','dock');
pause();
for k = 1 : Nframe
    k
    warning off;
I=vid_lc(:,:,k);
Ih=histeq(I); % this is the matlab function that performed histogram equalization with respect to uniform pdf by default
[CI1_e(k),CI2_e(k),CI3_e(k)]=my_contrast_index(Ih);
subplot(2,2,1);
imshow(I);
subplot(2,2,2);
imshow(Ih);
subplot(2,2,3); 
imhist(I);
subplot(2,2,4);
imhist(Ih);
if k==1
    pause();
else
pause(0.5);
end
end
%% comparison of indices before histogram equalization and after
close all;
% in order to calculate the gain of the improvement we may compare
% the indices before and after the improvement by ratio frame by frame
% in this case we should use ./
% CII stands for Contrast Index Improvememt
CII1=CI1_e./CI1;  %CI1_e/CI1
CII2=CI2_e./CI2;
CII3=CI3_e./CI3;

figure('WindowStyle','docked');
subplot(3,1,1);
plot(CII1);
ylabel('CII1');
subplot(3,1,2);
plot(CII2);
ylabel('CII2');
subplot(3,1,3);
plot(CII3);
ylabel('CII3');

% to conclude it is a common risk of histogram equalization to increase
% contrast in an exaggerate mode, even if contrast gain is so high, the visual
% assessment is not so good
 %% adaptive histogram equalization
  close all;
  figure('windowstyle','dock');
  pause();
  for k = 1 : Nframe
      I=vid_lc(:,:,k);
  Ia=adapthisteq(I,'Numtiles',[32 32]);
  [CI1_a(k),CI2_a(k),CI3_a(k)]=my_contrast_index(Ia);

  % Numtiles indicates the dimension of the local region Sxy
  % it is an adaptive and local version of histogram equalization
  % we called local regions as Sxy in the theoretical lesson
  % then the region moves ovee the image in a sliding window procedure
  % it is adaptive since the histogram equalization is applied on the
  % histogram calculated in the region Sxy
  subplot(2,2,1); imshow(I);
  subplot(2,2,2); imshow(Ia);
  subplot(2,2,3); 
  imhist(I);
  subplot(2,2,4);
  imhist(Ia);
  if k==1
      pause();
  else
  pause(0.5);
  end
  end
%%

close all;
% in order to calculate the gain of the improvement we may compare
% the indices before and after the improvement by ratio frame by frame
% in this case we should use ./
% CII stands for Contrast Index Improvememt
CII1=CI1_a./CI1; 
CII2=CI2_a./CI2;
CII3=CI3_a./CI3;

figure('WindowStyle','docked');
subplot(3,1,1);
plot(CII1);
ylabel('CII1');
subplot(3,1,2);
plot(CII2);
ylabel('CII2');
subplot(3,1,3);
plot(CII3);
ylabel('CII3');

%% optimized adaptisteq on the first frame
close all;
I=vid_lc(:,:,1);
% we are going to perform optimization of parameter "dimension of Sxy" only
% on the first frame and then apply the optimal result to every frame from
% 2 to 72
% in general it is not practical to optimze all the algorithms parameters
% over all the frames. Why?
% 1. The frames can be so many that the procedure is so long and time
% consuming
% 2. In practical cases you have not so many videos so that you can waste
% one entire video only for optimization
% the solution is to use only one or a few frames for optimization and the
% rest for analysis and testing
% even because it is a totally unsupervised procedure
clear CII2;
Nv=8:4:50; % 8 12 16 20 24 28 32 36 40 44 48 etc
% this is an different way to construct a vector of possible values for Nv
% in increasing order
% the sintax is different from the use of linspace
% linspace(a,b,N) provides a vector going from a to b in N equally spaced
% values, doesn't decide the step
% in syntax a:d:b you are deciding the starting value a and the step d not
% necessarily the ending value neither the number of elements
clear CII3;
for i = 1 : length(Nv)
    i
  BW=adapthisteq(I,'NumTiles',[Nv(i) Nv(i)]);
  CII2(i)=std(BW(:))/std(I(:));
  pause(0.5);
end

figure('windowstyle','dock'); 
subplot(2,1,1); plot(Nv,CII2); title('CII2');
xlabel('Numtiles');
ylabel('CII2');

%%
close all;
% subplot(2,1,2); plot(Nv,CII4); title('CII2');
N_o1=Nv(find(CII2==max(CII2),1,'first')); % it means that among all the possible solutions for maximum CII2 Matlab gives me back only the first one

% BW_o1=adapthisteq(I,'NumTiles',[N_o1 N_o1]);
figure('windowstyle','dock');
pause();
for k = 1 : Nframe
    k
I=vid_lc(:,:,k);
BW_o1=adapthisteq(I,'NumTiles',[N_o1 N_o1]);
vid_e(:,:,k)=BW_o1;
CII_o2(k)=std(BW_o1(:))/std(I(:));
imshow(BW_o1);
pause();
drawnow; % this is to force matlab to draw the figure immediately and not after the ending of the loop
end
% to conclude the result is good from a visual assessment and from a
% quantitative assessment
%%
figure('windowstyle','dock');
plot(CII_o2);
%% player of enhanced video
implay(vid_e);

%% histogram matching
close all; clc;
I1=vid_e(:,:,1); % we assume that we performed adapthisteq only in the first frame

% let's assume that we performed contrast enhancement only on the first
% frame of the video since it was a very time consuming procedure and the
% video was so big, e.g. 10 Mpx
% the task here is to match the histogram of the low contrast video vid_lc
% to that of a high contrast video but using only the first frame of the
% enhanced video
figure('windowstyle','dock');
pause();
for k = 1 : Nframe
   Ip=vid_lc(:,:,k);
   J = imhistmatch(Ip,I1); 
   % first input is the low contrast frame kth, second input is the high contrast target first frame 
   % histogram matching is applied despite the fact that the information of
   % the frames in input are different, the only fact is the general
   % appearance of the frames not the details.
   subplot(2,2,1);
   imshow(Ip); % this is the low contrast image
   subplot(2,2,2);
   imshow(J); % this is the matched image
   subplot(2,2,3);
   imhist(Ip); % this is the histogram of the low contrast image
   subplot(2,2,4);
   imhist(J); % this is the histogram of the matched imahe
   title(k);
   pause(0.5);
end

%% calculate contrast index over the matched image
close all;
figure('windowstyle','dock');
pause();
for k = 1 : Nframe
    k
    warning off;
Ip=vid_lc(:,:,k);
J = imhistmatch(Ip,I1); % first input is the low contrast frame, second input is the high contrast target frame
[CI1_e(k),CI2_e(k),CI3_e(k)]=my_contrast_index(J);
subplot(2,2,1);
imshow(Ip);
subplot(2,2,2);
imshow(J);
subplot(2,2,3); 
imhist(Ip);
subplot(2,2,4);
imhist(J);
if k==1
    pause()
else
pause(0.5);
end
end
%%
%% comparison of indices before histogram stretching and after
close all;
CII1=CI1_e./CI1;
CII2=CI2_e./CI2;
CII3=CI3_e./CI3;

figure('WindowStyle','docked');
subplot(3,1,1);
plot(CII1);
ylabel('CII1');
subplot(3,1,2);
plot(CII2);
ylabel('CII2');
subplot(3,1,3);
plot(CII3);
ylabel('CII3');
%% summary of contrast enhancement
% you have at least 4 approaches to perform contrast enhancement
% imadjust with parameters to optimize, th1=quantile(I(:),0.01), th2=quantile(I(:),0.99), gamma
% histeq no parameters. usually it causes a too high contrast and unwanted
% information, adapt for images with a sufficiently high contrast as a sort
% of refining
% adapthisteq with dimension parameter to optimize
% imhistmatch after contrast enhancement is performed at least on a frame
%% simulate overexposure 
close all; 
figure('WindowStyle','docked');
pause();
for k = 1 : Nframe
    k
warning off;
Ip=vid_lc(:,:,k);
vid_e(:,:,k) = imhistmatch(Ip,I1); % first input is the low contrast frame, second input is the high contrast target frame
subplot(2,2,1);
imshow(Ip);
subplot(2,2,2);
imshow(vid_e(:,:,k));
subplot(2,2,3); 
imhist(Ip);
subplot(2,2,4);
imhist(vid_e(:,:,k));
pause(0.5);
end
%%
% for homework you may turn to under_exposure decommenting the line and try
% again
vid_e2=vid_e;
% vid_e2(:,:,10:15)=max(0,vid_e(:,:,10:15)-0.2); % under-exposure max is needed to
% preserve from going below 0
vid_e2(:,:,10:15)=min(1,vid_e(:,:,10:15)+0.4); % over-exposure min is needed to preserve from exceeding 1
%%
close all; 
figure('WindowStyle','docked');
pause();
for k = 1 : Nframe
    k
warning off;
% Ip=vid_lc(:,:,k);
% vid_e(:,:,k) = imhistmatch(Ip,I1); % first input is the low contrast frame, second input is the high contrast target frame
% subplot(2,2,1);
% imshow(Ip);
subplot(2,1,1);
imshow(vid_e2(:,:,k));
subplot(2,1,2); 
% imhist(Ip);
% subplot(2,2,4);
imhist(vid_e2(:,:,k));
pause(0.5);
end



%%
% per risolvere lo scurimento
close all; 
figure('WindowStyle','docked');
pause();
for k = 1 : Nframe
    k
warning off;
Ip=vid_e2(:,:,k);
vid_e3(:,:,k) = imhistmatch(Ip,I1); % first input is the low contrast frame, second input is the high contrast target frame
subplot(2,2,1);
imshow(Ip);
subplot(2,2,2);
imshow(vid_e3(:,:,k));
subplot(2,2,3); 
imhist(Ip);
subplot(2,2,4);
imhist(vid_e3(:,:,k));
drawnow;
pause();
end