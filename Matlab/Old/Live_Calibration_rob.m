clear
%camera calibration parameters Iphone Rob

%originalImage = imread(imageFileNames{1});
%undistortedImage = undistortImage(originalImage, cameraParams);

using_webcam = false;
% TODO camera calibration

% TODO arduino sensor input
if using_webcam
    % Get image from webcam
    webcamlist
    cam = webcam(2);
    img = snapshot(cam);
    imwrite(img,'marker_demo.png'); % TODO update
else
    % Get demo image
    img = imread('marker_demo.png');
end

% Create a mask for the markers
img = createMask(img);

% Find circles in image mask and show 
[centroids,radii] = imfindcircles(img,[8,20]);
imshow(img);
viscircles(centroids,radii);
hold on;

% Sort the found circle centroids from left to right
centroids = sortrows(centroids);
dist=zeros(1,length(centroids)-1);
for d=1:length(centroids)-1
dist(1,d)= pdist(centroids(d:d+1,:),'euclidean');
end
%arclength=dist(1,1);
arclength=62;

theta=zeros(1,length(dist));
syms x
for d=1:length(dist)
eq1=dist(d)*x==2*arclength*sin(x/2);
theta(d)=vpasolve(eq1,x,pi/2);
end
r=zeros(1,length(dist));
r=arclength./theta;
K=1./r;


% Determine number of circles and lines and create a array for the lines
circle_count = numel(centroids)/2;
lines_count = circle_count-1; 
lines_array = zeros(lines_count,4);

% Store circle centroid cordinates in lines array
for n = 1:lines_count
    lines_array(n,:) = [centroids(n,1), centroids(n,2), centroids(n+1,1), centroids(n+1,2)];
    plot([lines_array(n,1), lines_array(n,3)],[lines_array(n,2), lines_array(n,4)]);
end

% calculate pixels per mm based on the first line
lreal=50; %real length in mm (dumy)
d = pdist([lines_array(1,1),lines_array(1,2);lines_array(1,3),lines_array(1,4)],'euclidean');
% TODO calculate based on average?
pixmm = d/lreal;

% TODO calculate angle/curvature/bezier-curve

clear cam

%% 
function [BW,maskedRGBImage] = createMask(RGB)

% Convert RGB image to chosen color space
I = rgb2ycbcr(RGB);

% Histogramsettings are found using the Color Thresholder app
% TODO Update thresholds
% Define thresholds for channel 1 based on histogram settings
channel1Min = 53.000;
channel1Max = 209.000;

% Define thresholds for channel 2 based on histogram settings
channel2Min = 70.000;
channel2Max = 111.000;

% Define thresholds for channel 3 based on histogram settings
channel3Min = 145.000;
channel3Max = 244.000;

% Create mask based on chosen histogram thresholds
sliderBW = (I(:,:,1) >= channel1Min ) & (I(:,:,1) <= channel1Max) & ...
    (I(:,:,2) >= channel2Min ) & (I(:,:,2) <= channel2Max) & ...
    (I(:,:,3) >= channel3Min ) & (I(:,:,3) <= channel3Max);
BW = sliderBW;

% Initialize output masked image based on input image.
maskedRGBImage = RGB;

% Set background pixels where BW is false to zero.
maskedRGBImage(repmat(~BW,[1 1 3])) = 0;

end