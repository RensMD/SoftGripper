clear;

% TODO arduino sensor input
% TODO calculate bezier-curve

usingWebcam = false;

if usingWebcam
    % Get image from webcam
    webcamlist
    cam = webcam(2);
    img = snapshot(cam);
    imwrite(img,'marker_demo.png'); % TODO update
else
    % Get demo image
    img = imread('marker_demo.png');
end

% Load webcam calibration parameters and use them to undistort the image
load('Camera Calibration\Calibration webcam\cameraParams.mat');
img = undistortImage(img,cameraParams);

% Create a mask for the markers
maskedImg = createMask(img);

% Find circles in the masked image, store their centroid-coordinates and radii
[centroidCoordinatesArray, radiiCircles] = imfindcircles(maskedImg,[8, 20]);

%  Show the found circles in the image
imshow(img);
viscircles(centroidCoordinatesArray,radiiCircles);
hold on;

% Determine the number of centroids, lines, and curves
centroidCount = length(centroidCoordinatesArray);
lineCount = centroidCount-1;
curveCount = lineCount-1; 

% Sort the found centroids from left to right
centroidCoordinatesArray = sortrows(centroidCoordinatesArray);
% TODO sort based on nearest point

% Create lines between centroids, calculate the length of the lines, and plot the lines
lineArray = zeros(lineCount,4);
lineLengthArray = zeros(1,lineCount);
for n = 1:lineCount
    
    lineArray(n,:) = [centroidCoordinatesArray(n,1), centroidCoordinatesArray(n,2), centroidCoordinatesArray(n+1,1), centroidCoordinatesArray(n+1,2)];
    lineLengthArray(1,n) = pdist(centroidCoordinatesArray(n:n+1,:),'euclidean');
    plot([lineArray(n,1), lineArray(n,3)], [lineArray(n,2), lineArray(n,4)]);

end

% Set reference length and calculate pixels per mm based on the first line
% % TODO switch of temporary solution"of taking longest line
% % TODO find solution for when lines are longer than the reference line
% % This problem is a result of the camera perspective
% referenceLength = distancesCentroids(1,1);
referenceLineLength = max(lineLengthArray);
realLineLengthmm = 10; % Real length between centroids in mm
pixelsPermm = referenceLineLength/realLineLengthmm;

% Calculate the angle (theta) between the centroids for each curve
% Ignore the first line, which is used for reference 
curveThetaArray = zeros(1,curveCount);
syms x;
for n = 1:curveCount
    
    % Take length of the current line, compare to reference line length, 
    % and calculate the resulting angle (theta) for the curve
    eq1 = lineLengthArray(n+1)*x == 2*referenceLineLength*sin(x/2);
    theta = vpasolve(eq1,x,pi/2);
   
    % If theta is too small, set to a fixed value
    if  theta > 1.0e-6 
        curveThetaArray(n) = theta;
    else
        curveThetaArray(n) = 1.0e-6;
    end
    
    % Check if the second centroid of the current line is above the previous line
    % if above the prvious line, the curvature is reversed
    lineSlope = (lineArray(n,4)- lineArray(n,2))./(lineArray(n,3)-lineArray(n,1)); 
    if lineArray(n+1,4) < (lineSlope*(lineArray(n+1,3)-lineArray(n,1))+lineArray(n,2))
        curveThetaArray(n) = curveThetaArray(n)*-1;
    end
    
end

curveRadiiArray = referenceLineLength./curveThetaArray;
curveCurvatureArray = 1./curveRadiiArray;

%% CONSTRUCT BELLOW SHAPE FIGURE

% Sensordata_array = zeros(1,linesCount);
% Sensordata_array=Read arduino

% Curvature_array = zeros(1,linesCount);
% Curvature_array=Sensormodel Function With Sensordata as input
% radiiCentroids=1./Curvature_array; %radius from curvature

hingeAngleArray = zeros(1,centroidCount); % Starting angles of the hinges, first hingle-angle equals zero
hingeCoordinatesArray = zeros(centroidCount,2); % XY coordinates of hinges, first hinge-position set to [0,0]
hingeCircleCenterCoordinatesArray = zeros(lineCount,2); % XY-coordinates of centers of circle segments between hinges

% Level the radii centroids
curveRadiiLeveledArray = 1-(abs((curveRadiiArray))).^-2; 

% Create a new window for the bellow figure, calculate and display the bellow curves
figure;
for n = 1:curveCount 
    
    hingeCircleCenterCoordinatesArray(n,:) = [(hingeCoordinatesArray(n,1)-sind(hingeAngleArray(n))*curveRadiiArray(n)), (hingeCoordinatesArray(n,2)-cosd(hingeAngleArray(n))*curveRadiiArray(n))];
    hingeAngleArray(n+1) = hingeAngleArray(n)+(360*referenceLineLength)/(2*pi*curveRadiiArray(n));
    hingeCoordinatesArray(n+1,:) = [hingeCircleCenterCoordinatesArray(n,1)+sind(hingeAngleArray(n+1))*curveRadiiArray(n), hingeCircleCenterCoordinatesArray(n,2)+cosd(hingeAngleArray(n+1))*curveRadiiArray(n)];
    
    hingeAngleDifference = linspace(90-hingeAngleArray(n+1),90-hingeAngleArray(n));
    x = curveRadiiArray(n)*cosd(hingeAngleDifference)+hingeCircleCenterCoordinatesArray(n,1);
    y = curveRadiiArray(n)*sind(hingeAngleDifference)+hingeCircleCenterCoordinatesArray(n,2);

    plot(x, y,'LineWidth',4,'color',[0, 1, (curveRadiiLeveledArray(n)-min(curveRadiiLeveledArray))/(max(curveRadiiLeveledArray)-min(curveRadiiLeveledArray))])
    axis equal;
    hold on;
    
end

% TODO overlay Bellow on original image
% TODO first hinge position equals first centroid of first curve in image
% TODO match hinge positions with centroid positions 

clear cam;

%% CREATE B&W IMAGE MASK
function [BW, maskedRGBImage] = createMask(RGB)

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