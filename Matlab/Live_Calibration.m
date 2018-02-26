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

% Create a B&W mask for the markers
imgMasked = createMask(img);

% Find markers in the masked image, store their center-coordinates and radii
[markerCenterCoords, markerRadii] = imfindcircles(imgMasked,[8, 20]);

%  Show the found markers in the image
imshow(img);
viscircles(markerCenterCoords,markerRadii);
hold on;

% Determine the number of markers, lines, and curves
markerCount = length(markerCenterCoords);
lineCount = markerCount-1;
curveCount = lineCount-1; 

% Sort the found centers from left to right
markerCenterCoords = sortrows(markerCenterCoords);
% TODO sort based on nearest point

% Create lines between centers, calculate the length of the lines, and plot the lines
lineCoords = zeros(lineCount,4);
lineLengths = zeros(lineCount,1);
for n = 1:lineCount
    
    % lineCoords: [x1,y1,x2,y2]
    lineCoords(n,:) = [markerCenterCoords(n,1), markerCenterCoords(n,2), markerCenterCoords(n+1,1), markerCenterCoords(n+1,2)];
    lineLengths(n,1) = pdist(markerCenterCoords(n:n+1,:),'euclidean');
    plot([lineCoords(n,1), lineCoords(n,3)], [lineCoords(n,2), lineCoords(n,4)]);

end

% Set reference length and calculate pixels per mm based on the first line
% % TODO switch of temporary solution"of taking longest line
% % TODO find solution for when lines are longer than the reference line
% % This problem is a result of the camera perspective
% referenceLength = distancescenters(1,1);
refLineLength = max(lineLengths);
realLineLengthmm = 10; % Real length between centers in mm
pixelsPermm = refLineLength/realLineLengthmm;

% Calculate the angle (theta) between the centers for each curve
% Ignore the first line, which is used for reference 
curveThetas = zeros(curveCount,1);
for n = 1:curveCount
    
    % Take length of the current line, compare to reference line length, 
    % and calculate the resulting angle (theta) for the curve
    syms x;
    eq1 = lineLengths(n+1)*x == 2*refLineLength*sin(x/2);
    theta = vpasolve(eq1,x,pi/2);
   
    % If theta is too small, set to a fixed value
    if  theta > 1.0e-6 
        curveThetas(n) = theta;
    else
        curveThetas(n) = 1.0e-6;
    end
    
    % Check if the second center-coord of the current line is above the previous line
    % if above the prvious line, the curvature is reversed
    lineSlope = (lineCoords(n,4)-lineCoords(n,2))./(lineCoords(n,3)-lineCoords(n,1)); 
    if lineCoords(n+1,4) < (lineSlope*(lineCoords(n+1,3)-lineCoords(n,1))+lineCoords(n,2))
        curveThetas(n) = curveThetas(n)*-1;
    end
    
end

curveRadii = refLineLength./curveThetas;
curveCurvature = 1./curveRadii;

%% CONSTRUCT BELLOW SHAPE FIGURE

% Sensordata_array = zeros(1,linesCount);
% Sensordata_array=Read arduino

% Curvature_array = zeros(1,linesCount);
% Curvature_array=Sensormodel Function With Sensordata as input
% radiicenters=1./Curvature_array; %radius from curvature

arcStartAngles = zeros(lineCount,1); % Starting angles of the hinges, first hingle-angle equals zero
arcHingeCoords = zeros(lineCount,2); % XY coordinates of hinges, first hinge-position set to [0,0]
arcCenterCoords = zeros(curveCount,2); % XY-coordinates of centers of arc segments between hinges

% Level the radii centers
curveRadiiLeveled = 1-(abs((curveRadii))).^-2; 

% Create a new window for the bellow figure, calculate and display the bellow curves
figure;
for n = 1:curveCount 
    
    arcCenterCoords(n,:) = [(arcHingeCoords(n,1)-sind(arcStartAngles(n))*curveRadii(n)), (arcHingeCoords(n,2)-cosd(arcStartAngles(n))*curveRadii(n))];
    arcStartAngles(n+1) = arcStartAngles(n)+(360*refLineLength)/(2*pi*curveRadii(n));
    arcHingeCoords(n+1,:) = [arcCenterCoords(n,1)+sind(arcStartAngles(n+1))*curveRadii(n), arcCenterCoords(n,2)+cosd(arcStartAngles(n+1))*curveRadii(n)];
    
    arcAngleDifference = linspace(90-arcStartAngles(n+1),90-arcStartAngles(n));
    x = curveRadii(n)*cosd(arcAngleDifference)+arcCenterCoords(n,1);
    y = curveRadii(n)*sind(arcAngleDifference)+arcCenterCoords(n,2);

    plot(x, y,'LineWidth',4,'color',[0, 1, (curveRadiiLeveled(n)-min(curveRadiiLeveled))/(max(curveRadiiLeveled)-min(curveRadiiLeveled))])
    axis equal;
    hold on;
    
end

% TODO overlay Bellow on original image
% TODO first hinge position equals first center of first curve in image
% TODO match hinge positions with center positions 

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