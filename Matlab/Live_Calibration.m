clear;

% TODO Arduino sensor input
% TODO Calculate bezier-curve

%% Settings
usingWebcam = false;
showTspResults = true;

%% Image processing
% Select source of image
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

%% Marker identification
% Find markers in the masked image, store their center-coordinates and radii
[markerCenterCoords, markerRadii] = imfindcircles(imgMasked,[8, 20]);

% Determine the number of markers, lines, and curves
markerCount = length(markerCenterCoords);
lineCount = markerCount-1;
curveCount = lineCount-1; 

% Sort the found centers by nearest neighbour
% TODO improve starting point identification
[~,I] = min(markerCenterCoords(:,1)); % starting point is most left
markerCenterCoords([1,I],:) = markerCenterCoords([I,1],:);
result = tspGA(markerCenterCoords,showTspResults);
markerCenterCoordsUnsorted = markerCenterCoords;
for n = 1:lineCount
    markerCenterCoords(n+1,:) = markerCenterCoordsUnsorted(result(n),:);
end

%  Show the found markers in the image
figure;
imshow(img);
viscircles(markerCenterCoords,markerRadii);
hold on;

%% Lines
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
% % TODO switch of temporary solution of taking the longest line
% % TODO find solution for when lines are longer than the reference line
% % This problem is a result of the camera perspective
% referenceLength = distancescenters(1,1);
refLineLength = max(lineLengths);
realLineLengthmm = 10; % Real length between centers in mm
pixelsPermm = refLineLength/realLineLengthmm;

%% Curve Thetas and Radii
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
% TODO turn into function with input curveRadii?
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

%% FUNCTIONS
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

%% FIND ORDER OF MARKERS WITH A TSP GENETIC ALGORITHM
% Adaptation of "Fixed start open TSP GA" by Joseph Kirk, see license
% TODO Add Kirk's licence
function varargout = tspGA(coords,show)
    
    % Initialize default configuration
    xy          = coords;
    dmat        = [];
    % Extract these parameters in case GA tuning is required?
    popSize     = 4;
    numIter     = 1e2;
    showResult  = show;
    if isempty(dmat)
        nPoints = size(xy,1);
        a = meshgrid(1:nPoints);
        dmat = reshape(sqrt(sum((xy(a,:)-xy(a',:)).^2,2)),nPoints,nPoints);
    end
    
    % Verify Inputs
    [N,dims] = size(xy);
    [nr,nc] = size(dmat);
    if N ~= nr || N ~= nc
        error('Invalid XY or DMAT inputs!')
    end
    n = N - 1; % Separate Start City
    
    % Sanity Checks
    popSize     = 4*ceil(popSize/4);
    numIter     = max(1,round(real(numIter(1))));
    showResult  = logical(showResult(1));
    
    % Initialize the Population
    pop = zeros(popSize,n);
    pop(1,:) = (1:n) + 1;
    for k = 2:popSize
        pop(k,:) = randperm(n) + 1;
    end
    
    % Run the GA
    globalMin = Inf;
    totalDist = zeros(1,popSize);
    distHistory = zeros(1,numIter);
    tmpPop = zeros(4,n);
    newPop = zeros(popSize,n);
    for iter = 1:numIter
        % Evaluate Each Population Member (Calculate Total Distance)
        for p = 1:popSize
            d = dmat(1,pop(p,1)); % Add Start Distance
            for k = 2:n
                d = d + dmat(pop(p,k-1),pop(p,k));
            end
            totalDist(p) = d;
        end
        
        % Find the Best Route in the Population
        [minDist,index] = min(totalDist);
        distHistory(iter) = minDist;
        if minDist < globalMin
            globalMin = minDist;
            optRoute = pop(index,:);
        end
        
        % Genetic Algorithm Operators
        randomOrder = randperm(popSize);
        for p = 4:4:popSize
            rtes = pop(randomOrder(p-3:p),:);
            dists = totalDist(randomOrder(p-3:p));
            [ignore,idx] = min(dists); %#ok
            bestOf4Route = rtes(idx,:);
            routeInsertionPoints = sort(ceil(n*rand(1,2)));
            I = routeInsertionPoints(1);
            J = routeInsertionPoints(2);
            for k = 1:4 % Mutate the Best to get Three New Routes
                tmpPop(k,:) = bestOf4Route;
                switch k
                    case 2 % Flip
                        tmpPop(k,I:J) = tmpPop(k,J:-1:I);
                    case 3 % Swap
                        tmpPop(k,[I J]) = tmpPop(k,[J I]);
                    case 4 % Slide
                        tmpPop(k,I:J) = tmpPop(k,[I+1:J I]);
                    otherwise % Do Nothing
                end
            end
            newPop(p-3:p,:) = tmpPop;
        end
        pop = newPop;   
    end
    
    if showResult
        % Plots the GA Results
        figure('Name','TSPOFS_GA | Results','Numbertitle','off');
        subplot(2,2,1);
        pclr = ~get(0,'DefaultAxesColor');
        if dims > 2, plot3(xy(:,1),xy(:,2),xy(:,3),'.','Color',pclr);
        else plot(xy(:,1),xy(:,2),'.','Color',pclr); end
        title('City Locations');
        subplot(2,2,2);
        imagesc(dmat([1 optRoute],[1 optRoute]));
        title('Distance Matrix');
        subplot(2,2,3);
        rte = [1 optRoute];
        if dims > 2
            plot3(xy(rte,1),xy(rte,2),xy(rte,3),'r.-', ...
                xy(1,1),xy(1,2),xy(1,3),'ro');
        else
            plot(xy(rte,1),xy(rte,2),'r.-',xy(1,1),xy(1,2),'ro');
        end
        title(sprintf('Total Distance = %1.4f',minDist));
        subplot(2,2,4);
        plot(distHistory,'b','LineWidth',2);
        title('Best Solution History');
        set(gca,'XLim',[0 numIter+1],'YLim',[0 1.1*max([1 distHistory])]);
    end
    
    % Return Output
    if nargout      
        varargout = {optRoute};
    end
    
end