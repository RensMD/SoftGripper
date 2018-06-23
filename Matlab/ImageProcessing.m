clear;
warning off;

%% Settings
% Markers
nMarkers = 6;

% Processing 
showImg = false;
showTspResults = false;

% Path
imageFolder = ('Images/');
datasetFolder = 'Datasets/';
currentFolder = ('18-06-21_12-06_S5-N600'); % Set folder

slash = '/';
nameOUT = '_OUT';
extMAT = '.mat';

% Camera Calibration data
load('Camera Calibration\Logitec C922\cameraParams.mat');

%% Get images
% Get image file names
pathImageFolder = join([imageFolder, currentFolder, slash]);
filePattern = sprintf('%s/*.png', pathImageFolder);
baseFileNames = dir(filePattern);

% Count number of images and create data matrix
numberOfImageFiles = length(baseFileNames);
dataMatrix = zeros(3, (1+(nMarkers*2)));

% Create a check if all images meet conditions
allCorrect = true;

%% Loop
for i=1:numberOfImageFiles
    imgPath = fullfile(pathImageFolder, baseFileNames(i).name);
    
    % Load image and crop
    img = imread(imgPath);
    img = imcrop(img,[120 65 420 230]); % Tune for image
      
    % Load webcam calibration parameters and use them to undistort the image
    img = undistortImage(img,cameraParams);

    % Create a B&W mask for the markers
    imgMasked = createMask(img);
   
    % Retrieve marker coordinates
    markerCenterCoords = identifyMarkers(imgMasked, showImg, showTspResults);
    
    if numel(markerCenterCoords) ~= nMarkers*2
        fprintf('[%d]\n', i);
        allCorrect = false;
    else 
        dataMatrix(i,:) = [i, reshape(markerCenterCoords.',1,[])];
    end
        
end  

%% Save Data Matrix
if allCorrect  
    saveFolder = join([datasetFolder, currentFolder, slash]);
    saveFile = join([saveFolder, currentFolder, nameOUT, extMAT]);
    
    mkdir(saveFolder);
    save(saveFile, 'dataMatrix');
end

    %% FUNCTIONS
%% CREATE B&W IMAGE MASK
function imgMasked = createMask(RGB)

    % Histogramsettings are found using the Color Thresholder app
    % TODO Update thresholds

    % Convert RGB image to chosen color space
    I = rgb2hsv(RGB);
    
    % Define thresholds for channel 1 based on histogram settings
    channel1Min = 0.991;
    channel1Max = 0.020;

    % Define thresholds for channel 2 based on histogram settings
    channel2Min = 0.683;
    channel2Max = 1.000;

    % Define thresholds for channel 3 based on histogram settings
    channel3Min = 0.192;
    channel3Max = 0.448;

    % Create mask based on chosen histogram thresholds
    sliderBW = ( (I(:,:,1) >= channel1Min) | (I(:,:,1) <= channel1Max) ) & ...
        (I(:,:,2) >= channel2Min ) & (I(:,:,2) <= channel2Max) & ...
        (I(:,:,3) >= channel3Min ) & (I(:,:,3) <= channel3Max);
    imgMasked = sliderBW;

%     % Initialize output masked image based on input image.
%     maskedRGBImage = RGB;
% 
%     % Set background pixels where BW is false to zero.
%     maskedRGBImage(repmat(~imgMasked,[1 1 3])) = 0;

end

%% Marker identification
function markerCenterCoords = identifyMarkers(imgMasked, showImg, showTspResults)
    % Find markers in the masked image, store their center-coordinates and radii
    [markerCenterCoords, markerRadii] = imfindcircles(imgMasked,[6, 20]);

    % Determine the number of markers, lines, and curves
    lineCount = length(markerCenterCoords) - 1;

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
    if showImg
        figure;
        imshow(imgMasked);
        viscircles(markerCenterCoords,markerRadii);
        hold on;
    end
end

%% FIND ORDER OF MARKERS WITH A TSP GENETIC ALGORITHM
% Adaptation of "Fixed start open TSP GA" by Joseph Kirk, see license
% TODO Add Kirk's licence
function optRoute = tspGA(coords,show)
    
    % Initialize default configuration
    xy          = coords;
    dmat        = [];
    % Extract these parameters in case GA tuning is required?
    popSize     = 8; % Multiple of 4
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
        if dims > 2
            plot3(xy(:,1),xy(:,2),xy(:,3),'.','Color',pclr);
        else
            plot(xy(:,1),xy(:,2),'.','Color',pclr); 
        end
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
    
%     % Return Output
%     if nargout      
%         varargout = {optRoute};
%     end
    
end