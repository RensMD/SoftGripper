clear;

%function [s0,s1,s2,s3]=cubic_spline(x,y)
% [s0,s1,s2,s3]=cubic_spline(x,y)
%
% computes the coefficents of a cubic spline 
% interpolant through the data points (x,y)
%
% The spline is defined as the piecewise cubic
% polynomial 
%
% S(x) = { Sk(x)    x(k) <= x <= x(k+1)
%      = { 0        otherwise
%
% The cubic polynomial Sk(x) is given by 
%
% Sk(x)  = sk0 + sk1*(x-x(k)) + sk2*(x-x(k))^2 + sk3*(x-x(k))^3
%
% The coefficents sk0, sk1, sk2, and sk3 for each of the 
% polynomials are returned in the vectors s0,s1,s2, and s3
% respectively.

%real_arc_length=1;
%f=(@x)(x^2);
%x=[-3:3]';
%y=[-3 -1 -0.5 0 0.5 1 2]';
%y=[1 1 1 1 1 1 1]';
%TODO: create point sets that are based on curves, where the points are at a
%distance of a fixed arc length of each other. Use these point sets to
%determine accuracy of fits.

%% Settings
usingWebcam = false;
showTspResults = false;

if usingWebcam
    % Get image from webcam
    webcamlist
    cam = webcam(2);
    img = snapshot(cam);
    imwrite(img,'marker_demo.png'); % TODO update
else
    % Get demo image
    img = imread('marker_demo_5.jpg');
end

% Load webcam calibration parameters and use them to undistort the image
%load('Camera Calibration\Calibration webcam\cameraParams.mat');
%img = undistortImage(img,cameraParams);

% Create a B&W mask for the markers
imgMasked = createMask(img);

%% Marker identification
% Find markers in the masked image, store their center-coordinates and radii
[markerCenterCoords, markerRadii] = imfindcircles(imgMasked,[8, 20]);
markerCount = length(markerCenterCoords);
lineCount = markerCount-1;

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
imshow(imgMasked);
viscircles(markerCenterCoords,markerRadii);
hold on

%% Cubic Spline

x=markerCenterCoords(:,1);
y=markerCenterCoords(:,2);

if any(size(x) ~= size(y)) || size(x,2) ~= 1
    error('inputs x and y must be column vectors of equal length');
end

n = length(x);

h = x(2:n) - x(1:n-1);
d = (y(2:n) - y(1:n-1))./h;

lower = h(1:end-1);
main  = 2*(h(1:end-1) + h(2:end));
upper = h(2:end);

T = spdiags([lower main upper], [-1 0 1], n-2, n-2);
rhs = 6*(d(2:end)-d(1:end-1));

m = T\rhs;

% Use natural boundary conditions where second derivative
% is zero at the endpoints

m = [ 0; m; 0];

s0 = y;
s1 = d - h.*(2*m(1:end-1) + m(2:end))/6;
s2 = m/2;
s3 =(m(2:end)-m(1:end-1))./(6*h);

% plot_cubic_spline(x,s0,s1,s2,s3)
%
% plots a cubic spline with break points x 
% and coefficents s0, s1, s2, s3

n = length(x);

inner_points = 20;
point_distance=zeros(inner_points-1,1);
arc_lengths=zeros(n-1,1);

for i=1:n-1
    xx = linspace(x(i),x(i+1),inner_points);
    xi = repmat(x(i),1,inner_points);
    yy = s0(i) + s1(i)*(xx-xi) + ... 
         s2(i)*(xx-xi).^2 + s3(i)*(xx - xi).^3;
    
    %approximate arc length by cumulative chordal distance
    coordinates=[xx',yy'];
        for ii=1:inner_points-1
            point_distance(ii)=pdist(coordinates(ii:ii+1,:));
        end
    arc_lengths(i)=sum(point_distance);
    plot(xx,yy,'b')
    hold on
    plot(x(i),0,'r');
    hold on
end

%plot data points
scatter(x,y)
set(gca,'Ydir','reverse')
hold off

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
    
    % Return Output
    if nargout      
        varargout = {optRoute};
    end
    
end