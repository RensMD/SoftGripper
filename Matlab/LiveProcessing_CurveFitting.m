clear;

cubicSpline = true;

load('Dataset/markerCenterCoords', 'markerCenterCoords');

% Determine the number of markers, lines, and curves
markerCount = length(markerCenterCoords);
lineCount = markerCount-1;
curveCount = lineCount-1; 

%% Curvefitting
if cubicSpline
    %% Cubic Spline
    x = markerCenterCoords(:,1);
    y = markerCenterCoords(:,2);

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

else 
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
end