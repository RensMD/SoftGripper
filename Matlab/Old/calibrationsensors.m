clear;

%% CALIBRATION
imagefiles = dir('*.JPG'); 
PictureForCalibration = imagefiles(1).name;
figure
imshow(num2str(PictureForCalibration));
[x,y]=ginput(2);
close
lreal=50; %real length in mm
X = [x(1),y(1);x(2),y(2)];
d = pdist(X,'euclidean');
pixmm=d/lreal; %pixels per mm (calibration)

%% SELECTING 'HINGES'     
n_files = length(imagefiles);    % Number of files found
n_angles = 3; %number of angles 
n_hinges = n_angles+1; %number of hinges
xcoords=zeros(n_hinges,n_files);
ycoords=zeros(n_hinges,n_files);

for it1=1:n_files
   currentfilename = imagefiles(it1).name;
   figure
   imshow(num2str(currentfilename))
   [xcoords(:,it1),ycoords(:,it1)]=ginput(n_hinges);
   cla reset
   close
end

%% MODELING SENSOR MOTION (ANGLE, CURVATURE, BEZIER CURVE)

%Angles
angles=zeros(n_angles,n_files);
for it2=1:n_files
    for it3=1:n_angles
    angles(it3,it2)=atand((ycoords(it3+1,it2)-ycoords(it3,it2))/(xcoords(it3+1,it2)-xcoords(it3,it2)));
    end
end


%Curvature
center_coords_y=zeros(n_angles,n_files);
center_coords_x=zeros(n_angles,n_files);
radia=zeros(n_angles,n_files);
curvatures=zeros(n_angles,n_files);
l_segment=10; %length of one segment in mm
l_arc=l_segment*pixmm;

%{
for file=1:n_files
    radia(1,it2)= 1/2 - (4*xcoords(1,it2)^2 - 8*xcoords(1,it2)*xcoords(2,it2) + 4*xcoords(2,it2)^2 - 4*ycoords(1,it2) + 4*ycoords(2,it2) + 1)^(1/2)/2
    (4*xcoords(1,it2)^2 - 8*xcoords(1,it2)*xcoords(2,it2) + 4*xcoords(2,it2)^2 - 4*ycoords(1,it2) + 4*ycoords(2,it2) + 1)^(1/2)/2 + 1/2;
   for angle=2:n_angles
       syms r x_center y_center
       f=
       center_coords=
       
       radia(angle,file)=pdist(center_coords(ang,fil,1)
   end
end


%arclength=2*pi*r*(arcangle/360)

%for calculation distance, make note if curvature is negative or
%positive(y1-y2)
%{



arcangle=atand((0.5*d)/r);
arclength=2*pi*r*(arcangle/360)
%r=... %r in pixels
R=r/pixmm; %Radius in mm
K=1/R; %curvature

%% LEAST SQUARE FIT WITH SENSOR DATA
%Load in excel data
%Angles
%Curvature
%Beziers Curve
%%%%%%%%%%%%%%%%%%%LeastSquareFit for each sensor with sensor data attached with pictures.

%}
%% CONSTRUCT BELLOW SHAPE
l_segment=10; %length of one segment in mm
%l_arc=l_segment*pixmm; %length of segment in pixels
l_arc=10;

Sensordata_array=zeros(1,n_angles);
%Sensordata_array=Read arduino
Curvature_array=zeros(1,n_angles);
%Curvature_array=Sensormodel Function With Sensordata as input
Radii_array=1./Curvature_array; %radius from curvature

Alpha_array=zeros(1,n_angles); %angles

p_hinges=zeros(n_hinges,2); %XY-coordinates of hinges, p1 set to [0,0]
c_circle=zeros(n_angles,2); %XY-coordinates of centers of circle segments
c_circle(1,:)=[p_hinges(1,1),p_hinges(1,2)-Radii_array(1)];
Alpha_array(1)=(360*l_arc)/(2*pi*Radii_array(1));
    
for angle=2:n_angles
    
    p_hinges(angle,:)=[c_circle(angle-1,1)+sind(Alpha_array(angle-1))*Radii_array(angle-1),c_circle(angle-1,2)+cosd(Alpha_array(angle-1))*Radii_array(angle-1)];
    Alpha_array(angle)=
end


%Read data directly from Arduino
%a=arduino('com4','Due')
sens1; %read in live using matlab
sens2;
sens3;
sens4;
sens5;


%Angles
%Curvature
%Beziers Curve


%curvature=zeros(nangles,nfiles);
%calculate middle between points, arc length known (needs to be calibrated). 
%arc length=f(angle between points(R,x1,y1,x2,y2), R). solve for R.
%lsegment=10; %should be scaled to image resolution, but is a constant.

%}
