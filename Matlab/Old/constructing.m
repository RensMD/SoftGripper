%% CONSTRUCT BELLOW SHAPE
n_angles=5;
n_hinges = n_angles+1; %number of hinges
l_segment=10; %length of one segment in mm
%l_arc=l_segment*pixmm; %length of segment in pixels
l_arc=10;

Sensordata_array=zeros(1,n_angles);
%Sensordata_array=Read arduino
Curvature_array=zeros(1,n_angles);
%Curvature_array=Sensormodel Function With Sensordata as input
%Radii_array=1./Curvature_array; %radius from curvature
Radii_array=[100 30 10000 -30 9999];
Alpha_array=zeros(1,n_hinges); %angles %first angle zero

p_hinges=zeros(n_hinges,2); %XY-coordinates of hinges, p1 set to [0,0]
c_circle=zeros(n_angles,2); %XY-coordinates of centers of circle segments
Radii_array_leveled=1-(abs((Radii_array))).^-2;
figure
for angle=1:n_angles  
    c_circle(angle,:)=[p_hinges(angle,1)-sind(Alpha_array(angle))*Radii_array(angle),p_hinges(angle,2)-cosd(Alpha_array(angle))*Radii_array(angle)];
    Alpha_array(angle+1)=Alpha_array(angle)+(360*l_arc)/(2*pi*Radii_array(angle));
    p_hinges(angle+1,:)=[c_circle(angle,1)+sind(Alpha_array(angle+1))*Radii_array(angle),c_circle(angle,2)+cosd(Alpha_array(angle+1))*Radii_array(angle)];
    
    degrees = linspace(90-Alpha_array(angle+1),90-Alpha_array(angle));
    x = Radii_array(angle)*cosd(degrees)+c_circle(angle,1);
    y = Radii_array(angle)*sind(degrees)+c_circle(angle,2);

    plot(x, y,'LineWidth',4,'color',[0 1 (Radii_array_leveled(angle)-min(Radii_array_leveled))/(max(Radii_array_leveled)-min(Radii_array_leveled))])
    axis equal
    hold on
end
hold off





