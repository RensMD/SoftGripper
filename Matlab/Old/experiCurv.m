%% Experiment with curvature integration

clear all; close all; 
l = 0.5*pi;
t=0:0.01:l;
%Make it a unit vector field 
while(1)
    for r = 100:-1:5
        r = r*0.1;
        f = @(t,x) [x(2)/(sqrt(x(2)^2+x(1)^2)); -x(1)/(sqrt(x(2)^2+x(1)^2))];  
        [t,x] = ode45(f,t,[-r;0]);
        plot(x(:,1)+r,x(:,2),'LineWidth',3);
        hold on 
        plot(x(158,1)+r,x(158,2),'r.','MarkerSize',40);
        hold off
        axis([-0.5 2 -0.5 2]); 
        pbaspect([1 1 1])
        grid on
        titl = sprintf('Curvature %f, End-effector:[%f, %f]^{T}',1/r,x(158,1)+r,x(158,2));
        title(titl); 
        drawnow
    end
    for r = 5:1:100
        r = r*0.1;
        f = @(t,x) [x(2)/(sqrt(x(2)^2+x(1)^2)); -x(1)/(sqrt(x(2)^2+x(1)^2))];  
        [t,x] = ode45(f,t,[-r;0]);
        plot(x(:,1)+r,x(:,2),'LineWidth',3);
        hold on 
        plot(x(158,1)+r,x(158,2),'r.','MarkerSize',40);
        hold off

        axis([-0.5 2 -0.5 2]); 
        pbaspect([1 1 1])
        grid on
        titl = sprintf('Curvature %f, End-effector:[%f, %f]^{T}',1/r,x(158,1)+r,x(158,2));
        title(titl); 
        drawnow
    end
end

%% See how to plot a vector field:
%     [u,v] = meshgrid(-l:0.1:l,-l:0.1:l);
%     x = v./(sqrt(v.^2+u.^2));
%     y = -u./(sqrt(v.^2+u.^2));
%     quiver(u,v,x,y);

%% Estimating curvature if somehow y_i is given
clear all; close all; clc; 
w=0.1;
while(1)
for i=1:50
    y = 0.5+0.01*i;
    l = 0.5*pi;
    t=0:0.01:l;        
    r = (w^2+y^2)/(2*w);
    f = @(t,x) [x(2)/(sqrt(x(2)^2+x(1)^2)); -x(1)/(sqrt(x(2)^2+x(1)^2))];  
    [t,x] = ode45(f,t,[-r;0]);

    plot(0,y,'rx','MarkerSize',20);
    hold on
    plot(-w,0,'r.','MarkerSize',20);
    plot(x(:,1)+r-w,x(:,2));
    axis([-0.5 2 -0.5 2]); 
    pbaspect([1 1 1])
    grid on
    hold off
    drawnow
end
for i=50:-1:1
    y = 0.5+0.01*i;
    l = 0.5*pi;
    t=0:0.01:l;        
    r = (w^2+y^2)/(2*w);
    f = @(t,x) [x(2)/(sqrt(x(2)^2+x(1)^2)); -x(1)/(sqrt(x(2)^2+x(1)^2))];  
    [t,x] = ode45(f,t,[-r;0]);

    plot(0,y,'rx','MarkerSize',20);
    hold on
    plot(-w,0,'r.','MarkerSize',20);
    plot(x(:,1)+r-w,x(:,2));
    axis([-0.5 2 -0.5 2]); 
    pbaspect([1 1 1])
    grid on
    hold off
    drawnow
end
end

%% Closed form approach is possible and probably much faster, check the papers, 
% for the inverse you might need to implement a numerical solver(Newton like). 









