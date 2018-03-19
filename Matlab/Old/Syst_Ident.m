%% Simple System Identification example
% Set up a standard LTI system, then from the input and output data
% retrieve it. 
clear all; close all; clc; 
%% Init
dt = 0.01;
t=0:dt:10;
N = length(t);

% Set up a random TF(MATLAB ex)
H = tf([0 0 1],[1 2 3]);
% check stability 
%[z,p,k]=zpkdata(H,'v');
%stepplot(H);

%% Set up noisy measurements
y = step(H,t);
yN = y + 0.05*randn(N,1);
u = ones(N,1);
% uN is of course not really needed. 
uN = u + 0.05*randn(N,1);
plot(y);
hold on
plot(u);
plot(yN);
plot(uN);
hold off

%% Try to fit a model
fun = @(x)step(tf([0 0 x(1)],[x(2) x(3) x(4)]),t)-yN; 
x0=[1,1,1,5];
options.Algorithm = 'levenberg-marquardt';
[x,resnorm,residual,exitflag,output] = lsqnonlin(fun,x0,[],[],options);

% check if the fit is improved
G_o = tf([0  0 x(1)],[x(2) x(3) x(4)]);
yo = step(G_o,t);
plot(t,y);
hold on
plot(t,yo);
 

