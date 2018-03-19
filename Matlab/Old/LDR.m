%% Example of how to read out sensor data through the Arduino and filter it "real-time"
% with a random moving average.
% For Linux, create a symbolic link as follows:
% ~# sudo ln -s /dev/ttyACM0 /dev/ttyS102

clear all; close all; clc; 

a = arduino('/dev/ttyS102', 'uno');
f = figure; 
i=1;
while(1)
av(i) = readVoltage(a,0)
avF(i) = 0;
if(i>5)
    avF(i) = 0.4*av(i)+0.25*av(i-1)+0.2*av(i-2)+0.15*av(i-3);
    plot(av);
    hold on
    plot(avF);
    axis([i-100 i+100 3 3.5]);
    drawnow;
end
i=i+1;
end

%somehow to get Arduino going again. 
clear a
