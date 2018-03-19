clear all;
clear a;
clear addrs;
clear isl29125;
close all;
clc; 

a = arduino();
addrs = scanI2CBus(a, 0)
TMP006 = i2cdev(a, '0x40')
ISL29125 = i2cdev(a, '0x44')

time = 200;
while time > 0
    data = read(TMP006, 2, 'uint8');
    temperature = (double(bitshift(int16(data(1)), 4)) + double(bitshift(int16(data(2)), -4))) * 0.0625;
    disp(data);
    disp(temperature);
    
%     data2 = read(ISL29125, 12, 'uint8');
%     disp(data2);
    
    time= time - 1;
    pause(1);
end