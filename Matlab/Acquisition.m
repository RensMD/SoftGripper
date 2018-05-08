clear;

% TODO store data in results matrix

%% Settings
nSamples = 3;

angleStarting = 0;
angleMax = 130;

presMin = 55;
presMax = 90;

angle=zeros(nSamples,1);
pressure=zeros(nSamples,1);

%% Setup

% Setup arduino communication
instrfind('Port','COM5') % Check if arduinoActuator is in use
instrfind('Port','COM6') % Check if arduinoActuator is in use

arduinoActuator = serial('COM5','BaudRate',9600);
arduinoSensor = serial('COM6','BaudRate',9600);
% arduinoActuator.Terminator = 'CR/LF';
arduinoSensor.Terminator = 'CR/LF';
fopen(arduinoActuator);
fopen(arduinoSensor);
fprintf(arduinoActuator, '%c', 'a');

% Setup webcam
webcamlist;
cam = webcam('C922 Pro Stream Webcam');
load('Camera Calibration\Calibration webcam\cameraParams.mat');

%% Loop
for i=1:nSamples
    
    % Move actuator
    randVal1 = num2str(round(0 + angleMax*rand));
    randVal2 = num2str(round(presMin + presMax*rand));
    fprintf(arduinoActuator,'R%s', randVal1); %send command to change angle obstacle to arduino
    pause(1);
    fprintf(arduinoActuator,'T%s', randVal2); %send command to change pressure
    pause(2);
    
    while arduinoActuator.BytesAvailable  == 0
        pause(1)
    end
    fgetl(arduinoActuator)
    
%     image = snapshot(cam); %take picture
%     thisFile=sprintf(nametemplate,i); %create filename
%     fullName=fullfile(savepath,thisFile); %folder included
%     imwrite(image,fullName); %write picture to file
%     pause(2)
    
%     % Get sensor data
%     command = 'a';
%     fprintf(arduinoSensor,command); % Send command
%     pause(2);
%     sensorData0 = fgetl(arduinoSensor)
%     sensorData1 = fgetl(arduinoSensor)
%     sensorData2 = fgetl(arduinoSensor)
%     sensorData3 = fgetl(arduinoSensor)
%     sensorData4 = fgetl(arduinoSensor)
%     
%     % Image processing
%     img = snapshot(cam);
%     imshow(img);
%     imwrite(img,'Demo Images/marker_demo.png'); % TODO update
    
    fprintf(arduinoActuator,'S'); %angle back to original
    pause (1);
    fprintf(arduinoActuator,'U'); % angle back to starting position
    pause(2);
end

% fprintf(' DONE ');
% hold off;
fclose(arduinoActuator);
fclose(arduinoSensor);
clear cam;