clear;

% TODO: Pause option, if user sees a problem, allow pausing of acquisition
% TODO: Check for incoming command "L" which means the bellow is leaking
% TODO: Allow fixing of bellow and send "F" to confirm fix. Send "R" afterwards and continue
% TODO: Check if sensor data makes sense
% TODO: break up samples in series. Check if everything is still ok after each series and save Data
% TODO: Allow cancellation of acquisition at any moment
% TODO: Allow enbaling and disabling of seccond measurement within same settings

%% Settings
nSamples = 600; % Multiple of 2

angleStarting = 0;
angleMax = 120;

presMin = 55;
presMax = 90;

%% Setup

% Setup Folders
format shortg
c = clock;
number = num2str(c(1:5),'%02d');
folderDate = join(number);
topFolderImages = 'Images/';
topFolderData = 'Datasets/';
slash = '/';
extPNG = '.png';
extMAT = '.mat';

% % Setup arduino communication
instrfind('Port','COM5') % Check if arduinoActuator is in use
instrfind('Port','COM8') % Check if arduinoSensor is in use

arduinoActuator = serial('COM5','BaudRate',9600);
arduinoSensor = serial('COM8','BaudRate',9600);
arduinoSensor.Terminator = 'CR/LF';
fopen(arduinoActuator);
fopen(arduinoSensor);
pause(5);

command = 'a';

% Setup Webcam
webcamlist;
cam = webcam('C922 Pro Stream Webcam');

pathFolderImages = [topFolderImages, folderDate];
pathFolderImages = join(pathFolderImages);
mkdir (pathFolderImages);

% Setup Data Matrix
dataMatrix = zeros(nSamples, 23);

%% Loop
for curSample=1:2:(nSamples)
    
    % Print current sample 
    fprintf('[%d/%d]\n', curSample, nSamples);
    
    % Create random values for angle and pressure
    randAngle = round(0 + angleMax*rand);
    randPress = round(presMin + (presMax-presMin)*rand);
    
    % Move actuator
    fprintf(arduinoActuator,'A%s', num2str(randAngle)); %send command to change angle obstacle to arduino
    pause(5);
    fprintf(arduinoActuator,'P%s', num2str(randPress)); %send command to change pressure
    pause(1);

    % Wait for comfirmation from arduino
    while arduinoActuator.BytesAvailable  == 0
        pause(1)
    end
    % TODO prevent "L" command from getting lost
    flushinput(arduinoActuator);
    pause(1);

    % Get and save first set of data
    % Get Data from cam and sensors
    img = snapshot(cam);
    fprintf(arduinoSensor,command);
    pause(1);
    sensorData0 = str2double(strsplit(fgetl(arduinoSensor),','));
    sensorData1 = str2double(strsplit(fgetl(arduinoSensor),','));
    sensorData2 = str2double(strsplit(fgetl(arduinoSensor),','));
    sensorData3 = str2double(strsplit(fgetl(arduinoSensor),','));
    sensorData4 = str2double(strsplit(fgetl(arduinoSensor),','));
    
    % Save Image to path
    number = num2str(curSample,'%04d');
    pathImage = [pathFolderImages, slash, number, extPNG];
    pathImage = join(pathImage);
    imwrite(img, pathImage);
    
    % Save Data matrix
    dataMatrix(curSample,1:23) = [curSample, randAngle, randPress, sensorData0, sensorData1, sensorData2, sensorData3, sensorData4];
    

    % Print current sample 
    fprintf('[%d/%d]\n', curSample+1, nSamples);
    
    % Break in between measurements
    pause(2);
    
    
    % Get and save second set of data
    % Get Data from cam and sensors
    img = snapshot(cam);
    fprintf(arduinoSensor,command);
    pause(1);
    sensorData0 = str2double(strsplit(fgetl(arduinoSensor),','));
    sensorData1 = str2double(strsplit(fgetl(arduinoSensor),','));
    sensorData2 = str2double(strsplit(fgetl(arduinoSensor),','));
    sensorData3 = str2double(strsplit(fgetl(arduinoSensor),','));
    sensorData4 = str2double(strsplit(fgetl(arduinoSensor),','));
    
    % Save Image to path
    number = num2str(curSample+1,'%04d');
    pathImage = [pathFolderImages, slash, number, extPNG];
    pathImage = join(pathImage);
    imwrite(img, pathImage);
    
    % Save Data matrix
    dataMatrix(curSample+1,1:23) = [curSample+1, randAngle, randPress, sensorData0, sensorData1, sensorData2, sensorData3, sensorData4];
  
   
    %Reset actuator positions
    fprintf(arduinoActuator,'R'); % angle back to original
    pause (6);
     
end

% Save data matrix
matriceName = join([topFolderData, folderDate, extMAT]);
save(matriceName, 'dataMatrix');

fclose(arduinoActuator);
fclose(arduinoSensor);
clear cam;

fprintf(' DONE ');