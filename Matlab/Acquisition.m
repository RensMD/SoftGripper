clear;

% TODO: Pause option, if user sees a problem, allow pausing of acquisition
% TODO: Check for incoming command "L" which means the bellow is leaking
% TODO: Allow fixing of bellow and send "F" to confirm fix. Send "R" afterwards and continue
% TODO: Check if sensor data makes sense
% TODO: break up samples in series. Check if everything is still ok after each series and save Data
% TODO: Allow cancellation of acquisition at any moment
% TODO: Allow enbaling and disabling of seccond measurement within same settings

%% Settings
% Data acuisition
nSensors = 4;
nMarkers = 6;
nSamples = 600; % Must be multiple of 2

% Obstacle
angleStarting = 0;
angleMax = 120;

% Pressure
presMin = 55;
presMax = 90;

%% Setup
% Setup Folders
nameDate = datestr(now, 'yy-mm-dd_HH-MM');
nameInfo = ['_S' num2str(nSensors) '-M' num2str(nMarkers) '-N' num2str(nSamples)];
nameIN = '_IN';

topFolderImages = 'Images/';
topFolderData = 'Datasets/';
slash = '/';

extPNG = '.png';
extMAT = '.mat';

pathName = join([nameDate, nameInfo]);
pathImageFolder = join([topFolderImages, pathName]);
pathDataFolder = join([topFolderData, pathName]);

mkdir (pathImageFolder);
mkdir (pathDataFolder);

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

% Setup Data Matrix
sensorData = zeros(nSensors, 4);
dataMatrix = zeros(nSamples, (3+(nSensors*4)));

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
    for n=1:nSensors
        sensorData(n,:) = str2double(strsplit(fgetl(arduinoSensor),','));
    end
    
    % Save Image to path
    sampleString = num2str(curSample,'%04d');
    pathImage = join([pathImageFolder, slash, sampleString, extPNG]);
    imwrite(img, pathImage);
    
    % Save Data matrix
    dataMatrix(curSample,:) = [curSample, randAngle, randPress, reshape(sensorData.',1,[])];
    

    % Print current sample 
    fprintf('[%d/%d]\n', curSample+1, nSamples);
    
    % Break in between measurements
    pause(2);
    
    
    % Get and save second set of data
    % Get Data from cam and sensors
    img = snapshot(cam);
    fprintf(arduinoSensor,command);
    pause(1);
    for n=1:nSensors
        sensorData(n,:) = str2double(strsplit(fgetl(arduinoSensor),','));
    end
    
    % Save Image to path
    sampleString = num2str(curSample+1,'%04d');
    pathImage = join([pathImageFolder, slash, sampleString, extPNG]);
    imwrite(img, pathImage);
    
    % Save Data matrix
    dataMatrix(curSample+1,:) = [curSample+1, randAngle, randPress, reshape(sensorData.',1,[])];
  
   
    %Reset actuator positions
    fprintf(arduinoActuator,'R'); % angle back to original
    pause (6);
     
end

% Save data matrix
matricePathName = join([pathDataFolder, slash, pathName, nameIN, extMAT]);
save(matricePathName, 'dataMatrix');

fclose(arduinoActuator);
fclose(arduinoSensor);
clear cam;

fprintf(' DONE ');