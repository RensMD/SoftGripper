clear;

%% Settings
nSamples = 4;

angleStarting = 0;
angleMax = 130;

presMin = 55;
presMax = 90;

%% Setup

% % Setup arduino communication
instrfind('Port','COM5') % Check if arduinoActuator is in use
instrfind('Port','COM8') % Check if arduinoActuator is in use

arduinoActuator = serial('COM5','BaudRate',9600);
arduinoSensor = serial('COM8','BaudRate',9600);
arduinoSensor.Terminator = 'CR/LF';
fopen(arduinoActuator);
fopen(arduinoSensor);

command = 'a';

% Setup Webcam
webcamlist;
cam = webcam('C922 Pro Stream Webcam');
load('Camera Calibration\Calibration webcam\cameraParams.mat');

% Setup Folders
c = clock;
folderDate = join(num2str(c(1:5),'%02d'));
topFolderImages = 'Images/';
topFolderData = 'Datasets/';
slash = '/';
extPNG = '.png';
extMAT = '.mat';

pathFolderImages = [topFolderImages, folderDate];
pathFolderImages = join(pathFolderImages);
mkdir (pathFolderImages);

% Setup Data Matrix
dataMatrix = zeros(nSamples, 23);

%% Loop
for curSample=1:(nSamples)
    
    % Create random values for angle and pressure
    randAngle = round(0 + angleMax*rand);
    randPress = round(presMin + presMax*rand);
    
    % Move actuator
    fprintf(arduinoActuator,'R%s', num2str(randAngle)); %send command to change angle obstacle to arduino
    pause(1);
    fprintf(arduinoActuator,'T%s', num2str(randPress)); %send command to change pressure

    % Prepare image path
    number = num2str(curSample,'%04d');
    pathImage = [pathFolderImages, slash, number, extPNG];
    pathImage = join(pathImage);
 
    % Wait for comfirmation from arduino
    while arduinoActuator.BytesAvailable  == 0
        pause(1)
    end
    
    % Get Data from cam and sensors
    img = snapshot(cam);
    fprintf(arduinoSensor,command);
    pause(1);
    sensorData0 = str2double(strsplit(fgetl(arduinoSensor),','));
    sensorData1 = str2double(strsplit(fgetl(arduinoSensor),','));
    sensorData2 = str2double(strsplit(fgetl(arduinoSensor),','));
    sensorData3 = str2double(strsplit(fgetl(arduinoSensor),','));
    sensorData4 = str2double(strsplit(fgetl(arduinoSensor),','));
    
    % Save Data
    imwrite(img, pathImage);
    dataMatrix(curSample,1:23) = [curSample, randAngle, randPress, sensorData0(2:5), sensorData1(2:5), sensorData2(2:5), sensorData3(2:5), sensorData4(2:5)];
    
    fprintf(arduinoActuator,'S'); % angle back to original
    pause (1);
    fprintf(arduinoActuator,'U'); % angle back to starting position
    pause(1);
     
end

% Save data matrix
matriceName = join([topFolderData, folderDate, extMAT]);
save(matriceName, 'dataMatrix');

fclose(arduinoActuator);
fclose(arduinoSensor);
clear cam;

fprintf(' DONE ');