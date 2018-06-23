# SoftGripper
Color Based Position Sensing of Soft Pneumatic Actuators Using Neural Nets 

## Calibration
### Acquisition
1. Move Soft Pneumatic Actuator to random position
1. Move Obstacle to random position
1. Get sensordata and pictures of position
1. Store actuator- and sensor-data

### Image Processing
1. Use computer vision to identify markers on Soft Pneumatic Actuator
1. Use TSP algorithm to sort marker positions
1. Store position data

### Create NN
1. Use MATLAB NN toolbox 

## Live Processing
1. Transform sensordata to position data using NN
1. Visualize position in real-time