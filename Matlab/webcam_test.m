clear

webcamlist
cam = webcam(2);

img = snapshot(cam);
img = rgb2gray(img);
imshow(img);
hold on;

thresh = img < 200;
[y,x] = find(thresh);
xmean = mean(x);
ymean = mean(y);
plot(xmean, ymean, 'r.', 'MarkerSize', 18);

clear cam