clear
%PERCEPTRON for or operator
input = [1 0 0; 1 0 1; 1 1 0; 1 1 1]; %Input values, first value of each input = 1 for bias
numIn = 4;
desired_out = [0;1;1;1]; %Desired output values
%bias = -1; %initialized at constant: any non-zero number between -1 and 1
coeff = 0.7; %learning rate
rand('state',sum(100*clock));
weights = -1*2.*rand(3,1); %first weight is bias

iterations = 1000;

for i = 1:iterations
     out = zeros(4,1);
     for j = 1:numIn
          y = input(j,1)*weights(1,1)+...
               input(j,2)*weights(2,1)+input(j,3)*weights(3,1);
          out(j) = 1/(1+exp(-y));
          delta = desired_out(j)-out(j);
          weights(1,1) = weights(1,1)+coeff*input(j,1)*delta;
          weights(2,1) = weights(2,1)+coeff*input(j,2)*delta;
          weights(3,1) = weights(3,1)+coeff*input(j,3)*delta;
     end
end

output=perceptron(weights,[1 1 0])

function predicted=perceptron(weights,input)
    y1=input(1)*weights(1,1)+input(2)*weights(2,1)+input(3)*weights(3,1);
    predicted = round(1/(1+exp(-y1)));
end

