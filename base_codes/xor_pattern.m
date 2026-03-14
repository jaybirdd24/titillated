Pclose all, clear all, clc, format compact
% number of samples of each class
K = 100;
% define 4 clusters of input data
q = .6; % offset of classes
A = [rand(1,K)-q; rand(1,K)+q];
B = [rand(1,K)+q; rand(1,K)+q];
C = [rand(1,K)+q; rand(1,K)-q];
D = [rand(1,K)-q; rand(1,K)-q];
% plot clusters
figure(1)
plot(A(1,:),A(2,:),'k+')
hold on
grid on
plot(B(1,:),B(2,:),'bd')
plot(C(1,:),C(2,:),'k+')
plot(D(1,:),D(2,:),'bd')
% text labels for clusters
text(.5-q,.5+2*q,'Class A')
text(.5+q,.5+2*q,'Class B')
text(.5+q,.5-2*q,'Class A')
text(.5-q,.5-2*q,'Class B')
%%
O = ones(1,100);
Z = zeros(1,100);

P = [A B C D];
T = [O Z O Z; Z O Z O];
clear O Z;
%%
% create a neural network
net = patternnet([4 2]); %two hidden layers with 4 neurons and 2 neuron

%% data division
net.divideParam.trainRatio = .70; % training set [%]
net.divideParam.valRatio = .15; % validation set [%]
net.divideParam.testRatio = 0.15; % test set [%]

net.layers{1}.transferFcn = 'logsig'; %set the transfer function of hidden layers
net.layers{2}.transferFcn = 'logsig';

net.trainFcn = 'trainlm'; %set trainig function
net.performFcn = 'mse'; %set cost function

%% train a neural network
[net,tr,Y,E] = train(net,P,T);
% show network
view(net)