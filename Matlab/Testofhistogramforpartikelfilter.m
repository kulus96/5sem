%% test of histogram before implementing in cpp
clear 
clc


alpha = 1; % Dont know what this is yet.
beta= 0.01; % Dont know what this is yet.
d = 200; % the true length of the beam.
varians= 1;
standartdevi=sqrt(varians);
lambda = 0.2; % a way of taking in to account if an object is in the enviroment but not in the map. 
gamma = 0.02; % conts to model random measurement
%sigma = 0.1; % how likely the maximum reading is.
syms y
bins =zeros(1,256);
w = 0:255;
for i = 0:255
   bins(i+1)=int(alpha*normpdf(y,d,varians)+gamma,i-0.5,i+0.5);
    
end

stem(w,bins)
%%

clc
clear
close all
standartdevi = 1;
varians =1;
M =200;
 fplot(@(x) (1/(standartdevi*sqrt(2*pi))*exp(-((x-M)^2)/(2*varians)))+0.1,[0,255],'r')
 ylim([0,1])