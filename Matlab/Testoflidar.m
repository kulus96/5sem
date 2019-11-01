clear
clc
close all
x = csvread("bigworldangle200.csv");
y = 0:size(x,2)-2;

x(end)=[];
plot(y,x);

M=mean(x)
V= var(x)

