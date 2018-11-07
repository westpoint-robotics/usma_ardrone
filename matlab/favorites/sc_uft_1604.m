
clc
clear all
close all

warning('off','MATLAB:legend:PlotEmpty')

meta.root = '/home/benjamin/ros/src/usma_ardrone/matlab/';
cd([meta.root])
addpath(genpath([meta.root]));
[meta, data] = uft_1604();