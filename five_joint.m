clc,clear

%%辨识 23456关节力矩传感器的参数

filename246 = 'joint_torques_246joint.csv';
filename3 = 'joint_torques_3joint.csv';
filename5 = 'joint_torques_5joint.csv';

opts = detectImportOptions(filename246);
opts.DataLines = [1, inf];  % 从第二行开始读取
data = readtable(filename246, opts);
data = table2array(data);

opts = detectImportOptions(filename3);
opts.DataLines = [1, inf];  % 从第二行开始读取
data_3 = readtable(filename3, opts);
data_3 = table2array(data_3);
opts = detectImportOptions(filename5);
opts.DataLines = [1, inf];  % 从第二行开始读取
data_5 = readtable(filename5, opts);
data_5 = table2array(data_5);




%2关节
V_2=data(:,1);
NM_2=data(:,4);
mean_V_2=mean(reshape(V_2, 1000, 361))'/1000;
mean_NM_2=mean(reshape(NM_2, 1000, 361))';
%4关节
V_4=data(:,2);
NM_4=data(:,5);
mean_V_4=mean(reshape(V_4, 1000, 361))'/1000;
mean_NM_4=mean(reshape(NM_4, 1000, 361))';
%4关节
V_6=data(:,3);
NM_6=data(:,6);
mean_V_6=mean(reshape(V_6, 1000, 361))'/1000;
mean_NM_6=mean(reshape(NM_6, 1000, 361))';
% 3关节
V_3=data_3(:,1);
NM_3=data_3(:,2);
mean_V_3=mean(reshape(V_3, 1000, 361))'/1000;
mean_NM_3=mean(reshape(NM_3, 1000, 361))';
% 5关节
V_5=data_5(:,1);
NM_5=data_5(:,2);
mean_V_5=mean(reshape(V_5, 1000, 361))'/1000;
mean_NM_5=mean(reshape(NM_5, 1000, 361))';


