clc
clear

%% File read
%从第二行第一列开始读。
M=csvread('速度给定速度反馈电流给定抓取.csv',1,0);
row_start = 1;
row_end =169400-1;

%%

%提取速度给定
Reference_speed=(M(row_start:row_end,2));
timex=(M(row_start:row_end,1))*0.000001;
Reference_speed_series=timeseries(Reference_speed,timex);   %rad/s

%%

%提取速度反馈
Speed_feedback=(M(row_start:row_end,3));
timey=(M(row_start:row_end,1))*0.000001;
Speed_feedback_series=timeseries(Speed_feedback,timey);   %rad/s


%%
%提取速度控制器输出（q轴电流给定）
Speed_PI_output=(M(row_start:row_end,4));
timez=(M(row_start:row_end,1))*0.000001;
Speed_PI_output_series=timeseries(Speed_PI_output,timez);   %A













