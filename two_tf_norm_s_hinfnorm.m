function [Hinf_norm] = two_tf_norm_s_hinfnorm(tf1,tf2)
%TWO_TF_NORM_S_HINFNORM 此处显示有关此函数的摘要
w = logspace(-3,3,1000);
% 获取频率响应（模长）
[mag1, ~, ~] = bode(tf1,w);  % mag1 是 G1 在不同频率下的增益
[mag2, ~, ~] = bode(tf2,w);  % mag2 是 G2 在不同频率下的增益

% 将mag1和mag2从3D矩阵转换为列向量
mag1 = squeeze(mag1);  
mag2 = squeeze(mag2);
% 计算模长之和
sum_mag = mag1 + mag2;
% 计算模长之和的无穷范数
Hinf_norm = max(sum_mag);

end

