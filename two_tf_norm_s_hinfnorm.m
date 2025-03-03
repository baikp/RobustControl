function [Hinf_norm] = two_tf_norm_s_hinfnorm(tf1,tf2)
%TWO_TF_NORM_S_HINFNORM �˴���ʾ�йش˺�����ժҪ
w = logspace(-3,3,1000);
% ��ȡƵ����Ӧ��ģ����
[mag1, ~, ~] = bode(tf1,w);  % mag1 �� G1 �ڲ�ͬƵ���µ�����
[mag2, ~, ~] = bode(tf2,w);  % mag2 �� G2 �ڲ�ͬƵ���µ�����

% ��mag1��mag2��3D����ת��Ϊ������
mag1 = squeeze(mag1);  
mag2 = squeeze(mag2);
% ����ģ��֮��
sum_mag = mag1 + mag2;
% ����ģ��֮�͵������
Hinf_norm = max(sum_mag);

end

