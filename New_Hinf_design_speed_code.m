s = tf('s');
Tsample = 0.0005;

%% 3 plants
Current_closed_loop = 6283/(s+6283);
Torque_coefficient = 0.06864;
Torque_to_mechanical_angular_speed_nominal = 1/(4.63*0.0001*s+0.0001);
Object_to_be_controlled_nominal = Current_closed_loop*Torque_coefficient*Torque_to_mechanical_angular_speed_nominal;
%注意1
Object_to_be_controlled_nominal = minreal(Object_to_be_controlled_nominal);


Torque_to_mechanical_angular_speed_upper = 1/(4.77*0.0001*s+0.0001);
Object_to_be_controlled_upper =Current_closed_loop*Torque_coefficient*Torque_to_mechanical_angular_speed_upper;
%注意2
Object_to_be_controlled_upper = minreal(Object_to_be_controlled_upper);


Torque_to_mechanical_angular_speed_lower = 1/(3.73*0.0001*s+0.0001);
Object_to_be_controlled_lower =Current_closed_loop*Torque_coefficient*Torque_to_mechanical_angular_speed_lower;
%注意3
Object_to_be_controlled_lower = minreal(Object_to_be_controlled_lower);


%step(Speed_closed_loop_nominal,'r')
bode(Object_to_be_controlled_nominal,'r',Object_to_be_controlled_lower,'b',Object_to_be_controlled_upper,'y')



%% W2
Object_to_be_controlled_set = stack(1,Object_to_be_controlled_nominal,Object_to_be_controlled_upper,Object_to_be_controlled_lower);
[GMult1,Info1] = ucover(Object_to_be_controlled_set,Object_to_be_controlled_nominal,1);
W2 = Info1.W1;
bodemag(Object_to_be_controlled_upper/Object_to_be_controlled_nominal-1,'b',Object_to_be_controlled_lower/Object_to_be_controlled_nominal-1,'y',W2,'r') 
%注意4
W2_Discrete=c2d(W2,Tsample,'zoh');
W2_Discrete_tustin=c2d(W2,Tsample,'tustin');


%% W1 nominal performance
% m*1.01 to ensure the magnitude of S is less than 6dB(modulus margin bigger than 0.5).
m = 0.5*1.01;
% bandwidth of 80Hz for position loop(here in rad).
wb = 80*3.14*2; 
% to limit the transfer function from reference to tracking error at low frequency.
W1_inv = tf([1 0],[m m*wb]); 
%To check high frequency gain is less than 6dB
% bodemag(W1_inv) 
W1 =  tf([m m*wb],[1 0.000001])

% 注意5
W1_Discrete=c2d(W1,Tsample,'zoh');

% 注意6
Object_to_be_controlled_nominal_Discrete = c2d(Object_to_be_controlled_nominal,0.0005,'zoh');


%% 
% First continuous controller 
[Kss,CL,GAM,INFO] = mixsyn(Object_to_be_controlled_nominal,W1,[],W2);
K = tf(Kss)
GAM


%% sensitivity function check 已经算出K，然后带回原来单位反馈闭环回路（传统的）
S = feedback(1,Object_to_be_controlled_set*K);
T = feedback(Object_to_be_controlled_set*K,1); 
bode(S);
bode(T);
step(T)

%% reduce the order of controller
figure
pzmap(Kss)

%% 
Kred = reduce(Kss);
[Knumred,Kdenred]=ss2tf(Kred.A,Kred.B,Kred.C,Kred.D);
K2 = tf(Knumred,Kdenred) 


%%
Sred = feedback(1,Object_to_be_controlled_set*Kred);
Tred = feedback(Object_to_be_controlled_set*Kred,1);
bode(Sred);
bode(Tred);
step(Tred)

%%
Kred_z = c2d(K2,Tsample,'zoh')

Object_to_be_controlled_nominal_discrete = c2d(Object_to_be_controlled_nominal,Tsample,'zoh');
Object_to_be_controlled_upper_discrete = c2d(Object_to_be_controlled_upper,Tsample,'zoh');
Object_to_be_controlled_lower_discrete = c2d(Object_to_be_controlled_lower,Tsample,'zoh');
Object_to_be_controlled_set_discrete = stack(1,Object_to_be_controlled_nominal_discrete,Object_to_be_controlled_upper_discrete,Object_to_be_controlled_lower_discrete);

Tred_z = feedback(Object_to_be_controlled_set_discrete*Kred_z,1);
step(Tred_z,0.03)



%% First discrete controller
[Kss,CL,GAM,INFO] = mixsyn(Object_to_be_controlled_nominal_Discrete,W1_Discrete,[],W2_Discrete);
K_discrete = tf(Kss)
GAM

%% 不能降阶，使用降阶函数会得到一个连续的控制器而不是离散的控制器。
T_2z = feedback(Object_to_be_controlled_set_discrete*K_discrete,1);
step(T_2z,0.03)








