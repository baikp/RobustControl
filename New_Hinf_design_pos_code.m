s = tf('s');

%% 3 plants
Speed_controller = 4.5+180.6/(s+0.00001);
Current_closed_loop = 6283/(s+6283);
Torque_coefficient = 0.06864;
Torque_to_mechanical_angular_speed_nominal = 1/(4.63*0.0001*s+0.00001);
Speed_open_loop_nominal = Speed_controller*Current_closed_loop*Torque_coefficient*Torque_to_mechanical_angular_speed_nominal;
Speed_closed_loop_nominal = Speed_open_loop_nominal/(1 + Speed_open_loop_nominal);
Object_to_be_controlled_nominal = Speed_closed_loop_nominal/(s+0.00001);
% Object_to_be_controlled_nominal = Speed_closed_loop_nominal/(s);

Torque_to_mechanical_angular_speed_upper = 1/(4.77*0.0001*s+0.00001);
Speed_open_loop_upper = Speed_controller*Current_closed_loop*Torque_coefficient*Torque_to_mechanical_angular_speed_upper;
Speed_closed_loop_upper = Speed_open_loop_upper/(1 + Speed_open_loop_upper);
Object_to_be_controlled_upper = Speed_closed_loop_upper/(s+0.00001);

Torque_to_mechanical_angular_speed_lower = 1/(3.53*0.0001*s+0.00001);
Speed_open_loop_lower = Speed_controller*Current_closed_loop*Torque_coefficient*Torque_to_mechanical_angular_speed_lower;
Speed_closed_loop_lower = Speed_open_loop_lower/(1 + Speed_open_loop_lower);
Object_to_be_controlled_lower = Speed_closed_loop_lower/(s+0.00001);

%step(Speed_closed_loop_nominal,'r')
bode(Object_to_be_controlled_nominal,'r',Object_to_be_controlled_lower,'b',Object_to_be_controlled_upper,'y')

%% W2
Object_to_be_controlled_set = stack(1,Object_to_be_controlled_nominal,Object_to_be_controlled_upper,Object_to_be_controlled_lower);
[GMult1,Info1] = ucover(Object_to_be_controlled_set,Object_to_be_controlled_nominal,3);
W2 = Info1.W1;
bodemag(Object_to_be_controlled_upper/Object_to_be_controlled_nominal-1,'b',Object_to_be_controlled_lower/Object_to_be_controlled_nominal-1,'y',W2,'r') 

%% W1 nominal performance
% m*1.01 to ensure the magnitude of S is less than 6dB(modulus margin bigger than 0.5).
m = 0.5*1.01;
% bandwidth of 5Hz for position loop(here in rad).
wb = 5*3.14*2; 
% to limit the transfer function from reference to tracking error at low frequency.
W1_inv = tf([1 0],[m m*wb]); 
%To check high frequency gain is less than 6dB
% bodemag(W1_inv) 
W1 =  tf([m m*wb],[1 0.000001])
% W1z = c2d(W1_stable,Ts,'zoh')


%% First controller
[Kss,CL,GAM,INFO] = mixsyn(Object_to_be_controlled_nominal,W1,[],W2);
%[Knum,Kden]= ss2tf(Kss.A,Kss.B,Kss.C,Kss.D);
K = tf(Kss) 
GAM

%% check robust stability + robust performance.
S_nominal = feedback(1,Object_to_be_controlled_nominal*K);
T_nominal = feedback(Object_to_be_controlled_nominal*K,1);

% Create combined system W1S + W2T
W1S = W1 * S_nominal;
W2T = W2 * T_nominal;

norm(W1S,inf)
norm(W2T,inf)

%% sensitivity function check 已经算出K，然后带回原来单位反馈闭环回路（含标称模型的回路）
S = feedback(1,Object_to_be_controlled_set*K);
T = feedback(Object_to_be_controlled_set*K,1); 
bode(S);
bode(T);
% step(T);

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
Speed_controller_1 = 4.5+180.6/(s);
Current_closed_loop_1 = 6283/(s+6283);
Torque_coefficient_1 = 0.06864;
Torque_to_mechanical_angular_speed_nominal_1 = 1/(4.63*0.0001*s);
Speed_open_loop_nominal_1 = Speed_controller_1*Current_closed_loop_1*Torque_coefficient_1*Torque_to_mechanical_angular_speed_nominal_1;
Speed_closed_loop_nominal_1 = Speed_open_loop_nominal_1/(1 + Speed_open_loop_nominal_1);
Object_to_be_controlled_nominal_1 = Speed_closed_loop_nominal_1/s;
Tred = feedback(Object_to_be_controlled_nominal_1*Kred,1);
bode(Tred);
step(Tred)

%%disretization
K2_z = c2d(K2,0.0005,'zoh');

%%
P = augw(Object_to_be_controlled_nominal_1,W1,[],W2)




