clear
T_delay= 0.08;
Ts = 0.01;
k = ceil(T_delay/Ts);
delay_dtf = tf([-0.5*T_delay 1],[0.5*T_delay 1],Ts);
%%

%Define the uncertain parameters a,b,c.
a = ureal('a',0.35,'Range',[0.2 0.5]);
b = ureal('b',0.1,'Range',[0.09 0.15]);
c = ureal('c',1,'Range',[0.8 1.2]);
%Define the uncertainty LTI model whose parameters ranges.
G_uncertain_temp = tf(c,[a 1])*tf([-0.5*b 1],[0.5*b 1]);

% G_uncertain = tf(c,[a 1]);
% G_uncertain = tf(1,[1 0 0] )*tf([-0.5*b 1],[0.5*b 1]);
%observe the step response,bode and Nyquist diagram.
% step(G_uncertain)
% bode(G_uncertain)
% nyquist(G_uncertain)
%generate a multimodel uncertainty set containing 100 models.
G_set=usample(G_uncertain_temp,100);
%bulid up the nominal model
G_nominal= tf(1,[0.35 1])*tf([-0.5*0.1 1],[0.5*0.1 1]);
% G_nominal= tf(1,[1 0 0] );
%Get the 1st order weighting function
[usys,infor] = ucover(G_set,G_nominal,1);
%In MATLAB, W1 is the function of W2 of the lecture slides
W2=infor.W1;
%Transfer function of W2;
W2_TF=tf(W2)
G_above = G_nominal*(1+W2);%Multiplicative uncertainty
G_below = G_nominal*(1-W2)
%The blue lines are the model set and the red one is the upper bound using W2.
figure
% bodemag(G_set,'b',G_above,'r+',G_below,'g+')
bodemag((G_nominal - G_set)/G_nominal,'b',infor.W1,'r+')
H = ultidyn("H",1);
G_uncertain = G_nominal*(1 + H*infor.W1);
figure;
bodemag(H*infor.W1,'b',infor.W1,'r+')

G_nominal_discrete = c2d(G_nominal,Ts,'zoh');
% G_nominal_discrete = tf(1,[0.35 1],Ts)*delay_dtf;
% G_uncertain_discrete = c2d(G_uncertain,Ts,'zoh');
G_set_discrete = c2d(G_set,Ts,'zoh');

% figure
% bodemag(W2_TF);hold on;
% bodemag(1 - W2_TF);
% legend('W2_TF','1 - W2_TF')
%% mixsyn

W1_discrete = makeweight(33,0.2*2*pi,0.5,Ts);

W3_discrete = makeweight(0.2,2.0*2*pi,20,Ts);

figure
bodemag(W2_TF,'+');
hold on;
bodemag(W3_discrete);
bodemag(W1_discrete);
hold off;
legend('W2_{TF}','W3','W1');
% W1_discrete = c2d(W1,Ts,'zoh');
W2_discrete = c2d(W2,Ts,'zoh');
% W3_discrete = c2d(W3,Ts,'zoh');
G_uncertain_discrete = G_nominal_discrete*(1 + H*W2_discrete);
% [Kss,CL,gamma] = mixsyn(G_nominal,W1,[],W3);
% K = tf(Kss) ;
% gamma
%%
C0 = tunablePID('C','pid',Ts);
a = realp('a',1);    % filter coefficient
b = realp('b',1);    % filter coefficient
F0 = tf([b 1],[a 1],Ts);    % filter parameterized by a

% Label the block I/Os
Wn = W2_discrete;    Wn.u = 'nw';   Wn.y = 'n';
Wr = W3_discrete;    Wr.u = 'rw';   Wr.y = 'r3';
Wd = W1_discrete;    Wd.u = 'dw';   Wd.y = 'd';
Sp = (1 - delay_dtf)*tf(1,[0.35 1],Ts); Sp.u = 'us'; Sp.y = 'ysp';

C0.u = 'e';   C0.y = 'u';
% F0.u = 'r';  F0.y = 'ky';
F0.u = 'rw';  F0.y = 'r';
G = G_nominal_discrete; G.u = 'us'; G.y = 'yd';
% G = G_uncertain; G.u = 'us'; G.y = 'yd';

% Specify summing junctions
Sum1 = sumblk('e = r - yf');
% Sum2 = sumblk('yf = y + n + ysp');
Sum2 = sumblk('yf = y + n');
Sum3 = sumblk('y = yd + d');
% Sum4 = sumblk('us = ky + u');
Sum4 = sumblk('us = u');

% Connect the blocks together
% T0 = connect(G,Wr,Wd,Wn,Sp,C0,F0,Sum1,Sum2,Sum3,Sum4,{'rw','nw','dw'},{'y'});
T0 = connect(G,Wd,Wn,C0,F0,Sum1,Sum2,Sum3,Sum4,{'rw','nw','dw'},{'y'});

T0.Blocks
%%
rng('default')
opt = hinfstructOptions('Display','final','RandomStart',10);
T = hinfstruct(T0,opt);

% [T,CLperf,info] = musyn(T0);

%%
showTunable(T)

C = getBlockValue(T,'C');
% F = getValue(F0,T.Blocks);  % propagate tuned parameters from T to F


% tf(F)
%%
% Sens_tilt = feedback(tf(1),G_uncertain*C);
Sens = feedback(tf(1),G_nominal_discrete*C);
T = feedback(G_nominal_discrete*C,tf(1));
% figure;
% bodemag(W1_discrete,'r--',Sens_tilt,'b',{1e-1,1e2}), grid, 
% title('Sensitivity function & W1'), legend('W1','S')

figure;
bodemag(W2_discrete,'r--',T,'b',{1e-1,1e2}), grid, 
title('Sensitivity function'), legend('W2','T')

norm(W2_discrete*T,inf)% robust stability
norm(W1_discrete*Sens,inf)% robust stability
two_tf_norm_s_hinfnorm(W2_discrete*T,W1_discrete*Sens) %robust stability + robust performance

%% 
F0 = tf([1/1.8 1],[1/2.3 1]);    % filter parameterized by a
F = c2d(F0,Ts,'zoh');
figure;
bodemag(W3_discrete,'g--',(F * C)*G_set_discrete / (G_set_discrete*C + 1),'r',{1e-1,1e2}), grid, 
title('Sensitivity function'), legend('W3','T')
figure;
subplot(2,1,1);
step((F * C)*G_set_discrete/(1 + G_set_discrete*C)), grid, title('Closed-loop response');
hold on;
step(G_nominal);
hold off;

subplot(2,1,2);
step((C)*G_set_discrete/(1 + G_set_discrete*C)), grid, title('Closed-loop response');
hold on;
step(G_nominal);
hold off;

figure;
tf_ = (F* C)*G_nominal_discrete/(1 + G_nominal_discrete*C);
pzmap(tf_);
title('with F');
plotZeroPolesOnBode((tf_));
title('with F');

figure;
tf_ = (C);
pzmap(tf_)
title('without F');


