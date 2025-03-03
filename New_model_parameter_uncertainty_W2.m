clear
%Define the uncertain parameters a,b,c.
a = ureal('a',0.35,'Range',[0.2 0.5]);
b = ureal('b',0.1,'Range',[0.05 0.15]);
c = ureal('c',1,'Range',[0.8 1.2]);
%Define the uncertainty LTI model whose parameters ranges.
G_uncertain = tf(c,[a 1])*tf([-0.5*b 1],[0.5*b 1]);

% G_uncertain = tf(c,[a 1]);
% G_uncertain = tf(1,[1 0 0] )*tf([-0.5*b 1],[0.5*b 1]);
%observe the step response,bode and Nyquist diagram.
% step(G_uncertain)
% bode(G_uncertain)
% nyquist(G_uncertain)
%generate a multimodel uncertainty set containing 100 models.
G_set=usample(G_uncertain,100);
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

% figure
% bodemag(W2_TF);hold on;
% bodemag(1 - W2_TF);
% legend('W2_TF','1 - W2_TF')

%% mixsyn

W1 = makeweight(33,0.6*2*pi,0.5);

W3 = makeweight(0.5,1.0*2*pi,20);

figure
bodemag(W2_TF,'+');
hold on;
bodemag(W3);
bodemag(W1);
hold off;
legend('W2_{TF}','W3','W1');

[Kss,CL,gamma] = mixsyn(G_nominal,W1,[],W3);
K = tf(Kss) ;
gamma

%% check robust stability + robust performance.
S_nominal = feedback(1,G_nominal*K);
T_nominal = feedback(G_nominal*K,1);

% Create combined system W1S + W2T
W1S = W1 * S_nominal;
W3T = W3 * T_nominal;

norm(W1S,inf)
norm(W3T,inf)
%% sensitivity function check 已经算出K，然后带回原来单位反馈闭环回路（含标称模型的回路）
S = feedback(1,G_set*K);
T = feedback(G_set*K,1); 
figure;
bode(S);
figure;
bode(T);
figure;
step(T);
%% 
figure
pzmap(Kss);
%%
Kred = reduce(Kss);
[Knumred,Kdenred]=ss2tf(Kred.A,Kred.B,Kred.C,Kred.D);
K2 = tf(Knumred,Kdenred);
%%
Sred = feedback(1,G_set*Kred);
Tred = feedback(G_set*Kred,1);
bode(Sred);
bode(Tred);
step(Tred)