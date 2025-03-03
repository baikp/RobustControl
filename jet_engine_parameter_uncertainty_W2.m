clear
%Define the uncertain parameters a,b,c.
a = ureal('a',10,'PlusMinus',[-2 2]);
b = ureal('b',2,'PlusMinus',[-1 1]);
c = ureal('c',5,'PlusMinus',[-1 1]);
%Define the uncertainty LTI model whose parameters ranges.
G_uncertain = tf(a,[1 b c]);
%observe the step response,bode and Nyquist diagram.
%step(G_uncertain)
%bode(G_uncertain)
%nyquist(G_uncertain)
%generate a multimodel uncertainty set containing 100 models.
G_set=usample(G_uncertain,100);
%bulid up the nominal model
G_nominal= tf(10,[1 2 5]);
%Get the 1st order weighting function
[usys,infor] = ucover(G_set,G_nominal,4);
%In MATLAB, W1 is the function of W2 of the lecture slides
W2=infor.W1;
%Transfer function of W2;
W2_TF=tf(W2)
G_above = G_nominal*(1+W2);%Multiplicative uncertainty
G_below = G_nominal*(1-W2)
%The blue lines are the model set and the red one is the upper bound using W2.
figure
bodemag(G_set,'b',G_above,'r',G_below,'g')
figure
bodemag((G_nominal - G_set)/G_nominal,'b',infor.W1,'r+')

% figure
% bodemag(G_below/G_nominal - 1,'b+');hold on;
% bodemag(G_above/G_nominal - 1,'y*');
% bodemag(W2,'r');hold off;
figure
bodemag(W2_TF);hold on;
bodemag(1 - W2_TF);
legend('W2_{TF}','1 - W2_{TF}')
