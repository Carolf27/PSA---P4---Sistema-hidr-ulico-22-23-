clc
clear all

%% declaração das variáveis

Ps=70*10^5;
xe0=0.082;

kq1=4.76*10^-4;
kq2=4.76*10^-4;

kc1=3.644*10^-12;
kc2=4.376*10^-12;

Vl1=3*10^-5;
Vl2=5*10^-5;

A1=1.2566*10^-3;
A2=8.7650*10^-4;

M=84;
Be=9000*10^5;
f=1500;
L=0.2;

P10=30*10^5;
P20=52*10^5;

Ve0=0;
x0=[P10 P20 Ve0 xe0];

V1=Vl1+A1*xe0;
V2=Vl2+A2*(L-xe0);


A=[(-Be/V1)*kc1 0 (-Be/V1)*A1 0;
    0 (-Be/V2)*kc2 (Be/V2)*A2 0; 
    A1/M -A2/M -f/M 0;
    0 0 1 0];

B=[(Be/V1)*kq1 0;
    (-Be/V2)*kq2 0;
    0 1; 
    0 0];

C=[0 0 1 0;
    0 0 0 1];

D=[0 0; 
    0 0];

ss_xe=ss(A,B,C,D);
F=tf(ss_xe)

Xe=F(2,1)
rlocus(Xe)
% step(feedback(ft1,1))

%% teste preliminar

z1= 0.0577;
z2=1.602;
k=59.276;
s=tf('s');

pid=(k*(s+z1)*(s+z2))/s

FT =pid*Xe; %MALHA ABERTA
 
F1=feedback(FT,1); %MALHA FECHADA
rlocus(FT)
step(F1)

%% procura do minimo local

t=0:1e-3:10;
u=ones(1,length(t));

global Xe u t;

parameter(1)=z1;
parameter(2)=z2;
parameter(3)=k;

options = optimset('Display','iter', 'MaxIter', 2^16, 'TolFun',1e-6,'Tolx',1e-6,'MaxFunEvals',2^16);
[parameter_out,MSE]=fminsearch(@fminsearchF1,parameter,options);

%% simulação do minimo local

z1 = parameter_out(1);
z2 = parameter_out(2);
k = parameter_out(3);

pid=(k*(s+z1)*(s+z2))/s;

FT =feedback(pid*Xe,1);

step(FT)


%% pid

kp=(z1+z2)*k;
ki=z1*z2*k;

%% procura do minimo global

problem = createOptimProblem('fmincon','objective',@(parameter)fminsearchF1(parameter),'x0',[0,0,0],'lb',[0,0,0],'ub',[100,100,1000],'options', optimoptions(@fmincon,'Algorithm','sqp'));
  
gs = GlobalSearch('Display','iter');
rng(10,'twister')
[parameter_out MSE] = run(gs,problem);

%% simulação do minimo global

z1_G = parameter_out(1);
z2_G = parameter_out(2);
k_G = parameter_out(3);

pid_G=(k_G*(s+z1_G)*(s+z2_G))/s;

FT_G =feedback(pid_G*Xe,1);

step(FT_G)

%% pid

kp_G=(z1_G+z2_G)*k_G;
ki_G=z1_G*z2_G*k_G;


