%% controlador BH aplicado ao modelo hidráulico
clear all
clc

%% declaração das variáveis modelo hidraulico-parametros iniciais

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

%% declaração das variávies BH-parametros iniciais

M0 = 480;
y = 170;
r0 = 0.62;
a = 3.2;
k1 = 11;
k2 = 0.9;
p = 2;
q = 2;