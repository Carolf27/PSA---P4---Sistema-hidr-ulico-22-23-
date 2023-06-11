clear all;
clc;
close all;

%% Parametros do modelo

% Pressão de alimentação
Ps=70e5;

% Pressões nas camaras
P10=30e5;
P20=52e5;

% Diâmetros do punção
dh=45e-3;
de=80e-3;

% Área do pistão nas câmaras 1 e 2
A1=1.2566e-3;
A2=8.765e-4;

% Commprimento 
L=0.20;

% Posição inicial
xe0=0.082;

% Volumes das tubagens de alimentação
VL1=3e-5;
VL2=5e-5;
V1=VL1+(A1*xe0);
V2=VL2+A2*(L-xe0);

% Atrito viscoso
f=1500;

% Módulo de compressibilidade efetivo do sistema 
be=9000e5;

% Massa em movimento
rho=7700;
M=84;

% Velocidade do êmbolo
ve0=0;

% Constante gravítica
g=9.8;

% Coeficientes caudal-pressão
kc1=3.644e-12;
kc2=4.376e-12;

% Ganhos de caudal na válvula
kq1=4.76e-4;
kq2=4.76e-4;



% Estado de espaços 

A=[(-be*kc1/V1) 0 (-be*A1/V1) 0 ; 
    0 (-be*kc2/V2) (be*A2/V2) 0 ;
    (A1/M) (-A2/M) (-f/M) 0;
    0 0 1 0]
B=[(be*kq1/V1) 0 ; (-be*kq2/V2) 0 ; 0 1 ;0 0;]
C=[0 0 1 0;
    0 0 0 1]
D=[0 0;0 0]

% Parâmetros do controlador do BH 
M0 = 480;
a = 3.2;
r0 =   0.62;
gama = 170;
k1 =11;
k2 = 0.9;



% Criar espaço de estados

SS = ss(A,B,C,D);
G=tf(SS);
G_xs = G(2,1); % fun transferencia - posição
G_vs = G(1,1); % fun transferencia - velocidade

% Polos

figure 
rlocus(G_xs)
rlocus(G_vs)

% Função de transferência 
s=tf('s');

t=0:0.001:10; % Definição do período a simular - 10 segundos
u=ones(1,length(t)); u(1:(find(t==1)))=0; % Definição da trajetória de entrada - step de 0 a 1 após 1 segundo

y = lsim(G_xs,u,t); % Resposta do controlador PID no sistema em malha fechada à trajetória de entrada
plot(t,y,t,u) % Representação gráfica da resposta

open('Modelo_inicial.slx')
sim('Modelo_inicial.slx')

%% Modelo com o controlador BH
open('Modelo_inicial_BH.slx')
sim('Modelo_inicial_BH.slx')