function cost_value=cost_fun_mod(parametro_mod)
global G_vs G_xs ts u v1 t1 x1 ;

% parametros prensa

% Pressão de alimentação
Ps=100e5;

% pressões nas camaras
P10=25e5;
P20=Ps-P10;

% Diâmetros do punção
dh=45e-3;
de=80e-3;
A1=(pi*de^2)/4;
A2=A1-(pi*dh^2)/4;

% Commprimento 
L=0.20;

% Posição inicial
xe0=0.082;

% Volumes das tubagens de alimentação
VL1=3e-5;
VL2=5e-5;
V1=VL1+(A1*xe0);
V2=VL2+A2*(L-xe0);

% caracteristicas do óleo
f=1500;
be=9000e5;

%massa (segundo a tese: Me + Mh + Mguias + Mmovida)???
rho=7700;

Mguias_movida=89;
ee=0.020;
Me=(A1*ee)*rho;
eh=L-ee;
Ah=(pi*dh^2)/4;
Mh=(Ah*eh)*rho;
M=Mguias_movida+Me+Mh;

% velocidade do êmbolo
ve0=0;

kq1 = parametro_mod(1);
kq2 = parametro_mod(2);
kc1 = parametro_mod(3);
kc2 = parametro_mod(4);
% 
A=[(-be*kc1/V1) 0 (-be*A1/V1) 0 ; 
    0 (-be*kc2/V2) (be*A2/V2) 0 ;
    (A1/M) (-A2/M) (-f/M) 0;
    0 0 1 0];
B=[(be*kq1/V1) 0 ; (-be*kq2/V2) 0 ; 0 1 ;0 0;];
C=[0 0 1 0;
    0 0 0 1];
D=[0 0;0 0];



s=tf('s');

SS = ss(A,B,C,D);
G=tf(SS);
G_vs = G(1,1);
G_xs = G(2,1);

% Descomentar para usar a otimização pela posição
% xs=lsim(G_xs,u,ts');
% xs=xs(1:20:end);
% 
% cost_value = sqrt(sum((x1-xs').^2));

% Descomentar para usar a otimização pela velocidade
vs=lsim(G_vs,u,ts)';
vs=vs(1:20:end);
cost_value = sqrt(sum((v1-vs).^2));
