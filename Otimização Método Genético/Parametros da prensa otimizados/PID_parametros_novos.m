%% parametros ja otimizados kq1 kq2 kc1 kc2

Ps=100*10^5;
xe0=0.082;

% valores originais----
% kq1=4.76*10^-4;
% kq2=4.76*10^-4;
% 
% kc1=3.644*10^-12;
% kc2=4.376*10^-12;

%valores otimizados----
kq1=0.0027459;
kq2=0.001091;

kc1=7e-13;
kc2=1e-12;

Vl1=3*10^-5;
Vl2=5*10^-5;

dh=45e-3;
de=80e-3;
A1=(pi*de^2)/4;
A2=A1-(pi*dh^2)/4;

L=0.2;

rho=7700;
c=0.2;
Mguias_movida=89;
ee=0.020;
Me=(A1*ee)*rho;
eh=L-ee;
Ah=(pi*dh^2)/4;
Mh=(Ah*eh)*rho;
M=Mguias_movida+Me+Mh;

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