clear all;
clc;
close all;
%% parametros prensa

% Pressão de alimentação
Ps=100e5;

% pressões nas camaras
P10=25e5;
P20=Ps-P10;

% Diâmetros do punção
dh=45e-3;
de=80e-3;

% Áreas 
A1=(pi*de^2)/4; % Área do êmbolo
A2=A1-(pi*dh^2)/4; % Área da haste

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

% Massa em movimento
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

% Constante gravítica
g=9.8;

% Coeficientes caudal-pressão (otimizados)
kc1=7e-13;
kc2=1e-12;

% Ganhos de caudal na válvula (otimizados)
kq1=0.0027459;
kq2=0.001091;


% Estado de espaços 

A=[(-be*kc1/V1) 0 (-be*A1/V1) 0 ; 
    0 (-be*kc2/V2) (be*A2/V2) 0 ;
    (A1/M) (-A2/M) (-f/M) 0;
    0 0 1 0]
B=[(be*kq1/V1) 0 ; (-be*kq2/V2) 0 ; 0 1 ;0 0;]
C=[0 0 1 0;
    0 0 0 1]
D=[0 0;0 0]




% Criar espaço de estados

SS = ss(A,B,C,D);
G=tf(SS);
G_xs = G(2,1); % fun transferencia - posição
G_vs = G(1,1); % fun transferencia - velocidade


%% Polos

figure 
rlocus(G_xs)
rlocus(G_vs)

%% Parâmetros do controlador
%% Originais (Tese)
% Variáveis

M0 = 480;
a = 3.2;
r0 =0.62;
gama = 170;
k1 =11;
k2 =0.9;

%% Otimização 1
M0 =451.952;
a = 1.088088;
r0 = 0.6444444;
gama = 110.5105;
k1 = 7.752753;
k2 = 0.228028;

%% Otimização 2
M0_2 =473.7738;
a_2 = 2.333333;
r0_2= 0.3987988;
gama_2 = 170.0701;
k1_2 = 6.651652;
k2_2 = 0.5465465;

%% Trajetórias de entradas
t=0:0.001:3;
u1=ones(1,length(t))*100;
u1(1:find(t==1))=0; % step 0 a 100

u2=100*sin(2*pi()*t); % onda sinusoidal 100 amp

u3=100*t; %rampa

u4=zeros(1,length(t));
u4(1:find(t==0.3))=100; % step 0 a 100
u4(find(t==1):find(t==1.31))=100;
u4(find(t==2):find(t==2.3))=100;

%% Implementação do controlador de BH no sistema
s = tf('s');

% Transformadas de Laplace das equações características do controlador BH -
% otimização 1
r=k1+k2*s;
i=1/s;
M1 = M0 + 1/(s-(gama*i^2));
c=M1*a*r/((a*r)^2+(r0)^2);

% Transformadas de Laplace das equações características do controlador BH -
% otimização 2
r_2=k1_2+k2_2*s;
i_2=1/s;
M1_2 = M0_2 + 1/(s-(gama_2*i_2^2));
c_2=M1_2*a_2*r_2/((a_2*r_2)^2+(r0_2)^2);

TF_MA = c*G_xs; % Controlador BH aplicado ao sistema em malha aberta
TF_MA_2=c_2*G_xs;

TF_MF=feedback(TF_MA,1); % Controlador BH aplicado ao sistema em malha fechada
TF_MF_2=feedback(TF_MA_2,1);

%% Reposta do modelo otimizado com o controlador Bh otimizado a várias trajetórias de entrada

% Step
subplot(4,1,1)
y1=lsim(TF_MF,u1,t);
y1_2=lsim(TF_MF_2,u1,t);
plot(t,u1,'m--',t,y1,'r',t,y1_2,'b','LineWidth',2)
title('Step - 0 a 100')

% Sinusoidal
subplot(4,1,2)
y2=lsim(TF_MF,u2,t);
y2_2=lsim(TF_MF_2,u2,t);
plot(t,u2,'m--',t,y2,'r',t,y2_2,'b','LineWidth',2)
title('Sinusoidal')

% Rampa
subplot(4,1,3)
y3=lsim(TF_MF,u3,t);
y3_2=lsim(TF_MF_2,u3,t);
plot(t,u3,'m--',t,y3,'r',t,y3_2,'b','LineWidth',2)
title('Rampa')

% Pulso
subplot(4,1,4)
y4=lsim(TF_MF,u4,t);
y4_2=lsim(TF_MF_2,u4,t);
plot(t,u4,'m--',t,y4,'r',t,y4_2,'b','LineWidth',2)
title('Pulso')


%% Otimização

% Parametros
n_iter=100; %n de iterações
NA=50; %n de ants
alpha =0.8; 
beta =0.2;
roh=0.7; %evaporation rate
n_param=6; % n parametros 
LB=[400 2 0.2 100 5 0.5]; %lower bound
UB=[500 5 0.8 200 15 1]; %upper bound
n_node=1000; %n de nós para cada parametro

% Inicialização de algumas variaveis
cost_best_prev=inf;
ant = zeros(NA,n_param);

cost = zeros(NA,1);
tour_selected_param = zeros(1,n_param);
param_mat = zeros(n_iter,n_param);
Nodes = zeros(n_node,n_param);
prob = zeros(n_node, n_param);

% Geração de nós
T=ones(n_node,n_param).*eps; %matriz das feromonas
dT=zeros(n_node,n_param); %mudança de feromonas
for i = 1:n_param
    Nodes(:,i)=linspace(LB(i),UB(i),n_node);
end

%% Iteration loop
global G_xs t u1 %% Consideramos o G_xs como variavel global pois representa o modelo e é inalterável

% Para cada iteração
for iter=1:n_iter
  
    for tour_i=1:n_param
        prob(:,tour_i)= (T(:,tour_i).^alpha) .* ((1./Nodes(:,tour_i)).^beta);
        prob(:,tour_i)=prob(:,tour_i)./sum(prob(:,tour_i));
    end
    % Para cada formiga
    for A=1:NA
        for tour_i=1:n_param
            node_sel=rand;
            node_ind=1;
            prob_sum=0;
            % Escolher um nó
            for j=1:n_node
                prob_sum=prob_sum+prob(j,tour_i);
                if prob_sum>=node_sel
                    node_ind=j;
                    break
                end
            end
            ant(A,tour_i)=node_ind;
            tour_selected_param(tour_i) = Nodes(node_ind, tour_i);
        end
        % Calcular o custo
        cost(A)=cost_func_bh_ot(tour_selected_param);
        clc
        disp(['Ant number: ' num2str(A)])
        disp(['Ant Cost: ' num2str(cost(A))])
        disp(['Ant Paramters: ' num2str(tour_selected_param)])
        if iter~=1
        disp(['iteration: ' num2str(iter)])
        disp('_________________')
        disp(['Best cost: ' num2str(cost_best)])
        for i=1:n_param
            tour_selected_param(i) = Nodes(ant(cost_best_ind,i), i);
        end
        disp(['Best paramters: ' num2str(tour_selected_param)])
        end
        
    end
    [cost_best,cost_best_ind]=min(cost);
    
    % Elitsem
    if (cost_best>cost_best_prev) && (iter~=1)
        [cost_worst,cost_worst_ind]=max(cost);
        ant(cost_worst_ind,:)=best_prev_ant;
        cost_best=cost_best_prev;
        cost_best_ind=cost_worst_ind;
    else
        cost_best_prev=cost_best;
        best_prev_ant=ant(cost_best_ind,:);
    end
    
    dT=zeros(n_node,n_param); % Change of Phormone
    for tour_i=1:n_param
        for A=1:NA
            dT(ant(A,tour_i),tour_i)=dT(ant(A,tour_i),tour_i)+cost_best/cost(A);
        end
    end
    
    T= roh.*T + dT;
  
end
