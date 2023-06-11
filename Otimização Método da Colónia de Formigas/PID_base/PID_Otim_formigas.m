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




% Criar espaço de estados

SS = ss(A,B,C,D);
G=tf(SS);
G_xs = G(2,1); % fun transferencia - posição
G_vs = G(1,1); % fun transferencia - velocidade


% Polos

figure 
rlocus(G_xs)
rlocus(G_vs)


%% Parâmetros do controlador PID 

% % Otimização 1
% z1 = 1.0417;
% z2 = 8.0981;
% k = 0.69069;
% 
% % Otimização 2
% z1 = 6.3144;
% z2 = 9.7232;
% k = 0.68402;
% 
% Otimização 3
z1 = 4.5815;
z2 = 7.0405;
k = 0.6684;

%% Função de transferência - Controlador PID 
s=tf('s');
PID = k*(s+z1)*(s+z2)/s; % Controlador PID
TF_MA=PID*G_xs; % Controlador PID aplicado ao sistema em malha aberta
rlocus(PID*G_xs) % Lugar de Raízes

TF_MF=feedback(TF_MA,1); % Controlador PID aplicado ao sistema em malha fechada
rlocus(TF_MA) % Lugar de Raízes

t=0:0.001:10; % Definição do período a simular - 10 segundos
u=ones(1,length(t)); u(1:(find(t==1)))=0; % Definição da trajetória de entrada - step de 0 a 1 após 1 segundo

y = lsim(TF_MF,u,t); % Resposta do controlador PID no sistema em malha fechada à trajetória de entrada
plot(t,y,t,u) % Representação gráfica da resposta


%% Ganhos Proporcional, Integral e Derivativo a ser implementado no controlador no simulink

kp=k*(z1+z2); % Ganho Proporcional
ki=k*z1*z2; % Ganho Integral
kd=k; % Ganho Derivativo

% Simulação do modelo com o controlador PID para várias trajetórias de
% entrada
open('PID_base_simulink_formigas.slx');
sim('PID_base_simulink_formigas');


%% Otimização

% Parametros de otimização
n_iter=200; %n de iterações
NA=50; %n de ants
alpha =0.8; 
beta =0.2;
roh=0.7; %evaporation rate
n_param=3; % n parametros 
LB=[2 2 0.5]; %lower bound
UB=[8 8 0.8]; %upper bound
n_node=10000; %n de nós para cada parametro

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

% Iteration loop

global G_xs t u %% Consideramos o G_xs como variavel global pois representa o modelo e é inalterável

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
        cost(A)=cost_func(tour_selected_param);
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
