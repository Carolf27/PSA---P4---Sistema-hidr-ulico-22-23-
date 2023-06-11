clear all;
clc;
close all;

%% Carregamento dos dados do ensaio experimental na prensa

load('teste2v.mat');
t= teste2v.X.Data; % tempo
x=teste2v.Y(2).Data; % posição
v=teste2v.Y(3).Data; % velocidade

%% Parametros do modelo

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

%% Coeficientes
% % Original (tese)
% % Coeficientes caudal-pressão
% kc1=7e-13;
% kc2=1e-12;
% 
% % Ganhos de caudal na válvula
% kq1=0.0027459;
% kq2=0.001091;
% 
% Otimização Posição
% Coeficientes caudal-pressão
kc1=7e-13;
kc2=1e-12;

% Ganhos de caudal na válvula
kq1=0.0027459;
kq2=0.001091;
% 
% % Otimização Velocidade
% % Coeficientes caudal-pressão
% kc1=7e-13;
% kc2=1e-12;
% 
% % Ganhos de caudal na válvula
% kq1=0.0027459;
% kq2=0.001091;


%% Estado de espaços 

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

%% Redução dos dados para o intervalo entre os 8 e 10 segundos ( 2 segundos )

v1=v(find(t==8):find(t==10)); % dados da velocidade entre os 8 e 10 s
t1=t(find(t==8):find(t==10)); % corte do tempo para o período entre os 8 e 10 s
t1=t1-t1(1); % reajuste do tempo para o período de 0 a 2 segundos
x1=x(find(t==8):find(t==10)); % dados da posição entre os 8 e 10 s

v_med = mean(v1(find(t1==1):find(t1==2))); % cálculo da velocidade média

%% Comparação da resposta do modelo ao ensaio da prensa

ts=0:0.001:2; % Definição do período a simular - 2 segundos
u=ones(1,length(ts))*140; % Definição da trajetória de entrada - step de 0 a 140 após 0.46 segundos - entrada dada no ensaio da prensa
u(1:find(ts==0.46))=0;

xs=lsim(G_xs,u,ts)'; % Resposta do modelo à trajetória de entrada - posição
xs=xs(1:20:end);
subplot(1,2,1)
plot(t1,xs,'b',t1,x1,'r','LineWidth',3) % Representação gráfica da resposta do modelo e do ensaio da prensa - posição
title('Posição','FontSize',20)
legend('Modelo','Real')
grid on


vs=lsim(G_vs,u,ts)'; % Resposta do modelo à trajetória de entrada - velocidade
vs=vs(1:20:end);
subplot(1,2,2)
plot(t1,vs,'b',t1,v1,'r','LineWidth',3) % Representação gráfica da resposta do modelo e do ensaio da prensa - velocidade
grid on
title('Velocidade','FontSize',20)
legend('Modelo','Real')

%% Otimização

% Parâmetros de Otimização

n_iter=500; %number of iteration
NA=100; % Number of Ants
alpha=0.8; % alpha
beta=0.2; % beta
roh=0.7; % Evaporation rate

n_param=4; % Number of paramters
LB=[1e-3 1e-3 1e-13 1e-13]; % lower bound
UB=[1e-1 1e-1 1e-10 1e-10]; % upper bound
n_node=500; % number of nodes for each param

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
global G_vs ts u v1 t1 x1 %% Consideramos o G_xs como variavel global pois representa o modelo e é inalterável
% 
start_time=tic();
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
        end_time=toc(start_time);
        % Calcular o custo
        cost(A)=cost_fun_mod(tour_selected_param);
        clc
        disp(['Ant number: ' num2str(A)])
        disp(['Ant Cost: ' num2str(cost(A))])
        disp(['Ant Paramters: ' num2str(tour_selected_param)])
        if iter~=1
        disp(['iteration: ' num2str(iter)])
        disp(['Time: ' num2str(end_time/60)])

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