function cost_value=cost_func_pid(parameter)

global G_xs t u ;

z1 = parameter(1);
z2 = parameter(2);
k = parameter(3);

s=tf('s');

s=tf('s');
PID = k*(s+z1)*(s+z2)/s; % Controlador PID
TF_MA=PID*G_xs; % Controlador PID aplicado ao sistema em malha aberta

TF_MF=feedback(TF_MA,1); % Controlador PID aplicado ao sistema em malha fechada

y = lsim(TF_MF,u,t); % Resposta do controlador PID no sistema em malha fechada à trajetória de entrada

erro = u' - y;

cost_value = mean(erro.^2);

