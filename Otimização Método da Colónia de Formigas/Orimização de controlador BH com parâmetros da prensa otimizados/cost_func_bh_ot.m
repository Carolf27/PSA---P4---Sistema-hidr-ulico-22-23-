function cost_value=cost_func_bh_ot(parameter)

global G_xs t u1;

s=tf('s');
M0 = parameter(1);
alpha = parameter(2);
r0 = parameter(3);
gama = parameter(4);
k1 = parameter(5);
k2 = parameter(6);

r=k1+k2*s;
i=1/s;

M = M0 + exp(-5*s);
c=M*alpha*r/((alpha*r)^2+(r0)^2);

TF = feedback(c*G_xs,1);

y = lsim(TF,u1,t);

erro = u1' - y;

cost_value = mean(erro.^2);
