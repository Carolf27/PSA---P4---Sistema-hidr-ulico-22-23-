function mse = fminsearchF1 (parameter)

global Xe u t; 

z1=parameter(1);
z2=parameter(2);
k=parameter(3);

s=tf('s');
pid=(k*(s+z1)*(s+z2))/s;

FT =feedback(pid*Xe,1);

y = lsim(FT,u,t);

erro=u'-y;

mse=mean((erro).^2); % transposta ', . o quadrado de cada elemento

if mse > 1e5
    mse=1e5;
end 