# ALGORTIMO GENÉTICO - COMO CORRER

## PASTA PID BASE 

- Simulação do modelo linear inicial, otimização dos parâmetros do controlador PID

**Atenção** ao modelo no simulink, para a maioria funcionou o algoritmo ode14x com step de 1e-3

Otimização PID

Aqui os resultados dos paramteros do PID veem na forma de k = [k1 k2 k3)
k(1) = kp
k(2) = ki
k(3) ) kd

1) Correr o PID_base_parametros (MATLAB)
2) Correr o PID_base_simulink (SIMULINK)
3) Correr o optimize_PID (MATLAB) - deve dar erro mas não faz mal, prosseguir
4) Correr o genetic_algorihtm (MATLAB)

'+'  opção de trocar os sinais de entrada e correr no simulink

## PASTA CONTROLADOR BH 

Otimização BH (extremamente demorado, 3h para 10 de população)
1) Correr o BH_ModeloHidraulico (MATLAB)
2) Correr o ControladorBH_ModeloHirdaulico (SIMULINK)
3) Correr o optimize_PID_BH (MATLAB)
4) Correr o genetic_algorithm_BH (MATLAB)

**Nota:**
Na otimização do BH, na parte do simulink, as variáveis (M0,y, r0,a,...) todas foram colocadas 
na forma de 'k' para o genetic algorithm da seguinte forma:

k1 = k(1)
k2 = k(2)
y = k(3)
M0 = k(4)
a = k(5)
r0 = k(6)

## PASTA PARAMETROS PRENSA OTIMIZADOS 

Otimização PID
1) Correr o PID_parametros_novos (MATLAB)
2) Correr o PID_base_simulink (SIMULINK) - opção de step de 0 a 1 e 0 a 100
3) Correr o optimize_PID (MATLAB) - deve dar erro mas não faz mal, prosseguir
4) Correr o genetic_algorihtm (MATLAB) -  aqui existem duas opções de codigo, uma para o step de 0 a 1 
e uma para o step de 0 a 100


