# MÉTODO DA COLÓNIA DAS FORMIGAS - COMO CORRER

Antes de correr algum dos códigos em ficheiro matlab correr a primeira secção de 'clear all, close all, clc' para limpar todas as variáveis para que não haja erros

## PASTA 'Otimização PID'
1. Abrir ficheiro matlab **'PID_Otim_formigas.'**
2. Correr a secção **'Parametros do modelo'** para definir os parâmetros do modelo, criar o espaço de estados do mesmo e obter as funções de trasnferência da posição (G_xs) e da velocidade (G_vs)
3. Na secção **'Parâmetros do controlador PID'** descomentar um dos conjuntos de valor de uma otimização 

![image](https://github.com/Carolf27/PSA---P4---Sistema-hidr-ulico-22-23-/assets/129117353/a8efb6f7-3657-46d4-8acd-87b2dcde8d2b)

(correr isto equivale a considerar os valores de z1, z2 e k resultantes da otimização 1)

4. Correr a secção **'Função de transferência - Controlador PID'** para aplicar o controlador ao sistema e obter a resposta
5. Correr a secção **'Ganhos Proporcional, Integral e Derivativo a ser implementado no controlador no simulink'** para simular o modelo em smulink para diferentes trajetórias de entrada
(este passo abre o ficheiro **'PID_base_simulink_formigas.slx'** e simula-o com os dados definidos no ficheiro matlab)

Para correr a otimização:
1. Correr a secção **'Otimização'** para inicializar os parâmetros da otimização e correr a simulação
(este passo utiliza a função definida em 'cost_fun.m')
2. O resultado da otimização são 3 valores: [z1 z2 k]

![image](https://github.com/Carolf27/PSA---P4---Sistema-hidr-ulico-22-23-/assets/129117353/fa3062d3-9af5-43e2-95f0-60f71ef20a5a)

(exemplo ilustrativo do resultado da otimização)

4. Para simular o modelo com os valores resultantes da otimização é necessário substituir estes valores na secção **'Parâmetros do controlador PID'**

## PASTA 'Otimização Controlador BH'
1. Abrir ficheiro matlab **'BH_Otim_formigas.m'**
2. Correr a secção **'Parametros do modelo'** para definir os parâmetros do modelo, criar o espaço de estados do mesmo e obter as funções de trasnferência da posição (G_xs) e da velocidade (G_vs)
3. Na secção **'Parâmetros do controlador'** descomentar um dos conjuntos de valor de uma otimização 


![image](https://github.com/Carolf27/PSA---P4---Sistema-hidr-ulico-22-23-/assets/129117353/09126c91-82b8-478a-8528-3ed2c1bc34bc)

(correr isto equivale a considerar os valores dos parâmetros do controlador originais)

4. Correr a secção **'Implementação do controlador de BH no sistema'** para aplicar o controlador ao sistema e obter a resposta
5. Correr a secção **'Simulação do modelo com o controlador PID para várias trajetórias de entrada'** para simular o modelo em smulink para diferentes trajetórias de entrada
(este passo abre o ficheiro 'BH_base_simulink_formigas.slx' e simula-o com os dados definidos no ficheiro matlab)

Para correr a otimização:
1. Correr a secção **'Otimização'** para inicializar os parâmetros da otimização e correr a simulação
(este passo utiliza a função definida em 'cost_fun_bh.m')
2. O resultado da otimização são 6 valores: [M0 a r0 gama k1 k2]

![image](https://github.com/Carolf27/PSA---P4---Sistema-hidr-ulico-22-23-/assets/129117353/2fb77241-95a0-4ce7-adfb-38e4e212612c)

(exemplo ilustrativo do resultado da otimização)

4. Para simular o modelo com os valores resultantes da otimização é necessário substituir estes valores na secção **'Parâmetros do controlador'**


## PASTA 'Parametrização do modelo da prensa'
1. Abrir ficheiro matlab **'Otim_modelo.m'**
2. Correr a secção **'Carregamento dos dados do ensaio experimental na prensa'** para carregar os dados do ensaio da prensa 
(nota: nesta secção é feito o **'load'** do ficheiro **'teste2v.mat'** por isso é necessário tê-lo na mesma pasta para correr)
3. Correr a secção **'Parametros do modelo'** para definir os parâmetros do modelo
4. Descomentar um conjunto de coeficientes da otimização da secção **'Coeficientes'** e correr para definir os coeficientes

![image](https://github.com/Carolf27/PSA---P4---Sistema-hidr-ulico-22-23-/assets/129117353/2c3620c6-cac9-4fd2-a1fd-99f374c74199)

(correr isto equivale a considerar os valores dos coeficientes do modelo resultantes da otimização pela posição)

5. Correr a secção **'Estado de espaços'** para criar o espaço de estados do modelo e obter as funções de trasnferência da posição (G_xs) e da velocidade (G_vs)
6. Correr a secção **'Redução dos dados para o intervalo entre os 8 e 10 segundos ( 2 segundos )'** para reduzir os dados dos ensaios para um intervalo de 2 segundos
7. Correr a secção **'Comparação da resposta do modelo ao ensaio da prensa'** para obter a resposta do modelo com os parâmetros otimizados e comparar os resultados da posição e da velocidade com as do ensaio

Para correr a otimização dos parâmetros do modelo:
1. Correr a secção **'Otimização'** para inicializar os parâmetros da otimização e correr a simulação
(este passo utiliza a função definida em 'cost_fun_mod.m')
2. O resultado da simulação são 4 valores: [kq1 kq2 kc1 kc2]

![image](https://github.com/Carolf27/PSA---P4---Sistema-hidr-ulico-22-23-/assets/129117353/5991590d-f25a-44e6-8ebe-159e7aac927e)

(exemplo ilustrativo do resultado da otimização)

4. Para simular o modelo com os valores resultantes da otimização é necessário substituir estes valores na secção **'Coeficientes'**

## PASTA 'Otimização do controlador BH com parâmetros otimizados da prensa'
1. Abrir ficheiro matlab **'BH_Otim_formigas_modelo_otimizado.m'**
2. Correr a secção **'Parametros modelo'** para definir os parâmetros do modelo otimizado, criar o espaço de estados do mesmo e obter as funções de trasnferência da posição (G_xs) e da velocidade (G_vs)
3. Descomentar um conjunto de valore das otimizações na secção **'Parâmetros do controlador'** e correr para definir os parâmetros do controlador

![image](https://github.com/Carolf27/PSA---P4---Sistema-hidr-ulico-22-23-/assets/129117353/36ff6a6a-925c-4162-bd24-dee55455ae1d)

(correr isto equivale a considerar os valores dos parâmetros do controlador resultantes da otimização 1)

4. Correr a secção **'Trajetórias de entradas'** para definir as diferentes trajetórias de entrada
5. Correr a secção **'Implementação do controlador de BH no sistema'** para aplicar o controlador ao sistema com os parâmetros otimizados
6. Correr a secção **'Reposta do modelo otimizado com o controlador Bh otimizado a várias trajetórias de entrada'** para obter a resposta do sistema com o controlador para as diferentes respostas e obter os gráficos representativos das mesmas 

Para correr a otimização dos parâmetros do modelo:
1. Correr a secção **'Otimização'** para inicializar os parâmetros da otimização e correr a simulação
(este passo utiliza a função definida em 'cost_fun_bh_ot.m')
2. O resultado da simulação são 6 valores: [M0 a r0 gama k1 k2]

![image](https://github.com/Carolf27/PSA---P4---Sistema-hidr-ulico-22-23-/assets/129117353/2fb77241-95a0-4ce7-adfb-38e4e212612c)

(exemplo ilustrativo do resultado da otimização)

4. Para simular o modelo com os valores resultantes da otimização é necessário substituir estes valores na secção **'Parâmetros do controlador'**






