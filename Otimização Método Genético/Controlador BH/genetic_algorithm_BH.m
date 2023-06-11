%% genetic algorithm pid
k = [11 0.9 170 480 3.2 0.62]
nvars = 6;
lb = [0 0 0 0 0 0];
ub = [20 2 300 500 5 2]; %upper bound

%GA options

ga_opt = optimoptions('ga','Display', 'off','Generations',10,'PopulationSize',10,'PlotFcn', @gaplotbestf);
obj_fn = @(k)optimize_PID_BH (k);

% GA Command

[k,best] = ga(obj_fn,nvars,[],[],[],[],lb,ub,[],ga_opt)

