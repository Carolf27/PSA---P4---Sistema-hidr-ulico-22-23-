%% genetic algorithm pid
k = [20 10 0.1]
nvars = 3;
lb = [0 0 0];
ub = [20 1 1]; %upper bound

%GA options

ga_opt = optimoptions('ga','Display', 'off','Generations',20,'PopulationSize',20,'PlotFcn', @gaplotbestf);
obj_fn = @(k)optimize_PID(k);

% GA Command

[k,best] = ga(obj_fn,nvars,[],[],[],[],lb,ub,[],ga_opt)

