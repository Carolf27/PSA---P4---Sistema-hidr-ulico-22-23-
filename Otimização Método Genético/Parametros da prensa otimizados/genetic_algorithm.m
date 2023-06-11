%% genetic algorithm pid para step de 0 a 1
k = [20 0.02 0.0001]
nvars = 3;
lb = [0 0 0];
ub = [20 0.1 0.1]; %upper bound

%GA options

ga_opt = optimoptions('ga','Display', 'off','Generations',10,'PopulationSize',10,'PlotFcn', @gaplotbestf);
obj_fn = @(k)optimize_PID(k);

% GA Command

[k,best] = ga(obj_fn,nvars,[],[],[],[],lb,ub,[],ga_opt)


% %% genetic algorithm pid para step de 0 a 100
% k = [0.5 0.02 0.0001]
% nvars = 3;
% lb = [0 0 0];
% ub = [5 0 0.01]; %upper bound
% 
% %GA options
% 
% ga_opt = optimoptions('ga','Display', 'off','Generations',10,'PopulationSize',10,'PlotFcn', @gaplotbestf);
% obj_fn = @(k)optimize_PID(k);
% 
% % GA Command
% 
% [k,best] = ga(obj_fn,nvars,[],[],[],[],lb,ub,[],ga_opt)



