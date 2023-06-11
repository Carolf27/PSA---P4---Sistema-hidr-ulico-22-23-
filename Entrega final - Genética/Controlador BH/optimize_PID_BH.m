function cost = optimize_PID_BH (k)
assignin('base','k',k);
sim('controladorBH_ModeloHidraulico.slx');
cost=ITAE(length(ITAE));
end