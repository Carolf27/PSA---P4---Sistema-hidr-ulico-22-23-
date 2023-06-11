function cost = optimize_PID(k)
assignin('base','k',k);
sim('PID_base_simulink.slx');
cost=ITAE(length(ITAE));
end