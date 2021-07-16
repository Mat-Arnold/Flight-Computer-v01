% plot teensy results

phase = teensyOut(end,:);
change = phase(2:end)> phase(1:end-1);
change = [0 change];
phaseChanges = find(change);



figure(12)
plot(time, -stateMatrix(14,:),'b');
hold on;
% filtered plot and noisy plot
plot(FilteredOutputs.time, -teensyOut(7,:),'r');
plot(FilteredOutputs.time, -sensorOutputs(15,:), 'g');
for k = 1:length(phaseChanges)
   xline(phaseChanges(k)*stepsize); 
end
hold off;
figure(13)
plot(time, stateMatrix(12,:), 'b');
hold on;
plot(FilteredOutputs.time, teensyOut(5,:), 'r');
plot(FilteredOutputs.time, sensorOutputs(13,:), 'g');
hold off;
figure(14)
plot(time, stateMatrix(13,:), 'b');
hold on;
plot(FilteredOutputs.time, teensyOut(6,:), 'r');
plot(FilteredOutputs.time, sensorOutputs(14,:), 'g');
hold off;