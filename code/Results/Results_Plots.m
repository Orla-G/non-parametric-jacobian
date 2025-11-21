% Load saved figures
c=hgload('Internal_Tendon_PATH.fig');
k=hgload('Internal_Tendon_ERROR.fig');
% Prepare subplots
figure
h(1)=subplot(1,2,1);
h(2)=subplot(1,2,2);
% Paste figures on the subplots
copyobj(allchild(get(c,'CurrentAxes')),h(1));
copyobj(allchild(get(k,'CurrentAxes')),h(2));
% Add legends
l(1)=legend(h(1),'Command Point', 'Trial 1', 'Trial 2', 'Trial 3', 'Trial 4', 'Trial 5', 'Planned Trajectory')
