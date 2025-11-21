%% Get actual inputs for initial Jacobian
% block1 = [0:10:150, 140:-10:10]; %for motors 1 and 2
% block2 = [0:-10:-150, -140:10:-10]; %for motor 3
block1 = [ones(1,15)*10, ones(1,15)*-10]; 
block2 = -block1; 
block3 = ones(1,length(block1))*0; 
step = [0:1:4*length(block1)];

%create actual inputs
dq1_a = [block1, block3, block3, block1, 0]; 
dq2_a = [block3, block1, block3, block1, 0]; 
dq3_a = [block3, block3, block2, block3, 0];

%% Commanded Microseconds plot
figure(11)
subplot(3,1,1)
plot(step, dq1_a,'o')
title('Servo 1','FontSize',14)
xlabel('Step', 'FontSize',10)
ylabel('Command ($\mu$s)','FontSmoothing','on', 'FontSize',10,'interpreter','latex')
ylim([-15 15])

subplot(3,1,2)
plot(step, dq2_a, 'o')
title('Servo 2','FontSize',14)
xlabel('Step', 'FontSize',10)
ylabel('Command ($\mu$s)','FontSmoothing','on','FontSize',10,'interpreter','latex')
ylim([-15 15])

subplot(3,1,3)
plot(step, dq3_a,'o')
title('Servo 3','FontSize',14)
xlabel('Step', 'FontSize',10)
ylabel('Command ($\mu$s)', 'FontSmoothing','on','FontSize',10,'interpreter','latex')
ylim([-15 15])

%% Initial Jacobian Results for Robot 1
Ji = [-0.357880120022413 0.359866614950458 -0.029610492190774;...
    0.0184914680026557 -0.171101492471721 0.0659979658814278];

EE = xlsread('Robot 1 - Run 1 Data/Robot1_EE_Position.xlsx');
EE = EE';
QQQ = pinv(Ji)*EE; 
dq1_r = QQQ(1,:); 
dq2_r = QQQ(2,:); 
dq3_r = QQQ(3,:); 

%% Initial Jacobian Results for Robot 2
Ji = [-0.31 0.229166666666667 -0.0391666666666667; 
    -0.0566666666666667 -0.259444444444444 -0.138888888888889]; 

EE = xlsread('results-fixedpoint/Robot 2 Results/Robot_2_initial_EE_Position.xlsx');
EE = EE';
QQQ = pinv(Ji)*EE; 
dq1_r = QQQ(1,:); 
dq2_r = QQQ(2,:); 
dq3_r = QQQ(3,:); 
    
%% Plot of recorded Position
figure(12)
plot(step, EE(1,:),'o', step, EE(2,:), 'x')
legend({'X Position','Y Position'},'FontSize',12)
title('Recorded End Effector Position','FontSize',14)
xlabel('Step','FontSize',10)
ylabel('Pixels','FontSize',10)

%% Plot results of actual input vs. calculated input
figure(1)
subplot(3,1,1)
plot(step, dq1_a,'o', step, dq1_r, 'x')
legend('Actual Input', 'Calculated Input','Location','NorthEast')
title('Servo 1','FontSize',14)
xlabel('Step', 'FontSize',10)
ylabel({'Command ($\mu$s)'}, 'FontSize',10,'interpreter','latex')
ylim([-15 15])

subplot(3,1,2)
plot(step, dq2_a, 'o', step, dq2_r, 'x')
title('Servo 2','FontSize',14)
xlabel('Step', 'FontSize',10)
ylabel({'Command ($\mu$s)'}, 'FontSize',10,'interpreter','latex')
ylim([-15 15])

subplot(3,1,3)
plot(step, dq3_a,'o', step, dq3_r, 'x')
title('Servo 3','FontSize',14)
xlabel('Step', 'FontSize',10)
ylabel('Command ($\mu$s)', 'FontSize',10,'interpreter','latex')
ylim([-15 15])
%suptitle('Actual Input vs Calculated Input')
%% Find Mean Squared Error
n = length(dq1_a); 
for i = 1:n
    q1_err(i) = (dq1_a(i) - dq1_r(i))^2; 
    q2_err(i) = (dq2_a(i) - dq2_r(i))^2;
    q3_err(i) = (dq3_a(i) - dq3_r(i))^2;
end
q1_err = sum(q1_err)/n
q2_err = sum(q2_err)/n
q3_err = sum(q3_err)/n

%% End Effector Comparison
figure(2)
Ji_EE = Ji*[dq1_a; dq2_a; dq3_a];
subplot(2,1,1)
plot(step, Ji_EE(1,:),step, EE(1,:),'o')
legend('Calcualted','Actual')
title('In X')
xlabel('Step')
ylabel('Pixels')

subplot(2,1,2)
plot(step, Ji_EE(2,:), step, EE(2,:),'o')
title('In Y')
xlabel('Step')
ylabel('Pixels')
    
