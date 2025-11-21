x_1=xlsread('X_232-Y_177.xlsx','A:A');
y_1=xlsread('X_232-Y_177.xlsx','G:G');

xp1=xlsread('X_232-Y_177.xlsx','B:B');
yp1=xlsread('X_232-Y_177.xlsx','D:D');

x_2=xlsread('X_245-Y_370.xlsx','A:A');
y_2=xlsread('X_245-Y_370.xlsx','G:G');

xp2=xlsread('X_245-Y_370.xlsx','B:B');
yp2=xlsread('X_245-Y_370.xlsx','D:D');

x_3=xlsread('X_364-Y_401.xlsx','A:A');
y_3=xlsread('X_364-Y_401.xlsx','G:G');

xp3=xlsread('X_364-Y_401.xlsx','B:B');
yp3=xlsread('X_364-Y_401.xlsx','D:D');

x_4=xlsread('X_447-Y_161.xlsx','A:A');
y_4=xlsread('X_447-Y_161.xlsx','G:G');

xp4=xlsread('X_447-Y_161.xlsx','B:B');
yp4=xlsread('X_447-Y_161.xlsx','D:D');

x_5=xlsread('X_463-Y_389.xlsx','A:A');
y_5=xlsread('X_463-Y_389.xlsx','G:G');

xp5=xlsread('X_463-Y_389.xlsx','B:B');
yp5=xlsread('X_463-Y_389.xlsx','D:D');
%%
for i=1:length(x_1)
    e_x1(i)=232-x_1(i);
    e_y1(i)=177-y_1(i);
    e1(i)=norm([e_x1(i),e_y1(i)]);
end
for i=1:length(x_2)
    e_x2(i)=245-x_2(i);
    e_y2(i)=370-y_2(i);
    e2(i)=norm([e_x2(i),e_y2(i)]);
end

for i=1:length(x_3)       
    e_x3(i)=364-x_3(i);
    e_y3(i)=401-y_3(i);
    e3(i)=norm([e_x3(i),e_y3(i)]); 
end

for i=1:length(x_4)     
    e_x4(i)=447-x_4(i);
    e_y4(i)=161-y_4(i);
    e4(i)=norm([e_x4(i),e_y4(i)]);  
end

for i=1:length(x_5)     
    e_x5(i)=463-x_5(i);
    e_y5(i)=389-y_5(i);
    e5(i)=norm([e_x5(i),e_y5(i)]); 
end
%%
figure('Position',[100 100 600 500],'Units','pixels')
targets=[232,177;
245,370;
364,401;
447,161;
463,389];

plot(targets(:,1),targets(:,2),'ko')
hold on
plot(xp1,yp1)
plot(xp2,yp2)
plot(xp3,yp3)
plot(xp4,yp4)
plot(xp5,yp5)

% plot(x_1,y_1,'r')
% plot(x_2,y_2,'b')
% plot(x_3,y_3,'g')
% plot(x_4,y_4,'c')
% plot(x_5,y_5,'m')

plot(x_1,y_1)%,'r')
plot(x_2,y_2)%,'b')
plot(x_3,y_3)%,'g')
plot(x_4,y_4)%,'c')
plot(x_5,y_5)%,'m')

% fig = gcf;
% fig.PaperUnits = 'inches';
% fig.PaperPosition = [0 0 1 1];

xlabel('X component of position in pixels')
ylabel('Y component of position in pixels')
legend('command points','traj 1','traj 2','traj 3','traj 4',...
    'traj 5','path 1','path 2','path  3'...
    ,'path  4','path  5','location','East')
title('Path followed by robot to reach command points - external tendons')
%set(gcf, 'Position',  [100, 100, 400, 400]);


%%
%figure(1100)
figure('Name','positionerror','Position',[100 100 600 500],'Units','pixels')
clf;
t=1:length(e1);
plot(t,e1)
hold on
plot((1:length(e2)),e2)
plot((1:length(e3)),e3)
plot((1:length(e4)),e4)
plot((1:length(e5)),e5)
title('Position error at each step for robot with external tendons')
xlabel('Steps')
ylabel('Norm of error in pixel space')
legend('x232 y177','x245 y370','x364 y401','x447 y161','x463 y389')
set(gcf, 'Position',  [100, 100, 400, 400])

%%
figure(1101)
clf;

cof = polyfit(t,e1,3);
fe1= polyval(cof, t); %use coefficients from above to estimate a new set of y values
plot(t,e1,'r:',t,fe1,'r') 

hold on

t2=1:length(e2);
cof2 = polyfit(t2,e2,3);
fe2= polyval(cof2, t2); %use coefficients from above to estimate a new set of y values
plot(t2,e2,'b:',t2,fe2,'b')

t3=1:length(e3);
cof3 = polyfit(t3,e3,3);
fe3= polyval(cof3, t3); %use coefficients from above to estimate a new set of y values
plot(t3,e3,'g:',t3,fe3,'g')

t4=1:length(e4);
cof4 = polyfit(t4,e4,3);
fe4= polyval(cof4, t4); %use coefficients from above to estimate a new set of y values
plot(t4,e4,'m:',t4,fe4,'m')

t5=1:length(e5);
cof5 = polyfit(t5,e5,3);
fe5= polyval(cof5, t5); %use coefficients from above to estimate a new set of y values
plot(t5,e5,'c:',t5,fe5,'c')

set(gcf, 'renderer', 'opengl')
title('Fitted postion error plot for robot with external tendons')
xlabel('Steps')
ylabel('Norm of error in pixel space')
legend('x232 y177','Cubic fit e1','x245 y370','Cubic fit e2','x364 y401','Cubic fit e3','x447 y161','Cubic fit e4','x463 y389','Cubic fit e5')


