x_1=xlsread('X_250-Y_273.xlsx','A:A');
y_1=xlsread('X_250-Y_273.xlsx','G:G');

xp1=xlsread('X_250-Y_273.xlsx','B:B');
yp1=xlsread('X_250-Y_273.xlsx','D:D');

x_2=xlsread('X_264-Y_142.xlsx','A:A');
y_2=xlsread('X_264-Y_142.xlsx','G:G');

xp2=xlsread('X_264-Y_142.xlsx','B:B');
yp2=xlsread('X_264-Y_142.xlsx','D:D');

x_3=xlsread('X_322-Y_306.xlsx','A:A');
y_3=xlsread('X_322-Y_306.xlsx','G:G');

xp3=xlsread('X_322-Y_306.xlsx','B:B');
yp3=xlsread('X_322-Y_306.xlsx','D:D');

x_4=xlsread('X_396-Y_279.xlsx','A:A');
y_4=xlsread('X_396-Y_279.xlsx','G:G');

xp4=xlsread('X_396-Y_279.xlsx','B:B');
yp4=xlsread('X_396-Y_279.xlsx','D:D');

x_5=xlsread('X_422-Y_150.xlsx','A:A');
y_5=xlsread('X_422-Y_150.xlsx','G:G');

xp5=xlsread('X_422-Y_150.xlsx','B:B');
yp5=xlsread('X_422-Y_150.xlsx','D:D');
%%

for i=1:length(x_1)
    e_x1(i)=250-x_1(i);
    e_y1(i)=273-y_1(i);
    e1(i)=norm([e_x1(i),e_y1(i)]);
end
for i=1:length(x_2)
    e_x2(i)=264-x_2(i);
    e_y2(i)=142-y_2(i);
    e2(i)=norm([e_x2(i),e_y2(i)]);
end

for i=1:length(x_3)        
    e_x3(i)=322-x_3(i);
    e_y3(i)=306-y_3(i);
    e3(i)=norm([e_x3(i),e_y3(i)]); 
end

for i=1:length(x_4)     
    e_x4(i)=396-x_4(i);
    e_y4(i)=279-y_4(i);
    e4(i)=norm([e_x4(i),e_y4(i)]);  
end

for i=1:length(x_5)     
    e_x5(i)=422-x_5(i);
    e_y5(i)=150-y_5(i);
    e5(i)=norm([e_x5(i),e_y5(i)]); 
end
%%
color=[0, 0.4470, 0.7410;
    0.8500, 0.3250, 0.0980;
    0.9290, 0.6940, 0.1250;
    0.4940, 0.1840, 0.5560;
    0.4660, 0.6740, 0.1880;
    0.3010, 0.7450, 0.9330;
    0.6350, 0.0780, 0.1840];

%%
figure('Name','paths','Position',[100 100 600 500],'Units','pixels')
clf;
set=[250,273;
264,142;
322,306;
396,279;
422,150];

plot(set(:,1),set(:,2),'ko')
hold on
plot(xp1,yp1)
plot(xp2,yp2)
plot(xp3,yp3)
plot(xp4,yp4)
plot(xp5,yp5)

plot(x_1,y_1)%,'r')
plot(x_2,y_2)%,'b')
plot(x_3,y_3)%,'g')
plot(x_4,y_4)%,'c')
plot(x_5,y_5)%,'m')



xlabel('X component of position in pixels')
ylabel('Y component of position in pixels')
legend('command points','traj 1','traj 2','traj 3','traj 4',...
    'traj 5','path 1','path 2','path  3'...
    ,'path  4','path  5','location','East')
title('Path followed by robot to reach command points - internal tendons')


%%
figure('Name','error','Position',[100 100 600 500],'Units','pixels')
plot(1:length(e1),e1)
hold on
plot(1:length(e2),e2)
plot((1:length(e3)),e3)
plot((1:length(e4)),e4)
plot((1:length(e5)),e5)
title('Position error at each step for internal tendons')
xlabel('Steps')
ylabel('Norm of error in pixel space')
legend('x250 y273','x264 y142','x322 y306','x396 y279','x422 y150')

%%

t=1:length(x_1);
figure(1100)
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
cof5 = polyfit(t5,e5,2);
fe5= polyval(cof5, t5); %use coefficients from above to estimate a new set of y values
plot(t5,e5,'c:',t5,fe5,'c')

%set(gcf, 'renderer', 'opengl')
title('Fitted postion error plot for internal tendons')
xlabel('Steps')
ylabel('Norm of error in pixel space')
legend('x250 y273','cubicfit e1','x264 y142','cubicfit e2','x322 y306','cubicfit e3','x396 y279','cubicfit e4','x422 y150','cubicfit e5')
