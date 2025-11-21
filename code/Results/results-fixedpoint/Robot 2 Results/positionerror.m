x_1=xlsread('X_354-Y_315-control_path_data.xlsx','A:A');
y_1=xlsread('X_354-Y_315-control_path_data.xlsx','G:G');

x_2=xlsread('X_372-Y_384_GOOD.xlsx','A:A');
y_2=xlsread('X_372-Y_384_GOOD.xlsx','G:G');

x_3=xlsread('X_398-Y_415-control_path_data.xlsx','A:A');
y_3=xlsread('X_398-Y_415-control_path_data.xlsx','G:G');

x_4=xlsread('X_451-Y_349-control_path_data_GOOD.xlsx','A:A');
y_4=xlsread('X_451-Y_349-control_path_data_GOOD.xlsx','G:G');

x_5=xlsread('X_469-Y_119-control_path_data_GOOD.xlsx','A:A');
y_5=xlsread('X_469-Y_119-control_path_data_GOOD.xlsx','G:G');
%%

for i=1:length(x_1)
    e_x1(i)=354-x_1(i);
    e_y1(i)=315-y_1(i);
    e1(i)=norm([e_x1(i),e_y1(i)]);
end
for i=1:length(x_2)
    e_x2(i)=372-x_2(i);
    e_y2(i)=384-y_2(i);
    e2(i)=norm([e_x2(i),e_y2(i)]);
end

for i=1:length(x_3)        
    e_x3(i)=398-x_3(i);
    e_y3(i)=415-y_3(i);
    e3(i)=norm([e_x3(i),e_y3(i)]); 
end

for i=1:length(x_4)     
    e_x4(i)=451-x_4(i);
    e_y4(i)=349-y_4(i);
    e4(i)=norm([e_x4(i),e_y4(i)]);  
end

for i=1:length(x_5)     
    e_x5(i)=469-x_5(i);
    e_y5(i)=119-y_5(i);
    e5(i)=norm([e_x5(i),e_y5(i)]); 
end
%%
figure(7000)
set=[354,315;
372,384;
398,415;
451,349;
469,119];

for i=1:5
    plot(set(i,1),set(i,2),'o')
    hold on
end

plot(x_1,y_1,'r')
plot(x_2,y_2,'b')
plot(x_3,y_3,'g')
plot(x_4,y_4,'c')
plot(x_5,y_5,'m')


%%
t=1:length(x_1);
figure(1100)
clf;
plot(t,e1)
hold on
plot((1:length(e2)),e2)
plot((1:length(e3)),e3)
plot((1:length(e4)),e4)
plot((1:length(e5)),e5)
title('Position error at each step for Cobian')
xlabel('Steps')
ylabel('Norm of error in pixel space')
legend('x354 y315','x372 y384','x398 y415','x451 y349','x469 y119')
