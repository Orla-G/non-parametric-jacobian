x_1=xlsread('X_232-Y_177.xlsx','A:A');
y_1=xlsread('X_232-Y_177.xlsx','G:G');

x_2=xlsread('X_245-Y_370.xlsx','A:A');
y_2=xlsread('X_245-Y_370.xlsx','G:G');

x_3=xlsread('X_364-Y_401.xlsx','A:A');
y_3=xlsread('X_364-Y_401.xlsx','G:G');

x_4=xlsread('X_447-Y_161.xlsx','A:A');
y_4=xlsread('X_447-Y_161.xlsx','G:G');

x_5=xlsread('X_463-Y_389.xlsx','A:A');
y_5=xlsread('X_463-Y_389.xlsx','G:G');

t=1:length(x_1)
figure(1100)
clf;
for i=1:length(x_1)
    e_x1(i)=232-x_1(i);
    e_y1(i)=177-y_1(i);
    e1(i)=norm(e_x1(i),e_y1(i));
end
for i=1:length(x_2)
    e_x2(i)=245-x_2(i);
    e_y2(i)=370-y_2(i)
    e2(i)=norm(e_x2(i),e_y2(i));
end

for i=1:length(x_3)        
    e_x3(i)=364-x_3(i);
    e_y3(i)=401-y_3(i)
    e3(i)=norm(e_x3(i),e_y3(i)); 
end

for i=1:length(x_4)     
    e_x4(i)=447-x_4(i);
    e_y4(i)=161-y_4(i)
    e4(i)=norm(e_x4(i),e_y4(i));  
end

for i=1:length(x_5)     
    e_x5(i)=463-x_5(i);
    e_y5(i)=389-y_5(i)
    e5(i)=norm(e_x5(i),e_y5(i)); 
end
plot(t,e1,'ro')
hold on
plot((1:length(e2)),e2,'bo')
plot((1:length(e3)),e3,'go')
plot((1:length(e4)),e4,'mo')
plot((1:length(e5)),e5,'vo')
title('Position error at each step for external threads')
xlabel('Steps')
ylabel('Norm of error in pixel space')
legend('x232 y177','x245 y370','x364 y401','x447 y161','x463 y389')
