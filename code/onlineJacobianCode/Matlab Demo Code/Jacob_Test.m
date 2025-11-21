%% Create inputs for initial Jacobian
%The idea is that 90deg is the neutral position. Each motor activates in a
%series. First q1 goes to 95, then q2 goes to 95 then q3 goes to 95. The
%motors all then go to 90, then to 85, then back to 90. 

%on the actual robot this is what we would feed into it to get that first
%Jacobian. 

%create a bunch of blocks I can throw together
b1 = [90:95]; 
b2 = [95:-1:90]; 
b3 = [90:-1:85]; 
b4 = [85:90]; 
b5 = ones(1,length(b1))*85; 
b6 = ones(1,length(b1))*90; 
b7 = ones(1,length(b1))*95; 

%create actual q's
q1 = [b1 b7 b7 b2 b6 b6 b3 b5 b5 b4 b6 b6]; 
q2 = [b6 b1 b7 b7 b2 b6 b6 b3 b5 b5 b4 b6];  
q3 = [b6 b6 b1 b7 b7 b2 b6 b6 b3 b5 b5 b4]; 

%plot q's
% figure()
% plot(q1); hold on; 
% plot(q2); hold on; 
% plot(q3); 
% title('Plot of q values')
% xlabel('Step')
% ylabel('degree')

%% Create outputs for initial Jacobian
%On the actual robot we would read these values from the above inputs

%initialize each matrix
x(1) = 0;
y(1) = 0;
z(1) = 0;
for i = (2:length(q1))
    %for the purposes of this demenstration q1 controls x, q2 controls y,
    %and q3 controls z
    
    %create some random values that will change x,y,z coordinates
    rx = randi([1 3],1); 
    ry = randi([1 3],1); 
    rz = randi([1 3],1);
    
    %Find X values
    if (q1(i) >  q1(i-1))
        x(i) = x(i-1) + rx;
    elseif (q1(i) < q1(i-1))
        x(i) = x(i-1) - rx; 
    else 
        x(i) = x(i-1);
    end
    
    %Find Y values
    if (q2(i) >  q2(i-1))
        y(i) = y(i-1) + ry;
    elseif (q2(i) < q2(i-1))
        y(i) = y(i-1) - ry; 
    else 
        y(i) = y(i-1);
    end
    
    %Find Z values
    if (q3(i) >  q3(i-1))
        z(i) = z(i-1) + rz;
    elseif (q3(i) < q3(i-1))
        z(i) = z(i-1) - rz; 
    else 
        z(i) = z(i-1);
    end    
end

figure();
plot3(x,y,z)
title('Plot of initial trajectory')
xlabel('X')
ylabel('Y')
zlabel('Z')
figure();
plot(x); hold on; 
plot(y); hold on; 
plot(z); 
title('Plot of x, y, z movement over time')
xlabel('Step')
ylabel('Magnitude')
legend('X','Y','Z')

%% Create Q and P Matrices

Q = [q1; q2; q3]; 
P = [x; y; z]; 

%% Create A and B matrix, solve for initial Jacobian
mX = Q(:,2)-Q(:,1); 
B = P(:,2)-P(:,1); 

for i = [3:(length(q1)-1)]
    mX(:,i-1) = Q(:,i)-Q(:,i-1); 
    B(:,i-1) = P(:,i)-P(:,i-1); 
end

%realized I had the dimensions reversed so take the transpose

display('the initial Jacobian is....'); 
Ji = B*mX'*inv(mX*mX') %We got it! 

%% Create Trajectory using Min Jerk from Class
%create time vector of how long we want this motion to take
T = [0:0.01:0.5]; 

%want to go from (0,0,0) to (10,5,7)
for i = [1:length(T)]
    Tx(i) = 800*T(i)^3-2400*T(i)^4+1920*T(i)^5; 
    Ty(i) = 400*T(i)^3-1200*T(i)^4+960*T(i)^5; 
    Tz(i) = 560*T(i)^3-1680*T(i)^4+1344*T(i)^5;  
end

Traj = [Tx; Ty; Tz;];

% figure();
% plot3(Tx,Ty,Tz)
% title('Plot of desired trajectory')
% xlabel('X')
% ylabel('Y')
% zlabel('Z')
% figure();
% plot(T,Tx,T,Ty,T,Tz)
% title('Plot of x, y, z movement over time')
% xlabel('Time')
% ylabel('Magnitude')
% legend('X','Y','Z')
%% Create 'actual' trajectory
%in the actual robot we would read the position of the end effector using
%our tracking system, use that data to update the Jacobian and then control
%our robot. In the case of using artifical data I have created a trajectory
%path that has white Gaussian noise with a signal to noise ratio of 10.
%This should replicate the fact that the robot will not exactly follow our
%path. 

% %Communication Systems Toolbox MUST be installed. 
ActTraj = awgn(Traj,25);
ActTraj(:,1) = [0;0;0]; %need to set initial position to zero
ActTraj(:,end) = [10,5,7];

%plot differences in data
figure();
subplot(3,1,1)
plot(T,Tx,T,ActTraj(1,:))
legend('Planned','Actual')
title('X data')
xlabel('Time')
ylabel('Magnitude')
subplot(3,1,2)
plot(T,Ty,T,ActTraj(2,:))
title('Y data')
xlabel('Time')
ylabel('Magnitude')
subplot(3,1,3)
plot(T,Tz,T,ActTraj(3,:))
title('Z data')
xlabel('Time')
ylabel('Magnitude')

%% Show that we can create q's and Jacobians at each time step
%will run through each time step and calculate the new Jacobian
%Traj(i) is the desired position on the path
%ActTraj(i) is the actual position on the path
%deltaQ(i) is the change in q1,q2,q3 required to go from the ActTraj(i) to the Traj(i)
%ActQ(i) is the Q's at each time step
%J is the Jacobian with deltaQ = inv(J)*deltaTraj

%Read the initial values of Q and EE position at starting point
ActQ = [90; 90; 90]; %assume we start at neutral Q which is 90,90,90

%In the actual robot we read the EE poisition but instead we just used the
%ActTraj we setup

%setup deltaQ matrix
deltaQ = [0;0;0]; 
TestTraj= [0;0;0]; 

%Running a test to make sure everything works 
deltaTraj = [0;0;0];

%number of time steps we want between calculating online Jacobian
n = 10;

%set J = Ji so the initial Jacobian is still accessible 
J = Ji; 

for i = (2:length(T))
    %we want to move to the next position on our planned trajectory
    %from our actual EE position
    deltaTraj(:,i) = Traj(:,i)-ActTraj(:,i-1); %3x1 vector
    %use the initial J matrix we found to solve for deltaQ needed
    deltaQ(:,i) = inv(J)*deltaTraj(:,i); %3x1 vector
    %Find next Q 
    ActQ(:,i) = ActQ(:,i-1)+deltaQ(:,i); %3xi vector
    
    %Creating a test traj, that shows the system compensating at each time
    %step to get to planned trajectory
    TestTraj(:,i) = ActTraj(:,i-1)+J*deltaQ(:,i);
    
    %send some command to servos to go to next Q position
    %DO STUFF HERE

    if i > n+1   
        %Calculate next Jacobian using last n time steps
        mX = (deltaQ(:,i-n+1:i)); %retrieve a nx3 matrix from ActQ
        B = (ActTraj(:,i-n+1:i)); %retrieve a nx3 matrix from ActTraj
        [mX,idx] = licols(mX); %get linearly ind. matrix from mX
        %use idx to create linerly ind. B matrix from B
        for j = 1:length(idx)
            if j ==1
                mB = B(:,idx(j)); 
            else 
                mB = [mB B(:,idx(j))];
            end
        end
        J = (mB*mX'*inv(mX*mX'))%Solve for new J;
    end 
end
    
figure()
plot(Traj(1,:)); 
hold on; 
plot(TestTraj(1,:)); 
xlabel('Time step') 
ylabel('Position')
legend('Planned Path','Controlled Path')
title('Plot of Planned Path and Controlled Path in X direction')


figure()
plot(deltaTraj(1,:))
hold on; 
plot(deltaQ(1,:))
hold on;
plot(deltaTraj(2,:))
hold on; 
plot(deltaQ(2,:))
hold on;
plot(deltaTraj(3,:))
hold on; 
plot(deltaQ(3,:))
hold on;
xlabel('Time step') 
ylabel('Change in Q or Traj')
legend('X', 'q1', 'y','q2','z','q3')
title('Delta comparision')

figure() 
plot(ActQ(1,:))
hold on; 
plot(ActQ(2,:))
hold on; 
plot(ActQ(3,:))
legend('q1','q2','q3')
xlabel('Time step') 
ylabel('degree')
title('Actuator position over time')

%% Function to make a linearly independ matrix 
%Taken from: 
%https://www.mathworks.com/matlabcentral/answers/108835-how-to-get-only-linearly-independent-rows-in-a-matrix-or-to-remove-linear-dependency-b-w-rows-in-a-m

function [Xsub,idx]=licols(X,tol)
%Extract a linearly independent set of columns of a given matrix X
%
%    [Xsub,idx]=licols(X)
%
%in:
%
%  X: The given input matrix
%  tol: A rank estimation tolerance. Default=1e-10
%
%out:
%
% Xsub: The extracted columns of X
% idx:  The indices (into X) of the extracted columns

     if ~nnz(X) %X has no non-zeros and hence no independent columns

         Xsub=[]; idx=[];
         return
     end

     if nargin<2, tol=1e-10; end

       [Q, R, E] = qr(X,0); 

       if ~isvector(R)
        diagr = abs(diag(R));
       else
        diagr = R(1);   
       end

       %Rank estimation
       r = find(diagr >= tol*diagr(1), 1, 'last'); %rank estimation

       idx=sort(E(1:r));

       Xsub=X(:,idx);   
end