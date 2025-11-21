%% Create inputs for initial Jacobian
%The idea is that 90deg is the neutral position. Each motor activates in a
%series going 10 degrees

%on the actual robot this is what we would feed into it to get that first
%Jacobian. 

%create a bunch of blocks I can throw together
b1 = [90:100, 100:-1:90];  
b2 = ones(1,length(b1))*90;  

%create actual q's
q1 = [b1 b2 b2]; 
q2 = [b2 b1 b2];  
q3 = [b2 b2 b1]; 

%plot q's
% figure()
% plot(q1); hold on; 
% plot(q2); hold on; 
% plot(q3); 
% title('Plot of q values')
% xlabel('Step')
% ylabel('degree')

%% Create outputs for initial Jacobian
%On the actual robot we would read these values from the above inputs, just
%going to worry about x and y (don't care where we are in z)
% assume following string placement q1: 90deg, q2: 225deg, q3: 315deg

%initialize each matrix
x(1) = 0;
y(1) = 0;
for i = (2:length(q1))
   if i <=22
       y(i) = y(i-1)+14.44/10*(q1(i)-q1(i-1));
       x(i) = 0; 
   elseif (i > 22 && i <=44)
       x(i) = x(i-1)+ 2*cos(5*pi/4)*(q2(i)-q2(i-1));  
       y(i) = y(i-1)+2*cos(5*pi/4)*(q2(i)-q2(i-1)); 
   elseif (i > 44 && i <= 66)
       x(i) = x(i-1)+2*cos(7*pi/4)*(q3(i)-q3(i-1));  
       y(i) = y(i-1)-2*cos(7*pi/4)*(q3(i)-q3(i-1)); 
   end   
end

figure();
plot(x,y)
title('Plot of initial trajectory')
xlabel('X')
ylabel('Y')
figure();
plot(x); hold on; 
plot(y); hold on; 
title('Plot of x, y movement over time')
xlabel('Step')
ylabel('Magnitude')
legend('X','Y')

%% Create Q and P Matrices

Q = [q1; q2; q3]; 
[Qi,idx] = licols(Q) 
P = [x; y]; 

%% Create A and B matrix, solve for initial Jacobian
mX = Q(:,2)-Q(:,1); 
B = P(:,2)-P(:,1); 

for i = [3:(length(q1)-1)]
    mX(:,i-1) = Q(:,i)-Q(:,i-1); 
    B(:,i-1) = P(:,i)-P(:,i-1); 
end

% [mX,idx] = licols(mX); %get linearly ind. matrix from mX
% %use idx to create linerly ind. B matrix from B
% for j = 1:length(idx)
%     if j ==1
%         mB = B(:,idx(j)); 
%     else 
%         mB = [mB B(:,idx(j))];
%     end
% end
[mXi,idx] = licols(mX)
%realized I had the dimensions reversed so take the transpose

display('the initial Jacobian is....'); 
Ji = B*mX'*inv(mX*mX') %We got it! 

%% Create Circular Trajectory
%create a circular trajectory
r=5; % radius
C=[0,0];
theta=0:2*pi/360:2*pi; % the angle
Traj=(r*[cos(theta')+C(1) sin(theta')+C(2)])'; %circular trajectory
Tx = Traj(1,:); 
Ty = Traj(2,:); 
%% Create 'actual' trajectory
%in the actual robot we would read the position of the end effector using
%our tracking system, use that data to update the Jacobian and then control
%our robot. In the case of using artifical data I have created a trajectory
%path that has white Gaussian noise with a signal to noise ratio of 10.
%This should replicate the fact that the robot will not exactly follow our
%path. 

% %Communication Systems Toolbox MUST be installed. 
ActTraj = awgn(Traj,30);
ActTraj(:,1) = [5;0]; %need to set initial position to zero
ActTraj(:,end) = [5;0];

%plot differences in data
figure();
subplot(2,1,1)
plot(theta,Tx,theta,ActTraj(1,:))
legend('Planned','Actual')
title('X data')
xlabel('Time')
ylabel('Magnitude')
subplot(2,1,2)
plot(theta,Ty,theta,ActTraj(2,:))
title('Y data')
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
TestTraj= [0;0]; 

%Running a test to make sure everything works 
deltaTraj = [5;0];

%number of time steps we want between calculating online Jacobian
n = 361;

%set J = Ji so the initial Jacobian is still accessible 
J = Ji; 

k = 0; %counter

for i = (2:length(theta))
    %we want to move to the next position on our planned trajectory
    %from our actual EE position
    deltaTraj(:,i) = Traj(:,i)-ActTraj(:,i-1); %3x1 vector
    %use the initial J matrix we found to solve for deltaQ needed
    deltaQ(:,i) = pinv(J)*deltaTraj(:,i); %3x1 vector
    %Find next Q 
    ActQ(:,i) = ActQ(:,i-1)+deltaQ(:,i); %3xi vector
    if ActQ(1,i) < 0
        ActQ(1,i) = 0;
    elseif ActQ(1,i) > 180
        ActQ(1,i) = 180;
    end
    if ActQ(2,i) < 0
        ActQ(2,i) = 0;
    elseif ActQ(2,i) > 180
        ActQ(2,i) = 180;
    end
    if ActQ(3,i) < 0
        ActQ(3,i) = 0;
    elseif ActQ(3,i) > 180
        ActQ(3,i) = 180;
    end
    
    %Creating a test traj, that shows the system compensating at each time
    %step to get to planned trajectory
    TestTraj(:,i) = ActTraj(:,i-1)+J*deltaQ(:,i);
    
    %send some command to servos to go to next Q position
    %DO STUFF HERE

    if i > n+1 && k > n  
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
        k = 0; 
    else
        k = k+1; 
    end 
end
    
figure()
plot(Traj(1,:),Traj(2,:)); 
hold on; 
plot(TestTraj(1,:),TestTraj(2,:)); 
xlabel('X Position') 
ylabel('Y Position')
legend('Planned Path','Controlled Path')
title('Plot of Planned Path and Controlled Path')


figure()
plot(deltaTraj(1,:))
hold on; 
plot(deltaQ(1,:))
hold on;
plot(deltaTraj(2,:))
hold on; 
plot(deltaQ(2,:))
hold on;
plot(deltaQ(3,:))
hold on;
xlabel('Time step') 
ylabel('Change in Q or Traj')
legend('X', 'q1', 'y','q2','q3')
title('Delta comparision')

figure() 
plot(theta,ActQ(1,:))
hold on; 
plot(theta, ActQ(2,:))
hold on; 
plot(theta,ActQ(3,:))
legend('q1','q2','q3')
xlabel('Time step') 
ylabel('degree')

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