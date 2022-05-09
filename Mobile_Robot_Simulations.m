%%Kinematic Simulation of a land-based mobile robot
close all; clear; clc; 

%% PART I - 
%In this part, you will give the angular velocities of each wheel and the
%robot will draw a path using forward kinematics

%%Simulation parameters
dt = 0.01; %step size
totalTime = 20; %Simulation time
t = 0:dt:totalTime; %Time span

%%Vehicle parameters
wheelR = 0.05; %radius of the wheel
lb = 0.1; %distance between wheel frame to vehicle frame


%%Initial Conditions
x0 = 1;
y0 = 1;
theta0 = pi/4; % initial theta

alpha = -pi/2; 
beta = 0;


zeta0 = [x0;y0;theta0]; % initializing pose matrix

zeta(:,1) = zeta0; % initializing pose matrix

%%Loop starts here
    for i = 1:length(t)
        theta = zeta(3,i);  % current orientation in radius.
        RotationMatrix = [cos(theta), sin(theta), 0;
                 -sin(theta), cos(theta), 0;
                 0, 0, 1]; % rotation matrix
             
        %%inputs
        psi_dot_1 = 15; % right wheel angular velocity
        psi_dot_2 = 12; % left wheel angular velocity
        
        psi_dot = [psi_dot_1; psi_dot_2];  
        
          A = [-sin(alpha+beta), cos(alpha+beta), lb*cos(beta);
               -sin(alpha+beta), cos(alpha+beta), -lb*cos(beta);
               cos(alpha+beta), sin(alpha+beta), lb*sin(beta);
               cos(alpha+beta), sin(alpha+beta), -lb*sin(beta);];
           % first two rows are rolling constraints
           % last two rows are no-sliding constraints
           
          B = [wheelR, 0;
               0, wheelR;
               0, 0;
               0, 0;];
           
          zeta_dot(:,i) = inv(RotationMatrix)*inv(transpose(A)*A)...
              *transpose(A)*B*psi_dot; % zeta_dot derivation
        
          zeta(:,i+1) = zeta(:,i) + dt * zeta_dot(:,i); %updating pose
    end

%%Plotting Functions

%%Animation (mobile robot motion animation)
l = 0.3; %length of the mobile robot
w = 2*lb; %width of the mobile robot

%I added third coordinates(zeros) for the following robot coordinates to 
%ease matrix multiplication with rotation matrix:

%mobile robot coordinates
rob_body = [-l/2, l/2, l/2, -l/2, -l/2;
         -w/2, -w/2, w/2, w/2 -w/2;
         0,0,0,0,0;];

%first wheel coordinates
wheel1 = [-l/2-wheelR, -l/2-wheelR, -l/2+wheelR, -l/2+wheelR, -l/2-wheelR;
          w/2, w/2+wheelR, w/2+wheelR, w/2, w/2;
          0,0,0,0,0;];

%second wheel coordinates
wheel2 = [-l/2-wheelR, -l/2-wheelR, -l/2+wheelR, -l/2+wheelR, -l/2-wheelR;
          -w/2, -w/2-wheelR, -w/2-wheelR, -w/2, -w/2;
          0,0,0,0,0;];

%castor wheel coordinates(simply a circle)
castor_r = 0.02;
xc=3*l/8;
yc=0;
th = 0:pi/216:2*pi;
xunit = castor_r * cos(th) + xc;
yunit = castor_r * sin(th) + yc;
castor_wheel = [xunit; yunit;zeros(size(xunit));];



figure()
set(gcf, 'Units', 'Normalized', 'OuterPosition', [0 0 1 1]);
set(gcf, 'Toolbar', 'none', 'Menu', 'none');
set(gcf,'color','w');
for i = 1:5:length(t) %animation starts here
    theta = zeta(3,i);
    RotationMatrix = [cos(theta),sin(theta),0;
             -sin(theta), cos(theta),0;
             0,0,1;]; %rotation matrix
    
    m = 2; %just a multiplicator to adjust the robot dimensions
    update_1 = inv(RotationMatrix)*rob_body * m;
    update_2 = inv(RotationMatrix)*wheel1 * m;
    update_3 = inv(RotationMatrix)*wheel2 * m;
    update_4 = inv(RotationMatrix)*castor_wheel * m;

    subplot(11,10,[2:9 12:19 22:29 32:39 42:49 52:59])
    fill(update_1(1,:) + zeta(1,i),update_1(2,:) + zeta(2,i),[0.9 0.7 0.1])
    hold on
    fill(update_2(1,:) + zeta(1,i),update_2(2,:) + zeta(2,i), 'r')
    hold on
    fill(update_3(1,:) + zeta(1,i),update_3(2,:) + zeta(2,i), 'r')
    hold on
    fill(update_4(1,:) + zeta(1,i),update_4(2,:) + zeta(2,i), 'y')
    hold on
    
    
    grid on
    axis([-2 4 -2 4]), axis square
    
    plot(zeta(1,1:i),zeta(2,1:i),'b-') % plotting TurtleBot's motion
    
    legend({'TurtleBot','','','','Path'},'FontSize',10)
    
    set(gca,'fontsize',12)
    xlabel('x,[m]');
    ylabel('y,[m]');
    title("TurtleBot's Motion")
    pause(0.001);
    hold off
    
    
    subplot(11,10,[71:74 81:84 91:94 101:104])
    plot(t(i), zeta(1,i),'r.-') % plotting x coordinates w.r.t. time
    set(gca, 'fontsize', 12)
    xlabel('t, [s]')
    ylabel('x, [m]')
    title('x coordinates in global frame vs time')
    hold on
    
    
    subplot(11,10,[77:80 87:90 97:100 107:110])
    plot(t(i), zeta(2, i), 'b.-') % plotting y coordinates w.r.t. time
    set(gca, 'fontsize', 12)
    xlabel('t, [s]')
    ylabel('y, [m]')
    title('y coordinates in global frame vs time')
    
    hold on
end %animation ends



%% PART II - 
%In this part, you will select a path and the robot will try to follow it
%using inverse kinematics

%path defining
x = -2:0.01:3; 
for i=1:length(x)
    y = (x(i)+3)*(x(i)+2)*(x(i)-1)*(x(i)-2)*(x(i)-3);
    path(i,:) = [3*x(i) + 6,y/5 + 11];
end %path defining ends


x0 = path(1,1); % initial x coordinate of the TurtleBot's center
y0 = path(1,2); % initial y coordinate of the TurtleBot's center
theta0 = pi/4; % initial theta, you can change them to whatever you want

    
zeta20 = [x0;y0;theta0]; % initializing pose matrix
zeta2(:,1) = zeta20; % initializing pose matrix

alpha = -pi/2; 
beta = 0;

%defining x and y coordinates
for i=2:length(path)
    zeta2(1,i) = path(i,1);
    zeta2(2,i) = path(i,2);   
end



zeta2_dot0 = [0;0;0]; % initializing pose_dot of the TurtleBot

zeta2_dot(:,1) = zeta2_dot0; % initializing pose_dot of the TurtleBot



% Simulation parameters
dt = 0.01; %step size

%%Vehicle parameters
wheelR = 0.05; %radius of the wheel
lb = 0.1; %distance between wheel frame to vehicle frame

%%Animation (mobile robot motion animation)
l = 0.3; %length of the mobile robot
w = 2*lb; %width of the mobile robot

%I added third coordinates(zeros) for the following robot coordinates to 
%ease matrix multiplication with rotation matrix:

%mobile robot coordinates
rob_body = [-l/2, l/2, l/2, -l/2, -l/2;
         -w/2, -w/2, w/2, w/2 -w/2;
         0,0,0,0,0;];

%first wheel coordinates
wheel1 = [-l/2-wheelR, -l/2-wheelR, -l/2+wheelR, -l/2+wheelR, -l/2-wheelR;
          w/2, w/2+wheelR, w/2+wheelR, w/2, w/2;
          0,0,0,0,0;];

%second wheel coordinates
wheel2 = [-l/2-wheelR, -l/2-wheelR, -l/2+wheelR, -l/2+wheelR, -l/2-wheelR;
          -w/2, -w/2-wheelR, -w/2-wheelR, -w/2, -w/2;
          0,0,0,0,0;];

%castor wheel coordinates(simply a circle)
castor_r = 0.02;
xc=3*l/8;
yc=0;
th = 0:pi/216:2*pi;
xunit = castor_r * cos(th) + xc;
yunit = castor_r * sin(th) + yc;
castor_wheel = [xunit; yunit;zeros(size(xunit));];



figure()
set(gcf, 'Units', 'Normalized', 'OuterPosition', [0 0 1 1]);
set(gcf, 'Toolbar', 'none', 'Menu', 'none');
set(gcf,'color','w');
for i = 1:length(path)-1 %animation starts here
    %theta = zeta2(3,i+1) - zeta2(3,i);
    scatter(path(end,1),path(end,2), 100,'r','o') % GOAL
    hold on
    text(path(end,1),path(end,2), "\leftarrow GOAL") % Goal showing
    hold on
    
    % Updating theta:
    theta = atan((zeta2(2,i+1) - zeta2(2,i))/(zeta2(1,i+1) - zeta2(1,i)));
    
    RotationMatrix = [cos(theta),sin(theta),0;
             -sin(theta), cos(theta),0;
             0,0,1;]; %rotation matrix
    
    m = 4; %just a multiplicator to adjust the robot dimensions
    update_1 = inv(RotationMatrix)*rob_body * m;
    update_2 = inv(RotationMatrix)*wheel1 * m;
    update_3 = inv(RotationMatrix)*wheel2 * m;
    update_4 = inv(RotationMatrix)*castor_wheel * m;

    plot(path(:,1),path(:,2),"k--d",'MarkerSize',5) % ploting desired path
    hold on
    
    % drawing the TurtleBot's updated positions
    fill(update_1(1,:)+ zeta2(1,i),update_1(2,:)+ zeta2(2,i),[0.9 0.7 0.1])
    hold on
    fill(update_2(1,:)+ zeta2(1,i),update_2(2,:)+ zeta2(2,i), 'r')
    hold on
    fill(update_3(1,:)+ zeta2(1,i),update_3(2,:)+ zeta2(2,i), 'r')
    hold on
    fill(update_4(1,:)+ zeta2(1,i),update_4(2,:)+ zeta2(2,i), 'y')
    hold on 
    grid on
    
    
    plot(zeta2(1,1:i),zeta2(2,1:i),'b-') % plotting TurtleBot's Motion 
    hold on
        
        zeta2(3,i+1) = theta;
        
        B2 = [wheelR, 0;
             0, wheelR;
             0, 0;
             0, 0;];
         
        A2 = [-sin(alpha+beta), cos(alpha+beta), lb*cos(beta);
               -sin(alpha+beta), cos(alpha+beta), -lb*cos(beta);
               cos(alpha+beta), sin(alpha+beta), lb*sin(beta);
               cos(alpha+beta), sin(alpha+beta), -lb*sin(beta);];
        % first two rows are rolling constraints
        % last two rows are no-sliding constraints
         
        
        zeta2_dot(:,i) = zeta2(:,i)*dt; % zeta_dot
             
        psi_dot=inv(transpose(B2)*B2)*transpose(B2)*...
            A2*RotationMatrix*zeta2_dot(:,i); % inverse kinematics
    
    % obstacles on the road :))
    obs3_x = [2 7 7 4 2];
    obs3_y = [15 15 10 3 10];
    fill(obs3_x,obs3_y,[0.7 0.7 0.7])
    
    obs2_x = [0 0 3];
    obs2_y = [5 -2 -2];
    fill(obs2_x,obs2_y,[0.7 0.7 0.7])
    
    obs1_x = [8 8 10 12 12 10];
    obs1_y = [-2 5 10 5 -2 -2];
    fill(obs1_x,obs1_y,[0.7 0.7 0.7])
    
    axis([-2 18 -1 15]), axis square
        
    legend({'','Desired Path','','','','',"TurtleBot's Path",...
        'Obstacles'},'FontSize',10)
    
    set(gca,'fontsize',12)
    xlabel('x,[m]');
    ylabel('y,[m]');
    title("Desired Path and TurtleBot's Path")
    pause(0.001);
    hold off
    
end
