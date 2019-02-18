function [  ] = simulateRR(  )
% MECH 498 - Intro to Robotics - Spring 2016
% Lab 3
% Solutions by Craig McDonald
%
%    DESCRIPTION - This function should run a dynamic simulation of the RR
%    robot for this assignment, and then play a video of the resulting
%    robot motion.
%
%    ADDITIONAL CODE NEEDED: Lots
%

% Shlok Sobti and Brian Ying
% For Part 1, set Tao = 0.
% For Part 2, 

close all;

% Initialize robot
robot = RRInit();
m1 = robot.m_1;
m2 = robot.m_2;
L1 = robot.l_1;
L2 = robot.l_2;
Mr1 = robot.m_r1;
Mr2 = robot.m_r2;

M1 = m1+Mr1;
M2 = m2+Mr2;
Lc1 = ((m1+0.5*Mr1)*L1)/M1;
Lc2 = ((m2+0.5*Mr2)*L2)/M2;
I1 = (1/12)*Mr1*L1^2 + m1*(L1/2)^2;
I2 = (1/12)*Mr2*L2^2 + m2*(L2/2)^2;
g = robot.g;

% Joint Torque Limit
tau_max = [200]; % [N-m] (Scalar)

% Time
dt = [0.01]; % [s]
t_f = [10]; % [s]

% Initial Conditions
X0 = [pi/3 pi/2 0 0];
%X0 = [0 0 0 0]; % DELETE
X_dot0 = [0 0 0 0];

% Control Gains (Scalar)
K_p = [300]; % 300 CD
K_v = [150]; % 150 CD

% Desired Angular Position
td1 = 0;
td2 = pi/2;


t = 0:dt:t_f;
X = []; % initialize variable to hold state vector
X_dot = []; % initialize variable to hold state vector derivatives

for i = 1:length(t)
    if i == 1
        X = X0;
        X_dot = X_dot0;
        
    else  
        % Dynamic Model
        M = [M1*Lc1^2+M2*(L1+Lc2*cos(X(i-1,2)))^2+(I1+I2) 0; 0 M2*Lc2^2+I2];
        C = [-2*M2*Lc2*sin(X(i-1,2))*(L1+Lc2*cos(X(i-1,2)))*X(i-1,3)*X(i-1,4); M2*Lc2*sin(X(i-1,2))*(L1+Lc2+cos(X(i-1,2)))*X(i-1,3)^2];
        G = [0; M2*g*Lc2*cos(X(i-1,2))];
        
        % Control torques
        tau = [-K_p*(X(i-1,1)-td1)-K_v*X(i-1,3) (-K_p*(X(i-1,2)-td2)-K_v*X(i-1,4))];
        %tau = [0 0]; % DELETE
      
        %Apply joint torque limits
        tau(tau>tau_max) = tau_max;
        tau(tau<-tau_max) = -tau_max;
        
        tdd(i,:) = [inv(M) * (tau' - G - C)]';
        
        % Trapezoidal Integration
        X(i,3) = X(i-1,3) + 0.5*(tdd(i-1,1)+tdd(i,1))*dt; % v1
        X(i,4) = X(i-1,4) + 0.5*(tdd(i-1,2)+tdd(i,2))*dt; % v2
        
        X(i,1) = X(i-1,1) + 0.5*(X(i-1,3)+X(i,3))*dt; % p1
        X(i,2) = X(i-1,2) + 0.5*(X(i-1,4)+X(i,4))*dt; % p2
        
        % Computes Energy at each time-step
        KE = (X(i,3)^2*(M1*Lc1^2 + I1))/2 + (M2*(X(i,3)*(L1*cos(X(i,1)) +...
              Lc2*cos(X(i,1))*cos(X(i,2))) - Lc2*X(i,4)*sin(X(i,1))*sin(X(i,2)))^2)/2 +...
              (I2*X(i,4)^2)/2 + (M2*(X(i,3)*(L1*sin(X(i,1)) + Lc2*cos(X(i,2))*sin(X(i,1))) +...
              Lc2*X(i,4)*cos(X(i,1))*sin(X(i,2)))^2)/2 + (Lc2^2*M2*X(i,4)^2*cos(X(i,2))^2)/2;
          
        PE = M2*g*Lc2*sin(X(i,2));
        
        K(i) = KE;
        U(i) = PE;
   
    end
end


% States Plot
figure
plot(t,X(:,1)), hold on
plot(t,X(:,2))
legend('Theta 1','Theta 2')

figure
plot(t,X(:,3)), hold on
plot(t,X(:,4))
legend('Velocity 1','Velocity 2')
% Energy Plot
E = U + K;
figure
plot(t(2:1000),K(2:1000)), hold on
plot(t(2:1000),U(2:1000))
plot(t(2:1000),E(2:1000))
legend('Kinetic Energy','Potential Energy','Total Energy')


% Graphical Simulation
robot.handles = drawRR([X0(1) X0(2)],robot);
for i = 2:length(t)
    setRR([X(i,1) X(i,2)],robot);
    pause(1e-6); % adjustable pause in seconds
end

end