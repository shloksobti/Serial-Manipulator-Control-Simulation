function [trajectory] = createRobTrajectory( via, rob )
% MECH 498/598 - Intro to Robotics - Spring 2016
% Lab 3
%
%    DESCRIPTION - Generate a joint position and velocity trajectory to be
%    used by the controller.
%
%    The input via is a 3x4 matrix containing via points for the end
%    effector. Each column contains a point in 3D space. The trajectory
%    should move sequentially through each via point. The best approach is
%    to simply move in a straight line in cartesian space between the
%    points. Use robIK() to find the corresponding trajectory in joint
%    space.
%
%    The output trajectory is a 7xn matrix. The first row will be
%    equally-spaced time stamps starting at time zero and finishing at time
%    t_f given to you below. Rows 2 through 4 contain joint angles,
%    and rows 5 7 should contain joint velocities.

% Shlok Sobti and Brian Ying

rob = robInit;
t_f = 30; % final time (do not change) [s]

point1 = via(:,1)';
point2 = via(:,2)';
t = 0:0.001:1;
C1 = repmat(point1,length(t),1)'+(point2-point1)'*t;

point2 = via(:,2)';
point3 = via(:,3)';
C2 = repmat(point2,length(t),1)'+(point3-point2)'*t;
C2 = C2(:,2:length(C2)-1);

point3 = via(:,3)';
point4 = via(:,4)';
C3 = repmat(point3,length(t),1)'+(point4-point3)'*t;

CartesianPath = [C1 C2 C3];

% % Define waypoints
% t = [0 10 20 30];
% points = [via(:,1)';via(:,2)';via(:,3)';via(:,4)'];
% x = points(:,1);
% y = points(:,2);
% z = points(:,3);
% 
% % Calculate spline for way points
% tq = 0:0.01:30;
% xq = interp1(t,x,tq,'spline');
% yq = interp1(t,y,tq,'spline');
% zq = interp1(t,z,tq,'spline');
% CartesianPath = [xq;yq;zq];

prev_joint_angles = [0 0 0];
joint_angles = [];
for i = 1:length(CartesianPath)
    [is, angles] = robIK( CartesianPath(:,i), prev_joint_angles, rob );
    joint_angles = [joint_angles;angles];
    prev_joint_angles = angles;
end
size(joint_angles);
trajectory(1,:) = [0:0.01:t_f]; %Time
trajectory(2:4,:) = joint_angles'; %Joint angles
trajectory(5:7,:) = zeros(3,length(CartesianPath)); %Joint velocities

end

