% simulate delta

%% initialize
clc;
clear all;
% set(gcf, 'Renderer', 'OpenGL')
set(gcf, 'color', 'white','units','pixels','position',[0 0 1920 1080]);
% set(gcf, 'color', 'white');
% figure('units','pixels','position',[0 0 1920 1080]) %this is added to get 1080p resolution mp4 file
%% parameters
% length of rod in m
r_fixedlink =0.524; % base to joint center- link of fixed frame
r_endlink = 1.244 ; % joint to end -effector length

%% base frame side length in m - the shape is triangular
% the dimensions given are from ABB IRB360 robot
%Rodrigo Torres Arrazate “Development of a URDF file for simulation and programming of a delta robot using ROS”, Feb, 2017.
fixed_edge = 0.576 ; % base edge length
%  endeff_edge= 0.076; % end effector edge length
endeff_edge= 0.476; % end effector edge length
parallel_width= 0.131;
parallel_length=r_endlink;
parameters = [endeff_edge,fixed_edge,r_endlink,r_fixedlink,parallel_width,parallel_length];
wireframe_on=0;
%% call workspace fn to calculate
%  gif('deltarobot.gif','DelayTime',0.3)

 workspace(parameters);
 xlabel('x')
 ylabel('y')
 zlabel('z')
hold on
%% kinematics
% here a specific trajectory is being followed for pick and place
% application
% for the trajectory, trajectory angles must be calculated via calling
% inversekinematics of delta robot

%actuator angles
no_traj_points= 5;
no_of_poses = 4;

% start pose
r_start = [0, 0 , -1.116];
%goal pose
r_goal=zeros(no_of_poses+1,3);

%setting goal positions
r_goal(1,:)=[-0.06558, -0.05222, -1.339];
r_goal(2,:)=r_start';
r_goal(3,:)=[0.2315, -0.4401, -1.258];
r_goal(4,:)=r_start';

% initialise the trajectory data matrix
trajectory=zeros(3,no_traj_points,no_of_poses);
angles=zeros(3,no_traj_points,no_of_poses);

for i=1:no_of_poses
    trajectory(:,:,i)=CalcTrajectory(r_goal(i,:),r_goal(i+1,:),no_traj_points);
    angles(:,:,i)= CalcTrajectoryAngles(trajectory(:,:,i), parameters)*pi/180;
end
%hold on
%% plot the picking motion  for each carton
ncartons= 3; % define no. of packages the robot has to pick
% gif('deltarobot5.gif','DelayTime',0.3)
vidObj = VideoWriter('deltavid','MPEG-4');
vidObj.FrameRate = 3;
open(vidObj)

while(ncartons~=0)
    for i=1:no_of_poses
        
        Animate(angles(:,:,i),trajectory(:,:,i),parameters,wireframe_on);
%         pause(0.03)% 60 HZ;
%          gif('DelayTime',0.3);
        frame = getframe(gcf);
%         pause(1.0); %slow down frames or 
%         vidObj.FrameRate = 1;
        writeVideo(vidObj,frame)
        
    end
    ncartons =  ncartons-1;
end
writeVideo(vidObj,getframe(gcf))
hold on
% pause(0.03)% 60 HZ;
%  gif('DelayTime',0.3);
writeVideo(vidObj,getframe(gcf))
workspace(parameters);
writeVideo(vidObj,getframe(gcf))
close(gcf)
 close(vidObj)
%  gif('DelayTime',0.3);