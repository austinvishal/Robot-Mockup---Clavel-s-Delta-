function [angles]= CalcTrajectoryAngles( traj,parameters )
% find inversekinematics for trajectory following
m=size(traj,2);
angles=zeros(3,m);

for i=1:m
    angles(:,i)=inverseKin([traj(1,i),traj(2,i),traj(3,i)],parameters);
end
end