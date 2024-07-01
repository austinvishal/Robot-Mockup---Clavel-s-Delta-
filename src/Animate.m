function [] = Animate( angles,traj,parameters,wireframe_on )
%ANIMATE Make Plot animate
%simulation step size
N=size(angles,2);
dt=0.1; % time step

for i=1:1 
    tic
    plotRobot(traj(:,i),angles(:,i),parameters,wireframe_on);
    toc
    pause(dt);
end

end

