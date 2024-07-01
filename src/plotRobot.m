function [] = plotRobot( traj,t, parameters,wireframe_on)
%plot position, given Pose
%parameters contains: [endeff_edge,fixed_edge,r_endlink,r_fixedlink];
if wireframe_on ==1
    facecolor='none';
    edgecolor='k';
else
    facecolor='k';
    edgecolor='none';
end
% initialize
%coords
x=traj(1); y=traj(2); z=traj(3);
t1=t(1); t2=t(2); t3=t(3);
%rod lengths in m:
r_endlink = parameters(3); %parallelogram
r_fixedlink = parameters(4); %base arm

%triangular side lengths in m:
endeff_edge = parameters(1); %end effector 
fixed_edge = parameters(2); %base

%parallelogram details
parallel_width=parameters(5);
parallel_length=parameters(6);
%% plot
%init
grid off

% 0.13,0.11,0.775,0.815  0.1 0.4 0.3 0.6
% set(0,'DefaultFigureUnits', 'normalized', 'DefaultFigurePosition', [0.1 0.4 0.4 0.6])
%to adjust the view
l=(fixed_edge+2*r_fixedlink);
axis([-l l -l l -l*2 l/2])
cla;


%calc and plot robot f plate
P_f1=[fixed_edge/2,-fixed_edge/(2*sqrt(3)),0];
P_f2=[0,fixed_edge/cos(30*pi/180)/2,0];
P_f3=[-fixed_edge/2,-fixed_edge/(2*sqrt(3)),0];


line([P_f1(1) P_f2(1)],[P_f1(2) P_f2(2)],[P_f1(3) P_f2(3)],'LineWidth',1.2)
line([P_f2(1) P_f3(1)],[P_f2(2) P_f3(2)],[P_f2(3) P_f3(3)])
line([P_f3(1) P_f1(1)],[P_f3(2) P_f1(2)],[P_f3(3) P_f1(3)])

hold on

%calc and plot robot end-effector pose
P_e1=traj'+[endeff_edge/2,-endeff_edge/(2*sqrt(3)),0];
P_e2=traj'+[0,endeff_edge/cos(30*pi/180)/2,0];
P_e3=traj'+[-endeff_edge/2,-endeff_edge/(2*sqrt(3)),0];

line([P_e1(1) P_e2(1)],[P_e1(2) P_e2(2)],[P_e1(3) P_e2(3)])
line([P_e2(1) P_e3(1)],[P_e2(2) P_e3(2)],[P_e2(3) P_e3(3)])
line([P_e3(1) P_e1(1)],[P_e3(2) P_e1(2)],[P_e3(3) P_e1(3)])

hold on

plot3(x,y,z,'o','color','green')%plotting the end-effector point
%% for joints
deg=360;
R=[cosd(deg) -sind(deg) 0 0
    sind(deg) cosd(deg) 0 -fixed_edge/(2*sqrt(3))
    0 0 1 0
    0 0 0 1];
%calc and plot joints f
P_F1=[0,-fixed_edge/(2*sqrt(3)),0];
%joint 1
[x0,y0,z0] = cylinder(1,10);
% x0=x0*0.75;
% y0=y0*0.75;
x0=x0*0.05;
y0=y0*0.05;
%  z0 = z0*0.085;
z0 = z0*0.085;
x1 = []; y1 = []; z1 = []; % To Initialize New Arrays
for i = 1:2
    cylin = R*[z0(i,:)-0.085/2; y0(i,:); x0(i,:); ones(1,11)];
% To Add 41 Ones
    x1 = [x1; cylin(1,:)];
    y1 = [y1; cylin(2,:)];
    z1 = [z1; cylin(3,:)];
end
surf(x1,y1,z1,'FaceColor',facecolor,'Facealpha',0.6, ...
'EdgeColor',edgecolor,'AmbientStrength',0.6)
%  camlight
%   light('Position',[2 10 10],'Style','infinite')
%   lighting gouraud
patch('Vertices',[x1(1,:)' y1(1,:)' z1(1,:)'], ...
'Faces',[1:10],'FaceColor',facecolor, ...
'EdgeColor',edgecolor,'AmbientStrength',0.6)
patch('Vertices',[x1(2,:)' y1(2,:)' z1(2,:)'], ...
'Faces',[1:10],'FaceColor',facecolor, ...
'EdgeColor',edgecolor,'AmbientStrength',0.6)

%use rotation matrix to calc other points
deg=120;
 R=[cosd(deg),-sind(deg),0;sind(deg),cosd(deg),0;0,0,1];
P_F2=(R*P_F1')';
R_1=[cosd(deg) -sind(deg) 0 fixed_edge/4
    sind(deg) cosd(deg) 0 (3^(1/2)*fixed_edge)/12
    0 0 1 0
    0 0 0 1];

[x0,y0,z0] = cylinder(1,10);
% x0=x0*0.75;
% y0=y0*0.75;
x0=x0*0.05;
y0=y0*0.05;
 z0 = z0*0.085;
x1 = []; y1 = []; z1 = []; % To Initialize New Arrays
for i = 1:2
    cylin =R_1*[z0(i,:)-0.085/2; y0(i,:); x0(i,:); ones(1,11)];
% To Add 41 Ones
    x1 = [x1; cylin(1,:)];
    y1 = [y1; cylin(2,:)];
    z1 = [z1; cylin(3,:)];
end
surf(x1,y1,z1,'FaceColor',facecolor,'Facealpha',0.6, ...
'EdgeColor',edgecolor,'AmbientStrength',0.6)
%  camlight
%   light('Position',[2 10 10],'Style','infinite')
%   lighting gouraud
patch('Vertices',[x1(1,:)' y1(1,:)' z1(1,:)'], ...
'Faces',[1:10],'FaceColor',facecolor, ...
'EdgeColor',edgecolor,'AmbientStrength',0.6)
patch('Vertices',[x1(2,:)' y1(2,:)' z1(2,:)'], ...
'Faces',[1:10],'FaceColor',facecolor, ...
'EdgeColor',edgecolor,'AmbientStrength',0.6)

deg=240;
P_F3=(R*P_F2')';
R_2=[cosd(deg) -sind(deg) 0 -fixed_edge/4
    sind(deg) cosd(deg) 0 (3^(1/2)*fixed_edge)/12
    0 0 1 0
    0 0 0 1];

[x0,y0,z0] = cylinder(1,10);
% x0=x0*0.75;
% y0=y0*0.75;
x0=x0*0.05;
y0=y0*0.05;
 z0 = z0*0.085;
x1 = []; y1 = []; z1 = []; % To Initialize New Arrays
for i = 1:2
    cylin =R_2*[z0(i,:)-0.085/2; y0(i,:); x0(i,:); ones(1,11)];
% To Add 41 Ones
    x1 = [x1; cylin(1,:)];
    y1 = [y1; cylin(2,:)];
    z1 = [z1; cylin(3,:)];
end
surf(x1,y1,z1,'FaceColor',facecolor,'Facealpha',0.6, ...
'EdgeColor',edgecolor,'AmbientStrength',0.6)
% light('Position',[1 -0.2 1],'Style','infinite')
 lightangle(gca,-45,30)
  camlight
%   light('Position',[2 10 10],'Style','infinite')
%   lighting gouraud
patch('Vertices',[x1(1,:)' y1(1,:)' z1(1,:)'], ...
'Faces',[1:10],'FaceColor',facecolor, ...
'EdgeColor',edgecolor,'AmbientStrength',0.6)
patch('Vertices',[x1(2,:)' y1(2,:)' z1(2,:)'], ...
'Faces',[1:10],'FaceColor',facecolor, ...
'EdgeColor',edgecolor,'AmbientStrength',0.6)
%Plot F1,F2,F3
hold on
% plot3(P_F1(1),P_F1(2),P_F1(3),'o','color','green')
% hold on
% plot3(P_F2(1),P_F2(2),P_F2(3),'o','color','green')
% hold on
% plot3(P_F3(1),P_F3(2),P_F3(3),'o','color','green')
% hold on
%%
P_J1=[0,-fixed_edge/(2*sqrt(3))-r_fixedlink*cos(t1),-r_fixedlink*sin(t1)];
deg=360;
Rj1=[cosd(deg) -sind(deg) 0 0
    sind(deg) cosd(deg) 0 -fixed_edge/(2*sqrt(3))-r_fixedlink*cos(t1)
    0 0 1 -r_fixedlink*sin(t1)
    0 0 0 1];
[x0,y0,z0] = cylinder(1,10);
% x0=x0*0.75;
% y0=y0*0.75;
x0=x0*0.05;
y0=y0*0.05;
%  z0 = z0*0.085;
z0 = z0*0.085;
x1 = []; y1 = []; z1 = []; % To Initialize New Arrays
for i = 1:2
    cylin = Rj1*[z0(i,:)-0.085/2; y0(i,:); x0(i,:); ones(1,11)];
% To Add 41 Ones
    x1 = [x1; cylin(1,:)];
    y1 = [y1; cylin(2,:)];
    z1 = [z1; cylin(3,:)];
end
surf(x1,y1,z1,'FaceColor',facecolor,'Facealpha',0.6, ...
'EdgeColor',edgecolor,'AmbientStrength',0.6)
%  camlight
%   light('Position',[2 10 10],'Style','infinite')
%   lighting gouraud
patch('Vertices',[x1(1,:)' y1(1,:)' z1(1,:)'], ...
'Faces',[1:10],'FaceColor',facecolor, ...
'EdgeColor',edgecolor,'AmbientStrength',0.6)
patch('Vertices',[x1(2,:)' y1(2,:)' z1(2,:)'], ...
'Faces',[1:10],'FaceColor',facecolor, ...
'EdgeColor',edgecolor,'AmbientStrength',0.6)
%%
P_J2=(R*[0,-fixed_edge/(2*sqrt(3))-r_fixedlink*cos(t2),-r_fixedlink*sin(t2)]')';
deg=120;
Rj2=[cosd(deg) -sind(deg) 0 P_J2(1)
    sind(deg) cosd(deg) 0 P_J2(2)
    0 0 1 P_J2(3)
    0 0 0 1];

[x0,y0,z0] = cylinder(1,10);
% x0=x0*0.75;
% y0=y0*0.75;
x0=x0*0.05;
y0=y0*0.05;
 z0 = z0*0.085;
x1 = []; y1 = []; z1 = []; % To Initialize New Arrays
for i = 1:2
    cylin =Rj2*[z0(i,:)-0.085/2; y0(i,:); x0(i,:); ones(1,11)];
% To Add 41 Ones
    x1 = [x1; cylin(1,:)];
    y1 = [y1; cylin(2,:)];
    z1 = [z1; cylin(3,:)];
end
surf(x1,y1,z1,'FaceColor',facecolor,'Facealpha',0.6, ...
'EdgeColor',edgecolor,'AmbientStrength',0.6)
%  camlight
%   light('Position',[2 10 10],'Style','infinite')
%   lighting gouraud
patch('Vertices',[x1(1,:)' y1(1,:)' z1(1,:)'], ...
'Faces',[1:10],'FaceColor',facecolor, ...
'EdgeColor',edgecolor,'AmbientStrength',0.6)
patch('Vertices',[x1(2,:)' y1(2,:)' z1(2,:)'], ...
'Faces',[1:10],'FaceColor',facecolor, ...
'EdgeColor',edgecolor,'AmbientStrength',0.6)


deg=-120;
R=[cosd(deg),-sind(deg),0;sind(deg),cosd(deg),0;0,0,1];
P_J3=(R*[0,-fixed_edge/(2*sqrt(3))-r_fixedlink*cos(t3),-r_fixedlink*sin(t3)]')';
deg=240;
Rj3=[cosd(deg) -sind(deg) 0 P_J3(1)
    sind(deg) cosd(deg) 0 P_J3(2)
    0 0 1 P_J3(3)
    0 0 0 1];

[x0,y0,z0] = cylinder(1,10);
% x0=x0*0.75;
% y0=y0*0.75;
x0=x0*0.05;
y0=y0*0.05;
 z0 = z0*0.085;
x1 = []; y1 = []; z1 = []; % To Initialize New Arrays
for i = 1:2
    cylin =Rj3*[z0(i,:)-0.085/2; y0(i,:); x0(i,:); ones(1,11)];
% To Add 41 Ones
    x1 = [x1; cylin(1,:)];
    y1 = [y1; cylin(2,:)];
    z1 = [z1; cylin(3,:)];
end
surf(x1,y1,z1,'FaceColor',facecolor,'Facealpha',0.6, ...
'EdgeColor',edgecolor,'AmbientStrength',0.6)
%  camlight
%   light('Position',[2 10 10],'Style','infinite')
%   lighting gouraud
patch('Vertices',[x1(1,:)' y1(1,:)' z1(1,:)'], ...
'Faces',[1:10],'FaceColor',facecolor, ...
'EdgeColor',edgecolor,'AmbientStrength',0.6)
patch('Vertices',[x1(2,:)' y1(2,:)' z1(2,:)'], ...
'Faces',[1:10],'FaceColor',facecolor, ...
'EdgeColor',edgecolor,'AmbientStrength',0.6)
%% link plot
% deg=360;
% Ru=[cosd(deg) -sind(deg) 0 0
%     sind(deg) cosd(deg) 0 -fixed_edge/(2*sqrt(3))
%     0 0 1 0
%     0 0 0 1];
 [x0,y0,z0] = cylindertwopoints(0.03,10,[P_F1(1),P_F1(2),P_F1(3)],[P_J1(1),P_J1(2),P_J1(3)]);
%  h = hgtransform;
 % [x0,y0,z0] = cylinder(1,10);
% x0=(x0*0.03);
% y0=(y0*0.03);
%  z0 = (-z0*2);
%  x1 = []; y1 = []; z1 = []; % To Initialize New Arrays
% for i = 1:2
%     cylin =Ru*[y0(i,:)+0.03/2; z0(i,:); x0(i,:); ones(1,11)];
% % To Add 41 Ones
%     x1 = [x1; cylin(1,:)];
%     y1 = [y1; cylin(2,:)];
%     z1 = [z1; cylin(3,:)];
% end
% surf(x1,y1,z1,'FaceColor','k','Facealpha',0.6, ...
% 'EdgeColor','none','AmbientStrength',0.6)
% patch('Vertices',[x1(1,:)' y1(1,:)' z1(1,:)'], ...
% 'Faces',[1:10],'FaceColor','k', ...
% 'EdgeColor','none','AmbientStrength',0.6)
% patch('Vertices',[x1(2,:)' y1(2,:)' z1(2,:)'], ...
% 'Faces',[1:10],'FaceColor','k', ...
% 'EdgeColor','none','AmbientStrength',0.6)
surf(x0,y0,z0,'FaceColor','r','Facealpha',0.6, ...
'EdgeColor','none','AmbientStrength',0.6)
patch('Vertices',[x0(1,:)' y0(1,:)' z0(1,:)'], ...
'Faces',[1:10],'FaceColor','r', ...
'EdgeColor','none','AmbientStrength',0.6)
patch('Vertices',[x0(2,:)' y0(2,:)' z0(2,:)'], ...
'Faces',[1:10],'FaceColor','r', ...
'EdgeColor','none','AmbientStrength',0.6)
% % Make it taller
% set(h, 'Matrix', makehgtform('scale', [0.01 0.01 1]))

%link 2
 [x0,y0,z0] = cylindertwopoints(0.03,10,[P_F2(1),P_F2(2),P_F2(3)],[P_J2(1),P_J2(2),P_J2(3)]);
surf(x0,y0,z0,'FaceColor','r','Facealpha',0.6, ...
'EdgeColor','none','AmbientStrength',0.6)
patch('Vertices',[x0(1,:)' y0(1,:)' z0(1,:)'], ...
'Faces',[1:10],'FaceColor','r', ...
'EdgeColor','none','AmbientStrength',0.6)
patch('Vertices',[x0(2,:)' y0(2,:)' z0(2,:)'], ...
'Faces',[1:10],'FaceColor','r', ...
'EdgeColor','none','AmbientStrength',0.6)

%link 3
 [x0,y0,z0] = cylindertwopoints(0.03,10,[P_F3(1),P_F3(2),P_F3(3)],[P_J3(1),P_J3(2),P_J3(3)]);
surf(x0,y0,z0,'FaceColor','r','Facealpha',0.6, ...
'EdgeColor','none','AmbientStrength',0.6)
patch('Vertices',[x0(1,:)' y0(1,:)' z0(1,:)'], ...
'Faces',[1:10],'FaceColor','r', ...
'EdgeColor','none','AmbientStrength',0.6)
patch('Vertices',[x0(2,:)' y0(2,:)' z0(2,:)'], ...
'Faces',[1:10],'FaceColor','r', ...
'EdgeColor','none','AmbientStrength',0.6)

%r_fixedlink link plot
%  line([P_F1(1) P_J1(1)],[P_F1(2) P_J1(2)],[P_F1(3) P_J1(3)],'color','red')
% line([P_F2(1) P_J2(1)],[P_F2(2) P_J2(2)],[P_F2(3) P_J2(3)],'color','red')
% line([P_F3(1) P_J3(1)],[P_F3(2) P_J3(2)],[P_F3(3) P_J3(3)],'color','red')
%plot J1, J2, J3
hold on
% plot3(P_J1(1),P_J1(2),P_J1(3),'o','color','green')
% hold on
% plot3(P_J2(1),P_J2(2),P_J2(3),'o','color','green')
% hold on
% plot3(P_J3(1),P_J3(2),P_J3(3),'o','color','green')
% hold on

%% calc and plot joint e
Transl=[0,-endeff_edge/(2*sqrt(3)),0];
P_E1=Transl+[x,y,z];

deg=360;
Re1=[cosd(deg) -sind(deg) 0 P_E1(1)
    sind(deg) cosd(deg) 0 P_E1(2)
    0 0 1 P_E1(3)
    0 0 0 1];
[x0,y0,z0] = cylinder(1,10);
% x0=x0*0.75;
% y0=y0*0.75;
% x0=x0*0.02;
x0=x0*0.05;
y0=y0*0.05;
%  z0 = z0*0.085;
z0 = z0*0.05;
x1 = []; y1 = []; z1 = []; % To Initialize New Arrays
for i = 1:2
    cylin = Re1*[z0(i,:)-0.02/2; y0(i,:); x0(i,:); ones(1,11)];
% To Add 41 Ones
    x1 = [x1; cylin(1,:)];
    y1 = [y1; cylin(2,:)];
    z1 = [z1; cylin(3,:)];
end
surf(x1,y1,z1,'FaceColor',facecolor,'Facealpha',0.6, ...
'EdgeColor',edgecolor,'AmbientStrength',0.6)
patch('Vertices',[x1(1,:)' y1(1,:)' z1(1,:)'], ...
'Faces',[1:10],'FaceColor',facecolor, ...
'EdgeColor',edgecolor,'AmbientStrength',0.6)
patch('Vertices',[x1(2,:)' y1(2,:)' z1(2,:)'], ...
'Faces',[1:10],'FaceColor',facecolor, ...
'EdgeColor',edgecolor,'AmbientStrength',0.6)


deg=120;
R=[cosd(deg),-sind(deg),0;sind(deg),cosd(deg),0;0,0,1];
P_E2=((R*Transl')+traj)';
Re2=[cosd(deg) -sind(deg) 0 P_E2(1)
    sind(deg) cosd(deg) 0 P_E2(2)
    0 0 1 P_E2(3)
    0 0 0 1];
[x0,y0,z0] = cylinder(1,10);
% x0=x0*0.75;
% y0=y0*0.75;
x0=x0*0.05;
y0=y0*0.05;
%  z0 = z0*0.085;
z0 = z0*0.05;
x1 = []; y1 = []; z1 = []; % To Initialize New Arrays
for i = 1:2
    cylin = Re2*[z0(i,:)-0.02/2; y0(i,:); x0(i,:); ones(1,11)];
% To Add 41 Ones
    x1 = [x1; cylin(1,:)];
    y1 = [y1; cylin(2,:)];
    z1 = [z1; cylin(3,:)];
end
surf(x1,y1,z1,'FaceColor',facecolor,'Facealpha',0.6, ...
'EdgeColor',edgecolor,'AmbientStrength',0.6)
patch('Vertices',[x1(1,:)' y1(1,:)' z1(1,:)'], ...
'Faces',[1:10],'FaceColor',facecolor, ...
'EdgeColor',edgecolor,'AmbientStrength',0.6)
patch('Vertices',[x1(2,:)' y1(2,:)' z1(2,:)'], ...
'Faces',[1:10],'FaceColor',facecolor, ...
'EdgeColor',edgecolor,'AmbientStrength',0.6)

deg=-120;
R=[cosd(deg),-sind(deg),0;sind(deg),cosd(deg),0;0,0,1];
P_E3=((R*Transl')+traj)';
Re3=[cosd(deg) -sind(deg) 0 P_E3(1)
    sind(deg) cosd(deg) 0 P_E3(2)
    0 0 1 P_E3(3)
    0 0 0 1];
[x0,y0,z0] = cylinder(1,10);
% x0=x0*0.75;
% y0=y0*0.75;
x0=x0*0.05;
y0=y0*0.05;
%  z0 = z0*0.085;
z0 = z0*0.05;
x1 = []; y1 = []; z1 = []; % To Initialize New Arrays
for i = 1:2
    cylin = Re3*[z0(i,:)-0.02/2; y0(i,:); x0(i,:); ones(1,11)];
% To Add 41 Ones
    x1 = [x1; cylin(1,:)];
    y1 = [y1; cylin(2,:)];
    z1 = [z1; cylin(3,:)];
end
surf(x1,y1,z1,'FaceColor',facecolor,'Facealpha',0.6, ...
'EdgeColor',edgecolor,'AmbientStrength',0.6)
patch('Vertices',[x1(1,:)' y1(1,:)' z1(1,:)'], ...
'Faces',[1:10],'FaceColor',facecolor, ...
'EdgeColor',edgecolor,'AmbientStrength',0.6)
patch('Vertices',[x1(2,:)' y1(2,:)' z1(2,:)'], ...
'Faces',[1:10],'FaceColor',facecolor, ...
'EdgeColor',edgecolor,'AmbientStrength',0.6)

%% re link plot

% %link 1
%  [x0,y0,z0] = cylindertwopoints(0.03,10,[P_J1(1),P_J1(2),P_J1(3)],[P_E1(1),P_E1(2),P_E1(3)]);
% surf(x0,y0,z0,'FaceColor',rgb('Coral'),'Facealpha',0.6, ...
% 'EdgeColor','none','AmbientStrength',0.6)
% patch('Vertices',[x0(1,:)' y0(1,:)' z0(1,:)'], ...
% 'Faces',[1:10],'FaceColor',rgb('Coral'), ...
% 'EdgeColor','none','AmbientStrength',0.6)
% patch('Vertices',[x0(2,:)' y0(2,:)' z0(2,:)'], ...
% 'Faces',[1:10],'FaceColor',rgb('Coral'), ...
% 'EdgeColor','none','AmbientStrength',0.6)
%link 2
%  [x0,y0,z0] = cylindertwopoints(0.03,10,[P_J2(1),P_J2(2),P_J2(3)],[P_E2(1),P_E2(2),P_E2(3)]);
% surf(x0,y0,z0,'FaceColor',rgb('Coral'),'Facealpha',0.6, ...
% 'EdgeColor','none','AmbientStrength',0.6)
% patch('Vertices',[x0(1,:)' y0(1,:)' z0(1,:)'], ...
% 'Faces',[1:10],'FaceColor',rgb('Coral'), ...
% 'EdgeColor','none','AmbientStrength',0.6)
% patch('Vertices',[x0(2,:)' y0(2,:)' z0(2,:)'], ...
% 'Faces',[1:10],'FaceColor',rgb('Coral'), ...
% 'EdgeColor','none','AmbientStrength',0.6)
%link 3
%  [x0,y0,z0] = cylindertwopoints(0.03,10,[P_J3(1),P_J3(2),P_J3(3)],[P_E3(1),P_E3(2),P_E3(3)]);
% surf(x0,y0,z0,'FaceColor',rgb('Coral'),'Facealpha',0.6, ...
% 'EdgeColor','none','AmbientStrength',0.6)
% patch('Vertices',[x0(1,:)' y0(1,:)' z0(1,:)'], ...
% 'Faces',[1:10],'FaceColor',rgb('Coral'), ...
% 'EdgeColor','none','AmbientStrength',0.6)
% patch('Vertices',[x0(2,:)' y0(2,:)' z0(2,:)'], ...
% 'Faces',[1:10],'FaceColor',rgb('Coral'), ...
% 'EdgeColor','none','AmbientStrength',0.6)

% line([P_J1(1) P_E1(1)],[P_J1(2) P_E1(2)],[P_J1(3) P_E1(3)],'color','red')
% line([P_J2(1) P_E2(1)],[P_J2(2) P_E2(2)],[P_J2(3) P_E2(3)],'color','red')
% line([P_J3(1) P_E3(1)],[P_J3(2) P_E3(2)],[P_J3(3) P_E3(3)],'color','red')

%% plot the parallelogram
deg=360;
Rj1=[cosd(deg) -sind(deg) 0 0
    sind(deg) cosd(deg) 0 -fixed_edge/(2*sqrt(3))-r_fixedlink*cos(t1)
    0 0 1 -r_fixedlink*sin(t1)
    0 0 0 1];
[x0,y0,z0] = cylinder(0.6,10);
% x0=x0*0.75;
% y0=y0*0.75;
x0=x0*0.05;
y0=y0*0.05;
%  z0 = z0*0.085;
% z0 = z0*0.085;
z0 = z0*parallel_width*2;
x1 = []; y1 = []; z1 = []; % To Initialize New Arrays
for i = 1:2
    cylin = Rj1*[z0(i,:)-parallel_width; y0(i,:); x0(i,:); ones(1,11)];
% To Add 41 Ones
    x1 = [x1; cylin(1,:)];
    y1 = [y1; cylin(2,:)];
    z1 = [z1; cylin(3,:)];
end
surf(x1,y1,z1,'FaceColor',rgb('DarkRed'),'Facealpha',1, ...
'EdgeColor','none','AmbientStrength',0.6)
patch('Vertices',[x1(1,:)' y1(1,:)' z1(1,:)'], ...
'Faces',[1:10],'FaceColor',rgb('DarkRed'), ...
'EdgeColor','none','AmbientStrength',0.6)
patch('Vertices',[x1(2,:)' y1(2,:)' z1(2,:)'], ...
'Faces',[1:10],'FaceColor',rgb('DarkRed'), ...
'EdgeColor','none','AmbientStrength',0.6)

deg=120;
Rj2=[cosd(deg) -sind(deg) 0 P_J2(1)
    sind(deg) cosd(deg) 0 P_J2(2)
    0 0 1 P_J2(3)
    0 0 0 1];

[x0,y0,z0] = cylinder(0.6,10);
% x0=x0*0.75;
% y0=y0*0.75;
x0=x0*0.05;
y0=y0*0.05;
 z0 = z0*parallel_width*2;
x1 = []; y1 = []; z1 = []; % To Initialize New Arrays
for i = 1:2
    cylin =Rj2*[z0(i,:)-parallel_width; y0(i,:); x0(i,:); ones(1,11)];
% To Add 41 Ones
    x1 = [x1; cylin(1,:)];
    y1 = [y1; cylin(2,:)];
    z1 = [z1; cylin(3,:)];
end
surf(x1,y1,z1,'FaceColor',rgb('DarkRed'),'Facealpha',0.6, ...
'EdgeColor','none','AmbientStrength',0.6)
patch('Vertices',[x1(1,:)' y1(1,:)' z1(1,:)'], ...
'Faces',[1:10],'FaceColor',rgb('DarkRed'), ...
'EdgeColor','none','AmbientStrength',0.6)
patch('Vertices',[x1(2,:)' y1(2,:)' z1(2,:)'], ...
'Faces',[1:10],'FaceColor',rgb('DarkRed'), ...
'EdgeColor','none','AmbientStrength',0.6)


deg=-120;
R=[cosd(deg),-sind(deg),0;sind(deg),cosd(deg),0;0,0,1];
% P_J3=(R*[0,-fixed_edge/(2*sqrt(3))-r_fixedlink*cos(t3),-r_fixedlink*sin(t3)]')';
deg=240;
Rj3=[cosd(deg) -sind(deg) 0 P_J3(1)
    sind(deg) cosd(deg) 0 P_J3(2)
    0 0 1 P_J3(3)
    0 0 0 1];

[x0,y0,z0] = cylinder(0.6,10);
% x0=x0*0.75;
% y0=y0*0.75;
x0=x0*0.05;
y0=y0*0.05;
 z0 = z0*parallel_width*2;
x1 = []; y1 = []; z1 = []; % To Initialize New Arrays
for i = 1:2
    cylin =Rj3*[z0(i,:)-parallel_width; y0(i,:); x0(i,:); ones(1,11)];
% To Add 41 Ones
    x1 = [x1; cylin(1,:)];
    y1 = [y1; cylin(2,:)];
    z1 = [z1; cylin(3,:)];
end
surf(x1,y1,z1,'FaceColor',rgb('DarkRed'),'Facealpha',0.6, ...
'EdgeColor','none','AmbientStrength',0.6)
patch('Vertices',[x1(1,:)' y1(1,:)' z1(1,:)'], ...
'Faces',[1:10],'FaceColor',rgb('DarkRed'), ...
'EdgeColor','none','AmbientStrength',0.6)
patch('Vertices',[x1(2,:)' y1(2,:)' z1(2,:)'], ...
'Faces',[1:10],'FaceColor',rgb('DarkRed'), ...
'EdgeColor','none','AmbientStrength',0.6)

%% plot parallelogram joints 1
deg=0;
 R=[cosd(deg),-sind(deg),0;sind(deg),cosd(deg),0;0,0,1];
P_J1hn=(R*[-parallel_width,-fixed_edge/(2*sqrt(3))-r_fixedlink*cos(t1),-r_fixedlink*sin(t1)]')';
P_J1hp=(R*[+parallel_width,-fixed_edge/(2*sqrt(3))-r_fixedlink*cos(t1),-r_fixedlink*sin(t1)]')';
deg=-90;
Rjp1=[cosd(deg) -sind(deg) 0 P_J1hn(1)
    sind(deg) cosd(deg) 0 P_J1hn(2)
    0 0 1 P_J1hn(3)
    0 0 0 1];
[x0,y0,z0] = cylinder(1,10);
% x0=x0*0.75;
% y0=y0*0.75;
x0=x0*0.05;
y0=y0*0.05;
%  z0 = z0*0.085;
z0 = z0*0.085-0.04;
x1 = []; y1 = []; z1 = []; % To Initialize New Arrays
for i = 1:2
    cylin = Rjp1*[z0(i,:); y0(i,:); x0(i,:); ones(1,11)];
% To Add 41 Ones
    x1 = [x1; cylin(1,:)];
    y1 = [y1; cylin(2,:)];
    z1 = [z1; cylin(3,:)];
end
surf(x1,y1,z1,'FaceColor',facecolor,'Facealpha',0.6, ...
'EdgeColor',edgecolor,'AmbientStrength',0.6)
patch('Vertices',[x1(1,:)' y1(1,:)' z1(1,:)'], ...
'Faces',[1:10],'FaceColor',facecolor, ...
'EdgeColor',edgecolor,'AmbientStrength',0.6)
patch('Vertices',[x1(2,:)' y1(2,:)' z1(2,:)'], ...
'Faces',[1:10],'FaceColor',facecolor, ...
'EdgeColor',edgecolor,'AmbientStrength',0.6)

Rjp2=[cosd(deg) -sind(deg) 0 P_J1hp(1)
    sind(deg) cosd(deg) 0 P_J1hp(2)
    0 0 1 P_J1hp(3)
    0 0 0 1];
[x0,y0,z0] = cylinder(1,10);
% x0=x0*0.75;
% y0=y0*0.75;
x0=x0*0.05;
y0=y0*0.05;
%  z0 = z0*0.085;
z0 = z0*0.085 -0.04;
x1 = []; y1 = []; z1 = []; % To Initialize New Arrays
for i = 1:2
    cylin = Rjp2*[z0(i,:); y0(i,:); x0(i,:); ones(1,11)];
% To Add 41 Ones
    x1 = [x1; cylin(1,:)];
    y1 = [y1; cylin(2,:)];
    z1 = [z1; cylin(3,:)];
end
surf(x1,y1,z1,'FaceColor',facecolor,'Facealpha',0.6, ...
'EdgeColor',edgecolor,'AmbientStrength',0.6)
patch('Vertices',[x1(1,:)' y1(1,:)' z1(1,:)'], ...
'Faces',[1:10],'FaceColor',facecolor, ...
'EdgeColor',edgecolor,'AmbientStrength',0.6)
patch('Vertices',[x1(2,:)' y1(2,:)' z1(2,:)'], ...
'Faces',[1:10],'FaceColor',facecolor, ...
'EdgeColor',edgecolor,'AmbientStrength',0.6)

%% plot parallelogram joints 2
deg=120;
 R=[cosd(deg),-sind(deg),0;sind(deg),cosd(deg),0;0,0,1];
 P_J2hn=(R*[-parallel_width,-fixed_edge/(2*sqrt(3))-r_fixedlink*cos(t2),-r_fixedlink*sin(t2)]')';
 P_J2hp=(R*[parallel_width,-fixed_edge/(2*sqrt(3))-r_fixedlink*cos(t2),-r_fixedlink*sin(t2)]')';
deg=120+90;
Rjp21=[cosd(deg) -sind(deg) 0 P_J2hn(1)
    sind(deg) cosd(deg) 0 P_J2hn(2)
    0 0 1 P_J2hn(3)
    0 0 0 1];
[x0,y0,z0] = cylinder(1,10);
% x0=x0*0.75;
% y0=y0*0.75;
x0=x0*0.05;
y0=y0*0.05;
%  z0 = z0*0.085;
z0 = z0*0.085-0.04;  % -0.04 is to keep the joint at the center
x1 = []; y1 = []; z1 = []; % To Initialize New Arrays
for i = 1:2
    cylin = Rjp21*[z0(i,:); y0(i,:); x0(i,:); ones(1,11)];
% To Add 41 Ones
    x1 = [x1; cylin(1,:)];
    y1 = [y1; cylin(2,:)];
    z1 = [z1; cylin(3,:)];
end
surf(x1,y1,z1,'FaceColor',facecolor,'Facealpha',0.6, ...
'EdgeColor',edgecolor,'AmbientStrength',0.6)
patch('Vertices',[x1(1,:)' y1(1,:)' z1(1,:)'], ...
'Faces',[1:10],'FaceColor',facecolor, ...
'EdgeColor',edgecolor,'AmbientStrength',0.6)
patch('Vertices',[x1(2,:)' y1(2,:)' z1(2,:)'], ...
'Faces',[1:10],'FaceColor',facecolor, ...
'EdgeColor',edgecolor,'AmbientStrength',0.6)

Rjp22=[cosd(deg) -sind(deg) 0 P_J2hp(1)
    sind(deg) cosd(deg) 0 P_J2hp(2)
    0 0 1 P_J2hp(3)
    0 0 0 1];
[x0,y0,z0] = cylinder(1,10);
% x0=x0*0.75;
% y0=y0*0.75;
x0=x0*0.05;
y0=y0*0.05;
%  z0 = z0*0.085;
z0 = z0*0.085-0.04;
x1 = []; y1 = []; z1 = []; % To Initialize New Arrays
for i = 1:2
    cylin = Rjp22*[z0(i,:); y0(i,:); x0(i,:); ones(1,11)];
% To Add 41 Ones
    x1 = [x1; cylin(1,:)];
    y1 = [y1; cylin(2,:)];
    z1 = [z1; cylin(3,:)];
end
surf(x1,y1,z1,'FaceColor',facecolor,'Facealpha',0.6, ...
'EdgeColor',edgecolor,'AmbientStrength',0.6)
patch('Vertices',[x1(1,:)' y1(1,:)' z1(1,:)'], ...
'Faces',[1:10],'FaceColor',facecolor, ...
'EdgeColor',edgecolor,'AmbientStrength',0.6)
patch('Vertices',[x1(2,:)' y1(2,:)' z1(2,:)'], ...
'Faces',[1:10],'FaceColor',facecolor, ...
'EdgeColor',edgecolor,'AmbientStrength',0.6)


% P_J3=(R*[0,-fixed_edge/(2*sqrt(3))-r_fixedlink*cos(t3),-r_fixedlink*sin(t3)]')';
%% plot parallelogram joints 3
deg=240;
 R=[cosd(deg),-sind(deg),0;sind(deg),cosd(deg),0;0,0,1];
 P_J3hn=(R*[-parallel_width,-fixed_edge/(2*sqrt(3))-r_fixedlink*cos(t3),-r_fixedlink*sin(t3)]')';
 P_J3hp=(R*[parallel_width,-fixed_edge/(2*sqrt(3))-r_fixedlink*cos(t3),-r_fixedlink*sin(t3)]')';
deg=240+90;
Rjp31=[cosd(deg) -sind(deg) 0 P_J3hn(1)
    sind(deg) cosd(deg) 0 P_J3hn(2)
    0 0 1 P_J3hn(3)
    0 0 0 1];
[x0,y0,z0] = cylinder(1,10);
% x0=x0*0.75;
% y0=y0*0.75;
x0=x0*0.05;
y0=y0*0.05;
%  z0 = z0*0.085;
z0 = z0*0.085-0.04;
x1 = []; y1 = []; z1 = []; % To Initialize New Arrays
for i = 1:2
    cylin = Rjp31*[z0(i,:); y0(i,:); x0(i,:); ones(1,11)];
% To Add 41 Ones
    x1 = [x1; cylin(1,:)];
    y1 = [y1; cylin(2,:)];
    z1 = [z1; cylin(3,:)];
end
surf(x1,y1,z1,'FaceColor',facecolor,'Facealpha',0.6, ...
'EdgeColor',edgecolor,'AmbientStrength',0.6)
patch('Vertices',[x1(1,:)' y1(1,:)' z1(1,:)'], ...
'Faces',[1:10],'FaceColor',facecolor, ...
'EdgeColor',edgecolor,'AmbientStrength',0.6)
patch('Vertices',[x1(2,:)' y1(2,:)' z1(2,:)'], ...
'Faces',[1:10],'FaceColor',facecolor, ...
'EdgeColor',edgecolor,'AmbientStrength',0.6)

Rjp32=[cosd(deg) -sind(deg) 0 P_J3hp(1)
    sind(deg) cosd(deg) 0 P_J3hp(2)
    0 0 1 P_J3hp(3)
    0 0 0 1];
[x0,y0,z0] = cylinder(1,10);
% x0=x0*0.75;
% y0=y0*0.75;
x0=x0*0.05;
y0=y0*0.05;
%  z0 = z0*0.085;
z0 = z0*0.085-0.04;
x1 = []; y1 = []; z1 = []; % To Initialize New Arrays
for i = 1:2
    cylin = Rjp32*[z0(i,:); y0(i,:); x0(i,:); ones(1,11)];
% To Add 41 Ones
    x1 = [x1; cylin(1,:)];
    y1 = [y1; cylin(2,:)];
    z1 = [z1; cylin(3,:)];
end
surf(x1,y1,z1,'FaceColor',facecolor,'Facealpha',0.6, ...
'EdgeColor',edgecolor,'AmbientStrength',0.6)
patch('Vertices',[x1(1,:)' y1(1,:)' z1(1,:)'], ...
'Faces',[1:10],'FaceColor',facecolor, ...
'EdgeColor',edgecolor,'AmbientStrength',0.6)
patch('Vertices',[x1(2,:)' y1(2,:)' z1(2,:)'], ...
'Faces',[1:10],'FaceColor',facecolor, ...
'EdgeColor',edgecolor,'AmbientStrength',0.6)


%% plot base parallelogram width link

deg=360;
Re1=[cosd(deg) -sind(deg) 0 P_E1(1)
    sind(deg) cosd(deg) 0 P_E1(2)
    0 0 1 P_E1(3)
    0 0 0 1];
[x0,y0,z0] = cylinder(0.6,10);

x0=x0*0.05;
y0=y0*0.05;
z0 = z0*parallel_width*2;
x1 = []; y1 = []; z1 = []; % To Initialize New Arrays
for i = 1:2
    cylin = Re1*[z0(i,:)-parallel_width; y0(i,:); x0(i,:); ones(1,11)];
% To Add 41 Ones
    x1 = [x1; cylin(1,:)];
    y1 = [y1; cylin(2,:)];
    z1 = [z1; cylin(3,:)];
end
surf(x1,y1,z1,'FaceColor',rgb('DarkRed'),'Facealpha',0.6, ...
'EdgeColor',edgecolor,'AmbientStrength',0.6)
patch('Vertices',[x1(1,:)' y1(1,:)' z1(1,:)'], ...
'Faces',[1:10],'FaceColor',rgb('DarkRed'), ...
'EdgeColor',edgecolor,'AmbientStrength',0.6)
patch('Vertices',[x1(2,:)' y1(2,:)' z1(2,:)'], ...
'Faces',[1:10],'FaceColor',rgb('DarkRed'), ...
'EdgeColor',edgecolor,'AmbientStrength',0.6)


deg=120;
% R=[cosd(deg),-sind(deg),0;sind(deg),cosd(deg),0;0,0,1];
% P_E2=((R*Transl')+traj)';
Re2=[cosd(deg) -sind(deg) 0 P_E2(1)
    sind(deg) cosd(deg) 0 P_E2(2)
    0 0 1 P_E2(3)
    0 0 0 1];
[x0,y0,z0] = cylinder(0.6,10);
x0=x0*0.05;
y0=y0*0.05;
z0 = z0*parallel_width*2;
x1 = []; y1 = []; z1 = []; % To Initialize New Arrays
for i = 1:2
    cylin = Re2*[z0(i,:)-parallel_width; y0(i,:); x0(i,:); ones(1,11)];
% To Add 41 Ones
    x1 = [x1; cylin(1,:)];
    y1 = [y1; cylin(2,:)];
    z1 = [z1; cylin(3,:)];
end
surf(x1,y1,z1,'FaceColor',rgb('DarkRed'),'Facealpha',0.6, ...
'EdgeColor',edgecolor,'AmbientStrength',0.6)
patch('Vertices',[x1(1,:)' y1(1,:)' z1(1,:)'], ...
'Faces',[1:10],'FaceColor',rgb('DarkRed'), ...
'EdgeColor',edgecolor,'AmbientStrength',0.6)
patch('Vertices',[x1(2,:)' y1(2,:)' z1(2,:)'], ...
'Faces',[1:10],'FaceColor',rgb('DarkRed'), ...
'EdgeColor',edgecolor,'AmbientStrength',0.6)

hold on
deg=-120;
% P_E3=((R*Transl')+traj)';
Re3=[cosd(deg) -sind(deg) 0 P_E3(1)
    sind(deg) cosd(deg) 0 P_E3(2)
    0 0 1 P_E3(3)
    0 0 0 1];
[x0a,y0a,z0a] = cylinder(0.6,10);
x0a=x0a*0.05;
y0a=y0a*0.05;
z0a = z0a*parallel_width*2;
x1a = []; y1a = []; z1a = []; % To Initialize New Arrays
for i = 1:2
    cylin = Re3*[z0a(i,:)-parallel_width; y0a(i,:); x0a(i,:); ones(1,11)];
% To Add 41 Ones
    x1a = [x1a; cylin(1,:)];
    y1a = [y1a; cylin(2,:)];
    z1a = [z1a; cylin(3,:)];
end
surf(x1a,y1a,z1a,'FaceColor',rgb('DarkRed'),'Facealpha',0.6, ...
'EdgeColor',edgecolor,'AmbientStrength',0.6)
patch('Vertices',[x1a(1,:)' y1a(1,:)' z1a(1,:)'], ...
'Faces',[1:10],'FaceColor',rgb('DarkRed'), ...
'EdgeColor',edgecolor,'AmbientStrength',0.6)
patch('Vertices',[x1a(2,:)' y1a(2,:)' z1a(2,:)'], ...
'Faces',[1:10],'FaceColor',rgb('DarkRed'), ...
'EdgeColor',edgecolor,'AmbientStrength',0.6)

%% plot end effector parallelogram joints
%% plot parallelogram joints e1
deg=90;
Rep1=[cosd(deg) -sind(deg) 0 P_E1(1)-parallel_width
    sind(deg) cosd(deg) 0 P_E1(2)
    0 0 1 P_E1(3)
    0 0 0 1];

[x0,y0,z0] = cylinder(1,10);
% x0=x0*0.75;
% y0=y0*0.75;
x0=x0*0.05;
y0=y0*0.05;
%  z0 = z0*0.085;
z0 = z0*0.085-0.04;
x1 = []; y1 = []; z1 = []; % To Initialize New Arrays
for i = 1:2
    cylin = Rep1*[z0(i,:); y0(i,:); x0(i,:); ones(1,11)];
% To Add 41 Ones
    x1 = [x1; cylin(1,:)];
    y1 = [y1; cylin(2,:)];
    z1 = [z1; cylin(3,:)];
end
surf(x1,y1,z1,'FaceColor',facecolor,'Facealpha',0.6, ...
'EdgeColor',edgecolor,'AmbientStrength',0.6)
patch('Vertices',[x1(1,:)' y1(1,:)' z1(1,:)'], ...
'Faces',[1:10],'FaceColor',facecolor, ...
'EdgeColor',edgecolor,'AmbientStrength',0.6)
patch('Vertices',[x1(2,:)' y1(2,:)' z1(2,:)'], ...
'Faces',[1:10],'FaceColor',facecolor, ...
'EdgeColor',edgecolor,'AmbientStrength',0.6)

deg=90;
Rep2=[cosd(deg) -sind(deg) 0 P_E1(1)+parallel_width
    sind(deg) cosd(deg) 0 P_E1(2)
    0 0 1 P_E1(3)
    0 0 0 1];
[x0,y0,z0] = cylinder(1,10);
% x0=x0*0.75;
% y0=y0*0.75;
x0=x0*0.05;
y0=y0*0.05;
%  z0 = z0*0.085;
z0 = z0*0.085-0.04;
x1 = []; y1 = []; z1 = []; % To Initialize New Arrays
for i = 1:2
    cylin = Rep2*[z0(i,:); y0(i,:); x0(i,:); ones(1,11)];
% To Add 41 Ones
    x1 = [x1; cylin(1,:)];
    y1 = [y1; cylin(2,:)];
    z1 = [z1; cylin(3,:)];
end
surf(x1,y1,z1,'FaceColor',facecolor,'Facealpha',0.6, ...
'EdgeColor',edgecolor,'AmbientStrength',0.6)
patch('Vertices',[x1(1,:)' y1(1,:)' z1(1,:)'], ...
'Faces',[1:10],'FaceColor',facecolor, ...
'EdgeColor',edgecolor,'AmbientStrength',0.6)
patch('Vertices',[x1(2,:)' y1(2,:)' z1(2,:)'], ...
'Faces',[1:10],'FaceColor',facecolor, ...
'EdgeColor',edgecolor,'AmbientStrength',0.6)

%% plot joints e2
deg=120;
R=[cosd(deg),-sind(deg),0;sind(deg),cosd(deg),0;0,0,1];
P_E2hn=((R*[-parallel_width,-endeff_edge/(2*sqrt(3)),0]')+traj)';
P_E2hp=((R*[parallel_width,-endeff_edge/(2*sqrt(3)),0]')+traj)';
deg=120+90;
Rep21=[cosd(deg) -sind(deg) 0 P_E2hn(1)
    sind(deg) cosd(deg) 0 P_E2hn(2)
    0 0 1 P_E2hn(3)
    0 0 0 1];

Rep22=[cosd(deg) -sind(deg) 0 P_E2hp(1)
    sind(deg) cosd(deg) 0 P_E2hp(2)
    0 0 1 P_E2hp(3)
    0 0 0 1];
[x0,y0,z0] = cylinder(1,10);
% x0=x0*0.75;
% y0=y0*0.75;
x0=x0*0.05;
y0=y0*0.05;
%  z0 = z0*0.085;
z0 = z0*0.085-0.04;  % -0.04 is to keep the joint at the center
x1 = []; y1 = []; z1 = []; % To Initialize New Arrays
for i = 1:2
    cylin = Rep21*[z0(i,:); y0(i,:); x0(i,:); ones(1,11)];
% To Add 41 Ones
    x1 = [x1; cylin(1,:)];
    y1 = [y1; cylin(2,:)];
    z1 = [z1; cylin(3,:)];
end
surf(x1,y1,z1,'FaceColor',facecolor,'Facealpha',0.6, ...
'EdgeColor',edgecolor,'AmbientStrength',0.6)
patch('Vertices',[x1(1,:)' y1(1,:)' z1(1,:)'], ...
'Faces',[1:10],'FaceColor',facecolor, ...
'EdgeColor',edgecolor,'AmbientStrength',0.6)
patch('Vertices',[x1(2,:)' y1(2,:)' z1(2,:)'], ...
'Faces',[1:10],'FaceColor',facecolor, ...
'EdgeColor',edgecolor,'AmbientStrength',0.6)

[x0,y0,z0] = cylinder(1,10);
% x0=x0*0.75;
% y0=y0*0.75;
x0=x0*0.05;
y0=y0*0.05;
%  z0 = z0*0.085;
z0 = z0*0.085-0.04;
x1 = []; y1 = []; z1 = []; % To Initialize New Arrays
for i = 1:2
    cylin = Rep22*[z0(i,:); y0(i,:); x0(i,:); ones(1,11)];
% To Add 41 Ones
    x1 = [x1; cylin(1,:)];
    y1 = [y1; cylin(2,:)];
    z1 = [z1; cylin(3,:)];
end
surf(x1,y1,z1,'FaceColor',facecolor,'Facealpha',0.6, ...
'EdgeColor',edgecolor,'AmbientStrength',0.6)
patch('Vertices',[x1(1,:)' y1(1,:)' z1(1,:)'], ...
'Faces',[1:10],'FaceColor',facecolor, ...
'EdgeColor',edgecolor,'AmbientStrength',0.6)
patch('Vertices',[x1(2,:)' y1(2,:)' z1(2,:)'], ...
'Faces',[1:10],'FaceColor',facecolor, ...
'EdgeColor',edgecolor,'AmbientStrength',0.6)

%%
% plot joints e3
deg=240;
R=[cosd(deg),-sind(deg),0;sind(deg),cosd(deg),0;0,0,1];
P_E3hn=((R*[-parallel_width,-endeff_edge/(2*sqrt(3)),0]')+traj)';
P_E3hp=((R*[parallel_width,-endeff_edge/(2*sqrt(3)),0]')+traj)';
deg=240+90;
Rep31=[cosd(deg) -sind(deg) 0 P_E3hn(1)
    sind(deg) cosd(deg) 0 P_E3hn(2)
    0 0 1 P_E3hn(3)
    0 0 0 1];

Rep32=[cosd(deg) -sind(deg) 0 P_E3hp(1)
    sind(deg) cosd(deg) 0 P_E3hp(2)
    0 0 1 P_E3hp(3)
    0 0 0 1];
[x0,y0,z0] = cylinder(1,10);
% x0=x0*0.75;
% y0=y0*0.75;
x0=x0*0.05;
y0=y0*0.05;
%  z0 = z0*0.085;
z0 = z0*0.085-0.04;  % -0.04 is to keep the joint at the center
x1 = []; y1 = []; z1 = []; % To Initialize New Arrays
for i = 1:2
    cylin = Rep31*[z0(i,:); y0(i,:); x0(i,:); ones(1,11)];
% To Add 41 Ones
    x1 = [x1; cylin(1,:)];
    y1 = [y1; cylin(2,:)];
    z1 = [z1; cylin(3,:)];
end
surf(x1,y1,z1,'FaceColor',facecolor,'Facealpha',0.6, ...
'EdgeColor',edgecolor,'AmbientStrength',0.6)
patch('Vertices',[x1(1,:)' y1(1,:)' z1(1,:)'], ...
'Faces',[1:10],'FaceColor',facecolor, ...
'EdgeColor',edgecolor,'AmbientStrength',0.6)
patch('Vertices',[x1(2,:)' y1(2,:)' z1(2,:)'], ...
'Faces',[1:10],'FaceColor',facecolor, ...
'EdgeColor',edgecolor,'AmbientStrength',0.6)

[x0,y0,z0] = cylinder(1,10);
% x0=x0*0.75;
% y0=y0*0.75;
x0=x0*0.05;
y0=y0*0.05;
%  z0 = z0*0.085;
z0 = z0*0.085-0.04;
x1 = []; y1 = []; z1 = []; % To Initialize New Arrays
for i = 1:2
    cylin = Rep32*[z0(i,:); y0(i,:); x0(i,:); ones(1,11)];
% To Add 41 Ones
    x1 = [x1; cylin(1,:)];
    y1 = [y1; cylin(2,:)];
    z1 = [z1; cylin(3,:)];
end
surf(x1,y1,z1,'FaceColor',facecolor,'Facealpha',0.6, ...
'EdgeColor',edgecolor,'AmbientStrength',0.6)
patch('Vertices',[x1(1,:)' y1(1,:)' z1(1,:)'], ...
'Faces',[1:10],'FaceColor',facecolor, ...
'EdgeColor',edgecolor,'AmbientStrength',0.6)
patch('Vertices',[x1(2,:)' y1(2,:)' z1(2,:)'], ...
'Faces',[1:10],'FaceColor',facecolor, ...
'EdgeColor',edgecolor,'AmbientStrength',0.6)

%% plot the four bar link between the two points
%four bar 1
[x0,y0,z0] = cylindertwopoints(0.03,10,[P_J1(1)-parallel_width,P_J1(2),P_J1(3)],[P_E1(1)-parallel_width,P_E1(2),P_E1(3)]);
surf(x0,y0,z0,'FaceColor',rgb('DarkRed'),'Facealpha',0.6, ...
'EdgeColor','none','AmbientStrength',0.6)
patch('Vertices',[x0(1,:)' y0(1,:)' z0(1,:)'], ...
'Faces',[1:10],'FaceColor',rgb('DarkRed'), ...
'EdgeColor','none','AmbientStrength',0.6)
patch('Vertices',[x0(2,:)' y0(2,:)' z0(2,:)'], ...
'Faces',[1:10],'FaceColor',rgb('DarkRed'), ...
'EdgeColor','none','AmbientStrength',0.6)

[x0,y0,z0] = cylindertwopoints(0.03,10,[P_J1(1)+parallel_width,P_J1(2),P_J1(3)],[P_E1(1)+parallel_width,P_E1(2),P_E1(3)]);
surf(x0,y0,z0,'FaceColor',rgb('DarkRed'),'Facealpha',0.6, ...
'EdgeColor','none','AmbientStrength',0.6)
patch('Vertices',[x0(1,:)' y0(1,:)' z0(1,:)'], ...
'Faces',[1:10],'FaceColor',rgb('DarkRed'), ...
'EdgeColor','none','AmbientStrength',0.6)
patch('Vertices',[x0(2,:)' y0(2,:)' z0(2,:)'], ...
'Faces',[1:10],'FaceColor',rgb('DarkRed'), ...
'EdgeColor','none','AmbientStrength',0.6)

% %four bar 2
[x0,y0,z0] = cylindertwopoints(0.03,10,[P_J2hn(1),P_J2hn(2),P_J2hn(3)],[P_E2hn(1),P_E2hn(2),P_E2hn(3)]);
surf(x0,y0,z0,'FaceColor',rgb('DarkRed'),'Facealpha',0.6, ...
'EdgeColor','none','AmbientStrength',0.6)
patch('Vertices',[x0(1,:)' y0(1,:)' z0(1,:)'], ...
'Faces',[1:10],'FaceColor',rgb('DarkRed'), ...
'EdgeColor','none','AmbientStrength',0.6)
patch('Vertices',[x0(2,:)' y0(2,:)' z0(2,:)'], ...
'Faces',[1:10],'FaceColor',rgb('DarkRed'), ...
'EdgeColor','none','AmbientStrength',0.6)

[x0,y0,z0] = cylindertwopoints(0.03,10,[P_J2hp(1),P_J2hp(2),P_J2hp(3)],[P_E2hp(1),P_E2hp(2),P_E2hp(3)]);
surf(x0,y0,z0,'FaceColor',rgb('DarkRed'),'Facealpha',0.6, ...
'EdgeColor','none','AmbientStrength',0.6)
patch('Vertices',[x0(1,:)' y0(1,:)' z0(1,:)'], ...
'Faces',[1:10],'FaceColor',rgb('DarkRed'), ...
'EdgeColor','none','AmbientStrength',0.6)
patch('Vertices',[x0(2,:)' y0(2,:)' z0(2,:)'], ...
'Faces',[1:10],'FaceColor',rgb('DarkRed'), ...
'EdgeColor','none','AmbientStrength',0.6)
% 
% %four bar 3
[x0,y0,z0] = cylindertwopoints(0.03,10,[P_J3hn(1),P_J3hn(2),P_J3hn(3)],[P_E3hn(1),P_E3hn(2),P_E3hn(3)]);
surf(x0,y0,z0,'FaceColor',rgb('DarkRed'),'Facealpha',0.6, ...
'EdgeColor','none','AmbientStrength',0.6)
patch('Vertices',[x0(1,:)' y0(1,:)' z0(1,:)'], ...
'Faces',[1:10],'FaceColor',rgb('DarkRed'), ...
'EdgeColor','none','AmbientStrength',0.6)
patch('Vertices',[x0(2,:)' y0(2,:)' z0(2,:)'], ...
'Faces',[1:10],'FaceColor',rgb('DarkRed'), ...
'EdgeColor','none','AmbientStrength',0.6)

[x0,y0,z0] = cylindertwopoints(0.03,10,[P_J3hp(1),P_J3hp(2),P_J3hp(3)],[P_E3hp(1),P_E3hp(2),P_E3hp(3)]);
surf(x0,y0,z0,'FaceColor',rgb('DarkRed'),'Facealpha',0.6, ...
'EdgeColor','none','AmbientStrength',0.6)
patch('Vertices',[x0(1,:)' y0(1,:)' z0(1,:)'], ...
'Faces',[1:10],'FaceColor',rgb('DarkRed'), ...
'EdgeColor','none','AmbientStrength',0.6)
patch('Vertices',[x0(2,:)' y0(2,:)' z0(2,:)'], ...
'Faces',[1:10],'FaceColor',rgb('DarkRed'), ...
'EdgeColor','none','AmbientStrength',0.6)
%%
% view(1.172640888945583e+02,15.379438256141702)
% view(-45.895019217079195,14.69414514991059)
% view(-54.842412448681586,9.248184279900045)
view(-41.379806416995386,24.620440619836735)
hold on
xlabel(['x = ' num2str(x)]);
ylabel(['y = ' num2str(y)]);
zlabel(['z = ' num2str(z)]);
xlim([-0.8 0.8])
ylim([-0.8 0.8])
zlim([-1.8 0.6])

end


%%
% [x0,y0,z0] = cylinder(0.6,10);
% % x0=x0*0.75;
% % y0=y0*0.75;
% x0=x0*0.05;
% y0=y0*0.05;
%  z0 = z0;
% x1 = []; y1 = []; z1 = []; % To Initialize New Arrays
% for i = 1:2
%     cylin =Rj2*[z0(i,:)*0.5-0.25; y0(i,:); x0(i,:); ones(1,11)];
% % To Add 41 Ones
%     x1 = [x1; cylin(1,:)];
%     y1 = [y1; cylin(2,:)];
%     z1 = [z1; cylin(3,:)];
% end
% surf(x1,y1,z1,'FaceColor',rgb('Coral'),'Facealpha',0.6, ...
% 'EdgeColor','none','AmbientStrength',0.6)
% patch('Vertices',[x1(1,:)' y1(1,:)' z1(1,:)'], ...
% 'Faces',[1:10],'FaceColor',rgb('Coral'), ...
% 'EdgeColor','none','AmbientStrength',0.6)
% patch('Vertices',[x1(2,:)' y1(2,:)' z1(2,:)'], ...
% 'Faces',[1:10],'FaceColor',rgb('Coral'), ...
% 'EdgeColor','none','AmbientStrength',0.6)



