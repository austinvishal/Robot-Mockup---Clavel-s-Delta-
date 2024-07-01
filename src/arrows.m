clf
% Set the limits and select a view
ax = axes('XLim',[-1 1],'YLim',[-1 1],'ZLim',[0 2]);
view(3); grid on; axis equal
xlabel('x'); ylabel('y'); zlabel('z');

[x y z] = cylinder(0.04);
h(1) = surface(x,y,z,'FaceColor','blue');
[x y z] = cylinder([0.2 0]);
h(2) = surface(x,y,z,'FaceColor','blue');
% Create group object and parent surfaces
t(1) = hgtransform('Parent',ax);
t(2) = hgtransform('Parent',t(1));
set(h(1),'Parent',t(1));
set(h(2),'Parent',t(2));

S = makehgtform('scale',[0.5 0.5 0.5]);
T = makehgtform('translate',[0 0 0.8]);
set(t(2),'Matrix',T*S);

% red axis [1 0 0]
t(3) = copyobj(t(1),ax);
h = findobj(t(3),'Type','surface');
set(h,'FaceColor','red');
R = makehgtform('yrotate',pi/2);
set(t(3),'Matrix',R);

% green axis [0 1 0]
t(4) = copyobj(t(1),ax);
h = findobj(t(4),'Type','surface');
set(h,'FaceColor','green');
R = makehgtform('xrotate',-pi/2);
set(t(4),'Matrix',R);