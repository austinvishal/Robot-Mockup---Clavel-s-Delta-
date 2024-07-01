%inverse kinematics
% we rotate the base frame by 120 degrees to find all three joint variables
function [Th,flag] =inverseKin (eeposition,parameters)
T = zeros(1,3);
transfangle = [0,120,-120];
for i = 1:3
    T(i) = jointParameter(eeposition,transfangle(i),parameters);
end
if (isnan(T(1)) || isnan(T(2)) || isnan(T(3)))
    flag = 1;
    Th = zeros(1,3);
else
    flag = 0;
    Th = [T(1), T(2), T(3)];
end
end

%% calculate the individual joint angle , this is basically inverse kinematics using
%geometry give the end effector pose, then its fed to the above function
%to calculate all three joint variable angles of the pkm

function [theta] = jointParameter(eeposition,transfangle,parameters)
rotMat =[cosd(transfangle), -sind(transfangle), 0;
    sind(transfangle), cosd(transfangle), 0;
    0, 0, 1];
%coord_in=[x0,y0,z0];
coord_param = rotMat*(eeposition)';
% parameters = [endeff_edge,fixed_edge,r_endlink,r_fixedlink];
endeff_edge = parameters(1); %end effector 
fixed_edge = parameters(2); %base
r_endlink = parameters(3); %parallelogram
r_fixedlink = parameters(4); %base arm

%Coordinates of the end effector
y = coord_param(2) - (endeff_edge/(2*sqrt(3))); %shift center to edge tan30 is root3
z = coord_param(3);
x = coord_param(1);

%projected point on yz plane
y1 = -0.5 * fixed_edge / sqrt(3);  % - f/2 * tan(30)
% solve two circle equations and obtain yj1 and zj1 from that theta
%% verify the below one symbolically 
a = (x^2 + y^2 + z^2 + r_fixedlink^2 - r_endlink^2 - y1^2)/(2*z);
b = (y1-y)/(z);

%discriminant
d = -(a+b*y1)^2 + r_fixedlink*(r_fixedlink*b^2 + r_fixedlink);
if d < 0
    
    
    theta = nan;
else
    
    yj = (y1 - a*b - sqrt(d))/(b^2 + 1); % choosing outer point
    zj = a + b*yj;
    theta = 180*atan(-zj/(y1 - yj))/pi;
    
    if yj>y1
        theta = theta + 180;
    end
end

end


