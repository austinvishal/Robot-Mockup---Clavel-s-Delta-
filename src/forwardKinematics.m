function [Pos_out,f] = forwardKinematics(T,parameters)
endeff_edge = parameters(1); %end effector 
fixed_edge = parameters(2); %base
r_endlink = parameters(3); %parallelogram
r_fixedlink = parameters(4); %base arm

sum = 0;
%defining geometry
t = (fixed_edge-endeff_edge)*tand(30)/2;
w = zeros(1,3);
theta1 = T(1);
theta2 = T(2);
theta3 = T(3);
j1 = [0, -(t+r_fixedlink*cosd(theta1)), -r_fixedlink*sind(theta1)];
j2 = [(t+r_fixedlink*cosd(theta2))*cosd(30), (t+r_fixedlink*cosd(theta2))*sind(30), -r_fixedlink*sind(theta2)];
j3 = [-(t+r_fixedlink*cosd(theta3))*cosd(30), (t+r_fixedlink*cosd(theta3))*sind(30), -r_fixedlink*sind(theta3)];
J1 = round(j1,3);
J2 = round(j2,3);
J3 = round(j3,3);
J = [J1;J2;J3];
for i = 1:3
    for j = 1:3
        sum = sum + J(i,j)^2;
    end
    w(i) = sum;
    sum = 0;
end

d = round(((J2(2)-J1(2))*J3(1)-(J3(2)-J1(2))*J2(1)),3);
a1 = round(((((J2(3)-J1(3))*(J3(2)-J1(2)))-((J3(3)-J1(3))*(J2(2)-J1(2))))/d),3);
a2 = round((-(((J2(3)-J1(3))*(J3(1)))-((J3(3)-J1(3))*(J2(1))))/d),3);
b1 = round((-(((w(2)-w(1))*(J3(2)-J1(2)))-((w(3)-w(1))*(J2(2)-J1(2))))/(2*d)),3);
b2 = round((-(((w(2)-w(1))*(J3(1)))-((w(3)-w(1))*(J2(1))))/(2*d)),3);

a = round(((a1^2)+(a2^2)+1),3);
b = round((2*((a1*b1)+a2*(b2-J1(2))-J1(3))),3);
c = round(((b1^2+((b2-J1(2))^2)+(J1(3)^2)-r_endlink^2)),3);
%solve geometry - algebraically
disc = (b^2) - 4*a*c;
if disc < 0
    %No solution!
    x0 = nan;
    y0 = nan;
    z0 = nan;
    f = 1;
else
    
    z0 = -0.5*(b + sqrt(disc)) / a; %get z
    x0 = -(a1*z0+b1); %find x
    y0 = a2*z0+b2; %find y
    f = 0;
end
    Pos_out = [x0,y0,z0]; %return vector
end

    