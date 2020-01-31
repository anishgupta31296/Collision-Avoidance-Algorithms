function [c,ceq]= MPC_objective_2(s,o_r_x, o_r_v, r, o, no_o, v, theta, theta1)
c=[];
for i=1:no_o
%c = [c; (r+o(i))^2-sum(o_r_x(i,:).*o_r_x(i,:)) + ((o_r_x(i,:)*(o_r_v(i,:)-u)')^2)/norm((o_r_v(i,:)-u))^2] ;
%c = [c; ((r+o(i))^2-sum(o_r_x(i,:).*o_r_x(i,:)) + ((o_r_x(i,:)*(o_r_v(i,:)+v*[cos(theta) sin(theta)]-v*s*[cos(theta1) sin(theta1)])')^2)/norm((o_r_v(i,:)+v*[cos(theta) sin(theta)]-v*s*[cos(theta1) sin(theta1)]))^2)] ;
c = [c; (norm((o_r_v(i,:)+v*[cos(theta) sin(theta)]-v*s*[cos(theta1) sin(theta1)]))^2)*((r+o(i))^2-sum(o_r_x(i,:).*o_r_x(i,:))) + ((o_r_x(i,:)*(o_r_v(i,:)+v*[cos(theta) sin(theta)]-v*s*[cos(theta1) sin(theta1)])')^2)] ;
end

ceq=[];
end
