function [c,ceq]= MPC_objective_1(theta_dot, time,o_r_x, o_r_v, r, o, no_o, v, theta)
c=[];
for i=1:no_o
%c = [c; (r+o(i))^2-sum(o_r_x(i,:).*o_r_x(i,:)) + ((o_r_x(i,:)*(o_r_v(i,:)-u)')^2)/norm((o_r_v(i,:)-u))^2] ;
c = [c; (norm((o_r_v(i,:)+v*[cos(theta) sin(theta)]-v*[cos(theta + theta_dot*time) sin(theta + theta_dot*time)]))^2)*((r+o(i))^2-sum(o_r_x(i,:).*o_r_x(i,:))) + ((o_r_x(i,:)*(o_r_v(i,:)+v*[cos(theta) sin(theta)]-v*[cos(theta + theta_dot*time) sin(theta + theta_dot*time)])')^2)] ;
end   
ceq=[];
end
