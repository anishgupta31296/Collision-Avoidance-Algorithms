function [c,ceq]= multiobs(u,o_r_x, o_r_v, r, o, no_o)
c=[];

for i=1:no_o
c = [c; ((norm((o_r_v(i,:)-u))^2)*((r+o(i))^2-sum(o_r_x(i,:).*o_r_x(i,:))) + ((o_r_x(i,:)*(o_r_v(i,:)-u)')^2))];
%c = [c; ((r+o(i))^2- o_r_x(i,1)^2 - o_r_x(i,2)^2)*(o_r_v(i,1)^2 + o_r_v(i,2)^2) + (o_r_x(i,1)*o_r_v(i,1) + o_r_x(i,2)*o_r_v(i,2))^2] ;

end   
ceq=[];
end