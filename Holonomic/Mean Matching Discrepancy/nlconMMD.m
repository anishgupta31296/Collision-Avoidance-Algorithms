function [c,ceq]= nlconMMD(u,o_r_x, o_r_v, r, o)
c=[];

for i=1:size(o_r_x,3)
    for j=1:size(o_r_x,1)
        c = [c; ((norm((o_r_v(j,:,i)-u))^2)*((r+o(i))^2-sum(o_r_x(j,:,i).*o_r_x(j,:,i))) + ((o_r_x(j,:,i)*(o_r_v(j,:,i)-u)')^2))];
    end
end
ceq=[];
end