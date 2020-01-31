function [a]=sensor_range(r_x , o_r_x,range)
if(norm(r_x-o_r_x)<range)
    a=1;
else
    a=0;
end
