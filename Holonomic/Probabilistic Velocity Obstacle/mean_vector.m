function [m]= mean_vector(x, y, v_1, v_2, o,du_x, du_y ,h)
m=[];
for i=1:6
    m = [m mean(h{i}(x,y,v_1,v_2,o,du_x, du_y))];
end
end
