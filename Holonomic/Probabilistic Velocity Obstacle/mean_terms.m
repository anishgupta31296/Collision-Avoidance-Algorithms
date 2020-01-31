function [m,h,f]= mean_terms(x, y, v_1, v_2, o, r)
x_size=size(x);
x_rows=x_size(1);
o=repmat(o,x_rows,1);
h5=@(r_x, r_y, v_x, v_y,o) (r+o)^2 - r_y^2 ;
h4=@(r_x, r_y, v_x, v_y,o) (r+o)^2 - r_x^2 ;
h3=@(r_x, r_y, v_x, v_y,o) -2*v_x*((r+o)^2 - (r_x^2 + r_y^2)) -2*((r_x*v_x + r_y*v_y)* r_x );
h2=@(r_x, r_y, v_x, v_y,o) -2*v_y*((r+o)^2 - (r_x^2 + r_y^2)) -2*((r_x*v_x + r_y*v_y)* r_y );
h1=@(r_x, r_y, v_x, v_y,o) (2*r_x*r_y); 
h0=@(r_x, r_y, v_x, v_y,o) ((r+o)^2)*(v_x^2 + v_y^2) - (v_x^2 + v_y^2)*(r_x^2 + r_y^2) + (v_x*r_x + v_y*r_y)^2;
f=@(r_x, r_y, v_x, v_y, o, u1,u2) (h5(r_x, r_y, v_x, v_y,o)*u1^2 + h4(r_x, r_y, v_x, v_y,o)*u2^2 + h3(r_x, r_y, v_x, v_y,o)*u1 + h2(r_x, r_y, v_x, v_y,o)*u2 + h1(r_x, r_y, v_x, v_y,o)*u1*u2 + h0(r_x, r_y, v_x, v_y,o));
h={h1, h2, h3, h4, h5, h0};
m=[];
for i=1:6
    m = [m mean(arrayfun(h{i},x,y,v_1,v_2,o))];
end
end