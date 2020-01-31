function h=function_handles(r,scale)
h5=@(r_x, r_y, v_x, v_y,o, du_x, du_y) scale*((r+o).^2 - r_y.^2) ;
h4=@(r_x, r_y, v_x, v_y,o, du_x, du_y) scale*((r+o).^2 - r_x.^2) ;
h3=@(r_x, r_y, v_x, v_y,o, du_x, du_y) scale*(-2*(v_x + du_x).*((r+o).^2 - (r_x.^2 + r_y.^2)) -2*((r_x.*(v_x + du_x) + r_y.*(v_y + du_y)).* r_x ));
h2=@(r_x, r_y, v_x, v_y,o, du_x, du_y) scale*(-2*(v_y + du_y).*((r+o).^2 - (r_x.^2 + r_y.^2)) -2*((r_x.*(v_x + du_x) + r_y.*(v_y + du_y)).* r_y ));
h1=@(r_x, r_y, v_x, v_y,o, du_x, du_y) scale*(2*r_x.*r_y); 
h0=@(r_x, r_y, v_x, v_y,o, du_x, du_y) scale*(((r+o).^2).*((v_x + du_x).^2 + (v_y + du_y).^2) - ((v_x + du_x).^2 + (v_y + du_y).^2).*(r_x.^2 + r_y.^2) + ((v_x + du_x).*r_x + (v_y + du_y).*r_y).^2);
f=@(r_x, r_y, v_x, v_y, o,du_x,du_y, u1,u2) (h5(r_x, r_y, v_x, v_y,o, du_x, du_y)*u1^2 + h4(r_x, r_y, v_x, v_y,o, du_x, du_y)*u2^2 + h3(r_x, r_y, v_x, v_y,o, du_x, du_y)*u1 + h2(r_x, r_y, v_x, v_y,o, du_x, du_y)*u2 + h1(r_x, r_y, v_x, v_y,o, du_x, du_y)*u1*u2 + h0(r_x, r_y, v_x, v_y,o, du_x, du_y));
h={h1, h2, h3, h4, h5, h0,f};
end