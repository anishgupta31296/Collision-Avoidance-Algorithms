function [a] =circ(x,y,r)
 th = 0:pi/50:2*pi;
 xunit = r * cos(th) + x;
 yunit = r * sin(th) + y;
 a = plot(xunit, yunit);
 axis square;
end

