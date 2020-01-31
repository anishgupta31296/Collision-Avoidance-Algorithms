function [] =circ(x,y,r,outline, color_array,line_width,pattern)
 th = 0:pi/50:2*pi;
 xunit = r * cos(th) + x;
 yunit = r * sin(th) + y;
 
 if(nargin==3) 
     a = plot(xunit, yunit);
 elseif(nargin==4)
     a = plot(xunit, yunit,'color',outline);
     hold on
 elseif(nargin==5)
     f= fill(xunit,yunit,color_array,'LineStyle','none'); 
     hold on
     a = plot(xunit, yunit,'color',outline);
     hold on
 elseif(nargin==6)           
     f= fill(xunit,yunit,color_array, 'LineStyle','none');
     hold on
     a = plot(xunit, yunit,'color',outline,'Linewidth',line_width);
     hold on
 elseif(nargin==7)
     f= fill(xunit,yunit,color_array,'LineStyle','none');
     hold on
     a = plot(xunit, yunit,pattern,'color',outline,'Linewidth',line_width);
     hold on
 end
 
 axis square;
end

