function [f]=NonLinearConstraint(u, head, vr, wr,relpx,relpy,relvx,relvy,robovx,robovy,R,dt)
 lindelta=u(1);
 angdelta=u(2);
 f=[];
 for i=1:length(relpx)
  t=(relpx(i).*(relvx(i)+(-1).*robovx+(lindelta+vr).*cos(head+dt.*(angdelta+ ...
    wr)))+relpy(i).*(relvy(i)+(-1).*robovy+(lindelta+vr).*sin(head+dt.*( ...
    angdelta+wr)))).^2+(R.^2+(-1).*relpx(i).^2+(-1).*relpy(i).^2).*((relvx(i)+( ...
    -1).*robovx+(lindelta+vr).*cos(head+dt.*(angdelta+wr))).^2+(relvy(i)+ ...
    (-1).*robovy+(lindelta+vr).*sin(head+dt.*(angdelta+wr))).^2);
  f=[f t];
  
 end
 
end

