
function a=tangent(P1, P2, r, o)
P = P1-P2;
d2 = dot(P,P);
Q0 = P2+r^2/d2*P;
T = r/d2*sqrt(d2-r^2)*P*[0,1;-1,0];
Q1 = Q0+T;
Q2 = Q0-T;
m1=(Q1(2)-P1(2))/(Q1(1)-P1(1));
m2=(Q2(2)-P1(2))/(Q2(1)-P1(1));

if(P1(1)>(P2(1)+r))
%fprintf("AAAA")    
F= @(d,m,X) X + d*[1, m]/sqrt(1 + m^2);
X1=F(-4.5, m1, P1);
X2=F(-o, m1, P1);
plot([X1(1), X2(1)], [X1(2), X2(2)], 'r'); 
X1=F(-4.5, m2, P1);
X2=F(-o, m2, P1);
plot([X1(1), X2(1)], [X1(2), X2(2)], 'r'); 
a=1;
elseif(P1(1)>(P2(1)-r) && P1(1)<(P2(1)+r) && P1(2)>P2(2))
%fprintf("IIII")    
F= @(d,m,X) X + d*[1, m]/sqrt(1 + m^2);
X1=F(-4.5, m1, P1);
X2=F(-o, m1, P1);
plot([X1(1), X2(1)], [X1(2), X2(2)], 'r'); 
X1=F(4.2, m2, P1);
X2=F(o, m2, P1);
plot([X1(1), X2(1)], [X1(2), X2(2)], 'r'); 
a=1;
elseif(P1(1)>(P2(1)-r) && P1(1)<(P2(1)+r)&& P1(2)<P2(2))
%fprintf("JJJJ")
F= @(d,m,X) X + d*[1, m]/sqrt(1 + m^2);
X1=F(4.5, m1, P1);
X2=F(o, m1, P1);
plot([X1(1), X2(1)], [X1(2), X2(2)], 'r'); 
X1=F(-4.5, m2, P1);
X2=F(-o, m2, P1);
plot([X1(1), X2(1)], [X1(2), X2(2)], 'r'); 
a=1;
elseif(P1(1)==(P2(1)+r) || P1(1)==(P2(1)-r))
 %fprintf("A");   
 if(isinf(m1))   
    X1=P1+[0,sign(P2(2)-P1(2))*4.5];
    X2=P1+[0,sign(P2(2)-P1(2))*o];
 else
    F= @(d,m,X) X + d*[1, m]/sqrt(1 + m^2);
    X1=F(sign(P2(1)-P1(1))*4.5, m1, P1);
    X2=F(sign(P2(1)-P1(1))*o, m1, P1);  
 end
    plot([X1(1), X2(1)], [X1(2), X2(2)], 'r');
    
 if(isinf(m2))   
    X1=P1+[0,sign(P2(2)-P1(2))*4.5];
    X2=P1+[0,sign(P2(2)-P1(2))*o];
 else
    F= @(d,m,X) X + d*[1, m]/sqrt(1 + m^2);
    X1=F(sign(P2(1)-P1(1))*4.5, m2, P1);
    X2=F(sign(P2(1)-P1(1))*o, m2, P1);  
 end
    plot([X1(1), X2(1)], [X1(2), X2(2)], 'r'); 
    a=1;
    
else
%fprintf("XXXX")    
F= @(d,m,X) X + d*[1, m]/sqrt(1 + m^2);
X1=F(4.5, m1, P1);
X2=F(o, m1, P1);
plot([X1(1), X2(1)], [X1(2), X2(2)], 'r'); 
X1=F(4.5, m2, P1);
X2=F(o, m2, P1);
plot([X1(1), X2(1)], [X1(2), X2(2)], 'r'); 
a=1;
end

end
