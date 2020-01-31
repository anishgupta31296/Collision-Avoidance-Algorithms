function [k]=kernel(x,y,d)
if(isempty(y))
    %{
    k=arrayfun(@(a) [a^2 1.414*a 1],x,'UniformOutput',false);
    k=reshape(cell2mat(k),3,length(k))';
    %}
    %fprintf("a");
    c=[];
    for i=0:d
    c=[c nchoosek(d,i)];
    end
    k=arrayfun(@(a) sqrt(c).*(a.^(0:d)),x,'UniformOutput',false);
    k=reshape(cell2mat(k),(d+1),length(k))';

else    
   % fprintf("b");
k=(sum(x.*y, 2) + 1).^d;  
end
end
