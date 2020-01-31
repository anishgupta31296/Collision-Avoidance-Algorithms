function v=variance_terms(m_terms,h, x, y, v_1, v_2, o) 
 x_size=size(x);
 x_rows=x_size(1);
 o=repmat(o,x_rows,1);
 coeff(:,:,1)=mean(arrayfun(h{5},x,y,v_1,v_2,o).^2) - m_terms(:,5,:).^2;
 coeff(:,:,2)=mean(arrayfun(h{4},x,y,v_1,v_2,o).^2) - m_terms(:,4,:).^2;
 coeff(:,:,3)=2*(mean(arrayfun(h{5},x,y,v_1,v_2,o).*arrayfun(h{3},x,y,v_1,v_2,o)) - m_terms(:,5,:).*m_terms(:,3,:));
 coeff(:,:,4)=2*(mean(arrayfun(h{4},x,y,v_1,v_2,o).*arrayfun(h{2},x,y,v_1,v_2,o)) - m_terms(:,4,:).*m_terms(:,2,:));
 coeff(:,:,5)=2*(mean(arrayfun(h{5},x,y,v_1,v_2,o).*arrayfun(h{6},x,y,v_1,v_2,o)) - m_terms(:,5,:).*m_terms(:,6,:)) + mean(arrayfun(h{3},x,y,v_1,v_2,o).^2) - m_terms(:,3,:).^2;
 coeff(:,:,6)=2*(mean(arrayfun(h{4},x,y,v_1,v_2,o).*arrayfun(h{6},x,y,v_1,v_2,o)) - m_terms(:,4,:).*m_terms(:,6,:)) + mean(arrayfun(h{2},x,y,v_1,v_2,o).^2) - m_terms(:,2,:).^2;
 coeff(:,:,7)=2*(mean(arrayfun(h{6},x,y,v_1,v_2,o).*arrayfun(h{3},x,y,v_1,v_2,o)) - m_terms(:,6,:).*m_terms(:,3,:));
 coeff(:,:,8)=2*(mean(arrayfun(h{6},x,y,v_1,v_2,o).*arrayfun(h{2},x,y,v_1,v_2,o)) - m_terms(:,6,:).*m_terms(:,2,:));
 coeff(:,:,9)=mean(arrayfun(h{6},x,y,v_1,v_2,o).^2) - m_terms(:,6,:).^2;
 coeff(:,:,10)=2*(mean(arrayfun(h{4},x,y,v_1,v_2,o).*arrayfun(h{5},x,y,v_1,v_2,o)) - m_terms(:,5,:).*m_terms(:,4,:)) + mean(arrayfun(h{1},x,y,v_1,v_2,o).^2) - m_terms(:,1,:).^2;
 coeff(:,:,11)=2*(mean(arrayfun(h{5},x,y,v_1,v_2,o).*arrayfun(h{1},x,y,v_1,v_2,o)) - m_terms(:,5,:).*m_terms(:,1,:));
 coeff(:,:,12)=2*(mean(arrayfun(h{4},x,y,v_1,v_2,o).*arrayfun(h{1},x,y,v_1,v_2,o)) - m_terms(:,4,:).*m_terms(:,1,:));
 coeff(:,:,13)=2*(mean(arrayfun(h{5},x,y,v_1,v_2,o).*arrayfun(h{2},x,y,v_1,v_2,o)) - m_terms(:,5,:).*m_terms(:,2,:)) + 2*(mean(arrayfun(h{3},x,y,v_1,v_2,o).*arrayfun(h{1},x,y,v_1,v_2,o)) - m_terms(:,3,:).*m_terms(:,1,:));
 coeff(:,:,14)=2*(mean(arrayfun(h{4},x,y,v_1,v_2,o).*arrayfun(h{3},x,y,v_1,v_2,o)) - m_terms(:,4,:).*m_terms(:,3,:)) + 2*(mean(arrayfun(h{2},x,y,v_1,v_2,o).*arrayfun(h{1},x,y,v_1,v_2,o)) - m_terms(:,2,:).*m_terms(:,1,:));
 coeff(:,:,15)=2*(mean(arrayfun(h{3},x,y,v_1,v_2,o).*arrayfun(h{2},x,y,v_1,v_2,o)) - m_terms(:,3,:).*m_terms(:,2,:)) + 2*(mean(arrayfun(h{6},x,y,v_1,v_2,o).*arrayfun(h{1},x,y,v_1,v_2,o)) - m_terms(:,6,:).*m_terms(:,1,:));

  %{
 coeff(5)=hh(3,3) + 2*hh(6,5) ; 
 coeff(6)=hh(2,2) + 2*hh(6,4);
 coeff(7)=2*hh(6,3)  ;
 coeff(8)=2*hh(6,2)  ;
 coeff(9)=hh(6,6) ; 
 coeff(10)=2*hh(5,4) + hh(1,1);
 coeff(11)=2*hh(5,1);
 coeff(12)=2*hh(4,1);
 coeff(13)=2*hh(5,2) +2*hh(3,1);
 coeff(14)=2*hh(4,3) +2*hh(2,1);
 coeff(15)=2*hh(3,2) +2*hh(6,1);
 %}
 v=coeff;
end