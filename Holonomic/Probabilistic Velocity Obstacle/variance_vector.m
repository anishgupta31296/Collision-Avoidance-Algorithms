function v=variance_vector(m_terms,h, x, y, v_1, v_2, o,du_x, du_y)
 coeff(:,:,1)=mean(h{5}(x,y,v_1,v_2,o,du_x, du_y).^2) - m_terms(:,5,:).^2;
 coeff(:,:,2)=mean(h{4}(x,y,v_1,v_2,o,du_x, du_y).^2) - m_terms(:,4,:).^2;
 coeff(:,:,3)=2*(mean(h{5}(x,y,v_1,v_2,o,du_x, du_y).*h{3}(x,y,v_1,v_2,o,du_x, du_y)) - m_terms(:,5,:).*m_terms(:,3,:));
 coeff(:,:,4)=2*(mean(h{4}(x,y,v_1,v_2,o,du_x, du_y).*h{2}(x,y,v_1,v_2,o,du_x, du_y)) - m_terms(:,4,:).*m_terms(:,2,:));
 coeff(:,:,5)=2*(mean(h{5}(x,y,v_1,v_2,o,du_x, du_y).*h{6}(x,y,v_1,v_2,o,du_x, du_y)) - m_terms(:,5,:).*m_terms(:,6,:)) + mean(h{3}(x,y,v_1,v_2,o,du_x, du_y).^2) - m_terms(:,3,:).^2;
 coeff(:,:,6)=2*(mean(h{4}(x,y,v_1,v_2,o,du_x, du_y).*h{6}(x,y,v_1,v_2,o,du_x, du_y)) - m_terms(:,4,:).*m_terms(:,6,:)) + mean(h{2}(x,y,v_1,v_2,o,du_x, du_y).^2) - m_terms(:,2,:).^2;
 coeff(:,:,7)=2*(mean(h{6}(x,y,v_1,v_2,o,du_x, du_y).*h{3}(x,y,v_1,v_2,o,du_x, du_y)) - m_terms(:,6,:).*m_terms(:,3,:));
 coeff(:,:,8)=2*(mean(h{6}(x,y,v_1,v_2,o,du_x, du_y).*h{2}(x,y,v_1,v_2,o,du_x, du_y)) - m_terms(:,6,:).*m_terms(:,2,:));
 coeff(:,:,9)=mean(h{6}(x,y,v_1,v_2,o,du_x, du_y).^2) - m_terms(:,6,:).^2;
 coeff(:,:,10)=2*(mean(h{4}(x,y,v_1,v_2,o,du_x, du_y).*h{5}(x,y,v_1,v_2,o,du_x, du_y)) - m_terms(:,5,:).*m_terms(:,4,:)) + mean(h{1}(x,y,v_1,v_2,o,du_x, du_y).^2) - m_terms(:,1,:).^2;
 coeff(:,:,11)=2*(mean(h{5}(x,y,v_1,v_2,o,du_x, du_y).*h{1}(x,y,v_1,v_2,o,du_x, du_y)) - m_terms(:,5,:).*m_terms(:,1,:));
 coeff(:,:,12)=2*(mean(h{4}(x,y,v_1,v_2,o,du_x, du_y).*h{1}(x,y,v_1,v_2,o,du_x, du_y)) - m_terms(:,4,:).*m_terms(:,1,:));
 coeff(:,:,13)=2*(mean(h{5}(x,y,v_1,v_2,o,du_x, du_y).*h{2}(x,y,v_1,v_2,o,du_x, du_y)) - m_terms(:,5,:).*m_terms(:,2,:)) + 2*(mean(h{3}(x,y,v_1,v_2,o,du_x, du_y).*h{1}(x,y,v_1,v_2,o,du_x, du_y)) - m_terms(:,3,:).*m_terms(:,1,:));
 coeff(:,:,14)=2*(mean(h{4}(x,y,v_1,v_2,o,du_x, du_y).*h{3}(x,y,v_1,v_2,o,du_x, du_y)) - m_terms(:,4,:).*m_terms(:,3,:)) + 2*(mean(h{2}(x,y,v_1,v_2,o,du_x, du_y).*h{1}(x,y,v_1,v_2,o,du_x, du_y)) - m_terms(:,2,:).*m_terms(:,1,:));
 coeff(:,:,15)=2*(mean(h{3}(x,y,v_1,v_2,o,du_x, du_y).*h{2}(x,y,v_1,v_2,o,du_x, du_y)) - m_terms(:,3,:).*m_terms(:,2,:)) + 2*(mean(h{6}(x,y,v_1,v_2,o,du_x, du_y).*h{1}(x,y,v_1,v_2,o,du_x, du_y)) - m_terms(:,6,:).*m_terms(:,1,:));

 v=coeff;
end