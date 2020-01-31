function [function_matrix,function_array]=function_samples_monte_carlo(samples_within_conf_robo,samples_within_conf_obst,R,scale,function_of_interest)

size_samples_robo=size(samples_within_conf_robo,1);
size_samples_obst=size(samples_within_conf_obst,1);
coll_cone_iter=1;
for robo_iter=1:size_samples_robo
      x0=samples_within_conf_robo(robo_iter,1);y0=samples_within_conf_robo(robo_iter,2);x0dot=samples_within_conf_robo(robo_iter,3);
        y0dot=samples_within_conf_robo(robo_iter,4);
    for obst_iter=1:size_samples_obst
     x1=samples_within_conf_obst(obst_iter,1); y1=samples_within_conf_obst(obst_iter,2);x1dot=samples_within_conf_obst(obst_iter,3);
        y1dot=samples_within_conf_obst(obst_iter,4); 
      function_matrix(robo_iter,obst_iter)=function_of_interest( x1,y1,x1dot,y1dot,x0,y0,x0dot,y0dot,scale,R );
      function_array(coll_cone_iter,:)=function_of_interest( x1,y1,x1dot,y1dot,x0,y0,x0dot,y0dot,scale,R );
      coll_cone_iter=coll_cone_iter+1;
    end
end

end


