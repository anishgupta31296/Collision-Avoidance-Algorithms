function [ind]=reduced_set_samples(sample, no_o, lambda, target_size, d)
options = optimoptions('quadprog', 'Display', 'off');
for i=1:no_o
 ss=generate_kernel_matrix(sample(:,:,i), sample(:,:,i), d);
 alpha=ones(1,length(sample));
 n=ss*alpha';
 I=eye(length(sample));
 H = 2*[I,-I]'* ss * [I,-I];
 f=[(-(2*n) + lambda); (2*n + lambda)];
 [r(i,:)]=quadprog(H,f,[],[],[],[],zeros(2*length(sample),1),inf*[alpha, alpha],[],options);
 [r_coeff(i, :),ind(i, :)]=maxk(abs(r(1:100)-r(101:200)), target_size);
 %objective= @(beta) alpha*ss*alpha' -2*alpha*ss*beta' + beta*ss*beta' + lambda*sum(abs(beta));
 %objective= @(beta) beta(1,:)*(-2*n +  1000) + beta(2,:)*(2*n +  1000) + (beta(1,:)-beta(2,:))*ss*(beta(1,:)-beta(2,:))' ;
 %beta=fmincon(objective, zeroes(2,length(sample)), [], [], [alpha, -alpha],[1000], zeroes(2,length(sample)), 10*[alpha;alpha], []);
end

