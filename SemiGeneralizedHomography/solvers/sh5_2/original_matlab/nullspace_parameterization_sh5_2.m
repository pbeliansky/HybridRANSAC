function [M] = nullspace_parameterization_sh5_2(q,p,c)
A = [0 -p(3,2) * q(1,2) 0 -p(3,2) * q(2,2) p(2,2) * q(2,2) p(2,2) * q(3,2) c(1,2) * q(1,2) c(1,2) * q(2,2); p(3,2) * q(1,2) 0 p(3,2) * q(2,2) 0 -p(1,2) * q(2,2) -p(1,2) * q(3,2) c(2,2) * q(1,2) c(2,2) * q(2,2); 0 -p(3,3) * q(1,3) 0 -p(3,3) * q(2,3) p(2,3) * q(2,3) p(2,3) * q(3,3) c(1,3) * q(1,3) c(1,3) * q(2,3); p(3,3) * q(1,3) 0 p(3,3) * q(2,3) 0 -p(1,3) * q(2,3) -p(1,3) * q(3,3) c(2,3) * q(1,3) c(2,3) * q(2,3); 0 -p(3,4) * q(1,4) 0 -p(3,4) * q(2,4) p(2,4) * q(2,4) p(2,4) * q(3,4) c(1,4) * q(1,4) c(1,4) * q(2,4); p(3,4) * q(1,4) 0 p(3,4) * q(2,4) 0 -p(1,4) * q(2,4) -p(1,4) * q(3,4) c(2,4) * q(1,4) c(2,4) * q(2,4); 0 -p(3,5) * q(1,5) 0 -p(3,5) * q(2,5) p(2,5) * q(2,5) p(2,5) * q(3,5) c(1,5) * q(1,5) c(1,5) * q(2,5); p(3,5) * q(1,5) 0 p(3,5) * q(2,5) 0 -p(1,5) * q(2,5) -p(1,5) * q(3,5) c(2,5) * q(1,5) c(2,5) * q(2,5);];
b = [-c(1,2) * q(3,2) -p(2,2) * q(1,2); -c(2,2) * q(3,2) p(1,2) * q(1,2); -c(1,3) * q(3,3) -p(2,3) * q(1,3); -c(2,3) * q(3,3) p(1,3) * q(1,3); -c(1,4) * q(3,4) -p(2,4) * q(1,4); -c(2,4) * q(3,4) p(1,4) * q(1,4); -c(1,5) * q(3,5) -p(2,5) * q(1,5); -c(2,5) * q(3,5) p(1,5) * q(1,5);];
M = A \ b;
 end
