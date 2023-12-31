%INVHOMOG Decompose an homography
%
% S = INVHOMOG(H, OPTIONS) decomposes the homography H (3x3)into the camera motion
% and the normal to the plane.
%
% In practice there are multiple solutions and S is a vector of structures
% with elements:
%  T   camera motion as a homogeneous transform matrix (4x4), translation not to scale
%  n   normal vector to the plane (3x3)
%
% Options::
% 'K',K   provide the camera intrinsic matrix
%
% Notes:
% - there are up to 4 solutions
% - only those solutions that obey the positive depth constraint are returned
%
% Reference::
% An invitation to 3D vision, section 5.3
%
% See also HOMOGRAPHY.


% Copyright (C) 1993-2011, by Peter I. Corke
%
% This file is part of The Robotics Toolbox for Matlab (RTB).
%
% RTB is free software: you can redistribute it and/or modify
% it under the terms of the GNU Lesser General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
%
% RTB is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU Lesser General Public License for more details.
%
% You should have received a copy of the GNU Leser General Public License
% along with RTB.  If not, see <http://www.gnu.org/licenses/>.

function solutions = invhomog(H, varargin)

opt.K = eye(3,3);

% optionally convert from projective to Euclidean homography
%     opt.K
if nargin > 1
    H = inv(opt.K) * H * opt.K;
end

% normalize H so that the second singular value is one
[U,S,V] = svd(H);

H = H/S(2,2);


% compute the SVD of the symmetric matrix H'*H = VSV'
[U,S,V] = svd(H'*H);


% ensure V is right-handed
if det(V) < 0,
    fprintf('det(V) was < 0\n');
    V = -V;
end

% get the squared singular values
s1 = S(1,1);
s3 = S(3,3);

v1 = V(:,1); v2 = V(:,2); v3 = V(:,3);

% pure the case of pure rotation all the singular values are equal to 1
if abs(s1-s3) < 100*eps
    warning('Homoography due to pure rotation');
    if det(H) < 0
        H = -H;
    end
    sol(1).T = r2t(H);
    sol(1).n = [];
else
    % compute orthogonal unit vectors
    u1 = (sqrt(1-s3)*v1 + sqrt(s1-1)*v3) / sqrt(s1-s3);
    u2 = (sqrt(1-s3)*v1 - sqrt(s1-1)*v3) / sqrt(s1-s3);
    %     disp('u1'); u1'
    %     disp('u2'); u2'
    
    U1 = [v2 u1 cross(v2,u1)];
    W1 = [H*v2 H*u1 skew(H*v2)*H*u1];
    
    U2 = [v2 u2 cross(v2,u2)];
    W2 = [H*v2 H*u2 skew(H*v2)*H*u2];
    
    % compute the rotation matrices
    R1 = W1*U1';
    R2 = W2*U2';
    
    % build the solutions, discard those with negative plane normals
    n = cross(v2, u1);
    sol(1).n = n;
    t = (H-R1)*n;
    sol(1).T = inv([R1 t; 0 0 0 1]);
    
    sol(2).n = -n;
    t = -(H-R1)*n;
    sol(2).T = inv([R1 t; 0 0 0 1]);
    
    
    
    
    n = cross(v2, u2);
    
    sol(3).n = n;
    t = (H-R2)*n;
    sol(3).T = inv([R2 t; 0 0 0 1]);
    
    sol(4).n = -n;
    t = -(H-R2)*n;
    sol(4).T = inv([R2 t; 0 0 0 1]);
    
    
end



solutions = sol;

end
function X = skew(x)

X = [   0 -x(3)  x(2) ;
    x(3)     0 -x(1) ;
    -x(2)  x(1)     0 ];
end