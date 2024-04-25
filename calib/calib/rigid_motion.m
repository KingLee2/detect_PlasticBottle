function [Y,dYdom,dYdT] = rigid_motion(X,om,T);

%rigid_motion.m
%
%[Y,dYdom,dYdT] = rigid_motion(X,om,T)
%
%Computes the rigid motion transformation Y = R*X+T, where R = rodrigues(om).
%
%INPUT: X: 3D structure in the world coordinate frame (3xN matrix for N points)
%       (om,T): Rigid motion parameters between world coordinate frame and camera reference frame
%               om: rotation vector (3x1 vector); T: translation vector (3x1 vector)
%
%OUTPUT: Y: 3D coordinates of the structure points in the camera reference frame (3xN matrix for N points)
%        dYdom: Derivative of Y with respect to om ((3N)x3 matrix)
%        dYdT: Derivative of Y with respect to T ((3N)x3 matrix)
%
%Definitions:
%Let P be a point in 3D of coordinates X in the world reference frame (stored in the matrix X)
%The coordinate vector of P in the camera reference frame is: Y = R*X + T
%where R is the rotation matrix corresponding to the rotation vector om: R = rodrigues(om);
%
%Important function called within that program:
%
%rodrigues.m: Computes the rotation matrix corresponding to a rotation vector



if nargin < 3,
   T = zeros(3,1);
   if nargin < 2,
      om = zeros(3,1);
      if nargin < 1,
         error('Need at least a 3D structure as input (in rigid_motion.m)');
         return;
      end;
   end;
end;

% 参数 X -> 物理坐标 [X;Y;Z]
% 参数 om 旋转阵 [1.9908  2.0304 -0.1722]
% 参数 T 平移矩阵  [-123.7378 -300.4677 978.1407]

% R =    -0.0015    0.9924    0.1231
%    0.9575    0.0369   -0.2861
%   -0.2884    0.1174   -0.9503
[R,dRdom] = rodrigues(om);%1*3的矩阵转换为3*3的矩阵

[m,n] = size(X);

Y = R*X + repmat(T,[1 n]);

if nargout > 1,
   

dYdR = zeros(3*n,9); % 9333*9
dYdT = zeros(3*n,3);

dYdR(1:3:end,1:3:end) =  X'; % 9333*9 其实只是一堆X的复制排列 
dYdR(2:3:end,2:3:end) =  X';
dYdR(3:3:end,3:3:end) =  X';

dYdT(1:3:end,1) =  ones(n,1);% 9333*3
dYdT(2:3:end,2) =  ones(n,1);
dYdT(3:3:end,3) =  ones(n,1);

% dRdom（9*3） =
%     0.3881   -0.5836    0.0495
%     0.1566    0.1404    0.1301
%     0.5179    0.4692    0.4318
%     0.0672    0.0492   -0.0648
%    -0.5871    0.3807    0.0508
%    -0.3831   -0.5355    0.5316
%    -0.5368   -0.4038    0.5230
%     0.4484    0.5191    0.4422
%    -0.2045   -0.2086   -0.0654
dYdom = dYdR * dRdom; %  乘完结果9333*3  dRdom，R对om求偏导

end;




