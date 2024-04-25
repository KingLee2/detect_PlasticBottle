function [xp,dxpdom,dxpdT,dxpdf,dxpdc,dxpdk,dxpdalpha] = project_points2_power(X,om,T,f,c,k,alpha)

%project_points2.m
%
%[xp,dxpdom,dxpdT,dxpdf,dxpdc,dxpdk] = project_points2(X,om,T,f,c,k,alpha)
%
%Projects a 3D structure onto the image plane.
%
%INPUT: X: 3D structure in the world coordinate frame (3xN matrix for N points)
%       (om,T): Rigid motion parameters between world coordinate frame and camera reference frame
%               om: rotation vector (3x1 vector); T: translation vector (3x1 vector)
%       f: camera focal length in units of horizontal and vertical pixel units (2x1 vector)
%       c: principal point location in pixel units (2x1 vector)
%       k: Distortion coefficients (radial and tangential) (4x1 vector)
%       alpha: Skew coefficient between x and y pixel (alpha = 0 <=> square pixels)
%
%OUTPUT: xp: Projected pixel coordinates (2xN matrix for N points)
%        dxpdom: Derivative of xp with respect to om ((2N)x3 matrix)
%        dxpdT: Derivative of xp with respect to T ((2N)x3 matrix)
%        dxpdf: Derivative of xp with respect to f ((2N)x2 matrix if f is 2x1, or (2N)x1 matrix is f is a scalar)
%        dxpdc: Derivative of xp with respect to c ((2N)x2 matrix)
%        dxpdk: Derivative of xp with respect to k ((2N)x4 matrix)
%
%Definitions:
%Let P be a point in 3D of coordinates X in the world reference frame (stored in the matrix X)
%The coordinate vector of P in the camera reference frame is: Xc = R*X + T
%where R is the rotation matrix corresponding to the rotation vector om: R = rodrigues(om);
%call x, y and z the 3 coordinates of Xc: x = Xc(1); y = Xc(2); z = Xc(3);
%The pinehole projection coordinates of P is [a;b] where a=x/z and b=y/z.
%call r^2 = a^2 + b^2.
%The distorted point coordinates are: xd = [xx;yy] where:
%
%xx = a * (1 + kc(1)*r^2 + kc(2)*r^4 + kc(5)*r^6)      +      2*kc(3)*a*b + kc(4)*(r^2 + 2*a^2);
%yy = b * (1 + kc(1)*r^2 + kc(2)*r^4 + kc(5)*r^6)      +      kc(3)*(r^2 + 2*b^2) + 2*kc(4)*a*b;
%
%The left terms correspond to radial distortion (6th degree), the right terms correspond to tangential distortion
%
%Finally, convertion into pixel coordinates: The final pixel coordinates vector xp=[xxp;yyp] where:
%
%xxp = f(1)*(xx + alpha*yy) + c(1)
%yyp = f(2)*yy + c(2)
%
%
%NOTE: About 90 percent of the code takes care fo computing the Jacobian matrices
%
%
%Important function called within that program:
%
%rodrigues.m: Computes the rotation matrix corresponding to a rotation vector
%
%rigid_motion.m: Computes the rigid motion transformation of a given structure


if nargin < 7,
    alpha = 0;
    if nargin < 6,
        k = zeros(5,1);
        if nargin < 5,
            c = zeros(2,1);
            if nargin < 4,
                f = ones(2,1);
                if nargin < 3,
                    T = zeros(3,1);
                    if nargin < 2,
                        om = zeros(3,1);
                        if nargin < 1,
                            error('Need at least a 3D structure to project (in project_points.m)');
                            return;
                        end;
                    end;
                end;
            end;
        end;
    end;
end;


[m,n] = size(X);

if length(k)==5
    k=zeros(20,1);
end
if nargout > 1,
    [Y,dYdom,dYdT] = rigid_motion(X,om,T); % dYdom = dYdR * dRdom
else
    Y = rigid_motion(X,om,T);
end;


inv_Z = 1./Y(3,:);

x = (Y(1:2,:) .* (ones(2,1) * inv_Z)) ;% 这个为公式里的P
zx = x(1,:);
zy = x(2,:); 

bb = (-zx .* inv_Z)'*ones(1,3);
cc = (-zy .* inv_Z)'*ones(1,3);

if nargout > 1,
    dxdom = ((inv_Z')*ones(1,3)) .* dYdom(1:3:end,:) + bb .* dYdom(3:3:end,:);%公式推导对的。
    dydom = ((inv_Z')*ones(1,3)) .* dYdom(2:3:end,:) + cc .* dYdom(3:3:end,:);

    dxdT = ((inv_Z')*ones(1,3)) .* dYdT(1:3:end,:) + bb .* dYdT(3:3:end,:);
    dydT = ((inv_Z')*ones(1,3)) .* dYdT(2:3:end,:) + cc .* dYdT(3:3:end,:);
end;


% Add distortion:
cdist = 1 + k(1).*zx + k(2).*zy + k(3).*zx.^2 + k(4).*zy.*zx + k(5).*zy.^2 + k(6).*zx.^3 ...
    + k(7).*zy.*zx.^2 + k(8).*zy.^2.*zx + k(9).*zy.^3 + k(10).*zx.^4 + k(11).*zy.*zx.^3 + k(12).*zy.^2.*zx.^2 ...
    + k(13).*zy.^3.*zx + k(14).*zy.^4 + k(15).*zx.^5 + k(16).*zy.*zx.^4 + k(17).*zy.^2.*zx.^3 ...
    + k(18).*zy.^3.*zx.^2 + k(19).*zy.^4.*zx + k(20).*zy.^5;
if nargout > 1,
    dcdistdom = ((k(1)+k(3).*zx.*2+k(4).*zy+k(6).*zx.^2.*3+k(7).*zy.*zx.*2+k(8).*zy.^2+k(10).*zx.^3.*4+ ...
        k(11).*zy.*zx.^2.*3+k(12).*zy.^2.*zx.*2+k(13).*zy.^3+k(15).*zx.^4.*5+k(16).*zy.*zx.^3.*4+ ...
        k(17).*zy.^2.*zx.^2.*3+k(18).*zy.^3.*zx.*2+k(19).*zy.^4))'*ones(1,3).*dxdom + ...
        ((k(2)+k(4).*zx+k(5).*zy.*2+k(7).*zx.^2+k(8).*zy.*2.*zx+k(9).*zy.^2.*3+k(11).*zx.^3+k(12).*zy.*2.*zx.^2+ ...
        k(13).*zy.^2.*3.*zx+k(14).*zy.^3.*4+k(16).*zx.^4+k(17).*zy.*2.*zx.^3+k(18).*zy.^2.*3.*zx.^2+ ...
        k(19).*zy.^3.*4.*zx+k(20).*zy.^4.*5))'*ones(1,3).*dydom;
    
    dcdistdT = ((k(1)+k(3).*zx.*2+k(4).*zy+k(6).*zx.^2.*3+k(7).*zy.*zx.*2+k(8).*zy.^2+k(10).*zx.^3.*4+ ...
        k(11).*zy.*zx.^2.*3+k(12).*zy.^2.*zx.*2+k(13).*zy.^3+k(15).*zx.^4.*5+k(16).*zy.*zx.^3.*4+ ...
        k(17).*zy.^2.*zx.^2.*3+k(18).*zy.^3.*zx.*2+k(19).*zy.^4))'*ones(1,3).*dxdT + ...
        ((k(2)+k(4).*zx+k(5).*zy.*2+k(7).*zx.^2+k(8).*zy.*2.*zx+k(9).*zy.^2.*3+k(11).*zx.^3+k(12).*zy.*2.*zx.^2+ ...
        k(13).*zy.^2.*3.*zx+k(14).*zy.^3.*4+k(16).*zx.^4+k(17).*zy.*2.*zx.^3+k(18).*zy.^2.*3.*zx.^2+ ...
        k(19).*zy.^3.*4.*zx+k(20).*zy.^4.*5))'*ones(1,3).*dydT;
    
    dcdistdk = [zx,zy,zx.^2,zy.*zx,zy.^2,zx.^3,zy.*zx.^2,zy.^2.*zx,zy.^3,zx.^4,zy.*zx.^3,zy.^2.*zx.^2,zy.^3.*zx,zy.^4,zx.^5,zy.*zx.^4,zy.^2.*zx.^3,zy.^3.*zx.^2,zy.^4.*zx,zy.^5];
    dcdistdk = reshape(dcdistdk,n,20);
end;

xd1 = zx .* cdist;
yd1 = zy .* cdist;

if nargout > 1,
    dxd1dom = (zx'*ones(1,3)) .* dcdistdom;
    dyd1dom = (zy'*ones(1,3)) .* dcdistdom;
    coeff = cdist'*ones(1,3);
    dxd1dom = dxd1dom + coeff.* dxdom;
    dyd1dom = dyd1dom + coeff.* dydom;

    dxd1dT = (zx'*ones(1,3)) .* dcdistdT;
    dyd1dT = (zy'*ones(1,3)) .* dcdistdT;
    dxd1dT = dxd1dT + coeff.* dxdT;
    dyd1dT = dyd1dT + coeff.* dydT;

    dxd1dk = (zx'*ones(1,20)) .* dcdistdk;%注意
    dyd1dk = (zy'*ones(1,20)) .* dcdistdk;
end;

% Add Skew:

xd3 = xd1 + alpha*yd1;
yd3 = yd1;

% Compute: dxd3dom, dxd3dT, dxd3dk, dxd3dalpha
if nargout > 1,
    dxd3dom = dxd1dom + alpha*dyd1dom;
    dyd3dom = dyd1dom;
    tmp= zeros(2*n,3);
    tmp(1:2:2*n,:) = dxd3dom;
    tmp(2:2:2*n,:) = dyd3dom;
    dxd3dom = tmp;
    
    dxd3dT = dxd1dT + alpha*dyd1dT;
    dyd3dT = dyd1dT;
    tmp= zeros(2*n,3);
    tmp(1:2:2*n,:) = dxd3dT;
    tmp(2:2:2*n,:) = dyd3dT;
    dxd3dT = tmp;
    
    dxd3dk = dxd1dk + alpha*dyd1dk;
    dyd3dk = dyd1dk;
    tmp= zeros(2*n,20);
    tmp(1:2:2*n,:) = dxd3dk;
    tmp(2:2:2*n,:) = dyd3dk;
    dxd3dk = tmp;
    
    dxd3dalpha = zeros(2*n,1);
    dxd3dalpha(1:2:2*n,:) = yd1';
end;


% Pixel coordinates:
if length(f)>1,
    vx = xd3 .* (f(1) * ones(1,n))  +  c(1)*ones(1,n);
    vy = yd3 .* (f(2) * ones(1,n))  +  c(2)*ones(1,n);
    xp = [vx;vy];
    if nargout > 1,
        coeff = reshape(f(:)*ones(1,n),2*n,1);% f 矩阵转一维
        dxpdom = (coeff*ones(1,3)) .* dxd3dom; % 对om求导
        dxpdT = (coeff*ones(1,3)) .* dxd3dT; % 对T求导
        dxpdk = (coeff*ones(1,20)) .* dxd3dk; % 对K求导
        dxpdalpha = (coeff) .* dxd3dalpha; % 对ALPHA求导
        dxpdf = zeros(2*n,2);
        dxpdf(1:2:end,1) = xd3';% 对f求导 xp = xd3.*f+c 所以变成xd3'
        dxpdf(2:2:end,2) = yd3';
    end;
else
    xp = f * xd3 + c*ones(1,n);
    if nargout > 1,
        dxpdom = f  * dxd3dom;
        dxpdT = f * dxd3dT;
        dxpdk = f  * dxd3dk;
        dxpdalpha = f .* dxd3dalpha;
        dxpdf = xd3(:);
    end;
end;

if nargout > 1,
    dxpdc = zeros(2*n,2);
    dxpdc(1:2:end,1) = ones(n,1);
    dxpdc(2:2:end,2) = ones(n,1);
end;


return;


