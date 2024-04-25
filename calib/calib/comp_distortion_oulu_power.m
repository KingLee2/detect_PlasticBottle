function [x] = comp_distortion_oulu_power(xd,k);

%comp_distortion_oulu.m
%
%[x] = comp_distortion_oulu(xd,k)
%
%Compensates for radial and tangential distortion. Model From Oulu university.
%For more informatino about the distortion model, check the forward projection mapping function:
%project_points.m
%
%INPUT: xd: distorted (normalized) point coordinates in the image plane (2xN matrix)
%       k: Distortion coefficients (radial and tangential) (4x1 vector)
%
%OUTPUT: x: undistorted (normalized) point coordinates in the image plane (2xN matrix)
%
%Method: Iterative method for compensation.
%
%NOTE: This compensation has to be done after the subtraction
%      of the principal point, and division by the focal length.


if length(k) == 1,
    
    [x] = comp_distortion(xd,k);
    
else
    
    x = xd; 				% initial guess
    zx = x(1,:);
    zy = x(2,:);
    for kk=1:20,
        
%         r_2 = sum(x.^2);
%         k_radial =  1 + k1 * r_2 + k2 * r_2.^2 + k3 * r_2.^3;
%         dk = 1 + k(6) * r_2 + k(7) * r_2.^2 + k(8) * r_2.^3;
%         k_radial = k_radial ./ dk;
%         
%         delta_x = [2*p1*x(1,:).*x(2,:) + p2*(r_2 + 2*x(1,:).^2 + k(9).*r_2 + k(10).*r_2.^2);
%         p1 * (r_2 + 2*x(2,:).^2)+2*p2*x(1,:).*x(2,:) + k(11).*r_2 + k(12).*r_2.^2];
%         x = (xd - delta_x)./(ones(2,1)*k_radial);
        k_radial=(1 + ...
            k(1).*zx + ...
            k(2).*zy + ...
            k(3).*zx.^2 + ...
            k(4).*zy.*zx + ...
            k(5).*zy.^2 + ...
            k(6).*zx.^3 + ...
            k(7).*zy.*zx.^2 + ...
            k(8).*zy.^2.*zx + ...
            k(9).*zy.^3 + ...
            k(10).*zx.^4 + ...
            k(11).*zy.*zx.^3 + ...
            k(12).*zy.^2.*zx.^2 + ...
            k(13).*zy.^3.*zx + ...
            k(14).*zy.^4 + ...
            k(15).*zx.^5 + ...
            k(16).*zy.*zx.^4 + ...
            k(17).*zy.^2.*zx.^3 + ...
            k(18).*zy.^3.*zx.^2 + ...
            k(19).*zy.^4.*zx + ...
            k(20).*zy.^5);
        x = xd./(ones(2,1)*k_radial);
            
    end;
    
end;
    
    
