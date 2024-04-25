function [XL,XR] = stereo_triangulation(xL,xR,om,T,fc_left,cc_left,kc_left,alpha_c_left,fc_right,cc_right,kc_right,alpha_c_right),

% [XL,XR] = stereo_triangulation(xL,xR,om,T,fc_left,cc_left,kc_left,alpha_c_left,fc_right,cc_right,kc_right,alpha_c_right),
%
% Function that computes the position of a set on N points given the left and right image projections.
% The cameras are assumed to be calibrated, intrinsically, and extrinsically.
%
% Input:
%           xL: 2xN matrix of pixel coordinates in the left image
%           xR: 2xN matrix of pixel coordinates in the right image
%           om,T: rotation vector and translation vector between right and left cameras (output of stereo calibration)
%           fc_left,cc_left,...: intrinsic parameters of the left camera  (output of stereo calibration)
%           fc_right,cc_right,...: intrinsic parameters of the right camera (output of stereo calibration)
%
% Output:
%
%           XL: 3xN matrix of coordinates of the points in the left camera reference frame
%           XR: 3xN matrix of coordinates of the points in the right camera reference frame
%
% Note: XR and XL are related to each other through the rigid motion equation: XR = R * XL + T, where R = rodrigues(om)
% For more information, visit http://www.vision.caltech.edu/bouguetj/calib_doc/htmls/example5.html
%
%
% (c) Jean-Yves Bouguet - Intel Corporation - April 9th, 2003



%--- Normalize the image projection according to the intrinsic parameters of the left and right cameras
% 相点归一， 去畸变，去焦距影响，去中心点。 转换为某种物理点通过小孔的成相点 Xm
xt = normalize_pixel(xL,fc_left,cc_left,kc_left,alpha_c_left);
xtt = normalize_pixel(xR,fc_right,cc_right,kc_right,alpha_c_right);

%--- Extend the normalized projections in homogeneous coordinates
xt = [xt;ones(1,size(xt,2))];
xtt = [xtt;ones(1,size(xtt,2))];

%--- Number of points:
N = size(xt,2);

%--- Rotation matrix corresponding to the rigid motion between left and right cameras:
R = rodrigues(om);


%--- Triangulation of the rays in 3D space:

% 左相机的 --> 物理点通过小孔的成相点(Xm), 通过左右相机相关关系。 旋转。 
% !! 这样的情况下，L R两个平面平行 u 为左 ， xtt 为右
u = R * xt;

n_xt2 = dot(xt,xt); % 左相机小孔后的数 点积  每个点x*x+y*y+z*z
n_xtt2 = dot(xtt,xtt);% 右相机小孔后的数 点积 每个点x*x+y*y+z*z

T_vect = repmat(T, [1 N]);% 构造平移矩阵


% 以下 三角公式求Z 未读懂
DD = n_xt2 .* n_xtt2 - dot(u,xtt).^2;% xl.xl.*xr.xr - (R*xl.xr)^2

dot_uT = dot(u,T_vect);
dot_xttT = dot(xtt,T_vect);
dot_xttu = dot(u,xtt);

NN1 = dot_xttu.*dot_xttT - n_xtt2 .* dot_uT;
NN2 = n_xt2.*dot_xttT - dot_uT.*dot_xttu;

Zt = NN1./DD;%求出左相机的Z
Ztt = NN2./DD;% 求出右相机的Z

% 求Z结束 计算左右物理点 这是一个虚拟的物理点。 因为移动了右相机的位置
X1 = xt .* repmat(Zt,[3 1]); % 给出左相机的X
X2 = R'*(xtt.*repmat(Ztt,[3,1])  - T_vect); %右相机 Xr = R'(xr -T); 等于把所有的T全加到右相机上。


%--- Left coordinates:
XL = 1/2 * (X1 + X2); % 把上面求出来的，相加，分一半，给左相机。

%--- Right coordinates: Xr = R*Xl + T
XR = R*XL + T_vect; % 根据左右相机对应，求右相机

