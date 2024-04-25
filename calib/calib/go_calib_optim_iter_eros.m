%go_calib_optim_iter
%
%Main calibration function. Computes the intrinsic andextrinsic parameters.
%Runs as a script.
%
%INPUT: x_1,x_2,x_3,...: Feature locations on the images
%       X_1,X_2,X_3,...: Corresponding grid coordinates
%
%OUTPUT: fc: Camera focal length
%        cc: Principal point coordinates
%        alpha_c: Skew coefficient
%        kc: Distortion coefficients
%        KK: The camera matrix (containing fc and cc)
%        omc_1,omc_2,omc_3,...: 3D rotation vectors attached to the grid positions in space
%        Tc_1,Tc_2,Tc_3,...: 3D translation vectors attached to the grid positions in space
%        Rc_1,Rc_2,Rc_3,...: 3D rotation matrices corresponding to the omc vectors
%
%Method: Minimizes the pixel reprojection error in the least squares sense over the intrinsic
%        camera parameters, and the extrinsic parameters (3D locations of the grids in space)
%
%Note: If the intrinsic camera parameters (fc, cc, kc) do not exist before, they are initialized through
%      the function init_intrinsic_param.m. Otherwise, the variables in memory are used as initial guesses.
%
%Note: The row vector active_images consists of zeros and ones. To deactivate an image, set the
%      corresponding entry in the active_images vector to zero.
%
%VERY IMPORTANT: This function works for 2D and 3D calibration rigs, except for init_intrinsic_param.m
%that is so far implemented to work only with 2D rigs.
%In the future, a more general function will be there.
%For now, if using a 3D calibration rig, quick_init is set to 1 for an easy initialization of the focal length
if ~exist('param_counter'),
    param_counter=17;
end
if ~exist('param_counter_long'),
    param_counter_long=22;
end

if ~exist('desactivated_images'),
    desactivated_images = [];
end;



if ~exist('est_aspect_ratio'),
    est_aspect_ratio = 1;
end;

if ~exist('est_fc');
    est_fc = [1;1]; % Set to zero if you do not want to estimate the focal length (it may be useful! believe it or not!)
end;

if ~exist('recompute_extrinsic'),
    recompute_extrinsic = 1; % Set this variable to 0 in case you do not want to recompute the extrinsic parameters
    % at each iterstion.
end;

if ~exist('MaxIter'),
    MaxIter = 30; % Maximum number of iterations in the gradient descent
end;

if ~exist('check_cond'),
    check_cond = 1; % Set this variable to 0 in case you don't want to extract view dynamically
end;

if ~exist('center_optim'),
    center_optim = 1; % Set this variable to 0 if your do not want to estimate the principal point
end;

if exist('est_dist'),
    if length(est_dist) == 4,
        est_dist = ones(12,1);
    end;
    if length(est_dist) == 5,
        est_dist = ones(12,1);
    end;
end;

if ~exist('est_dist'),
    est_dist = ones(12,1);
end;

if ~exist('est_alpha'),
    est_alpha = 1; % by default, do not estimate skew
end;

% 以上标志位: by eros
% desactivated_images 不进入计算的图
% est_aspect_ratio  如果接受FC0 == FC1 这个可以设置为0
% est_fc 计算FC, 如果不想算,可以关掉
% recompute_extrinsic 重算外参矩阵
% MaxIter 最大迭代次数
% check_cond  检测图是否有问题
% center_optim 中心点优化. 如果不想管中心点, 可以设置为0  
% est_dist % 参数数
% est_alpha 不计算ALPHA_C

% Little fix in case of stupid values in the binary variables:
center_optim = double(~~center_optim);
est_alpha = double(~~est_alpha);
est_dist = double(~~est_dist);
est_fc = double(~~est_fc);
est_aspect_ratio = double(~~est_aspect_ratio);

fprintf(1,'\n');

if ~exist('nx')&~exist('ny'),
    fprintf(1,'WARNING: No image size (nx,ny) available. Setting nx=640 and ny=480. If these are not the right values, change values manually.\n');
    nx = 640;
    ny = 480;
end;


check_active_images;


quick_init = 0; % Set to 1 for using a quick init (necessary when using 3D rigs)


% Check 3D-ness of the calibration rig:  
rig3D = 0;
for kk = ind_active,
    eval(['X_kk = X_' num2str(kk) ';']);
    if is3D(X_kk),
        rig3D = 1;
    end;
end;


if center_optim & (length(ind_active) < 2) & ~rig3D,
    fprintf(1,'WARNING: Principal point rejected from the optimization when using one image and planar rig (center_optim = 1).\n');
    center_optim = 0; %%% when using a single image, please, no principal point estimation!!!
    est_alpha = 0;
end;

if ~exist('dont_ask'),
    dont_ask = 0;
end;

if center_optim & (length(ind_active) < 5) & ~rig3D,
    fprintf(1,'WARNING: The principal point estimation may be unreliable (using less than 5 images for calibration).\n');
    %if ~dont_ask,
    %   quest = input('Are you sure you want to keep the principal point in the optimization process? ([]=yes, other=no) ');
    %   center_optim = isempty(quest);
    %end;
end;


% A quick fix for solving conflict
if ~isequal(est_fc,[1;1]),
    est_aspect_ratio=1;
end;
if ~est_aspect_ratio,
    est_fc=[1;1];
end;


if ~est_aspect_ratio,
    fprintf(1,'Aspect ratio not optimized (est_aspect_ratio = 0) -> fc(1)=fc(2). Set est_aspect_ratio to 1 for estimating aspect ratio.\n');
else
    if isequal(est_fc,[1;1]),
        fprintf(1,'Aspect ratio optimized (est_aspect_ratio = 1) -> both components of fc are estimated (DEFAULT).\n');
    end;
end;

if ~isequal(est_fc,[1;1]),
    if isequal(est_fc,[1;0]),
        fprintf(1,'The first component of focal (fc(1)) is estimated, but not the second one (est_fc=[1;0])\n');
    else
        if isequal(est_fc,[0;1]),
            fprintf(1,'The second component of focal (fc(1)) is estimated, but not the first one (est_fc=[0;1])\n');
        else
            fprintf(1,'The focal vector fc is not optimized (est_fc=[0;0])\n');
        end;
    end;
end;


if ~center_optim, % In the case where the principal point is not estimated, keep it at the center of the image
    fprintf(1,'Principal point not optimized (center_optim=0). ');
    if ~exist('cc'),
        fprintf(1,'It is kept at the center of the image.\n');
        cc = [(nx-1)/2;(ny-1)/2];
    else
        fprintf(1,'Note: to set it in the middle of the image, clear variable cc, and run calibration again.\n');
    end;
else
    fprintf(1,'Principal point optimized (center_optim=1) - (DEFAULT). To reject principal point, set center_optim=0\n');
end;


if ~center_optim & (est_alpha),
    fprintf(1,'WARNING: Since there is no principal point estimation (center_optim=0), no skew estimation (est_alpha = 0)\n');
    est_alpha = 0;  
end;

if ~est_alpha,
    fprintf(1,'Skew not optimized (est_alpha=0) - (DEFAULT)\n');
    alpha_c = 0;
else
    fprintf(1,'Skew optimized (est_alpha=1). To disable skew estimation, set est_alpha=0.\n');
end;


if ~prod(double(est_dist)),
    fprintf(1,'Distortion not fully estimated (defined by the variable est_dist):\n');
    if ~est_dist(1),
        fprintf(1,'     Second order distortion not estimated (est_dist(1)=0).\n');
    end;
    if ~est_dist(2),
        fprintf(1,'     Fourth order distortion not estimated (est_dist(2)=0).\n');
    end;
    if ~est_dist(5),
        fprintf(1,'     Sixth order distortion not estimated (est_dist(5)=0) - (DEFAULT) .\n');
    end;
    if ~prod(double(est_dist(3:4))),
        fprintf(1,'     Tangential distortion not estimated (est_dist(3:4)~=[1;1]).\n');
    end;
end;


% Check 3D-ness of the calibration rig:
rig3D = 0;
for kk = ind_active,
    eval(['X_kk = X_' num2str(kk) ';']);
    if is3D(X_kk),
        rig3D = 1;
    end;
end;

% If the rig is 3D, then no choice: the only valid initialization is manual!
if rig3D,
    quick_init = 1;
end;



alpha_smooth = 0.1; % set alpha_smooth = 1; for steepest gradient descent


% Conditioning threshold for view rejection
thresh_cond = 1e6;



% Initialization of the intrinsic parameters (if necessary)
% 先给定默认的中心点
if ~exist('cc'),
    fprintf(1,'Initialization of the principal point at the center of the image.\n');
    cc = [(nx-1)/2;(ny-1)/2];
    alpha_smooth = 0.1; % slow convergence
end;


if exist('kc'),
    if length(kc) == 4;
        fprintf(1,'Adding a new distortion coefficient to kc -> radial distortion model up to the 6th degree');
        kc = [kc;0];
    end;
end;

if ~exist('alpha_c'),
    fprintf(1,'Initialization of the image skew to zero.\n');
    alpha_c = 0;
    alpha_smooth = 0.1; % slow convergence
end;

if ~exist('fc') && quick_init,
    FOV_angle = 35; % Initial camera field of view in degrees
    fprintf(1,['Initialization of the focal length to a FOV of ' num2str(FOV_angle) ' degrees.\n']);
    fc = (nx/2)/tan(pi*FOV_angle/360) * ones(2,1);
    est_fc = [1;1];
    alpha_smooth = 0.1; % slow 
end;


if ~exist('fc'),
    % Initialization of the intrinsic parameters:
    fprintf(1,'Initialization of the intrinsic parameters using the vanishing points of planar patterns.\n')
    init_intrinsic_param; % The right way to go (if quick_init is not active)! !!计算第一次内参 第一次内参没有畸变方程. by eros
    alpha_smooth = 0.1; % slow convergence
    est_fc = [1;1];
end;


if ~exist('kc'),
    fprintf(1,'Initialization of the image distortion to zero.\n');
    kc = zeros(12,1);
    alpha_smooth = 0.1; % slow convergence
end;

if ~est_aspect_ratio,
    fc(1) = (fc(1)+fc(2))/2;
    fc(2) = fc(1);
end;

if length(kc)==5
    kc=[kc;zeros(7,1)];
end
if ~prod(double(est_dist)),
    % If no distortion estimated, set to zero the variables that are not estimated
    kc = kc .* est_dist;
end;



if ~prod(double(est_fc)),
    fprintf(1,'Warning: The focal length is not fully estimated (est_fc ~= [1;1])\n');
end;


% Initialization of the extrinsic parameters for global minimization:
comp_ext_calib_eros; % !!计算第一次外参,  by eros



% Initialization of the global parameter vector:

init_param = [fc;cc;alpha_c;kc;zeros(5,1)]; 

for kk = 1:n_ima,
    eval(['omckk = omc_' num2str(kk) ';']);
    eval(['Tckk = Tc_' num2str(kk) ';']);
    init_param = [init_param; omckk ; Tckk];    
end;



%-------------------- Main Optimization:

fprintf(1,'\nMain calibration optimization procedure - Number of images: %d\n',length(ind_active));


param = init_param;
change = 1;

iter = 0;

fprintf(1,'Gradient descent iterations: ');

param_list = param;

% 进入迭代计算 by eors 把change 从 1e-9 变成1e-11
while ((change > 1e-15) && iter < MaxIter),%
    
    fprintf(1,'%d...',iter+1);
    
    % To speed up: pre-allocate the memory for the Jacobian JJ3.
    % For that, need to compute the total number of points.
    
    % The first step consists of updating the whole vector of knowns (intrinsic + extrinsic of active
    % images) through a one step steepest gradient descent.
    % 更新内参矩阵
    
    f = param(1:2);
    c = param(3:4);
    alpha = param(5);
    k = param(6:param_counter);
    
    check_active_images;
    % Compute the size of the Jacobian matrix:
    N_points_views_active = N_points_views(ind_active);
    
    JJ3 = sparse([],[],[],param_counter_long + 6*n_ima,param_counter_long + 6*n_ima,126*n_ima + 225);
    ex3 = zeros(param_counter_long + 6*n_ima,1);
    
    % 迭代20张图 计算重投后的误差
    for kk = ind_active, %1:n_ima,
        %if active_images(kk),
        
        omckk = param(param_counter_long+6*(kk-1) + 1:param_counter_long+6*(kk-1) + 3); % 获取外参 旋转向量
        
        Tckk = param(param_counter_long+6*(kk-1) + 4:param_counter_long+6*(kk-1) + 6); % 外参 平移向量 X,Y,Z 3个方向
        
        if isnan(omckk(1)),
            fprintf(1,'Intrinsic parameters at frame %d do not exist\n',kk);
            return;
        end;
        
        eval(['X_kk = X_' num2str(kk) ';']);
        eval(['x_kk = x_' num2str(kk) ';']);
        
        Np = N_points_views(kk);
        
        if ~est_aspect_ratio,
            % omckk: rotation vector (3x1 vector);旋转向量
            % Tckk: translation vector (3x1 vector)平移向量 X,Y,Z 3个方向
            % f: camera focal length in units of horizontal and vertical
            % pixel units (2x1 vector) 焦距
            % c: principal point location in pixel units (2x1 vector) 中心点
            % k: Distortion coefficients (radial and tangential) (4x1
            % vector) 畸变参数
            % alpha: Skew coefficient between x and y pixel (alpha = 0 <=>
            % square pixels) 倾斜参数?
            [x,dxdom,dxdT,dxdf,dxdc,dxdk,dxdalpha] = project_points2_eros(X_kk,omckk,Tckk,f(1),c,k,alpha);%把3D空间投影到2D X 空间坐标, 
            dxdf = repmat(dxdf,[1 2]);
        else
            [x,dxdom,dxdT,dxdf,dxdc,dxdk,dxdalpha] = project_points2_eros(X_kk,omckk,Tckk,f,c,k,alpha);
        end;
        
        exkk = x_kk - x;
        A = [dxdf dxdc dxdalpha dxdk]';
        B = [dxdom dxdT]';
        JJ3(1:param_counter,1:param_counter) = JJ3(1:param_counter,1:param_counter) + sparse(A*A');
        JJ3(param_counter_long+6*(kk-1) + 1:param_counter_long+6*(kk-1) + 6,param_counter_long+6*(kk-1) + 1:param_counter_long+6*(kk-1) + 6) = sparse(B*B');
        
        AB = sparse(A*B');
        JJ3(1:param_counter,param_counter_long+6*(kk-1) + 1:param_counter_long+6*(kk-1) + 6) = AB;
        JJ3(param_counter_long+6*(kk-1) + 1:param_counter_long+6*(kk-1) + 6,1:param_counter) = (AB)';
        
        ex3(1:param_counter) = ex3(1:param_counter) + A*exkk(:);
        ex3(param_counter_long+6*(kk-1) + 1:param_counter_long+6*(kk-1) + 6) = B*exkk(:);
        
        % Check if this view is ill-conditioned: 检测图的状态
        if check_cond,
            JJ_kk = B'; %[dxdom dxdT];
            if (cond(JJ_kk)> thresh_cond),
                active_images(kk) = 0;
                fprintf(1,'\nWarning: View #%d ill-conditioned. This image is now set inactive. (note: to disactivate this option, set check_cond=0)\n',kk)
                desactivated_images = [desactivated_images kk];
                param(param_counter_long+6*(kk-1) + 1:param_counter_long+6*(kk-1) + 6) = NaN*ones(6,1); 
            end;
        end;
        
        %end;
        
    end;
    
    
    % List of active images (necessary if changed): 
    check_active_images;
    
    
    % The following vector helps to select the variables to update (for
    % only active images): 下面的数组是用来选择更新的变量
    selected_variables = [est_fc;center_optim*ones(2,1);est_alpha;est_dist;zeros(5,1);reshape(ones(6,1)*active_images,6*n_ima,1)];
    if ~est_aspect_ratio,
        if isequal(est_fc,[1;1]) | isequal(est_fc,[1;0]),
            selected_variables(2) = 0;
        end;
    end;
    ind_Jac = find(selected_variables)';
    
    % 只关注需要算的参数
    JJ3 = JJ3(ind_Jac,ind_Jac);
    ex3 = ex3(ind_Jac);
    
    JJ2_inv = inv(JJ3); % not bad for sparse matrices!!
    
    
    % Smoothing coefficient: 更新系数
    
%     alpha_smooth2 = 1-(1-alpha_smooth)^(iter+1); %set to 1 to undo any smoothing!
    
    param_innov = alpha_smooth*(JJ2_inv*ex3);% 计算更新参数
    
    
    param_up = param(ind_Jac) + param_innov;% 更新参数
    param(ind_Jac) = param_up;
    
    
    % New intrinsic parameters:新的内参
    
    fc_current = param(1:2);
    cc_current = param(3:4);

    if center_optim & ((param(3)<0)|(param(3)>nx)|(param(4)<0)|(param(4)>ny)),
        fprintf(1,'Warning: it appears that the principal point cannot be estimated. Setting center_optim = 0\n');
        center_optim = 0;
        cc_current = c;
    else
        cc_current = param(3:4);
    end;
    
    alpha_current = param(5);
    kc_current = param(6:param_counter);
    
    if ~est_aspect_ratio & isequal(est_fc,[1;1]),
        fc_current(2) = fc_current(1);
        param(2) = param(1);
    end;
    
    % Change on the intrinsic parameters: 改变内参  norm ==> sqrt(sum(X.^2));
    % 只关注fc 和 cc
    change = norm([fc_current;cc_current] - [f;c])/norm([fc_current;cc_current]);
    
    
    % Second step: (optional) - It makes convergence faster, and the region of convergence LARGER!!!
    % Recompute the extrinsic parameters only using compute_extrinsic.m (this may be useful sometimes)
    % The complete gradient descent method is useful to precisely update the intrinsic parameters.
    
    % 根据新算的内参矩阵,重算外参矩阵
    if recompute_extrinsic,
        MaxIter2 = 30;
        for kk =ind_active, %1:n_ima,
            %if active_images(kk),
            omc_current = param(param_counter_long+6*(kk-1) + 1:param_counter_long+6*(kk-1) + 3);
            Tc_current = param(param_counter_long+6*(kk-1) + 4:param_counter_long+6*(kk-1) + 6);
            eval(['X_kk = X_' num2str(kk) ';']);
            eval(['x_kk = x_' num2str(kk) ';']);
            [omc_current,Tc_current] = compute_extrinsic_init_eros(x_kk,X_kk,fc_current,cc_current,kc_current,alpha_current);%外参计算初始计算
            [omckk,Tckk,Rckk,JJ_kk] = compute_extrinsic_refine_eros(omc_current,Tc_current,x_kk,X_kk,fc_current,cc_current,kc_current,alpha_current,MaxIter2,thresh_cond);% 外参修正
            if check_cond,
                if (cond(JJ_kk)> thresh_cond),
                    active_images(kk) = 0;
                    fprintf(1,'\nWarning: View #%d ill-conditioned. This image is now set inactive. (note: to disactivate this option, set check_cond=0)\n',kk);
                    desactivated_images = [desactivated_images kk];
                    omckk = NaN*ones(3,1);
                    Tckk = NaN*ones(3,1);
                end;
            end;
            param(param_counter_long+6*(kk-1) + 1:param_counter_long+6*(kk-1) + 3) = omckk;
            param(param_counter_long+6*(kk-1) + 4:param_counter_long+6*(kk-1) + 6) = Tckk;
            %end;
        end;
    end;
    
    param_list = [param_list param];
    iter = iter + 1;
    
end;

fprintf(1,'done\n');



%%%--------------------------- Computation of the error of estimation:
% 计算结束 打印结果

fprintf(1,'Estimation of uncertainties...');


check_active_images;

solution = param;


% Extraction of the paramters for computing the right reprojection error:

fc = solution(1:2);
cc = solution(3:4);
alpha_c = solution(5);
kc = solution(6:param_counter);

for kk = 1:n_ima,
    
    if active_images(kk), 
        
        omckk = solution(param_counter_long+6*(kk-1) + 1:param_counter_long+6*(kk-1) + 3);%***   
        Tckk = solution(param_counter_long+6*(kk-1) + 4:param_counter_long+6*(kk-1) + 6);%*** 
        Rckk = rodrigues(omckk);
        
    else
        
        omckk = NaN*ones(3,1);   
        Tckk = NaN*ones(3,1);
        Rckk = NaN*ones(3,3);
        
    end;
    
    eval(['omc_' num2str(kk) ' = omckk;']);
    eval(['Rc_' num2str(kk) ' = Rckk;']);
    eval(['Tc_' num2str(kk) ' = Tckk;']);
    
end;


% Recompute the error (in the vector ex):
comp_error_calib_eros;

sigma_x = std(ex(:));

% Compute the size of the Jacobian matrix:
N_points_views_active = N_points_views(ind_active);

JJ3 = sparse([],[],[],param_counter_long + 6*n_ima,param_counter_long + 6*n_ima,126*n_ima + 225);

for kk = ind_active,
    
    omckk = param(param_counter_long+6*(kk-1) + 1:param_counter_long+6*(kk-1) + 3); 
    Tckk = param(param_counter_long+6*(kk-1) + 4:param_counter_long+6*(kk-1) + 6); 
    
    eval(['X_kk = X_' num2str(kk) ';']);
    
    Np = N_points_views(kk);
    
    %[x,dxdom,dxdT,dxdf,dxdc,dxdk,dxdalpha] = project_points2(X_kk,omckk,Tckk,fc,cc,kc,alpha_c);
    
    if ~est_aspect_ratio,
        [x,dxdom,dxdT,dxdf,dxdc,dxdk,dxdalpha] = project_points2_eros(X_kk,omckk,Tckk,fc(1),cc,kc,alpha_c);
        dxdf = repmat(dxdf,[1 2]);
    else
        [x,dxdom,dxdT,dxdf,dxdc,dxdk,dxdalpha] = project_points2_eros(X_kk,omckk,Tckk,fc,cc,kc,alpha_c);
    end;
    
    A = [dxdf dxdc dxdalpha dxdk]';
    B = [dxdom dxdT]';
    
    JJ3(1:param_counter,1:param_counter) = JJ3(1:param_counter,1:param_counter) + sparse(A*A');
    JJ3(param_counter_long+6*(kk-1) + 1:param_counter_long+6*(kk-1) + 6,param_counter_long+6*(kk-1) + 1:param_counter_long+6*(kk-1) + 6) = sparse(B*B');
    
    AB = sparse(A*B');
    JJ3(1:param_counter,param_counter_long+6*(kk-1) + 1:param_counter_long+6*(kk-1) + 6) = AB;
    JJ3(param_counter_long+6*(kk-1) + 1:param_counter_long+6*(kk-1) + 6,1:param_counter) = (AB)';
    
end;

JJ3 = JJ3(ind_Jac,ind_Jac);

JJ2_inv = inv(JJ3); % not bad for sparse matrices!!

param_error = zeros(6*n_ima+15,1);
param_error(ind_Jac) =  3*sqrt(full(diag(JJ2_inv)))*sigma_x;

solution_error = param_error;

if ~est_aspect_ratio && isequal(est_fc,[1;1]),
    solution_error(2) = solution_error(1);
end;


%%% Extraction of the final intrinsic and extrinsic paramaters:

extract_parameters_eros;

fprintf(1,'done\n');


fprintf(1,'\n\nCalibration results after optimization (with uncertainties):\n\n');
fprintf(1,'Focal Length:          fc = [ %3.5f   %3.5f ] +/- [ %3.5f   %3.5f ]\n',[fc;fc_error]);
fprintf(1,'Principal point:       cc = [ %3.5f   %3.5f ] +/- [ %3.5f   %3.5f ]\n',[cc;cc_error]);
fprintf(1,'Skew:             alpha_c = [ %3.5f ] +/- [ %3.5f  ]   => angle of pixel axes = %3.5f +/- %3.5f degrees\n',[alpha_c;alpha_c_error],90 - atan(alpha_c)*180/pi,atan(alpha_c_error)*180/pi);
fprintf(1,'Distortion:            kc = [ %3.5f   %3.5f   %3.5f   %3.5f  %5.5f %3.5f  %3.5f  %3.5f  %3.5f  %3.5f  %3.5f  %3.5f] +/- \n',[kc]);   
fprintf(1,'[%3.5f   %3.5f   %3.5f   %3.5f  %5.5f %3.5f  %3.5f  %3.5f  %3.5f  %3.5f  %3.5f  %3.5f]\n',[kc_error]);   
fprintf(1,'Pixel error:          err = [ %3.5f   %3.5f ]\n\n',err_std); 
fprintf(1,'Note: The numerical errors are approximately three times the standard deviations (for reference).\n\n\n')
%fprintf(1,'      For accurate (and stable) error estimates, it is recommended to run Calibration once again.\n\n\n')




