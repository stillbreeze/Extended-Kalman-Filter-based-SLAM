%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  16833 Robot Localization and Mapping  % 
%  Assignment #2                         %
%  EKF-SLAM                              %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear;
close all;
clc;

%==== TEST: Setup uncertainity parameters (try different values!) ===
sig_x = 0.25;
sig_y = 0.1;
sig_alpha = 0.1;
sig_beta = 0.01;
sig_r = 0.08;

%==== Generate sigma^2 from sigma ===
sig_x2 = sig_x^2;
sig_y2 = sig_y^2;
sig_alpha2 = sig_alpha^2;
sig_beta2 = sig_beta^2;
sig_r2 = sig_r^2;

%==== Open data file ====
fid = fopen('../data/data.txt');

%==== Read first measurement data ====
tline = fgets(fid);
arr = str2num(tline);
measure = arr';
t = 1;
 
%==== Setup control and measurement covariances ===
control_cov = diag([sig_x2, sig_y2, sig_alpha2]);
% measure_cov = diag([sig_beta2, sig_r2]);
measure_cov = diag([sig_r2, sig_beta2]);

%==== Setup initial pose vector and pose uncertainty ====
pose = [0 ; 0 ; 0];
pose_cov = diag([0.02^2, 0.02^2, 0.1^2]);

%==== TODO: Setup initial landmark vector landmark[] and covariance matrix landmark_cov[] ====
%==== (Hint: use initial pose with uncertainty and first measurement) ====
k = 6;
landmark = zeros(2*k,1);
landmark_cov = zeros(2*k);
for i = 1:2:2*k
   rot_1 = cos(pose(3) + measure(i));
   rot_2 = sin(pose(3) + measure(i));
   lx_r = rot_1;
   lx_beta = -measure(i+1)*rot_2;
   ly_r = rot_2;
   ly_beta = measure(i+1)*rot_1;
   landmark_jac = [lx_r lx_beta; ly_r ly_beta];
   landmark_cov(i:i+1,i:i+1) = landmark_jac * measure_cov * landmark_jac';
end

for i = 1:2:2*k
    lx = pose(1) + measure(i+1)*cos(pose(3) + measure(i));
    ly = pose(2) + measure(i+1)*sin(pose(3) + measure(i));
    landmark(i) = lx;
    landmark(i+1) = ly;
end

%==== Setup state vector x with pose and landmark vector ====
x = [pose ; landmark];

%==== Setup covariance matrix P with pose and landmark covariances ====
P = [pose_cov zeros(3, 2*k) ; zeros(2*k, 3) landmark_cov];


%==== Plot initial state and covariance ====
last_x = x;
drawTrajAndMap(x, last_x, P, 0);

%==== Read control data ====
F_x = [eye(3) zeros(3,2*k)];
tline = fgets(fid);
while ischar(tline)
    arr = str2num(tline);
    d = arr(1);
    alpha = arr(2);
    
    %==== TODO: Predict Step ====
    %==== (Notice: predict state x_pre[] and covariance P_pre[] using input control data and control_cov[]) ====
    
    % Write your code here...
    x_motion = [d*cos(x(3)); d*sin(x(3)); alpha];
    x_pre = x + F_x' * x_motion;
    G_x = [1 0 -d*sin(x(3)); 0 1 d*cos(x(3)); 0 0 1];
    G = [G_x zeros(3,2*k); zeros(2*k,3) eye(2*k)];
    %G_temp = F_x'*G_x*F_x;
    
    U_x = [cos(x(3)) sin(x(3)) 0; -sin(x(3)) cos(x(3)) 0; 0 0 1];
    U = U_x * control_cov * U_x';
    U_transformed = [U zeros(3,2*k); zeros(2*k,2*k+3)];
    P_pre = (G * P * G') + U_transformed;
    
    %==== Draw predicted state x_pre[] and covariance P_pre[] ====
    drawTrajPre(x_pre, P_pre);
    
    %==== Read measurement data ====
    tline = fgets(fid);
    arr = str2num(tline);
    measure = arr';
    
    %==== TODO: Update Step ====
    %==== (Notice: update state x[] and covariance P[] using input measurement data and measure_cov[]) ====
    
    % Write your code here...
    for j = 1:k
        delta_x = x_pre(3+(2*j)-1) - x_pre(1);
        delta_y = x_pre(4+(2*j)-1) - x_pre(2);
        delta = [delta_x; delta_y];
        q = delta'*delta;
        z_est = [sqrt(q); wrapToPi(atan2(delta_y,delta_x)-x_pre(3))];
        F_big = zeros(5,(2*k)+3);
        F_big(1:3,1:3) = eye(3);
        F_idx = 4+(2*j)-2;
        F_big(4:5,F_idx:F_idx+1) = eye(2);
        H = (1/q) * [-sqrt(q)*delta_x, -sqrt(q)*delta_y, 0,  sqrt(q)*delta_x, sqrt(q)*delta_y; delta_y, -delta_x, -q, -delta_y, delta_x] * F_big;
        K = P_pre * H' * inv(H * P_pre * H' + measure_cov);
        z = [measure(2*j);measure(2*j-1)];
        x_pre = x_pre + (K * (z - z_est));
        P_pre = (eye(2*k+3) - K*H) * P_pre;
    end
    
    x = x_pre;
    P = P_pre;
    
    %==== Plot ====   
    drawTrajAndMap(x, last_x, P, t)
    last_x = x;
    
    %==== Iteration & read next control data ===
    t = t + 1;
    tline = fgets(fid);
end

%==== EVAL: Plot ground truth landmarks ====

% Write your code here...
landmark_true = [3 6; 3 12; 7 8; 7 14; 11 6; 11 12];
landmark_pred = reshape(x(4:end), 2, k)';
scatter(landmark_true(:,1), landmark_true(:,2), '*b');
diff = (landmark_true - landmark_pred).^ 2;
euclidean_dist = sqrt(sum(diff, 2));

mahalanobis_dist = zeros(k, 1);
gt = landmark_true';
gt = gt(:);
for i=1:k
    dif = x(2*i+2:2*i+3)' - gt(2*i-1:2*i)';
    sig = P(2*i + 2:2*i + 3, 2*i + 2:2*i + 3);
    mahalanobis_dist(i) = sqrt(dif*inv(sig)*dif');
end

disp(euclidean_dist);
disp(mahalanobis_dist);

%==== Close data file ====
fclose(fid);
