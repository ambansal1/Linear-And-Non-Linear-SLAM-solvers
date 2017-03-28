% ERROR_NONLINEAR
% 16-831 Fall 2016 - *Stub* Provided
% Computes the total error of all measurements (odometry and landmark)
% given the current state estimate
%
% Arguments: 
%     x       - Current estimate of the state vector
%     odom    - Matrix that contains the odometry measurements
%               between consecutive poses. Each row corresponds to
%               a measurement. 
%                 odom(:,1) - x-value of odometry measurement
%                 odom(:,2) - y-value of odometry measurement
%     obs     - Matrix that contains the landmark measurements and
%               relevant information. Each row corresponds to a
%               measurement.
%                 obs(:,1) - idx of pose at which measurement was 
%                   made
%                 obs(:,2) - idx of landmark being observed
%                 obs(:,3) - x-value of landmark measurement
%                 obs(:,4) - y-value of landmark measurement
%     sigma_o - Covariance matrix corresponding to the odometry
%               measurements
%     sigma_l - Covariance matrix corresponding to the landmark
%               measurements
% Returns:
%     err     - total error of all measurements
%
function err = error_nonlinear(x, odom, obs, sigma_odom, sigma_landmark)


sigma_o = sigma_odom;
sigma_l = sigma_landmark;
n_poses = size(odom, 1) + 1;                % +1 for prior on the first pose
n_landmarks = max(obs(:,2));

n_odom = size(odom, 1);
n_obs  = size(obs, 1);

% Dimensions of state variables and measurements (all 2 in this case)
p_dim = 2;                                  % pose dimension
l_dim = 2;                                  % landmark dimension
o_dim = size(odom, 2);                      % odometry dimension
m_dim = size(obs(1, 3:end), 2);             % landmark measurement dimension

% A matrix is MxN, b is Nx1
N = p_dim*n_poses + l_dim*n_landmarks;
M = o_dim*(n_odom+1) + m_dim*n_obs;         % +1 for prior on the first pose

%% Initialize matrices
A = zeros(M, N);
b = zeros(M, 1);
observ = zeros(M,1);
observat = zeros(M,1);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%% Your code goes here %%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Adash = zeros(M, N);
odomJ = ((inv(sigma_o))^0.5)*[ -1, 0 ;  0, -1 ];

odomJ2 = [ 1, 0 ;  0, 1 ]
odomJ2 = ((inv(sigma_o))^0.5)*odomJ2;

tempm = eye((n_odom));
A(3:(2+o_dim*(n_odom)),1:(o_dim*(n_odom))) = kron(tempm,odomJ);
Adash(3:(2+o_dim*(n_odom)),3:(2+o_dim*(n_odom))) = kron(tempm,odomJ2);

A = A+Adash;
count  = o_dim*(n_odom+1)+1;

length = size(obs,1);
odom = odom';
b(3:2+o_dim*(n_odom)) = reshape(odom,(o_dim*(n_odom)),1);
sigmat = (inv(sigma_l))^(0.5);

sigma = sigmat(1);

sigmatemp = (inv(sigma_o))^(0.5);

sigma2 = sigmatemp(1);
A(1,1)= sigma2;
 A(2,2) = sigma2;
Astar = A./sigma2;
for  i = 1:length

temp = obs(i,:);

robotindex = temp(1);
landmarkindex = temp(2);
rx = x((2*(temp(1)-1))+1);
ry = x((2*(temp(1)-1))+2);
lx =  x(o_dim*(n_odom+1) +  (2*(temp(2)-1))+1);
ly =  x(o_dim*(n_odom+1) +  (2*(temp(2)-1))+2);
H = meas_landmark_jacobian(rx,ry,lx,ly);
H = H.*sigma;
A(count,(p_dim*(n_poses) + l_dim*(temp(2)-1) +1):(p_dim*(n_poses) + l_dim*(temp(2)-1) +2))  = H(1,3:4) ;
A(count, ((2*(temp(1)-1))+1): (2*(temp(1)-1))+2) = H(1,1:2);

A(count+1,(p_dim*(n_poses) + l_dim*(temp(2)-1) +1):(p_dim*(n_poses) + l_dim*(temp(2)-1) +2))  = H(2,3:4) ;
A(count+1, ((2*(temp(1)-1))+1): (2*(temp(1)-1))+2) = H(2,1:2);
b(count) = temp(3);
b(count+1) = temp(4);

h = meas_landmark(rx, ry, lx, ly);
observ(count,1) = h(1);
observ((count+1),1) = h(2);

count = count +2 ;

end

 A(1,1)= 1;
 A(2,2) = 1;
observ1 = Astar*x;
observat = observ+observ1;

% x = solve_pinv(A, b);
% [traj, landmarks] = format_solution(x, n_poses, n_landmarks, p_dim, l_dim);
b1 = b;
b1(3:o_dim*(n_odom+1)) = (-observat(3:o_dim*(n_odom+1))+b(3:o_dim*(n_odom+1))).*sigma2;
b1((o_dim*(n_odom+1)+1):end) = (-observat(o_dim*(n_odom+1)+1:end)+ b(o_dim*(n_odom+1)+1:end)).*sigma;

b = b1;

err = b'*b;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%% Your code goes here %%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%