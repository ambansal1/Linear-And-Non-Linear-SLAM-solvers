% CREATE_AB_LINEAR
% 16-831 Fall 2016 - *Stub* Provided
% Computes the A and b matrices for the 2D linear SLAM problem
%
% Arguments: 
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
%     A       - MxN matrix
%     b       - Mx1 vector
%
function [As, b] = create_Ab_linear(odom, obs, sigma_o, sigma_l)


% Useful Constants
n_poses = size(odom, 1) + 1; % +1 for prior on the first pose
n_landmarks = max(obs(:,2));

n_odom = size(odom, 1);
n_obs  = size(obs, 1);

% Dimensions of state variables and measurements (all 2 in this case)
p_dim = 2;
l_dim = 2;
o_dim = size(odom, 2);
m_dim = size(obs(1, 3:end), 2);

% A matrix is MxN, b is Nx1
N = p_dim*n_poses + l_dim*n_landmarks;
M = o_dim*(n_odom+1) + m_dim*n_obs;     % +1 for prior on the first pose

% Initialize matrices
A = zeros(M, N);
b = zeros(M, 1);

% Add odometry and landmark measurements to A, b - including prior on first
% pose



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
for  i = 1:length

temp = obs(i,:);


A(count,(p_dim*(n_poses) + l_dim*(temp(2)-1) +1):(p_dim*(n_poses) + l_dim*(temp(2)-1) +2))  = [sigma,0] ;
A(count, ((2*(temp(1)-1))+1): (2*(temp(1)-1))+2) = [-sigma,0];

A(count+1,(p_dim*(n_poses) + l_dim*(temp(2)-1) +1):(p_dim*(n_poses) + l_dim*(temp(2)-1) +2))  = [0,sigma] ;
A(count+1, ((2*(temp(1)-1))+1): (2*(temp(1)-1))+2) = [0,-sigma];
b(count) = temp(3);
b(count+1) = temp(4);
count = count +2 ;

end
 A(1,1)= 1;
 A(2,2) = 1;
% x = solve_pinv(A, b);
% [traj, landmarks] = format_solution(x, n_poses, n_landmarks, p_dim, l_dim);
b(3:o_dim*(n_odom+1)) = b(3:o_dim*(n_odom+1)).*sigma2;
b((o_dim*(n_odom+1)+1):end) = b(o_dim*(n_odom+1)+1:end).*sigma;

%% Make A a sparse matrix 
As = sparse(A);