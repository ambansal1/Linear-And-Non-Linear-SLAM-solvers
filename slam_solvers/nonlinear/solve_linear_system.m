% SOLVE_LINEAR_SYSTEM
% 16-831 Fall 2016 - *Stub* Provided
% Solve the linear system with your method of choice
%
% Arguments: 
%     A     - A matrix from linear system that you generate yourself
%     b     - b vector from linear system that you generate yourself
%
% Returns:
%     x     - solution to the linear system, computed using your method of
%             choice
%
function x = solve_linear_system(A, b)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%% Your code goes here %%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
[L,p,S] = chol((A'*A),'lower','matrix');

bdash = S'*A'*b;
z = forward_sub(L,bdash);
y = back_sub(L', z);

x = S*y;

R = L';