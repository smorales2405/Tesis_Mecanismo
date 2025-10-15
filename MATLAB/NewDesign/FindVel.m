function v = FindVel(v0, L, dir_or_omega, n)
% FindVel - Modified version for Option 1
% Calculates velocity at a point
%
% For rotational motion (4 arguments):
%   v0 = velocity of first point
%   L = length of vector to second point on the link
%   omega = angular velocity of link
%   n = unit normal to vector btw first and second points
%   v = velocity of second point
%
% For linear motion (3 arguments):
%   v0 = velocity of first point
%   L = velocity magnitude
%   dir = direction vector
%   v = velocity of second point

if nargin == 4
    % Rotational motion - original formula
    omega = dir_or_omega;
    v = v0 + omega * L * n;
elseif nargin == 3
    % Linear motion - direct velocity
    dir = dir_or_omega;
    v = v0 + L * dir;
else
    error('FindVel requires either 3 or 4 arguments');
end

end
