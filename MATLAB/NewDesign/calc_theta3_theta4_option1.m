function [theta3, theta4] = calc_theta3_theta4_option1(rA, rB, rD, rE, b, e)
% calc_theta3_theta4_option1
% Calculates theta3 and theta4 for Option 1 configuration where:
% - DE is always vertical (theta5 = pi/2)
% - Point C must lie on line EP
% - EP can rotate freely around E
%
% Inputs:
%   rA - Position of point A (2x1 vector)
%   rB - Position of point B (2x1 vector)
%   rD - Position of point D (2x1 vector)
%   rE - Position of point E (2x1 vector)
%   b  - Length of link BC (scalar)
%   e  - Length of link EP (scalar)
%
% Outputs:
%   theta3 - Angle of link BC
%   theta4 - Angle of link EP

% Calculate current positions
Bx = rB(1); By = rB(2);
Ex = rE(1); Ey = rE(2);

% We need to find theta3 such that C lies on a line passing through E
% Position of C: C = B + b*[cos(theta3); sin(theta3)]
% 
% For C to lie on line EP, we need to satisfy:
% The constraint that C is at distance b from B AND
% C must be on some line passing through E

% This is a geometric problem: find the intersection of:
% 1. Circle centered at B with radius b
% 2. Lines passing through E

% We can solve this by finding theta3 such that when we place C,
% there exists a valid theta4 that makes C lie on EP

% Vector from E to B
EB = rB - rE;
EB_dist = norm(EB);

% If B is too far from E, there might be no solution
if EB_dist > b + e
    warning('No solution: B is too far from E');
    % Return approximate solution
    theta3 = atan2(rE(2)-rB(2), rE(1)-rB(1));
    theta4 = theta3;
    return;
end

% We'll use an iterative approach to find theta3
% Initial guess: point C toward E
theta3_init = atan2(Ey - By, Ex - Bx);

% Optimization function: minimize the distance from C to line EP
options = optimset('Display', 'off', 'TolFun', 1e-10, 'TolX', 1e-10);

% Define the objective function
obj_func = @(th3) compute_error(th3, Bx, By, Ex, Ey, b, e);

% Find optimal theta3
theta3 = fminsearch(obj_func, theta3_init, options);

% Normalize angle to [0, 2*pi]
theta3 = mod(theta3, 2*pi);

% Calculate position of C with found theta3
Cx = Bx + b * cos(theta3);
Cy = By + b * sin(theta3);

% Calculate theta4: angle from E to C
theta4 = atan2(Cy - Ey, Cx - Ex);

% Normalize angle to [0, 2*pi]
theta4 = mod(theta4, 2*pi);

end

function error = compute_error(theta3, Bx, By, Ex, Ey, b, e)
    % Calculate position of C for given theta3
    Cx = Bx + b * cos(theta3);
    Cy = By + b * sin(theta3);
    
    % Vector from E to C
    ECx = Cx - Ex;
    ECy = Cy - Ey;
    EC_dist = sqrt(ECx^2 + ECy^2);
    
    % Error is the difference between EC distance and platform length e
    % We want C to be on the platform, so it should be at some distance <= e from E
    if EC_dist <= e
        error = 0;  % C is on the platform
    else
        error = (EC_dist - e)^2;  % Penalty for being outside platform range
    end
    
    % Add a small penalty to prefer configurations where C is closer to the middle of EP
    % This helps convergence and gives more stable solutions
    ideal_dist = e * 0.5;  % Prefer C to be in the middle of the platform
    error = error + 0.01 * (EC_dist - ideal_dist)^2;
end