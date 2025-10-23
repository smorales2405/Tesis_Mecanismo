function [theta3, theta4] = calc_theta3_theta4_opt(rB, rE, b, e)
% calc_theta3_theta4_option1
% Calculates theta3 and theta4 for Option 1 configuration where:
% - DE is always vertical (theta5 = pi/2)
% - BC must be PERPENDICULAR to PE (key constraint!)
% - PE rotates freely around E
%
% Inputs:
%   rA - Position of point A (2x1 vector)
%   rB - Position of point B (2x1 vector)  
%   rD - Position of point D (2x1 vector)
%   rE - Position of point E (2x1 vector)
%   b  - Length of link BC (scalar)
%   e  - Length of link PE (scalar)
%
% Outputs:
%   theta3 - Angle of link BC
%   theta4 - Angle of link PE

% Calculate current positions
Bx = rB(1); By = rB(2);
Ex = rE(1); Ey = rE(2);

% The key constraint: BC ⊥ PE
% This means: θ3 = θ4 ± π/2
% 
% We need to find θ4 such that when BC is perpendicular to PE,
% point C lies on the line defined by PE

% Vector from E to B
EB = [Bx - Ex; By - Ey];
EB_dist = norm(EB);

% We'll solve for theta4 using the constraint that C must lie on line EP
% when BC is perpendicular to EP

% Define the objective function
options = optimset('Display', 'off', 'TolFun', 1e-12, 'TolX', 1e-12);

% Initial guess for theta4
theta4_init = atan2(By - Ey, Bx - Ex);

% Optimization to find theta4
obj_func = @(th4) perpendicular_constraint_error(th4, Bx, By, Ex, Ey, b, e);
theta4 = fminsearch(obj_func, theta4_init, options);

% Normalize theta4
theta4 = mod(theta4, 2*pi);

% Calculate theta3 - BC must be perpendicular to PE
% We need to determine which perpendicular direction (+90° or -90°)
theta3_option1 = theta4 + pi/2;
theta3_option2 = theta4 - pi/2;

% Calculate C position for both options
C1 = [Bx; By] + b * [cos(theta3_option1); sin(theta3_option1)];
C2 = [Bx; By] + b * [cos(theta3_option2); sin(theta3_option2)];

% Vector from E to P
EP = e * [cos(theta4); sin(theta4)];
P = [Ex; Ey] + EP;

% Check which C is closer to the line EP (should be exactly on it)
% Calculate distance from C to line EP
dist1 = point_to_line_distance(C1, [Ex; Ey], P);
dist2 = point_to_line_distance(C2, [Ex; Ey], P);

if dist1 < dist2
    theta3 = theta3_option1;
else
    theta3 = theta3_option2;
end

% Normalize theta3
theta3 = mod(theta3, 2*pi);

end

function error = perpendicular_constraint_error(theta4, Bx, By, Ex, Ey, b, e)
    % Calculate position of C when BC is perpendicular to PE
    % Try both perpendicular directions
    
    theta3_1 = theta4 + pi/2;
    theta3_2 = theta4 - pi/2;
    
    % Position of C for both options
    C1x = Bx + b * cos(theta3_1);
    C1y = By + b * sin(theta3_1);
    C2x = Bx + b * cos(theta3_2);
    C2y = By + b * sin(theta3_2);
    
    % Vector from E in direction theta4 (direction of EP)
    EP_dir = [cos(theta4); sin(theta4)];
    
    % Vector from E to C for both options
    EC1 = [C1x - Ex; C1y - Ey];
    EC2 = [C2x - Ex; C2y - Ey];
    
    % Project EC onto EP direction
    % C should lie on line EP, so EC should be parallel to EP_dir
    % This means the cross product should be zero
    
    % Cross product in 2D (scalar value)
    cross1 = EC1(1)*EP_dir(2) - EC1(2)*EP_dir(1);
    cross2 = EC2(1)*EP_dir(2) - EC2(2)*EP_dir(1);
    
    % Return minimum error (one of them should be zero)
    error = min(abs(cross1), abs(cross2))^2;
    
    % Also check that C is within reasonable distance from E
    dist1 = norm(EC1);
    dist2 = norm(EC2);
    
    % Add penalty if C is too far from the platform
    if dist1 > e
        error = error + (dist1 - e)^2;
    end
    if dist2 > e
        error = error + (dist2 - e)^2;
    end
end

function dist = point_to_line_distance(point, line_start, line_end)
    % Calculate perpendicular distance from point to line
    % point: [x; y]
    % line_start, line_end: [x; y] vectors defining the line
    
    % Vector from line_start to line_end
    line_vec = line_end - line_start;
    line_length = norm(line_vec);
    
    if line_length == 0
        dist = norm(point - line_start);
        return;
    end
    
    % Unit vector along the line
    line_unit = line_vec / line_length;
    
    % Vector from line_start to point
    point_vec = point - line_start;
    
    % Project point_vec onto line
    projection_length = dot(point_vec, line_unit);
    
    % Find the closest point on the line
    if projection_length < 0
        closest = line_start;
    elseif projection_length > line_length
        closest = line_end;
    else
        closest = line_start + projection_length * line_unit;
    end
    
    % Distance from point to closest point on line
    dist = norm(point - closest);
end