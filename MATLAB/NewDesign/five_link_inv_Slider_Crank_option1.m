close all;clear;clc;

%% Plot Configuration
set(0, 'DefaultLineLineWidth', 2);
set(0,'DefaultLineMarkerSize',5);
set(0,'defaultAxesFontSize',10);

%% Linkage dimensions 
a = 3;      % crank length (m) [r2]
d = 5*a;    % length between ground pins A and D (m) [r1]
b = 1.5*a;  % Rocker length (m) [r3]
e = 30;     % Slider length EP (m) [length of platform]

%% Actuator Configuration (DE)
% Mode selection: 'fixed', 'extend_contract', 'extend_only'
actuator_mode = 'extend_contract';  % Change this to select mode

c_initial = 1.5*a;  % Initial length of DE (m)
c_min = 1*a;      % Minimum length of DE (m)
c_max = 3*a;      % Maximum length of DE (m)
c_velocity = 25.0;   % Actuator velocity (m/s) - positive for extension, negative for contraction

% For fixed mode
c_fixed = 1.5*a;    % Fixed length when actuator_mode = 'fixed'

%% Ground Configuration
theta1 = 0*(pi/180); % Angle between AD and 'x' axis

% Ground pins
rA = [0; 0]; % ground pin at A (origin)
[eAD,nAD] = UnitVector(theta1);
rD = FindPos(rA, d, eAD); % ground pin at D

%% Simulation configuration
tf = 4;      % Set Time in seconds
dt = 0.01;   % Set Sampling time
t = 0:dt:tf; % Time vector

% Variables to accumulate pos, vel and acc of P
rP_v = zeros(2,size(t,2)); 
vP_v = zeros(2,size(t,2)); 
aP_v = zeros(2,size(t,2)); 

% Vector generation
theta2 = zeros(size(t));
theta3 = zeros(size(t));
theta4 = zeros(size(t));
c_length = zeros(size(t)); % Variable length of DE

%% Initialize Crank Motion
theta2(1) = 0*pi/180;  % Set Crank initial angle
fth2 = 8*pi;           % Set final theta2 angle
w2 = fth2/tf;          % Angular velocity (rad/s)
for i = 2:tf/dt 
    theta2(i) = theta2(i-1) + dt*w2;
end

%% Initialize Actuator Motion
% Set initial actuator state
c_length(1) = c_initial;
actuator_direction = 1;  % 1 for extension, -1 for contraction

% Generate actuator length trajectory based on mode
for i = 2:length(t)
    switch actuator_mode
        case 'fixed'
            c_length(i) = c_fixed;
            
        case 'extend_contract'
            % Calculate next position
            c_next = c_length(i-1) + actuator_direction * c_velocity * dt;
            
            % Check bounds and reverse direction if needed
            if c_next >= c_max
                c_length(i) = c_max;
                actuator_direction = -1;  % Start contracting
            elseif c_next <= c_min
                c_length(i) = c_min;
                actuator_direction = 1;   % Start extending
            else
                c_length(i) = c_next;
            end
            
        case 'extend_only'
            % Extend to maximum and stay there
            c_next = c_length(i-1) + c_velocity * dt;
            c_length(i) = min(c_next, c_max);
    end
end

%% Plot Settings
LinkColor = [14 103 180]/255;
% Dynamic axis limits based on mechanism dimensions
f = 1.2;
xl = -(e/2)*f; xu = (e/2)*f; 
yl = -(a)*f; yu = (c_max)*f;
sp = 0.75;

%% Main Simulation Loop
figure(1),
for k = 1:(tf/dt+1)
    
    % Find position of B
    [eAB,nAB] = UnitVector(theta2(k));
    rB = FindPos(rA, a, eAB);
    
    % Find position of E (vertical actuator, Option 1)
    theta5 = pi/2;  % DE is always vertical
    [eDE,nDE] = UnitVector(theta5);
    rE = FindPos(rD, c_length(k), eDE);
    
    % Find position of C and angle theta4 using Option 1 kinematics
    [theta3(k), theta4(k)] = calc_theta3_theta4_option1(rA, rB, rD, rE, b, e);
    
    % VERIFICATION: Check that BC is perpendicular to PE
    angle_diff = abs(theta3(k) - theta4(k));
    angle_diff = mod(angle_diff, 2*pi);
    if abs(angle_diff - pi/2) > 0.01 && abs(angle_diff - 3*pi/2) > 0.01
        warning('BC is not perpendicular to PE at time %.2f', t(k));
    end
    
    % Calculate position of C
    [eBC,nBC] = UnitVector(theta3(k));
    rC = FindPos(rB, b, eBC);
    
    % Find position of P
    [eEP,nEP] = UnitVector(theta4(k));
    rP = FindPos(rE, e, eEP);
    rP_v(:,k) = rP;
    
    % Velocities (simplified for Option 1)
    % Angular velocity of BC
    if k > 1
        w3 = (theta3(k) - theta3(k-1))/dt;
        w4 = (theta4(k) - theta4(k-1))/dt;
        c_vel = (c_length(k) - c_length(k-1))/dt;
    else
        w3 = 0; w4 = 0; c_vel = 0;
    end
    
    vD = [0;0]; 
    vE = FindVel(vD, c_vel, [0;1]);  % E moves vertically
    vP = FindVel(vE, e, w4, nEP);
    vP_v(:,k) = vP;
    
    % Accelerations (simplified)
    if k > 1
        a3 = (w3 - w3_prev)/dt;
        a4 = (w4 - w4_prev)/dt;
        c_acc = (c_vel - c_vel_prev)/dt;
    else
        a3 = 0; a4 = 0; c_acc = 0;
        w3_prev = 0; w4_prev = 0; c_vel_prev = 0;
    end
    
    aD = [0;0];
    aE = [0; c_acc];  % E accelerates only vertically
    aP = FindAcc(aE, e, w4, a4, eEP, nEP);
    aP_v(:,k) = aP;
    
    % Store previous values
    w3_prev = w3; w4_prev = w4; c_vel_prev = c_vel;
    
    % Plot Simulation
    plot_inv_Slider_Crank_option1(rA,rB,rC,rD,rE,rP,rP_v,c_length(k),c_min,c_max,sp,t(k), LinkColor);
    axis([xl xu yl yu]); 
    axis equal
    hold off;
    
    % Add status text
    % text(xu-5, yu-1, sprintf('Time: %.2f s', t(k)), 'FontSize', 10);
    % text(xu-5, yu-2, sprintf('Actuator: %.2f m', c_length(k)), 'FontSize', 10);
    % text(xu-5, yu-3, sprintf('Mode: %s', actuator_mode), 'FontSize', 10);
    
    pause(dt);
end

%% Plot Results
% Plot position, velocity and acceleration of P
figure(2), clf;
subplot(3,1,1), plot(t,rP_v(1,:),'-','Color',"#1171BE"); grid on; hold on;
plot(t,rP_v(2,:),'-','Color',"#8516D1");
ylabel('Position (m)'); title('Position of P'); 
legend('$P_x$','$P_y$','interpreter','latex');

subplot(3,1,2), plot(t,vP_v(1,:),'-','Color',"#2FBEEF"); grid on; hold on;
plot(t,vP_v(2,:),'-','Color',"#D1048B");
ylabel('Velocity (m/s)'); title('Velocity of P'); 
legend('$V_x$','$V_y$','interpreter','latex');

subplot(3,1,3), plot(t,aP_v(1,:),'-','Color',"#EDB120"); grid on; hold on;
plot(t,aP_v(2,:),'-','Color',"#3BAA32"); xlabel('Time (s)'); 
ylabel('Acceleration (m/s^2)'); title('Acceleration of P'); 
legend('$A_x$','$A_y$','interpreter','latex');

% Plot angles and actuator length
figure(3), clf;
subplot(3,1,1), plot(t, theta3*180/pi); grid on;
ylabel('Angle (deg)'); title('Angle $\theta_3$ (BC)', 'interpreter','latex');

subplot(3,1,2), plot(t, theta4*180/pi); grid on;
ylabel('Angle (deg)'); title('Angle $\theta_4$ (EP)', 'interpreter','latex');

subplot(3,1,3), plot(t, c_length, 'r-', 'LineWidth', 2); grid on; hold on;
plot([0 tf], [c_min c_min], 'k--', 'LineWidth', 1);
plot([0 tf], [c_max c_max], 'k--', 'LineWidth', 1);
ylabel('Length (m)'); xlabel('Time (s)');
title('Actuator Length DE', 'interpreter','latex');
legend('Length', 'Min', 'Max', 'Location', 'best');