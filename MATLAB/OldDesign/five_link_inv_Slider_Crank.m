close all;clear;clc;

%Plot Configuration
set(0, 'DefaultLineLineWidth', 2);
set(0,'DefaultLineMarkerSize',5);
set(0,'defaultAxesFontSize',10);

% Linkage dimensions 
a = 3; % crank length (m) [r2]
d = 5*a; % length between ground pins A and D (m) [r1]
b = 1.5*a; % Rocker length (m) [r3]
e = 20; % Slider length (m) [r4 + CP]
c = 1.5*a; % Slider heigth (m) [r5]

%a = 0.015; % crank length (m) [r2]
%d = 0.12; % length between ground pins A and D (m) [r1]
%b = 0.045; % [r3]
%e = 0.25; % Slider length(m) [r4 + CP]
%c = 0.045; % Rocker length (m) [r5]

theta1 = 0*(pi/180); % Angle between AD and 'x' axis
d_sc=0.1; bs=1; %Slide base lengths

% Ground pins
rA = [0; 0]; % ground pin at A (origin)
[eAD,nAD] = UnitVector(theta1);
rD = FindPos(rA,d,eAD); % ground pin at D

%Simulation configuration
tf = 2; %Set Time in seconds
dt = 0.01; %Set Sampling time
t = 0:dt:tf; %Time vector

%Variables to accumulate pos, vel and acc of P
rP_v = zeros(2,size(t,2)); 
vP_v = zeros(2,size(t,2)); 
aP_v = zeros(2,size(t,2)); 

%Vector generation
theta2 = zeros(size(t));
theta3 = zeros(size(t));
r4 = zeros(size(t));

theta2(1) = 0*pi/180; %Set Crank initial angle
fth2 = 4*pi; %Set final theta2 angle
%Angular velocity (rad/s)
w2=fth2/tf; %Set clockwise (-) or counterclockwise movement (+)
for i = 2:tf/dt 
    theta2(i) = theta2(i-1)+dt*w2;
end

%Color definitions for plots
LinkColor = [14 103 180]/255;
%Axis lims
f = 1.25;
xl = -(e-d)*f; xu = d*f; yl = -a*f; yu = (a+b)*f; sp = 0.5;

figure(1),
for k = 1:(tf/dt+1)

    % Find position of B
    [eAB,nAB] = UnitVector(theta2(k));
    rB = FindPos(rA,a,eAB);
    
    % Find position of C
    [theta3(k), r4(k)] = calc_theta3_r4(d, a, b, c, theta1, theta2(k));
    %theta3(k) = -theta3(k);
    theta3(k) = 2*pi-mod(theta3(k), 2*pi);
    [eBC,nBC] = UnitVector(theta3(k));
    rC = FindPos(rB,b-d_sc,eBC);

    % Find position of E
    theta5 = theta3(k);
    [eDE,nDE] = UnitVector(theta5);
    rE = FindPos(rD,c,eDE);
    
    % Find position of P
    %theta4 = theta3(k) - pi/2;
    theta4 =  theta3(k) + pi/2;
    [eEP,nEP] = UnitVector(theta4);
    rP = FindPos(rE,e,eEP);
    rP_v(:,k) = rP;

    %Positions of the slide base limits
    rBS = FindPos(rB,b-d_sc,eBC);
    rL1 = FindPos(rBS,bs/2,eEP);
    rL2 = FindPos(rBS,bs/2,-eEP);

    % Velocities
    w3 = w2*(a*sin(theta2(k)-theta3(k))/r4(k));
    vD = [0;0]; aD = [0;0]; 
    vE = FindVel( vD, c, w3 ,nDE);   
    vP = FindVel( vE, e, w3, nEP);
    vP_v(:,k) = vP;
    
    % Accelerations
    v4 = a*w2*(cos(theta3(k)-theta2(k))+(c-b)/r4(k)*sin(theta3(k)-theta2(k)));
    a3 = 1/r4(k)*(-w3*v4+a*w2*(w2-w3)*cos(theta2(k)-theta3(k)));
    aE = FindAcc(aD, c, w3, a3, eDE, nDE);
    aP = FindAcc(aE, e, w3, a3, eEP, nEP);
    aP_v(:,k) = aP;

    %Plot Simulation
    plot_inv_Slider_Crank_mod(rA,rB,rC,rBS,rL1,rL2,rD,rE,rP,rP_v,sp,t(k), LinkColor);
    axis([xl xu yl yu]); 
    axis equal
    hold off;
    pause(dt);
        
end

% Plot position, velocity and acceleration of P
figure(2), clf;
subplot(3,1,1), plot(t,rP_v(1,:),'-','Color',"#1171BE"); grid on; hold on;
plot(t,rP_v(2,:),'-','Color',"#8516D1"); %xlabel('Time (s)'); 
ylabel('Position (m)'); title('Position of P'); 
legend('$P_x$','$P_y$','interpreter','latex');
subplot(3,1,2), plot(t,vP_v(1,:),'-','Color',"#2FBEEF"); grid on; hold on;
plot(t,vP_v(2,:),'-','Color',"#D1048B"); %xlabel('Time (s)'); 
ylabel('Velocity (m/s)'); title('Velocity of P'); 
legend('$P_x$','$P_y$','interpreter','latex');
subplot(3,1,3), plot(t,aP_v(1,:),'-','Color',"#EDB120"); grid on; hold on;
plot(t,aP_v(2,:),'-','Color',"#3BAA32"); xlabel('Time (s)'); 
ylabel('Acceleration (m/s^2)'); title('Acceleration of P'); 
legend('$P_x$','$P_y$','interpreter','latex');

% Plot theta 3
figure(3), clf; 
plot(t(1:end-1),theta3(1:end-1)),
xlabel('Time (s)'); ylabel('Angular Position (rad)'); title('$\theta_3$ vs Time','interpreter','latex'); 
legend('$\theta_3$','interpreter','latex');
