
function [theta3, r4] = calc_theta3_r4(r1, r2, r3, r5, theta1, theta2)
% Calcula theta3 y r4 (que es r_j en el modelo del libro)
% usando el método del Fifth Case de Dyer & Constans

% Paso 1: Vector de cierre
b_vec = - r1 * [cos(theta1); sin(theta1)] - r2 * [cos(theta2); sin(theta2)];
bx = b_vec(1); by = b_vec(2);
b = norm(b_vec);
alpha = atan2(by, bx);  % dirección de b

% Paso 2: Ángulos relativos del modelo
gamma = pi/2;    % entre r3 y r4
beta  = pi;      % entre r3 y r5

% Paso 3: Calcular coeficientes c y d de la cuadrática (ec. 2.49)
c = 2 * r3 * cos(gamma) + 2 * r5 * cos(gamma - beta);
d = r3^2 + r5^2 + 2 * r3 * r5 * cos(beta) - b^2;

% Paso 4: Resolver cuadrática (ec. 2.50)
discriminant = (c/2)^2 - d;
if discriminant < 0
    error('No hay solución real para r4');
end
r4 = -c/2 + sqrt(discriminant);  % usamos la raíz positiva
r4 = (r4 - 12) * (-1) + 12;

% Paso 5: Calcular A y B (ec. 2.51 y 2.52)
A = (r3 * sin(gamma) + r5 * sin(gamma - beta)) / b;
B = (r3 * cos(gamma) + r4 + r5 * cos(gamma - beta)) / b;

% Paso 6: Calcular zeta*
zeta = atan2(A, B);

% Paso 7: Determinar zeta (ec. 2.53)
% if A > 0 && B > 0
%     zeta = zeta_star;
% elseif A > 0 && B < 0
%     zeta = pi - zeta_star;
% elseif A < 0 && B < 0
%     zeta = pi + zeta_star;
% elseif A < 0 && B > 0
%     zeta = 2*pi - zeta_star;
% elseif A == 0 && B > 0
%     zeta = 0;
% end

% Paso 8: Calcular theta3
theta3 = alpha - zeta + gamma;

end
