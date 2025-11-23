function [theta2, c_length, success] = inverse_kinematics_5bar(O_y, theta4)
% inverse_kinematics_5bar - Cinemática inversa para mecanismo de 5 barras
% 
% Entradas:
%   O_y     - Altura del punto medio O de la barra PE (m)
%   theta4  - Ángulo de la barra PE (rad)
%
% Salidas:
%   theta2    - Ángulo del crank (rad)
%   c_length  - Longitud del actuador DE (m)
%   success   - true si se encontró solución válida

% Inicializar
success = false;
theta2 = 0;
c_length = 0;

% Parámetros del mecanismo
a = 3;      % crank length
d = 15;     % distance AD
b = 4.5;    % length BC
e = 30;     % platform length

%% Paso 1: Calcular longitud del actuador directamente
c_length = O_y - (e/2)*sin(theta4);

% Verificar límites físicos del actuador
c_min = 1.0*a;  % Ajustar según tu sistema
c_max = 3.0*a;  % Ajustar según tu sistema

if c_length < c_min || c_length > c_max
    warning('Longitud del actuador fuera de límites: %.2f m', c_length);
    return;
end

%% Paso 2: Calcular posición de E
E_x = d;
E_y = c_length;

%% Paso 3: Calcular theta2 analíticamente
% La ecuación geométrica es:
% B está en un círculo de radio 'a' centrado en A(0,0)
% C debe estar sobre la línea PE
% BC tiene longitud b y es perpendicular a PE

% Dirección de PE
PE_dir = [cos(theta4); sin(theta4)];
PE_normal = [-sin(theta4); cos(theta4)];  % Normal a PE

% Para que BC sea perpendicular a PE y C esté en PE:
% B = C + b*PE_normal  (o B = C - b*PE_normal)

% C está en la línea PE, parametrizada como: C = E + t*PE_dir
% donde t es el parámetro a lo largo de PE

% B = A + a*[cos(theta2); sin(theta2)]
% Sustituyendo: A + a*[cos(theta2); sin(theta2)] = E + t*PE_dir ± b*PE_normal

% Esto da dos ecuaciones:
% a*cos(theta2) = E_x + t*cos(theta4) ± b*(-sin(theta4))
% a*sin(theta2) = E_y + t*sin(theta4) ± b*cos(theta4)

% Elevando al cuadrado y sumando: a^2 = (E + t*PE_dir ± b*PE_normal - A)^2

% Resolvemos para las dos opciones de signo
solutions_theta2 = [];

for sign_b = [-1, 1]
    % Para cada dirección perpendicular
    
    % La ecuación cuadrática en t es:
    % |E + t*PE_dir + sign_b*b*PE_normal|^2 = a^2
    
    % Expandiendo:
    % t^2 + 2*t*(E·PE_dir) + |E|^2 + 2*sign_b*b*(E·PE_normal) + b^2 = a^2
    
    % Coeficientes de la ecuación cuadrática
    A_coef = 1;  % |PE_dir|^2 = 1
    B_coef = 2*(E_x*cos(theta4) + E_y*sin(theta4));
    C_coef = E_x^2 + E_y^2 + 2*sign_b*b*(E_x*(-sin(theta4)) + E_y*cos(theta4)) + b^2 - a^2;
    
    % Discriminante
    discriminant = B_coef^2 - 4*A_coef*C_coef;
    
    if discriminant >= 0
        % Dos soluciones para t
        t1 = (-B_coef + sqrt(discriminant))/(2*A_coef);
        t2 = (-B_coef - sqrt(discriminant))/(2*A_coef);
        
        for t = [t1, t2]
            % Posición de C
            C_x = E_x + t*cos(theta4);
            C_y = E_y + t*sin(theta4);
            
            % Posición de B
            B_x = C_x + sign_b*b*(-sin(theta4));
            B_y = C_y + sign_b*b*cos(theta4);
            
            % Calcular theta2 desde B
            dist_AB = sqrt(B_x^2 + B_y^2);
            
            % Verificar que B está en el círculo correcto
            if abs(dist_AB - a) < 1e-6
                theta2_candidate = atan2(B_y, B_x);
                
                % Verificar que C está dentro del rango de la plataforma
                dist_EC = sqrt((C_x - E_x)^2 + (C_y - E_y)^2);
                if dist_EC >= 0 && dist_EC <= e
                    solutions_theta2 = [solutions_theta2; theta2_candidate];
                end
            end
        end
    end
end

% Seleccionar la mejor solución
if ~isempty(solutions_theta2)
    % Si hay múltiples soluciones, elegir la más razonable
    % Por ejemplo, la que da theta2 en el rango [0, 2*pi]
    solutions_theta2 = mod(solutions_theta2, 2*pi);
    
    % Preferir soluciones donde el crank no esté en posición extrema
    % (evitar theta2 cerca de pi/2 o 3*pi/2 si es posible)
    best_idx = 1;
    best_score = inf;
    
    for i = 1:length(solutions_theta2)
        th2 = solutions_theta2(i);
        % Penalizar posiciones verticales del crank
        score = min(abs(th2 - pi/2), abs(th2 - 3*pi/2));
        if score > best_score
            best_score = score;
            best_idx = i;
        end
    end
    
    theta2 = solutions_theta2(best_idx);
    success = true;
else
    warning('No se encontró solución válida para theta2');
end

end