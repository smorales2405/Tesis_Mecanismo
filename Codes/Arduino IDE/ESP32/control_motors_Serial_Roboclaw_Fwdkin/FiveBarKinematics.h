// FiveBarKinematics.h
#ifndef FIVE_BAR_KINEMATICS_H
#define FIVE_BAR_KINEMATICS_H

#include <Arduino.h>
#include <math.h>

// Estructura para almacenar resultados
struct KinematicsResult {
    float O_y;      // Posición Y del punto medio de PE (cm)
    float theta4;   // Ángulo de la barra PE (radianes)
    float O_x;      // Posición X del punto medio de PE (cm) - opcional
    bool valid;     // Indica si la solución es válida
};

// Estructura para configuración del mecanismo
struct MechanismConfig {
    float a = 7.03;     // Longitud del crank AB (cm)
    float b = 8.345;    // Longitud de BC (cm)
    float d = 52.0;     // Distancia horizontal AD (cm)
    float e = 70.0;     // Longitud de la plataforma PE (cm)
    float Dy = -3.6;    // Offset vertical del punto D (cm)
    float c_min = 6.0;  // Longitud mínima del actuador (cm)
    float c_max = 20.3; // Longitud máxima del actuador (cm)
};

class FiveBarMechanism {
private:
    MechanismConfig config;
    float theta3_prev = 0;  // Para mantener continuidad
    float theta4_prev = 0;
    bool has_previous = false;
    
    // Función auxiliar para calcular diferencia angular mínima
    float angleDifference(float angle1, float angle2) {
        float diff = angle1 - angle2;
        while (diff > PI) diff -= 2*PI;
        while (diff < -PI) diff += 2*PI;
        return diff;
    }
    
    // Función auxiliar para normalizar ángulo a [0, 2*PI]
    float normalizeAngle(float angle) {
        while (angle < 0) angle += 2*PI;
        while (angle >= 2*PI) angle -= 2*PI;
        return angle;
    }
    
    // Calcula theta3 y theta4 (versión simplificada de calc_theta3_theta4_nopt)
    bool calculateTheta3Theta4(float Bx, float By, float Ex, float Ey, 
                               float &theta3, float &theta4) {
        
        // Vector de E a B
        float EBx = Bx - Ex;
        float EBy = By - Ey;
        float EB_dist = sqrt(EBx*EBx + EBy*EBy);
        
        // Verificar si hay solución posible
        if (EB_dist > config.e + config.b) {
            // B está muy lejos
            theta4 = atan2(EBy, EBx);
            theta3 = theta4 + PI/2;
            return false;
        }
        
        // Teorema de Pitágoras para triángulo rectángulo EBC
        float EC_dist_sq = EB_dist*EB_dist - config.b*config.b;
        
        if (EC_dist_sq < 0) {
            // Configuración singular
            if (has_previous) {
                theta3 = theta3_prev;
                theta4 = theta4_prev;
            } else {
                theta4 = atan2(EBy, EBx);
                theta3 = theta4 + PI/2;
            }
            return false;
        }
        
        float EC_dist = sqrt(EC_dist_sq);
        
        // Ángulo de EB respecto al eje x
        float angle_EB = atan2(EBy, EBx);
        
        // Ángulo en E entre EC y EB
        float angle_at_E = acos(EC_dist / EB_dist);
        
        // Dos posibles posiciones para theta4
        float theta4_opt1 = angle_EB - angle_at_E;
        float theta4_opt2 = angle_EB + angle_at_E;
        
        // Para cada theta4, evaluar las perpendiculares
        float best_theta3 = 0;
        float best_theta4 = 0;
        float best_error = 1e6;
        
        for (int i = 0; i < 2; i++) {
            float th4 = (i == 0) ? theta4_opt1 : theta4_opt2;
            
            // Dos opciones perpendiculares para theta3
            for (int j = 0; j < 2; j++) {
                float th3 = th4 + ((j == 0) ? PI/2 : -PI/2);
                
                // Calcular posición de C
                float Cx = Bx + config.b * cos(th3);
                float Cy = By + config.b * sin(th3);
                
                // Vector EC
                float ECx = Cx - Ex;
                float ECy = Cy - Ey;
                float EC_norm = sqrt(ECx*ECx + ECy*ECy);
                
                if (EC_norm < 0.001) continue;  // Evitar división por cero
                
                ECx /= EC_norm;
                ECy /= EC_norm;
                
                // Dirección de PE
                float PE_dir_x = cos(th4);
                float PE_dir_y = sin(th4);
                
                // Error de alineación (producto cruz en 2D)
                float cross_product = abs(ECx*PE_dir_y - ECy*PE_dir_x);
                
                // Error de perpendicularidad
                float BC_dir_x = cos(th3);
                float BC_dir_y = sin(th3);
                float perp_error = abs(BC_dir_x*PE_dir_x + BC_dir_y*PE_dir_y);
                
                // Error total
                float total_error = cross_product + perp_error;
                
                // Si tenemos historia, penalizar cambios grandes
                if (has_previous) {
                    float dth3 = angleDifference(th3, theta3_prev);
                    float dth4 = angleDifference(th4, theta4_prev);
                    total_error += (dth3*dth3 + dth4*dth4) * 10;
                }
                
                // Actualizar mejor solución
                if (total_error < best_error) {
                    best_error = total_error;
                    best_theta3 = th3;
                    best_theta4 = th4;
                }
            }
        }
        
        theta3 = best_theta3;
        theta4 = best_theta4;
        
        // Guardar para la próxima iteración
        theta3_prev = theta3;
        theta4_prev = theta4;
        has_previous = true;
        
        return (best_error < 0.1);  // Retornar true si el error es aceptable
    }
    
public:
    // Constructor
    FiveBarMechanism() {}
    
    // Constructor con configuración personalizada
    FiveBarMechanism(MechanismConfig cfg) : config(cfg) {}
    
    // Función principal de cinemática directa
    KinematicsResult forwardKinematics(float theta2, float c_length) {
        KinematicsResult result;
        result.valid = false;
        
        // Validar entrada
        if (c_length < config.c_min || c_length > config.c_max) {
            return result;  // Actuador fuera de límites
        }
        
        // Paso 1: Calcular posición de B
        float Bx = config.a * cos(theta2);
        float By = config.a * sin(theta2);
        
        // Paso 2: Calcular posición de E (actuador vertical)
        float Ex = config.d;
        float Ey = config.Dy + c_length;  // D_y + longitud actuador
        
        // Paso 3: Calcular theta3 y theta4
        float theta3, theta4;
        bool success = calculateTheta3Theta4(Bx, By, Ex, Ey, theta3, theta4);
        
        if (!success) {
            return result;  // No se pudo resolver la cinemática
        }
        
        // Paso 4: Calcular posición del punto medio O de PE
        // P = E + e*[cos(theta4), sin(theta4)]
        // O = (E + P)/2 = E + (e/2)*[cos(theta4), sin(theta4)]
        
        float O_x = Ex + (config.e/2) * cos(theta4);
        float O_y = Ey + (config.e/2) * sin(theta4);
        
        // Llenar resultado
        result.O_x = O_x;
        result.O_y = O_y;
        result.theta4 = theta4;
        result.valid = true;
        
        return result;
    }
    
    // Resetear estado previo (útil al iniciar)
    void reset() {
        has_previous = false;
        theta3_prev = 0;
        theta4_prev = 0;
    }
    
    // Obtener configuración actual
    MechanismConfig getConfig() {
        return config;
    }
    
    // Actualizar configuración
    void setConfig(MechanismConfig cfg) {
        config = cfg;
        reset();  // Resetear estado al cambiar configuración
    }
};

#endif