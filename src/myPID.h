typedef struct {
    double kp;              // Ganho proporcional
    double ki;              // Ganho integral
    double kd;              // Ganho derivativo
    double setpoint;        // Ponto de ajuste
    double integral;        // Componente integral
    double int_sat;         // Ponto de ajuste
    double previous_error;  // Erro anterior
    double output_min;      // Saída mínima
    double output_max;      // Saída máxima
} pid_controller_t;


double pid_compute(pid_controller_t *pid, double measurement) {
    double error = pid->setpoint - measurement;
    pid->integral +=  error ;
    double derivative = error - pid->previous_error;
    pid->previous_error = error;
    double output = (pid->kp * error) + (pid->ki * pid->integral) + (pid->kd * derivative);
    // Limita a saída entre os valores mínimo e máximo
    if (output > pid->output_max) {
        output = pid->output_max;
    } else if (output < pid->output_min) {
        output = pid->output_min;
    }
    return output;
}
