#ifndef PWM_INPUT_H
#define PWM_INPUT_H

void update_adrc_gains(const struct shell *shell, size_t argc, char **argv){
	float dt = strtof(argv[0], NULL);
    float wo = strtof(argv[1], NULL);
    float bo = strtof(argv[2], NULL);
    float kp = strtof(argv[3], NULL);
    float kd = strtof(argv[4], NULL);
}


void readings(const struct shell *shell, size_t argc, char **argv){
	printk("%0.1f,%0.1f,%0.1f,%d,%d,%lld,%0.4f\r\n",233.3f,24.5f,6.3f,1330,1530,k_uptime_get(),0.232f);
}


#endif // PWM_INPUT_H
