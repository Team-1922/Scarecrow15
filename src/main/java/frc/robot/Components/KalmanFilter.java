package frc.robot.Components;

import java.lang.Math;

public class KalmanFilter {

    private int size;
    private double total = 0.0;
    private int index = 0;
    private double samples[];

    public KalmanFilter(int size) {
        this.size = size;
        samples = new double[size];
        for (int i = 0; i < size; i++) samples[i] = 0d;
    }


/*
    int kalman_filter(){
        //prediction
        x_hat_k_a_priori = x_hat_k_minus_1;
        P_k_a_priori = P_k_minus_1 + Q;
        
        //obtaining z_k: my tank height = 25.4, calculates %
        digitalWrite(trigger, LOW);
        delayMicroseconds(2);
        digitalWrite(trigger, HIGH);
        delayMicroseconds(10);
        digitalWrite(trigger, LOW);
        duration = pulseIn(echo, HIGH);
        z_k = (25.4-duration*0.017)/25.4; 
        
        //innovation 
        K_k = P_k_a_priori * (P_k_a_priori + R);
        x_hat_k = x_hat_k_a_priori + K_k * (z_k - x_hat_k_a_priori);
        P_k = (1 - K_k) * P_k_a_priori;
        
        return x_hat_k;
    }
*/
  

}