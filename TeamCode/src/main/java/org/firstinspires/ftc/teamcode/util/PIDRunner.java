package org.firstinspires.ftc.teamcode.util;
public class PIDRunner {
    double kP = 0.0;
    double kI = 0.0;
    double kD = 0.0;
    double oldTime = 0.0;
    double integralError;
    double lastError;
    public PIDRunner(double kP, double kI, double kD, double oldTime) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.oldTime = oldTime;
        integralError = 0.0;
        lastError = 0.0;
    }

    public double calculate(double desired, double actual, double newTime) {
        double dt = newTime - oldTime;
        oldTime = newTime;
        if (dt > 0.2){
            dt = 0.02;
        }
        // first calculate new error
        double error = desired - actual;
        // calculate integral
        integralError += error * dt;
        // calculate derivative
        double derivativeError = (error - lastError) / dt;
        // calculate output
        double output = kP * error + kI * integralError + kD * derivativeError;
        // update last error
        lastError = error;
        // return output
        return output;
    }

    public void reset() {
        integralError = 0.0;
    }

    public void setConstants(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    
    
}
