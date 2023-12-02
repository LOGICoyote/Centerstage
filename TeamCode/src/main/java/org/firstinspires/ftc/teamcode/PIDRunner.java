package org.firstinspires.ftc.teamcode;

public class PIDRunner {
    double kP = 0.0;
    double kI = 0.0;
    double kD = 0.0;
    double dt = 0.0;
    double integralError;
    double lastError;
    public PIDRunner(double kP, double kI, double kD, double dt) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.dt = dt;
        integralError = 0.0;
        lastError = 0.0;
    }

    public double calculate(double desired, double actual) {
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
