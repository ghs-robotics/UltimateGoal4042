package org.firstinspires.ftc.teamcode.robot_components;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class PIDController {
    public double k_P; // TO DO: change to private later
    public double k_I;
    public double k_D;
    private double p_error;
    private double i_error;
    private double d_error;
    private double toleranceRadius; // PID won't adjust within this range
    private double min;
    private double max;
    private double prevError;
    private double prevTime; // measured in seconds!
    private ElapsedTime time;

    public PIDController(double Kp, double Ki, double Kd, double tolerance, double min, double max) {
        k_P = Kp;
        k_I = Ki;
        k_D = Kd;
        toleranceRadius = tolerance;
        this.min = min;
        this.max = max;
        time = new ElapsedTime();
        resetValues();
    }

    // Default values for min and max are -1.0 and 1.0, respectively
    public PIDController(double Kp, double Ki, double Kd, double tolerance) {
        this(Kp, Ki, Kd, tolerance, -1.0, 1.0);
    }

    // Restarts the PID controller
    public void resetValues() {
        p_error = 0;
        i_error = 0;
        d_error = 0;
        prevError = 0;
        prevTime = 0;
        time.reset();
    }

    // Calculates the power value to be sent to the motor(s)
    public double calcVal(double error) {
        // If the error is small enough, the robot won't adjust
        if (Math.abs(error) < toleranceRadius) { return 0; }

        // Calculates the different errors
        double deltaTime = time.seconds() - prevTime;
        p_error = error;
        i_error += error * deltaTime;
        d_error = (error - prevError) / deltaTime;

        // Updates the "prev" variables for the next loop
        prevError = error;
        prevTime = time.seconds();

        // Returns the PID value to be sent to the motor
        return Range.clip(k_P * p_error + k_I * i_error + k_D * d_error, min, max);
    }
}