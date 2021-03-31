package org.firstinspires.ftc.teamcode.robot_components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class PowerLauncher {

    public double leftPower = 0;
    public double rightPower = 0;
    public boolean running = false;

    // Target speed for the two motors in ticks per second
    public double leftTargetVelocity = 0;
    public double rightTargetVelocity = 0;

    // Variables for calculating the motor velocities
    private long prevLeftPos = 0;
    private long prevRightPos = 0;
    private double prevLeftSeconds = 0;
    private double prevRightSeconds = 0;

    private ElapsedTime elapsedTime;

    // PowerLauncher's motors plus a servo for adjusting shot angle
    public DcMotor leftMotor;
    public DcMotor rightMotor;
    public Servo launcherServo;

    // Constructs a PowerLauncher object
    public PowerLauncher(HardwareMap hardwareMap) {

        // Initializes motors and servo and add them to the phone config
        leftMotor = hardwareMap.get(DcMotor.class, "leftDiffyMotor"); // TODO : CHANGE CONFIG
        rightMotor = hardwareMap.get(DcMotor.class, "rightDiffyMotor"); // TODO : CHANGE CONFIG
        launcherServo = hardwareMap.get(Servo.class, "launcherServo"); // TODO : ADD TO CONFIG

        // Defines the forward direction for diffy motors
        leftMotor.setDirection(DcMotor.Direction.REVERSE);
        rightMotor.setDirection(DcMotor.Direction.FORWARD);

        // Initialize and reset the timer
        elapsedTime = new ElapsedTime();
        elapsedTime.reset();
    }

     public void adjustShooterVelocity(){
         if (getLeftVelocity() > leftTargetVelocity){
             leftPower -= 0.001;
         } else {
             leftPower += 0.001;
         }
         if (getRightVelocity() > rightTargetVelocity){
             rightPower -= 0.001;
         } else {
             rightPower += 0.001;
         }
         sendPowers();
     }

    // Calculates left motor speed in ticks per second
    private double getLeftVelocity() {
        long deltaTicks = (leftMotor.getCurrentPosition() - prevLeftPos);
        double deltaTime = elapsedTime.seconds() - prevLeftSeconds;
        prevLeftPos = leftMotor.getCurrentPosition();
        prevLeftSeconds = elapsedTime.seconds();
        return (deltaTicks / deltaTime);
    }

    // Calculates right motor speed in ticks per second
    private double getRightVelocity() {
        long deltaTicks = (rightMotor.getCurrentPosition() - prevRightPos);
        double deltaTime = elapsedTime.seconds() - prevRightSeconds;
        prevRightPos = rightMotor.getCurrentPosition();
        prevRightSeconds = elapsedTime.seconds();
        return (deltaTicks / deltaTime);
    }

    // Resets the encoder encoder position's of the motors to zero
    public void resetDiffyMotors() { // TODO : USE???
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // Sends powers to the actual motors
    public void sendPowers() {
        leftPower = Range.clip(leftPower, -1.0, 1.0);
        rightPower = Range.clip(rightPower, -1.0, 1.0);
        leftMotor.setPower(leftPower);
        rightMotor.setPower(rightPower);
    }

    // Sends the same specified power to both motors
    public void sendPowers(double power) {
        leftPower = power;
        rightPower = power;
        sendPowers();
    }

    // Toggles the two launcher motors between off and full power
    public void toggle() {
        if (leftPower != 0 || rightPower != 0) {
            sendPowers(0);
        } else {
            sendPowers(1.0);
        }
    }
}