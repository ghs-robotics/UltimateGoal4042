package org.firstinspires.ftc.teamcode.robot_components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class PowerLauncher {

    public static final double PERFECT_LAUNCH_ANGLE = 0.421; // Perfect launch angle
    public static final double INDEXER_BACK_POS = 0.400;
    public static final double INDEXER_FORWARD_POS = 0.810;

    public double leftPower = 0;
    public double rightPower = 0;

    // launchAngle should range between 0.7 (horizontal) and 0.85 (very steep); launcher vertical at 1.0
    public double launchAngle = PERFECT_LAUNCH_ANGLE;
    public double indexerAngle = INDEXER_BACK_POS;

    public boolean running = false;

    // Target speed for the two motors in ticks per second
    public double leftTargetVelocity = 1700;
    public double rightTargetVelocity = 1700;

    // Variables for calculating the motor velocities
    private long prevLeftPos = 0;
    private long prevRightPos = 0;
    private double prevLeftSeconds = 0;
    private double prevRightSeconds = 0;

    private ElapsedTime elapsedTime;

    // Motors and servos
    public DcMotor leftMotor;
    public DcMotor rightMotor;
    public Servo launchAngleServo;
    public Servo indexerServo;

    // Constructs a PowerLauncher object
    public PowerLauncher(HardwareMap hardwareMap) {

        // Initializes motors and servos and adds them to the phone config
        leftMotor = hardwareMap.get(DcMotor.class, "leftLaunchMotor");
        rightMotor = hardwareMap.get(DcMotor.class, "rightLaunchMotor");
        launchAngleServo = hardwareMap.get(Servo.class, "launchAngleServo");
        indexerServo = hardwareMap.get(Servo.class, "indexerServo");

        // Defines the forward direction for motors and servos
        leftMotor.setDirection(DcMotor.Direction.REVERSE);
        rightMotor.setDirection(DcMotor.Direction.FORWARD);
        launchAngleServo.setDirection(Servo.Direction.FORWARD);
        indexerServo.setDirection(Servo.Direction.FORWARD);

        // Initializes and resets the timer
        elapsedTime = new ElapsedTime();
        elapsedTime.reset();
    }

    // Tries to achieve target velocities
    public void adjustShooterVelocity(){
        if (getRightVelocity() > rightTargetVelocity){
            leftPower -= 0.001;
            rightPower -= 0.001;
        } else {
            leftPower += 0.001;
            rightPower += 0.001;
        }
        sendPowers();
    }

    // Increase/decrease the launch angle
    public void changeLaunchAngle(double change) {
        setLaunchAngle(launchAngle + change);
    }

    // Increase/decrease the indexer angle
    public void changeIndexerAngle(double change) {
        setIndexerAngle(indexerAngle + change);
    }

    // Calculates left motor speed in ticks per second
    public double getLeftVelocity() {
        long deltaTicks = (leftMotor.getCurrentPosition() - prevLeftPos);
        double deltaTime = elapsedTime.seconds() - prevLeftSeconds;
        prevLeftPos = leftMotor.getCurrentPosition();
        prevLeftSeconds = elapsedTime.seconds();
        return (deltaTicks / deltaTime);
    }

    // Calculates right motor speed in ticks per second
    public double getRightVelocity() {
        long deltaTicks = (rightMotor.getCurrentPosition() - prevRightPos);
        double deltaTime = elapsedTime.seconds() - prevRightSeconds;
        prevRightPos = rightMotor.getCurrentPosition();
        prevRightSeconds = elapsedTime.seconds();
        return (deltaTicks / deltaTime);
    }

    // Moves the indexer servo, which launches a ring
    public void index() {
        indexerAngle = INDEXER_FORWARD_POS;
        indexerServo.setPosition(indexerAngle);
        wait(0.6); // TODO : CHANGE
        indexerAngle = INDEXER_BACK_POS;
        indexerServo.setPosition(indexerAngle);
    }

    // Resets the encoder encoder position's of the motors to zero
    public void resetMotors() { // TODO : USE???
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // Rotate servos to default positions
    public void resetServos() {
        indexerAngle = INDEXER_BACK_POS;
        launchAngle = PERFECT_LAUNCH_ANGLE;
        indexerServo.setPosition(indexerAngle);
        launchAngleServo.setPosition(launchAngle);
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

    // Sets a launch angle
    public void setLaunchAngle(double angle) {
        launchAngle = angle;
        launchAngleServo.setPosition(launchAngle);
    }

    // Sets indexer angle
    public void setIndexerAngle(double angle) {
        indexerAngle = angle;
        indexerServo.setPosition(indexerAngle);
    }

    // Toggles the two launcher motors between off and full power
    public void toggle() {
        if (leftPower != 0 || rightPower != 0) {
            sendPowers(0);
        } else {
            sendPowers(1.0);
        }
        running = !running;
    }

    // Turn launcher off
    public void toggleOff() {
        sendPowers(0.0);
        running = false;
    }

    // Turn launcher on
    public void toggleOn() {
        sendPowers(0.85);
        running = true;
//        double start = elapsedTime.seconds();
//        while (elapsedTime.seconds() - start < 2) {
//            adjustShooterVelocity();
//        }
    }

    // Makes the robot wait (i.e. do nothing) for a specified number of seconds
    public void wait(double seconds) {
        double start = elapsedTime.seconds();
        while (elapsedTime.seconds() - start < seconds) {}
    }
}