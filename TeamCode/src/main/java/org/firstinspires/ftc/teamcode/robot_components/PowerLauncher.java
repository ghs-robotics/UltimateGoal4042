package org.firstinspires.ftc.teamcode.robot_components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class PowerLauncher {

    // Launcher angles
    public static double PERFECT_LAUNCH_ANGLE = 0.766; // Default launch angle
    public static double VERTICAL_OFFSET = 0.030; // Added to PERFECT LAUNCH ANGLE
    public static double LOADING_OFFSET = -0.040; // Added to PERFECT LAUNCH ANGLE

    public static final double INDEXER_BACK_POS = 0.420;
    public static final double INDEXER_FORWARD_POS = 0.860;

    // 1700 ticks per second (or anything higher) gives the best results
    public static final double PERFECT_SHOOTER_VELOCITY = 1800; // Ticks per second

    // Motor powers
    public double leftPower = 0;
    public double rightPower = 0;

    public double launchAngle = PERFECT_LAUNCH_ANGLE + LOADING_OFFSET;
    public double indexerAngle = INDEXER_BACK_POS;

    public boolean running = false; // If the launcher is running

    private int queue = 0; // Keeps track of how many rings are "in line" to be launched

    // Target speed for the two motors in ticks per second
    public double leftTargetVelocity = PERFECT_SHOOTER_VELOCITY; // Not currently used because encoder isn't working
    public double rightTargetVelocity = PERFECT_SHOOTER_VELOCITY;

    // Variables for calculating the motor velocities
    private long prevLeftPos = 0;
    private long prevRightPos = 0;
    private double prevLeftSeconds = 0;
    private double prevRightSeconds = 0;
    private double timeStamp = 0; // For regulating the speed of changing the incline
    private double queueTimeStamp = 0; // For the queue of rings to be launched

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

    public void changeLaunchAngleGradually(double change) {
        if (elapsedTime.seconds() - timeStamp > 0.01) { // TODO : ADJUST
            changeLaunchAngle(change);
            timeStamp = elapsedTime.seconds();
        }
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

    // Handle the queue of rings
    public int handleQueue(int queue) {
        this.queue = queue;
        if (!running) {
            toggleOn();
            queueTimeStamp = elapsedTime.seconds();
        }
        if (elapsedTime.seconds() - queueTimeStamp > 1.6) { // TODO : ADJUST
            setIndexerAngle(INDEXER_BACK_POS);
            queueTimeStamp = elapsedTime.seconds();
            this.queue--;
            if (this.queue == 0) {
                toggleOff();
            }
            return this.queue;
        }
        else if (elapsedTime.seconds() - queueTimeStamp > 1.0) { // TODO : ADJUST
            setIndexerAngle(INDEXER_FORWARD_POS);
        }
        return queue;
    }

    // Moves the indexer servo, which launches a ring
    public void index() {
        setIndexerAngle(INDEXER_FORWARD_POS);
        wait(0.4); // TODO : CHANGE
        setIndexerAngle(INDEXER_BACK_POS);
    }

    public boolean hasLoadingLaunchAngle() {
        return launchAngle == PERFECT_LAUNCH_ANGLE + LOADING_OFFSET;
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
        launchAngle = PERFECT_LAUNCH_ANGLE + VERTICAL_OFFSET;
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

    // Updates the default angle
    public void setCurrentAngleToDefault() {
        PERFECT_LAUNCH_ANGLE = launchAngle;
    }

    // Sets indexer angle
    public void setIndexerAngle(double angle) {
        indexerAngle = angle;
        indexerServo.setPosition(indexerAngle);
    }

    // Sets a launch angle
    public void setLaunchAngle(double angle) {
        launchAngle = angle;
        launchAngleServo.setPosition(launchAngle);
    }

    // Sets launcher to loading position
    public void setLaunchAngleLoading() {
        setLaunchAngle(PERFECT_LAUNCH_ANGLE + LOADING_OFFSET);
    }

    // Sets launcher to vertical
    public void setLaunchAngleVertical() {
        setLaunchAngle(PERFECT_LAUNCH_ANGLE + VERTICAL_OFFSET);
    }

    // Sets a launch angle
    public void setLaunchAnglePerfect() {
        setLaunchAngle(PERFECT_LAUNCH_ANGLE);
    }

    // Sets launcher to hit powershots
    public void setLaunchAnglePowershot() {
        setLaunchAngle(PERFECT_LAUNCH_ANGLE - 0.025);
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
    }

    // Makes the robot wait (i.e. do nothing) for a specified number of seconds
    public void wait(double seconds) {
        double start = elapsedTime.seconds();
        while (elapsedTime.seconds() - start < seconds) {}
    }

    // Adjust shooter velocity while waiting
    public void waitAndAdjustVelocity(double seconds) {
        double start = elapsedTime.seconds();
        while (elapsedTime.seconds() - start < seconds) {
            adjustShooterVelocity();
        }
    }

}