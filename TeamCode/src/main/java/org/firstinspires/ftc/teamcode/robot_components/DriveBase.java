package org.firstinspires.ftc.teamcode.robot_components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

// Contains the basic code for a mecanum wheel drive base; should be extended by a child Robot class
public class DriveBase {

    // Motor powers
    protected double leftFrontPower = 0;
    protected double rightFrontPower = 0;
    protected double leftRearPower = 0;
    protected double rightRearPower = 0;

    // Drive speed ranges from 0 to 1
    protected double speed = 1;

    // For meta-drive
    public int metaOffset = 90; // Specifies the direction of meta mode; 90 is optimal for driver

    // Mecanum wheel drive motors
    public DcMotor leftFrontDrive;
    public DcMotor rightFrontDrive;
    public DcMotor leftRearDrive;
    public DcMotor rightRearDrive;

    public Gyro gyro; // Keeps track of robot's angle
    public PIDController gyroPID; // Controls the angle
    public double targetGyroAngle = 0; // gyroscope will target this angle

    // For displaying things on the DS phone
    public Telemetry telemetry;

    public ElapsedTime elapsedTime;

    // Constructs a DriveBase object with four drive motors
    public DriveBase(HardwareMap hardwareMap, Telemetry telemetry) {

        // These are the names to use in the phone config (in quotes below)
        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFrontDrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFrontDrive");
        leftRearDrive = hardwareMap.get(DcMotor.class, "leftRearDrive");
        rightRearDrive = hardwareMap.get(DcMotor.class, "rightRearDrive");

        // TODO : WHAT ABOUT ZERO POWER BEHAVIOR? FOR AUTO?

        // Default is to have the launcher be the front
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftRearDrive.setDirection(DcMotor.Direction.REVERSE);
        rightRearDrive.setDirection(DcMotor.Direction.FORWARD);

        // Initializes gyro and sets starting angle to zero
        gyro = new Gyro(hardwareMap);
        resetGyroAngle();

        // gyroPID works best when Ki = 0
        gyroPID = new PIDController(0.0330, 0.0000, 0.0020, 1.1); // TODO : SET TOLERANCE TO 1.1

        // Initializes telemetry
        this.telemetry = telemetry;

        elapsedTime = new ElapsedTime();
        elapsedTime.reset();
    }

    // Displays drive motor powers on the DS phone
    public void addTelemetryData() {
        telemetry.addData("LF power: ", leftFrontPower);
        telemetry.addData("RF power: ", rightFrontPower);
        telemetry.addData("LR power: ", leftRearPower);
        telemetry.addData("RR power: ", rightRearPower);
        telemetry.update();
    }

    // Turns the robot to a desired angle (if called repeatedly)
    public void adjustAngle() {
        calculateDrivePowers(0, 0, getGyroPIDValue());
        sendDrivePowers();
    }

    // Calculates powers for mecanum wheel drive
    public void calculateDrivePowers(double x, double y, double rot) { // rot is rotation
        calculateDrivePowers(x, y, rot, false);
    }

    // Calculates powers for mecanum wheel drive
    public void calculateDrivePowers(double x, double y, double rot, boolean meta) { // rot is rotation
        double r = Math.hypot(x, y); // TODO : MULTIPLY BY SQRT(2) ?
        double robotAngle = Math.atan2(y, x) - Math.PI / 4;
        if (meta) { // Assuming launcher is front
            robotAngle -= Math.toRadians(gyro.getAngle() + metaOffset);
        }
        leftFrontPower = Range.clip(r * Math.cos(robotAngle) + rot, -1.0, 1.0) * speed;
        rightFrontPower = Range.clip(r * Math.sin(robotAngle) - rot, -1.0, 1.0) * speed;
        leftRearPower = Range.clip(r * Math.sin(robotAngle) + rot, -1.0, 1.0) * speed;
        rightRearPower = Range.clip(r * Math.cos(robotAngle) - rot, -1.0, 1.0) * speed;
    }

    // Finds an equivalent gyro angle (mod 360) within range of the actual current robot angle
    public double getReasonableGyroAngle(double desiredAngle) {
        double actualAngle = gyro.getAngle();
        while (Math.abs(actualAngle - desiredAngle)  > 190) {
            if (desiredAngle < actualAngle) {
                desiredAngle += 360;
            } else {
                desiredAngle -= 360;
            }
        }
        return desiredAngle;
    }

    public double getAbsoluteGyroError() {
        return Math.abs(getGyroError());
    }

    // Returns how many seconds have passed since the timer was last reset
    public double getElapsedSeconds() {
        return elapsedTime.seconds();
    }

    public double getGyroError() {
        return targetGyroAngle - gyro.getAngle();
    }

    public double getGyroPIDValue() {
        return -gyroPID.calcVal(getGyroError());
    }

    // Resets the timer
    public void resetElapsedTime() {
        elapsedTime.reset();
    }

    // Sets current angle as zero
    public void resetGyroAngle() {
        gyro.resetAngle();
    }

    // Makes the robot rotate to a certain angle
    public void rotateToPos(double angle, double maxFineTuning) {
        targetGyroAngle = getReasonableGyroAngle(angle);
        gyroPID.resetValues();
        double t = getElapsedSeconds();
        while(getAbsoluteGyroError() > 5 && elapsedTime.seconds() - t < 5) {
            adjustAngle();
        }
        t = getElapsedSeconds();
        gyroPID.resetValues();
        while ((getAbsoluteGyroError() > 1)
                && elapsedTime.seconds() - t < maxFineTuning) {
            adjustAngle();
        }
        stopDrive();
    }

    // Sends power to drive motors
    public void sendDrivePowers() {
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftRearDrive.setPower(leftRearPower);
        rightRearDrive.setPower(rightRearPower);
    }

    public void sendDrivePowers(double x, double y, double rot) {
        calculateDrivePowers(x, y, rot);
        sendDrivePowers();
    }

    // Sets speed to desired value
    public void setDriveSpeed(double speed) {
        this.speed = speed;
    }

    // Makes the robot stop driving
    public void stopDrive() {
        sendDrivePowers(0, 0, 0);
    }

    // Toggles the drive speed between 50% and normal
    public void toggleSpeed() {
        speed = (speed == 1 ? 0.5 : 1);
    }

    // Displays telemetry values and updates the powers being sent to the drive motors
    public void updateDrive() {
        sendDrivePowers();
        addTelemetryData();
    }

    // Makes the robot wait (i.e. do nothing) for a specified number of seconds
    public void wait(double seconds) {
        double start = getElapsedSeconds();
        while (getElapsedSeconds() - start < seconds) {}
    }
}
