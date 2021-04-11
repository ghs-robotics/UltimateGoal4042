package org.firstinspires.ftc.teamcode.robot_components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
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

    // Drive orientation (+1 means launcher side is front, -1 means intake side is front)
    protected double orientation = 1;

    // For meta-drive
    public boolean usingMeta = false; // Doesn't use meta drive by default
    public int metaOffset = 0; // Specifies the direction of meta mode; default will be facing the tower goal

    // Mecanum wheel drive motors
    public DcMotor leftFrontDrive;
    public DcMotor rightFrontDrive;
    public DcMotor leftRearDrive;
    public DcMotor rightRearDrive;

    // Keeps track of robot's angle
    public Gyro gyro;

    // For displaying things on the DS phone
    public Telemetry telemetry;

    // Constructs a DriveBase object with four drive motors
    public DriveBase(HardwareMap hardwareMap, Telemetry telemetry) {

        // These are the names to use in the phone config (in quotes below)
        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFrontDrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFrontDrive");
        leftRearDrive = hardwareMap.get(DcMotor.class, "leftRearDrive");
        rightRearDrive = hardwareMap.get(DcMotor.class, "rightRearDrive");

        // Default is to have the launcher be the front
        setLauncherSideAsFront();

        // Initializes gyro and sets starting angle to zero
        gyro = new Gyro(hardwareMap);
        resetGyroAngle();

        // Initializes telemetry
        this.telemetry = telemetry;
    }

    // Displays drive motor powers on the DS phone
    public void addTelemetryData() {
        telemetry.addData("LF power: ", leftFrontPower);
        telemetry.addData("RF power: ", rightFrontPower);
        telemetry.addData("LR power: ", leftRearPower);
        telemetry.addData("RR power: ", rightRearPower);
        telemetry.update();
    }

    // Calculates powers for mecanum wheel drive
    public void calculateDrivePowers(double x, double y, double rot) { // rot is rotation
        rot *= orientation;
        double r = Math.hypot(x, y);
        double robotAngle = Math.atan2(y, x) - Math.PI / 4;
        if (usingMeta) {
            robotAngle += Math.toRadians(gyro.getAngle() + metaOffset);
        }
        leftFrontPower = Range.clip(r * Math.cos(robotAngle) + rot, -1.0, 1.0) * speed;
        rightFrontPower = Range.clip(r * Math.sin(robotAngle) - rot, -1.0, 1.0) * speed;
        leftRearPower = Range.clip(r * Math.sin(robotAngle) + rot, -1.0, 1.0) * speed;
        rightRearPower = Range.clip(r * Math.cos(robotAngle) - rot, -1.0, 1.0) * speed;
    }

    // Sets current angle as zero
    public void resetGyroAngle() {
        gyro.resetAngle();
    }

    // Sends power to drive motors
    public void sendDrivePowers() {
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftRearDrive.setPower(leftRearPower);
        rightRearDrive.setPower(rightRearPower);
    }

    // Sets speed to desired value
    public void setDriveSpeed(double speed) {
        this.speed = speed;
    }

    // When driving, intake side will be forward
    public void setIntakeSideAsFront() {
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftRearDrive.setDirection(DcMotor.Direction.FORWARD);
        rightRearDrive.setDirection(DcMotor.Direction.REVERSE);
        orientation = -1;
    }

    // When driving, launcher side will be forward
    public void setLauncherSideAsFront() {
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftRearDrive.setDirection(DcMotor.Direction.REVERSE);
        rightRearDrive.setDirection(DcMotor.Direction.FORWARD);
        orientation = 1;
    }

    // Makes the robot stop driving
    public void stopDrive() {
        leftFrontPower = 0;
        rightFrontPower = 0;
        leftRearPower = 0;
        rightRearPower = 0;
        sendDrivePowers();
    }

    // Toggles between launcher and intake side being the front (when driving)
    public void switchDriveDirection() {
        if (orientation == -1) {
            setLauncherSideAsFront();
        } else {
            setIntakeSideAsFront();
        }
    }

    // Displays telemetry values and updates the powers being sent to the drive motors
    public void updateDrive() {
        addTelemetryData();
        sendDrivePowers();
    }
}
