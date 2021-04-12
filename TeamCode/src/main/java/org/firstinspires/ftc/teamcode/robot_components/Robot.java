package org.firstinspires.ftc.teamcode.robot_components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.cv_objects.CVObject;
import org.firstinspires.ftc.teamcode.cv_objects.FieldFloor;
import org.firstinspires.ftc.teamcode.cv_objects.FieldWall;
import org.firstinspires.ftc.teamcode.cv_objects.Ring;
import org.firstinspires.ftc.teamcode.cv_objects.StarterStack;
import org.firstinspires.ftc.teamcode.cv_objects.TowerGoal;
import org.firstinspires.ftc.teamcode.cv_objects.WobbleGoal;
import org.firstinspires.ftc.teamcode.data.FieldPosition;
import org.firstinspires.ftc.teamcode.data.HSVConstants;

public class Robot extends DriveBase implements HSVConstants, FieldPosition {

    public CameraManager camera; // Manages the webcam and phone camera

    // Objects to be detected through CV
    public CVObject target;
    public FieldFloor floor;
    public FieldWall wall;
    public Ring ring;
    public StarterStack stack;
    public TowerGoal tower;
    public WobbleGoal wobble;

    // Used in several methods for regulating motor powers for in automated functions
    private double x = 0;
    private double y = 0;

    // Robot variables and objects
    private double intakePower = 0;
    public double armAngle = 0.45; // Up position
    public double clawAngle = 0.15; // Closed position

    public DcMotor intakeMotor;
    public Servo armServo;
    public Servo clawServo;

    public PowerLauncher powerLauncher;

    // PID controllers
    public PIDController towerXPID; // For the x-position of the tower goal
    public PIDController towerWPID; // For the width of the tower goal
    public PIDController xPID;
    public PIDController yPID;
    public PIDController wPID; // For the y-position of the robot

    // Constructs a robot object with methods that we can use in both Auto and TeleOp
    public Robot(HardwareMap hardwareMap, Telemetry telemetry) {

        // Calls the constructor in DriveBase, which handles all the drive base motors
        super(hardwareMap, telemetry);

        // These are the names to use in the phone config (in quotes below)
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        armServo = hardwareMap.get(Servo.class, "armServo");
        clawServo = hardwareMap.get(Servo.class, "clawServo");

        // Defines the forward direction for each of our motors/servos
        intakeMotor.setDirection(DcMotor.Direction.REVERSE);
        armServo.setDirection(Servo.Direction.FORWARD);
        clawServo.setDirection(Servo.Direction.FORWARD);

        // Initializes some other useful tools/objects for our robot
        powerLauncher = new PowerLauncher(hardwareMap);
        camera = new CameraManager(hardwareMap);

        // Initializing PID objects

        // xPID works best on its own with following values: 0.0900, 0.0015, 0.0075
        // When working together with wPID, having Ki and Kd be zero works best
        towerXPID = new PIDController(0.0400, 0.0015, 0.0000, 2);

        // wPID works best on its own with following values: 0.0750, 0.0010, 0.0080
        // Having Ki and Kd be zero normally works fine though
        towerWPID = new PIDController(0.0450, 0.0010, 0.0000, 2);

        xPID = new PIDController(0.0200, 0.0000, 0.0000, 2); // Could be better
        yPID = new PIDController(0.0200, 0.0000, 0.0000, 2); // TODO : UPDATE
        wPID = new PIDController(0.0250, 0.0000, 0.0000, 2); // Could be better

        CVDetectionPipeline web = camera.webcamPipeline;
        CVDetectionPipeline phone = camera.phoneCamPipeline;

        floor = new FieldFloor(phone, yPID);
        wall = new FieldWall(web);
        ring = new Ring(phone, xPID, wPID);
        stack = new StarterStack(web);
        tower = new TowerGoal(web, xPID, wPID);
        wobble = new WobbleGoal(phone, xPID, wPID);
        target = tower;
    }






    // ---------------------------------------------------------------------------------------------
    // ---------------------------------------------------------------------------------------------
    // ------------------------------------   HELPER METHODS   -------------------------------------
    // ---------------------------------------------------------------------------------------------
    // ---------------------------------------------------------------------------------------------


    // Displays a bunch of useful values on the DS phone
    @Override
    public void addTelemetryData() {
//        telemetry.addData("crosshair: ", camera.webcamPipeline.crosshairHSV);

        if (!tower.isIdentified()) {
            telemetry.addData("NOTE: ", "TOWER NOT IDENTIFIED");
        }
        if (!floor.isIdentified()) {
            telemetry.addData("NOTE: ", "FLOOR NOT IDENTIFIED");
        }

        telemetry.addData("angle = ", gyro.getAngle());
        telemetry.addData("launchAngle: ", "" + powerLauncher.launchAngle);
        telemetry.addData("indexerAngle: ", "" + powerLauncher.indexerAngle);
        telemetry.addData("gyro angle: ", "" + gyro.getAngle());
        telemetry.addData("objectX = ", "" + target.x);
        telemetry.addData("objectY = ", "" + target.y);
        telemetry.addData("width = ", "" + target.w);
        telemetry.addData("height = ", "" + target.h);
        telemetry.addData("(x, y)", "( " + x + ", " + y + " )");
        telemetry.addData("Kp (w): ", wPID.k_P);
        telemetry.addData("Ki (w): ", wPID.k_I);
        telemetry.addData("Kd (w): ", wPID.k_D);
        telemetry.update();
    }

    // Classifies the starter stack; TODO: NEEDS TO BE TESTED ADJUSTED
    public int identifyRingConfig() {
        return stack.findConfig();
    }

    // To use at the start of each OpMode that uses CV
    public void initWithCV() {
        camera.initCamera();
        initWithoutCV();
    }

    public void initWithoutCV() {
        resetServos();
        resetGyroAngle();
        wait(1.0);
    }

    // Launches a ring by moving the shooterServo
    public void launchRings(int rings) { // TODO : CUT DOWN ON TIME
        powerLauncher.toggleOn();
        wait(0.7);
        for (int i = 0; i < rings; i++) {
            powerLauncher.index();
            wait(0.7);
        }
        powerLauncher.toggleOff();
    }

    // Resets all PID controllers
    public void resetPIDs() {
        towerXPID.resetValues();
        towerWPID.resetValues();
        xPID.resetValues();
        yPID.resetValues();
        wPID.resetValues();
        gyroPID.resetValues();
    }

    // Sets servos to starting positions
    public void resetServos() {
        clawServo.setPosition(clawAngle);
        wait(0.6);
        armServo.setPosition(armAngle);
        powerLauncher.resetServos();
    }

    // Run intake with specified power; negative values make intake run backward
    public void runIntake(double power) {
        intakePower = power;
        intakeMotor.setPower(intakePower);
    }

    // Toggles the wobble gripper/claw
    public void toggleClaw() {
        // Default angle is 0.15 (which means the gripper is closed)
        clawAngle = (clawAngle == 0.48 ? 0.15 : 0.48);
        clawServo.setPosition(clawAngle);
    }

    // Turns the arm
    public void turnArm() {
        // Default angle is 0.5 (which is the up position)
        armAngle = (armAngle == 0.88 ? 0.45 : 0.88);
        armServo.setPosition(armAngle);
    }




    // ---------------------------------------------------------------------------------------------
    // ---------------------------------------------------------------------------------------------
    // ---------------------------------   AUTOMATED FUNCTIONS   -----------------------------------
    // ---------------------------------------------------------------------------------------------
    // ---------------------------------------------------------------------------------------------

    // Makes the robot line up with the tower goal and shoot three rings
    public void adjustAndShoot(int rings) {
        moveToPos(PERFECT_LAUNCH_POS, 0.8, 2.0, 5.0);
        launchRings(rings);
    }

    // Makes the robot line up with the tower goal (if called repeatedly)
    // Make sure the gyro angle and target values are correct before calling this!!!
    public void adjustPosition() {
        if (!tower.isIdentified()) {
            chaseObject(floor);
        } else {
            chaseObject(tower);
        }
    }

    // Makes the robot chase the target object (if called repeatedly)
    // Make sure the gyro angle and target values are correct before calling this (and reset PIDs)!!!
    private void chaseObject(CVObject target) {
        target.updateData();
        x = tower.getXPIDValue();
        y = tower.getWPIDValue();
        calculateDrivePowers(0, 0, getGyroPIDValue()); // TODO : replace with x, y
        sendDrivePowers();
        addTelemetryData();
    }

    // Calling move(0, 0.4, 3.0) makes the robot move forward 3.5 feet (a little over 1 ft/sec)
    public void move(double x, double y, double seconds) {
        calculateDrivePowers(x, y, 0);
        sendDrivePowers();
        wait(seconds);
        stopDrive();
    }

    public void moveToPos(int[] pos) {
        moveToPos(pos, 0.0, 2.0, 5.0);
    }

    public void moveToPos(int[] pos, double maxFineTuning) {
        moveToPos(pos, 0.0, maxFineTuning, 5.0);
    }

    public void moveToPos(int[] pos, double minFineTuning, double maxFineTuning) {
        moveToPos(pos, minFineTuning, maxFineTuning, 5.0);
    }

    // Makes the robot move to a certain position relative to the tower goal
    public void moveToPos(int[] pos, double minFineTuning, double maxFineTuning, double maxBroadTuning) {
        setLauncherSideAsFront();
        tower.setTargetXW(pos);
        floor.setTargetY(280); // TODO : UPDATE
        rotateToPos(0.0, 0.0);
        tower.updateData();
        floor.updateData();
        double t = getElapsedSeconds();
        while(  (!tower.isIdentified()
                || tower.getErrorW() > 8
                || tower.getErrorX() > 8
                || getGyroError() > 8)
                && elapsedTime.seconds() - t < maxBroadTuning) {
            adjustPosition();
        }
        t = getElapsedSeconds();

        // Start fresh by resetting these
        resetPIDs();

        while (elapsedTime.seconds() - t < minFineTuning || (elapsedTime.seconds() - t < maxFineTuning &&
                (leftRearPower != 0 || rightRearPower != 0 || leftFrontPower != 0 || rightFrontPower != 0))) {
            adjustPosition();
        }
        stopDrive();
    }

    // Makes robot move forward and pick up wobble goal
    public void pickUpWobbleGoal() {
        stopDrive();
        turnArm();
        toggleClaw();
        move(0, -0.7, 0.5);
        toggleClaw();
        wait(0.4);
        turnArm();
    }
}

// Documentation: https://ftctechnh.github.io/ftc_app/doc/javadoc/index.html