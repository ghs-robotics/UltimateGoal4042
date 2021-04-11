package org.firstinspires.ftc.teamcode.robot_components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Robot extends DriveBase implements HSVConstants, FieldPosition {

    public CameraManager camera; // Manages the webcam and phone camera

    private int targetX = 100;
    private int targetY = 140;
    private int targetWidth = 95;
    public double targetGyroAngle = 0; // gyroscope will target this angle
    private int objectX = 0;
    private int objectY = 0;
    private int objectWidth = 0;
    private int objectHeight = 0;
    private double x = 0; // Used in several methods
    private double y = 0; // Used in several methods

    private String targetObject = "ring"; // Default targetObject
    private static boolean objectIdentified = false; // The program will know when the object isn't in view

    // Robot variables and objects
    private double intakePower = 0;
    public double armAngle = 0.45; // Up position
    public double clawAngle = 0.15; // Closed position
    public DcMotor intakeMotor;
    public Servo armServo;
    public Servo clawServo;

    public PowerLauncher powerLauncher;

    public ElapsedTime elapsedTime;

    // PID controllers
    public PIDController towerXPID; // For the x-position of the tower goal
    public PIDController towerWPID; // For the width of the tower goal
    public PIDController xPID;
    public PIDController wPID; // For the y-position of the robot
    public PIDController gyroPID; // Controls the angle

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
        elapsedTime = new ElapsedTime();
        elapsedTime.reset();

        // Initializing PID objects

        // xPID works best on its own with following values: 0.0900, 0.0015, 0.0075
        // When working together with wPID, having Ki and Kd be zero works best
        towerXPID = new PIDController(0.0400, 0.0015, 0.0000, 2);

        // wPID works best on its own with following values: 0.0750, 0.0010, 0.0080
        // Having Ki and Kd be zero normally works fine though
        towerWPID = new PIDController(0.0450, 0.0010, 0.0000, 2);

        // Could be better
        xPID = new PIDController(0.0200, 0.0000, 0.0000, 2);

        // Could be better
        wPID = new PIDController(0.0250, 0.0000, 0.0000, 3);

        // gyroPID works best when Ki = 0
        gyroPID = new PIDController(0.0330, 0.0000, 0.0020, 2);
    }



    // ---------------------------------------------------------------------------------------------
    // ---------------------------------------------------------------------------------------------
    // ------------------------------------   HELPER METHODS   -------------------------------------
    // ---------------------------------------------------------------------------------------------
    // ---------------------------------------------------------------------------------------------


    // Displays a bunch of useful values on the DS phone
    @Override
    public void addTelemetryData() {
        telemetry.addData("crosshair: ", camera.webcamPipeline.crosshairValue);
        telemetry.addData("rightLaunchPower: ", "" + powerLauncher.rightPower);
        telemetry.addData("rightLaunchVelocity: ", "" + powerLauncher.getRightVelocity());
        telemetry.addData("launchAngle: ", "" + powerLauncher.launchAngle);
        telemetry.addData("indexerAngle: ", "" + powerLauncher.indexerAngle);
        telemetry.addData("gyro angle: ", "" + gyro.getAngle());
        telemetry.addData("objectX = ", "" + objectX);
        telemetry.addData("objectY = ", "" + objectY);
        telemetry.addData("width = ", "" + objectWidth);
        telemetry.addData("height = ", "" + objectHeight);
        telemetry.update();
    }

    private void checkIfObjectIdentified() {
        objectIdentified = (objectWidth != 0);
        if (!objectIdentified) {
            x = 0;
            y = 0;
        }
    }

    // Returns how many seconds have passed since the timer was last reset
    public double getElapsedSeconds() {
        return elapsedTime.seconds();
    }

    // To use at the start of each OpMode that uses CV
    public void init() {
        resetServos();
        resetGyroAngle();
        camera.initCamera();
        wait(1.0);
    }

    // Resets the timer
    public void resetElapsedTime() {
        elapsedTime.reset();
    }

    // Resets all PID controllers
    public void resetPIDs() {
        towerXPID.resetValues();
        towerWPID.resetValues();
        xPID.resetValues();
        wPID.resetValues();
        gyroPID.resetValues();
    }

    // Sets servos to starting positions
    public void resetServos() {
        clawServo.setPosition(clawAngle);
        wait(0.4);
        armServo.setPosition(armAngle);
        powerLauncher.resetServos();
    }

    private void setTargetCoordinates(int x, int y, int w, String target) {
        targetObject = target;
        resetPIDs();
        targetX = x;
        targetY = y;
        targetWidth = w;
    }

    // Set a target and use default values for the target position
    public void setTargetToRing() { setTargetToRing(140, 70); } // Originally: y = 220
    public void setTargetToTower() { setTargetToTower(PERFECT_LAUNCH_POS[0], PERFECT_LAUNCH_POS[1]); }
    public void setTargetToWobble() { setTargetToWobble(33, 80); } // originally y = 160 (before changed to width)

    // Switches the object that the robot is trying to detect to a ring
    public void setTargetToRing(int x, int w) {
        camera.phoneCamPipeline.setTargetTo("ring");
        setTargetCoordinates(x, 0, w, "ring");
    }

    public void setTargetToStack() {
        camera.webcamPipeline.setTargetTo("stack");
        targetObject = "stack";
    }

    // Switches the object that the robot is trying to detect to the tower goal
    public void setTargetToTower(int x, int w) {
        camera.webcamPipeline.setTargetTo("tower");
        setTargetCoordinates(x, 0, w, "tower");
    }

    // Switches the object that the robot is trying to detect to the wobble goal
    public void setTargetToWobble(int x, int w) {
        camera.phoneCamPipeline.setTargetTo("wobble");
        setTargetCoordinates(x, 0, w, "wobble");
    }

    // Updates the coordinates of the object being detected on the screen
    // If target is "tower," this uses the webcam; otherwise, phoneCam
    public void updateObjectValues() {
        int[] val = camera.getObjectData(targetObject);
        objectX = val[0];
        objectY = val[1];
        objectWidth = val[2];
        objectHeight = val[3];
    }

    // Makes the robot wait (i.e. do nothing) for a specified number of seconds
    public void wait(double seconds) {
        double start = getElapsedSeconds();
        while (getElapsedSeconds() - start < seconds) {}
    }



    // ---------------------------------------------------------------------------------------------
    // ---------------------------------------------------------------------------------------------
    // ---------------------------------   MECHANICAL FUNCTIONS   ----------------------------------
    // ---------------------------------------------------------------------------------------------
    // ---------------------------------------------------------------------------------------------

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

    // Toggles the drive speed between 50% and normal
    public void toggleSpeed() {
        speed = (speed == 1 ? 0.65 : 1);
    }

    // Turns the arm
    public void turnArm() {
        // Default angle is 0.5 (which is the up position)
        armAngle = (armAngle == 0.88 ? 0.45 : 0.88);
        armServo.setPosition(armAngle);
    }

    // Classifies the starter stack; TODO: NEEDS TO BE TESTED ADJUSTED
    public int identifyRingConfig() {
        setTargetToStack();
        updateObjectValues();
        if (5 <= objectHeight && objectHeight <= 18) { // typically about 10
            return Config.ONE;
        } else if (19 <= objectHeight && objectHeight <= 30) { // typically about 21
            return Config.FOUR;
        } else {
            return Config.ZERO;
        }
    }



    // ---------------------------------------------------------------------------------------------
    // ---------------------------------------------------------------------------------------------
    // ---------------------------------   AUTOMATED FUNCTIONS   -----------------------------------
    // ---------------------------------------------------------------------------------------------
    // ---------------------------------------------------------------------------------------------

    // Turns the robot to a desired angle (if called repeatedly)
    public void adjustAngle() {
        calculateDrivePowers(0, 0, -gyroPID.calcVal(targetGyroAngle - gyro.getAngle()));
        sendDrivePowers();
    }

    // Makes the robot line up with the tower goal and shoot three rings
    public void adjustAndShoot(int rings) {
        moveToPos(PERFECT_LAUNCH_POS, 0.8, 2.0, 5.0);
        launchRings(rings);
    }

    // Displays important values on the phone screen
    private void chaseObject(double x, double y, double rotation) {
        calculateDrivePowers(x, y, rotation);
        sendDrivePowers();

        String t = targetObject;

        if (!objectIdentified) {
            telemetry.addData("NOTE: ", "OBJECT NOT IDENTIFIED");
        }

        telemetry.addData("angle = ", gyro.getAngle());
        telemetry.addData(t + "X = ", objectX + " (target = " + targetX + ")");
        telemetry.addData(t + "Y = ", objectY + " (target = " + targetY + ")");
        telemetry.addData("width = ", objectWidth + " (target = " + targetWidth + ")");
        telemetry.addData("height = ", objectHeight);
        telemetry.addData("(x, y)", "( " + x + ", " + y + " )");
        telemetry.addData("Kp (w): ", wPID.k_P);
        telemetry.addData("Ki (w): ", wPID.k_I);
        telemetry.addData("Kd (w): ", wPID.k_D);
        telemetry.update();
    }

    // Makes the robot chase the closest ring (if called repeatedly)
    public void chaseRing() {
        if (!camera.phoneCamPipeline.targetObject.equals("ring")) { setTargetToRing(); }
        updateObjectValues();
        x = -xPID.calcVal(targetX - objectX);
        y = wPID.calcVal(targetWidth - objectWidth);
        checkIfObjectIdentified();
        chaseObject(x, y, -gyroPID.calcVal(targetGyroAngle - gyro.getAngle()));
    }

    // Makes the robot line up with the tower goal (if called repeatedly)
    public void chaseTower() {
        if (!camera.webcamPipeline.targetObject.equals("tower")) { setTargetToTower(); }
        updateObjectValues();
        x = -towerXPID.calcVal(targetX - objectX);
        y = towerWPID.calcVal(targetWidth - objectWidth);
        checkIfObjectIdentified();
        chaseObject(x, y, -gyroPID.calcVal(targetGyroAngle - gyro.getAngle()));
    }

    // Makes the robot chase the wobble goal (if called repeatedly)
    public void chaseWobble() {
        if (!camera.phoneCamPipeline.targetObject.equals("wobble")) { setTargetToWobble(); }
        updateObjectValues();
        x = -xPID.calcVal(targetX - objectX);
        y = wPID.calcVal(targetWidth - objectWidth);
        checkIfObjectIdentified();
        chaseObject(x, y, -gyroPID.calcVal(targetGyroAngle - gyro.getAngle()));
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
        setTargetToTower(pos[0], pos[1]); // Setting targetX and targetWidth
        resetPIDs();
        rotateToPos(0.0, 0.0);
        updateObjectValues();
        double t = getElapsedSeconds();
        while((Math.abs(targetWidth - objectWidth) > 8
                || Math.abs(targetX - objectX) > 8
                || Math.abs(targetGyroAngle - gyro.getAngle()) > 8)
                && elapsedTime.seconds() - t < maxBroadTuning) {
            chaseTower();
        }
        t = getElapsedSeconds();

        // Start fresh by resetting these
        resetPIDs();

        while (elapsedTime.seconds() - t < minFineTuning || (elapsedTime.seconds() - t < maxFineTuning &&
                (leftRearPower != 0 || rightRearPower != 0 || leftFrontPower != 0 || rightFrontPower != 0))) {
            chaseTower();
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

    // Makes the robot rotate to a certain angle
    public void rotateToPos(double angle, double maxFineTuning) {
        double actualAngle = gyro.getAngle();
        while (Math.abs(actualAngle - angle)  > 190) {
            if (angle < actualAngle) {
                angle += 360;
            } else {
                angle -= 360;
            }
        }
        targetGyroAngle = angle;
        resetPIDs();
        double t = getElapsedSeconds();
        while(Math.abs(targetGyroAngle - gyro.getAngle()) > 5 && elapsedTime.seconds() - t < 5) {
            adjustAngle();
        }
        t = getElapsedSeconds();
        resetPIDs();
        while ((Math.abs(targetGyroAngle - gyro.getAngle()) > 1)
                && elapsedTime.seconds() - t < maxFineTuning) {
            adjustAngle();
        }
        stopDrive();
    }
}

// Documentation: https://ftctechnh.github.io/ftc_app/doc/javadoc/index.html