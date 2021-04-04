package org.firstinspires.ftc.teamcode.robot_components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Robot implements HSVConstants, FieldPositions {

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
    private double leftFrontPower = 0;
    private double rightFrontPower = 0;
    private double leftRearPower = 0;
    private double rightRearPower = 0;
    private double intakePower = 0;
    public double armAngle = 0.45; // Up position
    public double clawAngle = 0.15; // Closed position
    public double speed = 1;
    public double config = 0;

    // Drive orientation (+1 means launcher side is front, -1 means intake side is front)
    private double orientation = 1;

    public DcMotor leftFrontDrive;
    public DcMotor rightFrontDrive;
    public DcMotor leftRearDrive;
    public DcMotor rightRearDrive;
    public DcMotor intakeMotor;
    public Servo armServo;
    public Servo clawServo;

    public PowerLauncher powerLauncher;

    public ElapsedTime elapsedTime;
    public Gyro gyro;
    public Telemetry telemetry;

    // PID controllers
    public PIDController towerXPID; // For the x-position of the tower goal
    public PIDController towerWPID; // For the width of the tower goal
    public PIDController xPID;
    public PIDController wPID; // For the y-position of the robot
    public PIDController gyroPID; // Controls the angle

    // Constructs a robot object with methods that we can use in both Auto and TeleOp
    public Robot(HardwareMap hardwareMap, Telemetry telemetry) {

        // These are the names to use in the phone config (in quotes below)
        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFrontDrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFrontDrive");
        leftRearDrive = hardwareMap.get(DcMotor.class, "leftRearDrive");
        rightRearDrive = hardwareMap.get(DcMotor.class, "rightRearDrive");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        armServo = hardwareMap.get(Servo.class, "armServo");
        clawServo = hardwareMap.get(Servo.class, "clawServo");

        // Defines the forward direction for each of our motors/servos; default is launcher
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftRearDrive.setDirection(DcMotor.Direction.REVERSE);
        rightRearDrive.setDirection(DcMotor.Direction.FORWARD);
        intakeMotor.setDirection(DcMotor.Direction.REVERSE);
        armServo.setDirection(Servo.Direction.FORWARD);
        clawServo.setDirection(Servo.Direction.FORWARD);

        // Initializes some other useful tools for our robot (the gyroscope, the timer, etc.)
        powerLauncher = new PowerLauncher(hardwareMap);
        gyro = new Gyro(hardwareMap);
        gyro.resetAngle();
        camera = new CameraManager(hardwareMap);
        elapsedTime = new ElapsedTime();
        elapsedTime.reset();
        this.telemetry = telemetry;

        // Initializing PID objects

        // xPID works best on its own with following values: 0.0900, 0.0015, 0.0075
        // When working together with wPID, having Ki and Kd be zero works best
        towerXPID = new PIDController(0.0900, 0.0000, 0.0000, 2);

        // wPID works best on its own with following values: 0.0750, 0.0010, 0.0080
        // Having Ki and Kd be zero normally works fine though
        towerWPID = new PIDController(0.0750, 0.0000, 0.0000, 2);

        // Could be better
        xPID = new PIDController(0.0200, 0.0010, 0.0000, 2);

        // Could be better
        wPID = new PIDController(0.0250, 0.0010, 0.0000, 3);

        // gyroPID works best when Ki = 0
        gyroPID = new PIDController(0.0330, 0.0000, 0.0020, 2);
    }



    // ---------------------------------------------------------------------------------------------
    // ---------------------------------------------------------------------------------------------
    // ------------------------------------   HELPER METHODS   -------------------------------------
    // ---------------------------------------------------------------------------------------------
    // ---------------------------------------------------------------------------------------------

    // Calculates powers for mecanum wheel drive
    public void calculateDrivePowers(double x, double y, double rotation) {
        rotation *= orientation;
        double r = Math.hypot(x, y);
        double robotAngle = Math.atan2(y, x) - Math.PI / 4;
        leftFrontPower = Range.clip(r * Math.cos(robotAngle) + rotation, -1.0, 1.0) * speed;
        rightFrontPower = Range.clip(r * Math.sin(robotAngle) - rotation, -1.0, 1.0) * speed;
        leftRearPower = Range.clip(r * Math.sin(robotAngle) + rotation, -1.0, 1.0) * speed;
        rightRearPower = Range.clip(r * Math.cos(robotAngle) - rotation, -1.0, 1.0) * speed;
    }

    private void checkIfObjectIdentified() {
        objectIdentified = (objectWidth != 0);
        if (!objectIdentified) {
            x = 0;
            y = 0;
        }
    }

    // Returns how many seconds have passed since the timer was last reset
    public double getElapsedTimeSeconds() {
        return elapsedTime.seconds();
    }

    // To use at the start of each OpMode that uses CV
    public void init() {
        resetServos();
        resetGyroAngle();
        camera.initCamera();
    }

    // Resets the timer
    public void resetElapsedTime() {
        elapsedTime.reset();
    }

    public void resetGyroAngle() {
        gyro.resetAngle();
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
        armServo.setPosition(armAngle);
        clawServo.setPosition(clawAngle);
        powerLauncher.resetServos();
    }

    // Sends power to drive motors
    public void sendDrivePowers() {
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftRearDrive.setPower(leftRearPower);
        rightRearDrive.setPower(rightRearPower);
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

    // Makes the robot stop driving
    public void stopDrive() {
        leftFrontPower = 0;
        rightFrontPower = 0;
        leftRearPower = 0;
        rightRearPower = 0;
        sendDrivePowers();
    }

    // Updates the powers being sent to the drive motors
    public void updateDrive() {
        //Displays motor powers on the phone
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
        sendDrivePowers();
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
        double start = getElapsedTimeSeconds();
        while (getElapsedTimeSeconds() - start < seconds) {}
    }



    // ---------------------------------------------------------------------------------------------
    // ---------------------------------------------------------------------------------------------
    // ---------------------------------   MECHANICAL FUNCTIONS   ----------------------------------
    // ---------------------------------------------------------------------------------------------
    // ---------------------------------------------------------------------------------------------

    // Launches a ring by moving the shooterServo
    public void launchRings(int rings) { // TODO : CUT DOWN ON TIME
        powerLauncher.toggleOn();
        powerLauncher.waitAndAdjustVelocity(0.5);
        for (int i = 0; i < rings; i++) {
            powerLauncher.index();
            powerLauncher.waitAndAdjustVelocity(0.4);
        }
        powerLauncher.toggleOff();
    }

    // Run intake with specified power; negative values make intake run backward
    public void runIntake(double power) {
        intakePower = power;
        intakeMotor.setPower(intakePower);
    }

    public void switchDriveDirection() {
        if (leftFrontDrive.getDirection().equals(DcMotor.Direction.FORWARD)) {
            leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
            rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
            leftRearDrive.setDirection(DcMotor.Direction.REVERSE);
            rightRearDrive.setDirection(DcMotor.Direction.FORWARD);
        } else {
            leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
            rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
            leftRearDrive.setDirection(DcMotor.Direction.FORWARD);
            rightRearDrive.setDirection(DcMotor.Direction.REVERSE);
        }
        orientation *= -1.0;
    }

    public void setForwardDirection(String dir) {

        if (dir.equals("launcher")) {
            leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
            rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
            leftRearDrive.setDirection(DcMotor.Direction.REVERSE);
            rightRearDrive.setDirection(DcMotor.Direction.FORWARD);
            orientation = 1;
        } else if (dir.equals("intake")) {
            leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
            rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
            leftRearDrive.setDirection(DcMotor.Direction.FORWARD);
            rightRearDrive.setDirection(DcMotor.Direction.REVERSE);
            orientation = -1;
        }
    }

    // Toggles the wobble gripper/claw
    public void toggleClaw() {
        // Default angle is 0.15 (which means the gripper is closed)
        clawAngle = (clawAngle == 0.48 ? 0.15 : 0.48);
        clawServo.setPosition(clawAngle);
    }

    // Turns the power launcher motors on or off
    public void togglePowerLauncher() {
        powerLauncher.toggle();
    }

    // Toggles the drive speed between 50% and normal
    public void toggleSpeed() {
        speed = (speed == 1 ? 0.5 : 1);
    }

    // Turns the arm
    public void turnArm() {
        // Default angle is 0.5 (which is the up position)
        armAngle = (armAngle == 0.88 ? 0.45 : 0.88);
        armServo.setPosition(armAngle);
    }

    // Classifies the starter stack; TODO: NEEDS TO BE TESTED ADJUSTED
    public Config identifyRingConfig() {
        setTargetToStack();
        updateObjectValues();
        if (5 <= objectHeight && objectHeight <= 15) { // typically about 10
            return Config.ONE;
        } else if (16 <= objectHeight && objectHeight <= 30) { // typically about 21
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
        moveToPos(PERFECT_LAUNCH_POS, 3.0);
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
        chaseObject(x, y, -gyroPID.calcVal(targetGyroAngle - gyro.getAngle())); // TODO : change back
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
        moveToPos(pos, 1.0);
    }

    // Makes the robot move to a certain position relative to the tower goal
    public void moveToPos(int[] pos, double maxSeconds) {
        moveToPos(pos, maxSeconds, false);
    }

    // Makes the robot move to a certain position relative to the tower goal
    public void moveToPos(int[] pos, double maxSeconds, boolean backup) {
        moveToPos(pos, 0.0, maxSeconds, backup);
    }

    // Makes the robot move to a certain position relative to the tower goal
    public void moveToPos(int[] pos, double minSeconds, double maxSeconds, boolean backup) {
        setForwardDirection("launcher");
        setTargetToTower(pos[0], pos[1]); // Setting targetX and targetWidth
        resetPIDs();
        updateObjectValues();
        double t = getElapsedTimeSeconds();
        while((Math.abs(targetWidth - objectWidth) > 8
                || Math.abs(targetX - objectX) > 8
                || Math.abs(targetGyroAngle - gyro.getAngle()) > 8)
                && elapsedTime.seconds() - t < 5) {
            chaseTower();
            if (objectIdentified && backup) {
                move(0, -0.5, 1.0);
            }
        }
        t = getElapsedTimeSeconds();

        // Start fresh by resetting these
        resetPIDs();

        while (elapsedTime.seconds() - t < minSeconds || elapsedTime.seconds() - t < maxSeconds &&
                (leftRearPower != 0 || rightRearPower != 0 || leftFrontPower != 0 || rightFrontPower != 0)) {
            chaseTower();
        }
        stopDrive();
    }

    // Makes robot move forward and pick up wobble goal
    public void pickUpWobbleGoal() {
        setTargetToWobble();
        resetPIDs();
        updateObjectValues();
        double t = getElapsedTimeSeconds();
        while (elapsedTime.seconds() - t < 1.0 || elapsedTime.seconds() - t < 2.0 &&
                (leftRearPower != 0 || rightRearPower != 0 || leftFrontPower != 0 || rightFrontPower != 0)) {
            chaseWobble();
        }
        stopDrive();
        turnArm();
        toggleClaw();
        move(0, 0.45, 1.9);
        toggleClaw();
        wait(0.6);
        turnArm();
    }

    // Makes the robot rotate to a certain angle
    public void rotateToPos(int angle, int maxSeconds) {
        targetGyroAngle = angle;
        resetPIDs();
        double t = getElapsedTimeSeconds();
        while(Math.abs(targetGyroAngle - gyro.getAngle()) > 5 && elapsedTime.seconds() - t < 5) {
            adjustAngle();
        }
        t = getElapsedTimeSeconds();
        resetPIDs(); // TODO : TEST THIS LINE
        while ((Math.abs(targetGyroAngle - gyro.getAngle()) > 1)
                && elapsedTime.seconds() - t < maxSeconds) {
            adjustAngle();
        }
        stopDrive();
    }
}

//Documentation: https://ftctechnh.github.io/ftc_app/doc/javadoc/index.html