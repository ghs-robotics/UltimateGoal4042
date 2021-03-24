package org.firstinspires.ftc.teamcode.robot_components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Scalar;

public class Robot {
    // HSV constants
    public static final Scalar LOWER_RING_HSV = new Scalar(74, 153, 144); // original values: 74, 153, 144
    public static final Scalar UPPER_RING_HSV = new Scalar(112, 242, 255); // original values: 112, 242, 255
    public static final Scalar LOWER_TOWER_HSV = new Scalar(0, 80, 100); // original values: 0, 124, 60
    public static final Scalar UPPER_TOWER_HSV = new Scalar(80, 255, 220); // original values: 54, 212, 255
    public static final Scalar LOWER_WOBBLE_HSV = new Scalar(0, 117, 0);
    public static final Scalar UPPER_WOBBLE_HSV = new Scalar(77, 255, 97);

    // CV detection variables
    public static Scalar lower = LOWER_RING_HSV; // We identify rings by default to start out
    public static Scalar upper = UPPER_RING_HSV;
    public static double cover = 0; // The fraction of the top part of the camera screen that is
    // covered, which is useful when we don't want the phone to detect anything beyond the field

    private boolean objectNotIdentified = false; // The program will know when the object isn't in view
    public CameraManager cameraManager;

    private int targetX = 100;
    private int targetY = 140;
    private int targetWidth = 95;
    private int objectX = 0;
    private int objectY = 0;
    private int objectWidth = 0;
    private int objectHeight = 0;
    private double x = 0;
    private double y = 0;
    public double targetAngle = 0; // gyroscope will target this angle

    private String currentTargetObject = "ring";

    // Robot variables and objects
    private double leftFrontPower = 0;
    private double rightFrontPower = 0;
    private double leftRearPower = 0;
    private double rightRearPower = 0;
    private double intakePower = 0;
    public double armAngle = 0.45; // Up position
    public double grabAngle = 0.15; // Closed position
    public double shooterAngle = 0.58; // Back (regular) position
    public double speed = 1;
    public double config = 0;

    public DcMotor leftFrontDrive;
    public DcMotor rightFrontDrive;
    public DcMotor leftRearDrive;
    public DcMotor rightRearDrive;
    public DcMotor intakeMotor;
    public Servo armServo;
    public Servo grabServo;
    public Servo shooterServo;

    public Diffy diffy;

    public ElapsedTime elapsedTime;
    public Gyro gyro;
    public Telemetry telemetry;

    // PID controllers
    public PIDController xPID; // For the x-position of the robot
    public PIDController yPID; // For the y-position of the robot
    public PIDController wPID; // For the width of the tower goal
    public PIDController gyroPID; // Controls the angle

    // Creates a robot object with methods that we can use in both Auto and TeleOp
    public Robot(HardwareMap hardwareMap, Telemetry telemetry) {

        // These are the names to use in the phone config (in quotes below)
        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFrontDrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFrontDrive");
        leftRearDrive = hardwareMap.get(DcMotor.class, "leftRearDrive");
        rightRearDrive = hardwareMap.get(DcMotor.class, "rightRearDrive");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        armServo = hardwareMap.get(Servo.class, "armServo");
        grabServo = hardwareMap.get(Servo.class, "grabServo");
        shooterServo = hardwareMap.get(Servo.class, "shooterServo");

        // Defines the forward direction for each of our motors/servos
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftRearDrive.setDirection(DcMotor.Direction.REVERSE);
        rightRearDrive.setDirection(DcMotor.Direction.FORWARD);
        intakeMotor.setDirection(DcMotor.Direction.REVERSE);
        armServo.setDirection(Servo.Direction.FORWARD);
        grabServo.setDirection(Servo.Direction.FORWARD);
        shooterServo.setDirection(Servo.Direction.FORWARD);

        // Initializes some other useful tools for our robot (the gyroscope, the timer, etc.)
        diffy = new Diffy(hardwareMap);
        gyro = new Gyro(hardwareMap);
        gyro.resetAngle();
        cameraManager = new CameraManager(hardwareMap);
        elapsedTime = new ElapsedTime();
        elapsedTime.reset();
        this.telemetry = telemetry;

        // Initializing PID objects
        xPID = new PIDController(0.0120, 0.0022, 0.0015, 2);
        yPID = new PIDController(0.0200, 0.0025, 0.0010, 2);
        wPID = new PIDController(0.0450, 0.0015, 0.0020, 2); //0.0440, 0.0016, 0.0010
        gyroPID = new PIDController(0.0330, 0.0000, 0.0020, 2); //works best when Ki = 0
    }

    // To use at the start of each OpMode that uses CV
    public void init() {
        resetServos();
        cameraManager.initCamera();
    }

    // Updates the coordinates of the object being detected on the screen
    public void updateObjectValues() {
        int[] val = cameraManager.getObjectData(currentTargetObject);
        objectX = val[0];
        objectY = val[1];
        objectWidth = val[2];
        objectHeight = val[3];
    }

    // Switches the object that the robot is trying to detect to a ring
    public void setTargetToRing(int x, int y) {
        currentTargetObject = "ring";
        xPID.resetValues();
        yPID.resetValues();
        cover = 0;
        lower = LOWER_RING_HSV;
        upper = UPPER_RING_HSV;
        targetX = x;
        targetY = y;
    }

    // Switches the object that the robot is trying to detect to the wobble goal
    public void setTargetToWobble(int x, int y) {
        currentTargetObject = "wobble";
        xPID.resetValues();
        yPID.resetValues();
        cover = 0;
        lower = LOWER_WOBBLE_HSV;
        upper = UPPER_WOBBLE_HSV;
        targetX = x;
        targetY = y;
    }

    // Switches the object that the robot is trying to detect to the tower goal
    public void setTargetToTower(int x, int w) {
        currentTargetObject = "tower";
        xPID.resetValues();
        wPID.resetValues();
        cover = 0;
        lower = LOWER_TOWER_HSV;
        upper = UPPER_TOWER_HSV;
        targetX = x;
        targetWidth = w;
    }

    // Set a target and use default values for the target position
    public void setTargetToRing() { setTargetToRing(230, 190); } // Originally: y = 220
    public void setTargetToWobble() { setTargetToWobble(60, 160); }
    public void setTargetToTower() { setTargetToTower(140, 70); }

    // Sets servos to starting positions
    public void resetServos() {
        armServo.setPosition(armAngle);
        grabServo.setPosition(grabAngle);
        shooterServo.setPosition(shooterAngle);
    }

    // Makes the robot stop driving
    public void stopDrive() {
        leftFrontPower = 0;
        rightFrontPower = 0;
        leftRearPower = 0;
        rightRearPower = 0;
        sendDrivePowers();
    }

    // Calculates powers for mecanum wheel drive
    public void calculateDrivePowers(double x, double y, double rotation) {
        double r = Math.hypot(x, y);
        double robotAngle = Math.atan2(y, x) - Math.PI / 4;
        leftFrontPower = Range.clip(r * Math.cos(robotAngle) + rotation, -1.0, 1.0) * speed;
        rightFrontPower = Range.clip(r * Math.sin(robotAngle) - rotation, -1.0, 1.0) * speed;
        leftRearPower = Range.clip(r * Math.sin(robotAngle) + rotation, -1.0, 1.0) * speed;
        rightRearPower = Range.clip(r * Math.cos(robotAngle) - rotation, -1.0, 1.0) * speed;
    }

    // Sends power to drive motors
    public void sendDrivePowers() {
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftRearDrive.setPower(leftRearPower);
        rightRearDrive.setPower(rightRearPower);
    }

    // Updates the powers being sent to the drive motors
    public void updateDrive() {
        //Displays motor powers on the phone
//        telemetry.addData("encoderPos", "" + diffy.getPosition());
        telemetry.addData("leftDiffy", "" + diffy.leftDiffyPower);
        telemetry.addData("rightDiffy", "" + diffy.rightDiffyPower);
        telemetry.addData("shooterAngle", "" + shooterAngle);
        telemetry.addData("angle", "" + gyro.getAngle());
        telemetry.addData("config: ", "" + config);
        telemetry.update();
        sendDrivePowers();
    }

    // Turns the robot to a desired angle (if called repeatedly)
    public void adjustAngle() {
        calculateDrivePowers(0, 0, -gyroPID.calcVal(targetAngle - gyro.getAngle()));
        sendDrivePowers();
    }

    // Displays important values on the phone screen
    private void chaseObject(double x, double y, double rotation) {
        calculateDrivePowers(x, y, rotation);
        sendDrivePowers();

        String t = currentTargetObject;

        if (objectNotIdentified) {
            telemetry.addData("ATTENTION: ", "OBJECT NOT IDENTIFIED");
        }

        if (t.equals("tower")) {
            telemetry.addData("towerX = ", objectX + " (target = " + targetX + ")");
            telemetry.addData("towerY = ", objectY);
            telemetry.addData("width = ", objectWidth + " (target = " + targetWidth + ")");
            telemetry.addData("height = ", objectHeight);
            telemetry.addData("(x, y)", "( " + x + ", " + y + " )");
            telemetry.addData("Kp (w): ", wPID.k_P);
            telemetry.addData("Ki (w): ", wPID.k_I);
            telemetry.addData("Kd (w): ", wPID.k_D);
        } else {
            telemetry.addData(t + "X = ", objectX + " (target = " + targetX + ")");
            telemetry.addData(t + "Y = ", objectY + " (target = " + targetY + ")");
            telemetry.addData("width = ", objectWidth);
            telemetry.addData("height = ", objectHeight);
            telemetry.addData("(x, y)", "( " + x + ", " + y + " )");
            telemetry.addData("Kp (x): ", xPID.k_P);
            telemetry.addData("Ki (x): ", xPID.k_I);
            telemetry.addData("Kd (x): ", xPID.k_D);
        }

        telemetry.addData("HSV MIN, MAX: ", lower + ", " + upper);
        telemetry.update();
    }

    // Makes the robot chase the closest ring (if called repeatedly)
    public void chaseRing() {
        if (!currentTargetObject.equals("ring")) { setTargetToRing(); }
        updateObjectValues();

        x = xPID.calcVal(targetX - objectX);
        y = -yPID.calcVal(targetY - objectY);

        double h = objectHeight;
        double w = objectWidth;
        double r = 1.0 * w / h;

        // Testing to make sure the detected object is a ring
        if (!(h > 8 && h < 23 && w > 32 && w < 90 && r > 1.5 && r < 5)) { // 8,23,22,90,1.5,7
            // TODO: Make sure we aren't double-checking this condition in the pipeline or here!
            x = 0;
            y = 0;
        }

        chaseObject(x, y, 0);
    }

    // Makes the robot chase the wobble goal (if called repeatedly); TO DO: NEEDS GYRO IMPLEMENTATION
    public void chaseWobble() {
        if (!currentTargetObject.equals("wobble")) { setTargetToWobble(); }
        updateObjectValues();

        double dy = targetY - objectY;
        double dx = targetX - objectX;

        // Adjust x and y values according to how far away the robot is from the target
        if (Math.abs(dx) < 10) {
            x = 0;
        } else {
            x += Range.clip(dx / 400.0, -0.2, 0.2);
        }
        if (Math.abs(dy) < 10) {
            y = 0;
        } else {
            y -= Range.clip(dy / 400.0, -0.2, 0.2);
        }

        // Make sure the robot doesn't go too fast
        y = Range.clip(y, -0.6, 0.6);
        x = Range.clip(x, -0.6, 0.6);

        double h = objectHeight;
        double w = objectWidth;

        // Testing to make sure the detected object is a wobble goal
        if (!(h > 10 && h < 200 && w > 10 && w < 200)) {
            x = 0;
            y = 0;
        }

        chaseObject(x, y, 0);
    }

    // Makes the robot line up with the tower goal (if called repeatedly)
    public void chaseTower() {
        if (!currentTargetObject.equals("tower")) { setTargetToTower(); }
        updateObjectValues();

        x = xPID.calcVal(targetX - objectX);
        y = -wPID.calcVal(targetWidth - objectWidth);

        if (!(objectWidth > 40 && objectWidth < 150)) { // TODO : perhaps adjust these values
            x = 0;
            y = 0;
            objectNotIdentified = true;
        } else if (objectNotIdentified == true) {
            objectNotIdentified = false;
        }

//        chaseObject(x, y, -gyroPID.calcVal(targetAngle - gyro.getAngle()));
        chaseObject(x, y, 0); // TODO : comment out
//        chaseObject(0, 0, 0);
    }

    public void moveToPos(int[] pos) {
        moveToPos(pos, 1.0);
    }

    // Makes the robot move to a certain position relative to the tower goal
    public void moveToPos(int[] pos, double maxSeconds) {
        setTargetToTower(pos[0], pos[1]); // Setting targetX and targetWidth
        updateObjectValues();
        double t = getElapsedTimeSeconds();
        while((Math.abs(targetWidth - objectWidth) > 8 || Math.abs(targetX - objectX) > 8)
                && elapsedTime.seconds() - t < 5) {
            chaseTower();
        }
        t = getElapsedTimeSeconds();
        while ((leftRearPower != 0 || rightRearPower != 0 || leftFrontPower != 0
                || rightFrontPower != 0) && elapsedTime.seconds() - t < maxSeconds) {
            chaseTower();
        }
        //stopDrive();
    }

    // Makes the robot rotate to a certain angle
    public void rotateToPos(int angle, int maxSeconds) {
        targetAngle = angle;
        double t = getElapsedTimeSeconds();
        while(Math.abs(targetAngle - gyro.getAngle()) > 5 && elapsedTime.seconds() - t < 5) {
            adjustAngle();
        }
        t = getElapsedTimeSeconds();
        while ((Math.abs(targetAngle - gyro.getAngle()) > 1)
                && elapsedTime.seconds() - t < maxSeconds) {
            adjustAngle();
        }
        //stopDrive();
    }

    // Makes the robot line up with the tower goal and shoot three rings
    public void adjustAndShoot() {
        setTargetToTower(95,80);
        updateObjectValues();
        double t = getElapsedTimeSeconds();
        while(Math.abs(targetWidth - objectWidth) > 2 || Math.abs(targetX - objectX) > 2 && elapsedTime.seconds() - t < 4) {
            chaseTower();
        }
        t = getElapsedTimeSeconds();
        chaseTower();
        toggleShooter();
        while ((leftRearPower != 0 || rightRearPower != 0 || leftFrontPower != 0
                || rightFrontPower != 0) && elapsedTime.seconds() - t < 3) {
            chaseTower();
        }
        stopDrive();
        for (int i = 0; i < 3; i++) {
            launchRing();
            wait(0.4);
        }
        toggleShooter();
    }

    // Toggles the drive speed between 50% and normal
    public void toggleSpeed() {
        speed = (speed == 1 ? 0.5 : 1);
    }

    // Turns the shooter motor on or off
    public void toggleShooter() { diffy.toggleShooter(0); }

    // Turns the intake motor on or off
    public void toggleIntake() {
        intakePower = (intakePower == 0 ? 0.6 : 0);
        intakeMotor.setPower(intakePower);
    }

    // Turns the arm
    public void turnArm() {
        // Default angle is 0.5 (which is the up position)
        armAngle = (armAngle == 0.88 ? 0.45 : 0.88);
        armServo.setPosition(armAngle);
    }

    // Toggles the wobble gripper
    public void toggleGrab() {
        // Default angle is 0.15 (which means the gripper is closed)
        grabAngle = (grabAngle == 0.48 ? 0.15 : 0.48);
        grabServo.setPosition(grabAngle);
    }

    // Launches a ring by moving the shooterServo
    public void launchRing() {
        shooterAngle = 0.46; // Forward position
        shooterServo.setPosition(shooterAngle);
        wait(0.5);
        shooterAngle = 0.58; // Back position
        shooterServo.setPosition(shooterAngle);
    }

    // Makes robot move forward and pick up wobble goal
    public void pickUpWobbleGoal(double sec) {
        turnArm();
        toggleGrab();
        calculateDrivePowers(0,-0.4,0);
        sendDrivePowers();
        wait(sec); //Adjust this later
        stopDrive();
        toggleGrab();
        wait(0.6);
        turnArm();
    }

    // Classifies the starter stack; TODO: NEEDS TO BE TESTED ADJUSTED
    public void identifyRingConfig() {
        setTargetToRing();
        updateObjectValues();
        if (!(objectWidth == 0)) {
            config = 1.0 * objectHeight / objectWidth;
        }
    }

    // Resets the timer
    public void resetElapsedTime() {
        elapsedTime.reset();
    }

    // Returns how many seconds have passed since the timer was last reset
    public double getElapsedTimeSeconds() {
        return elapsedTime.seconds();
    }

    // Makes the robot wait (i.e. do nothing) for a specified number of seconds
    public void wait(double seconds) {
        double start = getElapsedTimeSeconds();
        while (getElapsedTimeSeconds() - start < seconds) {}
    }
}

//Documentation: https://ftctechnh.github.io/ftc_app/doc/javadoc/index.html