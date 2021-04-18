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
import org.firstinspires.ftc.teamcode.data.FieldPositions;
import org.firstinspires.ftc.teamcode.data.HSVConstants;

public class Robot extends DriveBase implements HSVConstants, FieldPositions {

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

    // Stores data for automated functions in TeleOp
    private double phaseTimeStamp = 0;

    // Robot variables and objects
    private double intakePower = 0;
    public double armAngle = 0.37; // Up position
    public double clawAngle = 0.92; // Closed position

    public DcMotor intakeMotor;
    public Servo armServo;
    public Servo clawServo;

    public PowerLauncher powerLauncher;

    // PID controllers
    public PIDController towerXPID; // For the x-position of the tower goal
    public PIDController towerWPID; // For the width of the tower goal
    public PIDController xPID;
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
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        armServo.setDirection(Servo.Direction.FORWARD);
        clawServo.setDirection(Servo.Direction.FORWARD);

        // Initializes some other useful tools/objects for our robot
        powerLauncher = new PowerLauncher(hardwareMap);
        camera = new CameraManager(hardwareMap);

        // Initializing PID objects

        // xPID works best on its own with following values: 0.0900, 0.0015, 0.0075
        // When working together with wPID, having Ki and Kd be zero works best
        towerXPID = new PIDController(0.0400, 0.0015, 0.0000, 1);

        // wPID works best on its own with following values: 0.0750, 0.0010, 0.0080
        // Having Ki and Kd be zero normally works fine though
        towerWPID = new PIDController(0.0450, 0.0010, 0.0000, 1);

        xPID = new PIDController(0.0200, 0.0000, 0.0000, 1); // Could be better
        wPID = new PIDController(0.0250, 0.0000, 0.0000, 1); // Could be better

        CVDetectionPipeline web = camera.webcamPipeline;
        CVDetectionPipeline phone = camera.phoneCamPipeline;

        floor = new FieldFloor(phone, new PIDController(0.0600, 0.0035, 0.0020, 1)); // yPID
        wall = new FieldWall(phone, new PIDController(0.0300, 0.0020, 0.0000, 0.5)); // hPID
        ring = new Ring(phone, xPID, wPID);
        stack = new StarterStack(web);
        tower = new TowerGoal(web, towerXPID, towerWPID);
        wobble = new WobbleGoal(phone, xPID, wPID);
        target = stack; // TODO : CHANGE
    }





    // ---------------------------------------------------------------------------------------------
    // ---------------------------------------------------------------------------------------------
    // ------------------------------------   HELPER METHODS   -------------------------------------
    // ---------------------------------------------------------------------------------------------
    // ---------------------------------------------------------------------------------------------

    // Call before using CV for field localization
    public void activateFieldLocalization() {
        tower.activate();
        wall.activate();
    }

    // Displays a bunch of useful values on the DS phone
    @Override
    public void addTelemetryData() {
        telemetry.addData("TARGET", "" + target.toString());
        telemetry.addData("config", "" + identifyRingConfig());

        telemetry.addData("CURRENT LAUNCH ANGLE", "" + powerLauncher.launchAngle);
        telemetry.addData("PERFECT LAUNCH ANGLE", "" + powerLauncher.PERFECT_LAUNCH_ANGLE);
        telemetry.addData("", "");
        telemetry.addData("phonecam crosshair: ", camera.phoneCamPipeline.crosshairHSV);
        telemetry.addData("webcam crosshair: ", camera.webcamPipeline.crosshairHSV);

        telemetry.addData("gyro angle", "" + gyro.getAngle());
        telemetry.addData("TOWER", "" + tower.toString());
        telemetry.addData("WALL", "" + wall.toString());
        telemetry.addData("(x, y)", "( " + x + ", " + y + " )");
//        telemetry.addData("Kp", target.depthPID.k_P);
//        telemetry.addData("Ki", target.depthPID.k_I);
//        telemetry.addData("Kd", target.depthPID.k_D);
        telemetry.update();
    }

    // Call after using CV for field localization in order to reduce lag in teleop
    public void deactivateFieldLocalization() {
        tower.deactivate();
        wall.deactivate();
    }

    // Classifies the starter stack; TODO: NEEDS TO BE TESTED ADJUSTED
    public int identifyRingConfig() {
        return stack.findConfig();
    }

    // To use at the start of each OpMode that uses CV
    public void initWithCV() {
        camera.initCamera();
        initWithoutCV();
        tower.activate();
        wall.activate();
    }

    public void initWithoutCV() {
        resetServos();
        resetGyroAngle();
        wait(1.0);
    }

    // Launches a ring by moving the shooterServo
    public void indexRings(int rings) {
        for (int i = 0; i < rings; i++) {
            powerLauncher.index();
            wait(0.4);
        }
        powerLauncher.toggleOff();
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
        clawAngle = (clawAngle == 0.55 ? 0.92 : 0.55);
        clawServo.setPosition(clawAngle);
    }

    // Turns the arm
    public void turnArm() {
        // Default angle is 0.37 (which is the up position)
        armAngle = (armAngle == 0.88 ? 0.37 : 0.88);
        armServo.setPosition(armAngle);
    }

    // Turns the arm but not as far
    public void turnArmAlmost() {
        // Default angle is 0.37 (which is the up position)
        armAngle = (armAngle == 0.8 ? 0.37 : 0.8);
        armServo.setPosition(armAngle);
    }






    // ---------------------------------------------------------------------------------------------
    // ---------------------------------------------------------------------------------------------
    // ---------------------------------   AUTOMATED FUNCTIONS   -----------------------------------
    // ---------------------------------------------------------------------------------------------
    // ---------------------------------------------------------------------------------------------


    // Makes the robot line up with the tower goal (if called repeatedly)
    // Make sure the following things are accounted for before calling this:
    // targetGyroAngle, target values, reset PIDs, activate object, set launcher side as front
    public void adjustPosition() {
        if (getAbsoluteGyroError() > 4) {
            adjustAngle();
        }
        // When robot is too close to front of field
        else if (!tower.isIdentified() /* || (10 < floor.y && floor.y < 80) || (wall.isIdentified() && wall.h < 35) */) {
            chaseObject(wall);
        }
        else {
            chaseObject(tower);
        }
    }

    // Makes the robot chase the target object (if called repeatedly)
    // Make sure the following things are accounted for before calling this:
    // gyro angle, target values, reset PIDs, activate object, set launcher side as front
    public void chaseObject(CVObject target) {
        x = target.getBreadthPIDValue();
        y = target.getDepthPIDValue();
        calculateDrivePowers(x, y, getGyroPIDValue());
        updateDrive();
    }

    // Hardcoded movement
    public void move(double x, double y, double seconds) {
        move(x, y, seconds, false);
    }

    public void move(double x, double y, double seconds, boolean gyro) {
        if (gyro) {
            double t = getElapsedSeconds();
            while (getElapsedSeconds() - t < seconds) {
                calculateDrivePowers(x, y, getGyroPIDValue());
                sendDrivePowers();
            }
        } else {
            calculateDrivePowers(x, y, 0);
            sendDrivePowers();
            wait(seconds);
        }
        stopDrive();
    }

    // Automated move to position function that uses phases and must be called repeatedly
    // This allows us to terminate the function early (because we can just set phase to be 0)
    public int moveInPhases(int phase) {
        if (phase == 3) {
            tower.activate();
            wall.activate();
            targetGyroAngle = getReasonableGyroAngle(0);
            phaseTimeStamp = elapsedTime.seconds();
            phase--;
        }
        else if (phase == 2) {
            if ((!tower.isIdentified()
                    || tower.getAbsErrorW() > 8
                    || tower.getAbsErrorX() > 8
                    || tower.getAbsErrorX() > 8
                    || getAbsoluteGyroError() > 4)
                    && elapsedTime.seconds() - phaseTimeStamp < 5.0) {
                adjustPosition();
            } else {
                phaseTimeStamp = elapsedTime.seconds();
                tower.resetPIDs();
                wall.resetPIDs();
                phase--;
            }
        }
        else if (phase == 1) {
            if (elapsedTime.seconds() - phaseTimeStamp < 2.0 &&
                    (leftRearPower != 0 || rightRearPower != 0 || leftFrontPower != 0 || rightFrontPower != 0)) {
                adjustPosition();
            } else {
                stopDrive();
                phase--;
            }
        }
        return phase;
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
        tower.activate();
        wall.activate();
        tower.setTargetXW(pos);
        rotateToPos(0.0, 0.0);
        double t = getElapsedSeconds();
        while(  (!tower.isIdentified()
                || tower.getAbsErrorW() > 8
                || tower.getAbsErrorX() > 8
                || tower.getAbsErrorX() > 8
                || getAbsoluteGyroError() > 4)
                && elapsedTime.seconds() - t < maxBroadTuning) {
            adjustPosition();
        }
        t = getElapsedSeconds();

        // Start fresh by resetting these
        tower.resetPIDs();
        wall.resetPIDs();

        while (elapsedTime.seconds() - t < minFineTuning || (elapsedTime.seconds() - t < maxFineTuning &&
                (leftRearPower != 0 || rightRearPower != 0 || leftFrontPower != 0 || rightFrontPower != 0))) {
            adjustPosition();
        }
        stopDrive();
//        tower.deactivate(); // TODO
//        wall.deactivate();
    }

    // Move to a certain distance from the back wall
    // Only call this if the robot is already at least 2 feet from the back wall!
    public void moveUsingFloor(int targetY, double minFineTuning, double maxFineTuning, double maxBroadTuning) {
        floor.activate();
        floor.setTargetY(targetY);
        rotateToPos(0.0, 0.0);
        double t = getElapsedSeconds();
        while(  (floor.getAbsErrorY() > 3 || getAbsoluteGyroError() > 4)
                && elapsedTime.seconds() - t < maxBroadTuning) {
            chaseObject(floor);
        }
        t = getElapsedSeconds();

        // Start fresh by resetting this
        floor.resetPIDs();

        while (elapsedTime.seconds() - t < minFineTuning || (elapsedTime.seconds() - t < maxFineTuning &&
                (leftRearPower != 0 || rightRearPower != 0 || leftFrontPower != 0 || rightFrontPower != 0))) {
            chaseObject(floor);
        }
        stopDrive();
    }

    // Move to a certain distance from the back wall
    // Only call this if the robot is already at least 2 feet from the back wall!
    public void moveUsingWall(int wallH, double minFineTuning, double maxFineTuning, double maxBroadTuning) {
        wall.activate();
        wall.setTargetH(wallH);
        rotateToPos(0.0, 0.0);
        double t = getElapsedSeconds();
        while(  (wall.getAbsErrorH() > 3 || getAbsoluteGyroError() > 4)
                && elapsedTime.seconds() - t < maxBroadTuning) {
            chaseObject(wall);
        }
        t = getElapsedSeconds();

        // Start fresh by resetting this
        wall.resetPIDs();

        while (elapsedTime.seconds() - t < minFineTuning || (elapsedTime.seconds() - t < maxFineTuning &&
                (leftRearPower != 0 || rightRearPower != 0 || leftFrontPower != 0 || rightFrontPower != 0))) {
            chaseObject(wall);
        }
        stopDrive();
    }

    // Makes robot move forward and pick up wobble goal
    public void pickUpWobbleGoal() {
        wall.activate();
        stopDrive();
        turnArm();
        toggleClaw();
        double t = getElapsedSeconds();
        calculateDrivePowers(0, -0.3, 0);
        sendDrivePowers();
        while (wall.h < 94 && getElapsedSeconds() - t < 3.0) {
            calculateDrivePowers(0, -0.3, getGyroPIDValue());
            sendDrivePowers();
        }
        wall.deactivate();
        stopDrive();
        toggleClaw();
        wait(0.5);
        turnArm();
    }
}

// Documentation: https://ftctechnh.github.io/ftc_app/doc/javadoc/index.html