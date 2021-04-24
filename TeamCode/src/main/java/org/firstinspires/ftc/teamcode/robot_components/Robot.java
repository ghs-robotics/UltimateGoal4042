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
    public double armAngle = 0.42; // TODO : init position is 0
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

        // When working together with wPID, having Ki and Kd be zero works best
        towerXPID = new PIDController(0.0400, 0.0015, 0.0000, 0); // Used to have tolerance of 1

        // Having Ki and Kd be zero normally works fine though
        towerWPID = new PIDController(0.0450, 0.0010, 0.0000, 0); // Used to have tolerance of 1

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
        target = tower;
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
        telemetry.addData("CURRENT LAUNCH ANGLE", "" + powerLauncher.launchAngle);
        telemetry.addData("PERFECT LAUNCH ANGLE", "" + powerLauncher.PERFECT_LAUNCH_ANGLE);
        telemetry.addLine();

        telemetry.addLine("" + target.toString());
        telemetry.addLine("STREAMING: " + camera.isStreaming());
        telemetry.addLine("CONFIG: " + identifyRingConfig());
        telemetry.addLine("LAUNCHER POWER: " + powerLauncher.leftPower);

        telemetry.addLine();
//        telemetry.addData("phonecam crosshair: ", camera.phoneCamPipeline.crosshairHSV);
//        telemetry.addData("webcam crosshair: ", camera.webcamPipeline.crosshairHSV);

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

    // Classifies the starter stack
    public int identifyRingConfig() {
        return stack.findConfig();
    }

    // To use at the start of each OpMode that uses CV
    public void initWithCV() {
        camera.initCamera();
        resetServos();
        resetGyroAngle();
        tower.activate();
        wall.activate();
    }

    public void initWithoutCV() {
        initWithCV();
        camera.stopStreaming();
        camera.stopStreaming();
    }

    // Launches a ring by moving the shooterServo
    public void indexRings(int rings) {
        for (int i = 0; i < rings; i++) {
            powerLauncher.index();
            wait(0.3);
        }
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

    // Turn arm to specified position
    private void setArmAngle(double armAngle) {
        this.armAngle = armAngle;
        armServo.setPosition(armAngle);
    }

    // Toggles the wobble gripper/claw
    public void toggleClaw() {
        // Default angle is 0.15 (which means the gripper is closed)
        clawAngle = (clawAngle == 0.55 ? 0.92 : 0.55);
        clawServo.setPosition(clawAngle);
    }

    // Turns the arm
    public void turnArm() {
        // Default angle is 0.42 (which is the up position)
        setArmAngle(armAngle != 0.42 ? 0.42 : 0.88);
    }

    // Good position for grabbing a wobble goal
    public void turnArmDownFull() {
        setArmAngle(0.88);
    }

    // Good position for placing a wobble goal during auto
    public void turnArmDownSlight() {
        setArmAngle(0.80);
    }

    // Good position for holding the wobble goal right above ground
    public void turnArmDownDrag() {
        setArmAngle(0.75);
    }

    // For holding the wobble above the wall height
    public void turnArmUpFull() {
        setArmAngle(0.42);
    }






    // ---------------------------------------------------------------------------------------------
    // ---------------------------------------------------------------------------------------------
    // ---------------------------------   AUTOMATED FUNCTIONS   -----------------------------------
    // ---------------------------------------------------------------------------------------------
    // ---------------------------------------------------------------------------------------------


    // Makes the robot line up with the tower goal (if called repeatedly)
    // Make sure the following things are accounted for before calling this:
    // targetGyroAngle, target values, reset PIDs, activate object, set launcher side as front
    public void adjustPosition(double minAbsVal) {
        if (getAbsoluteGyroError() > 4) {
            adjustAngle();
        }
        // When robot is too close to front of field
        else if (!tower.isIdentified()) {
            chaseObject(wall, minAbsVal);
        }
        else {
            chaseObject(tower, minAbsVal);
        }
    }

    // Makes the robot chase the target object (if called repeatedly)
    // Make sure the following things are accounted for before calling this:
    // gyro angle, target values, reset PIDs, activate object, set launcher side as front
    public void chaseObject(CVObject target, double minAbsVal) {
        x = target.getBreadthPIDValue(minAbsVal); // 0.115
        y = target.getDepthPIDValue(minAbsVal);
        calculateDrivePowers(x, y, getGyroPIDValue());
        updateDrive();
    }

    // Hardcoded movement
    public void move(double x, double y, double seconds) {
        move(x, y, seconds, false);
    }

    public void move(double x, double y, double seconds, boolean gyro) {
        if (gyro) {
            double t = elapsedSecs();
            while (elapsedSecs() - t < seconds) {
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

    public int moveInPhases(int phase) {
        return moveInPhases(phase, 0.0, 2.0, 5.0);
    }

    // Automated move to position function that uses phases and must be called repeatedly
    // This allows us to terminate the function early (because we can just set phase to be 0)
    public int moveInPhases(int phase, double minFineTuning, double maxFineTuning, double maxBroadTuning) {
        if (phase == 4) {
            tower.activate();
            wall.activate();
            targetGyroAngle = getReasonableGyroAngle(0);
            phaseTimeStamp = elapsedSecs();
            phase--;
        }
        else if (phase == 3) {
            if (camera.isStreaming() && camera.isWebcamReady()) {
                phaseTimeStamp = elapsedSecs();
                phase--;
            }
        }
        else if (phase == 2) {
            if ((!tower.isIdentified()
                    || tower.getAbsErrorW() > 8 // TODO : TRY MAKING THESE SMALLER
                    || tower.getAbsErrorX() > 8
                    || getAbsoluteGyroError() > 4)
                    && elapsedSecs() - phaseTimeStamp < maxBroadTuning) {
                adjustPosition(0);
            } else {
                phaseTimeStamp = elapsedSecs();
                tower.resetPIDs();
                wall.resetPIDs();
                phase--;
            }
        }
        else if (phase == 1) {
            if (elapsedSecs() - phaseTimeStamp < minFineTuning
                    || (elapsedSecs() - phaseTimeStamp < maxFineTuning && driveMotorsRunning())) {
                adjustPosition(0.115);
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

    public void moveToPos(int[] pos, double minFineTuning, double maxFineTuning, double maxBroadTuning) {
        moveToPos(pos, minFineTuning, maxFineTuning, maxBroadTuning, 0);
    }

    // Makes the robot move to a certain position relative to the tower goal
    public void moveToPos(int[] pos, double minFineTuning, double maxFineTuning, double maxBroadTuning, double minAbsVal) {
        tower.activate();
        wall.activate();
        tower.setTargetXW(pos);
        rotateToPos(0.0, 0.0);
        double t = elapsedSecs();
        while(  (!tower.isIdentified()
                || tower.getAbsErrorW() > 8
                || tower.getAbsErrorX() > 8
                || getAbsoluteGyroError() > 4)
                && elapsedSecs() - t < maxBroadTuning) {
            adjustPosition(0);
        }
        t = elapsedSecs();

        // Start fresh by resetting these
        tower.resetPIDs();
        wall.resetPIDs();

        while (elapsedSecs() - t < minFineTuning
                || (elapsedSecs() - t < maxFineTuning && driveMotorsRunning())) {
            adjustPosition(minAbsVal);
        }
        stopDrive();
    }

    // Move to a certain distance from the back wall
    // Only call this if the robot is already at least 2 feet from the back wall!
    public void moveUsingFloor(int targetY, double minFineTuning, double maxFineTuning, double maxBroadTuning) {
        floor.activate();
        floor.setTargetY(targetY);
        rotateToPos(0.0, 0.0);
        double t = elapsedSecs();
        while(  (floor.getAbsErrorY() > 3 || getAbsoluteGyroError() > 4)
                && elapsedSecs() - t < maxBroadTuning) {
            chaseObject(floor, 0);
        }
        t = elapsedSecs();

        // Start fresh by resetting this
        floor.resetPIDs();

        while (elapsedSecs() - t < minFineTuning || (elapsedSecs() - t < maxFineTuning && driveMotorsRunning())) {
            chaseObject(floor, 0);
        }
        stopDrive();
    }

    // Move to a certain distance from the back wall
    // Only call this if the robot is already at least 2 feet from the back wall!
    public void moveUsingWall(int wallH, double minFineTuning, double maxFineTuning, double maxBroadTuning) {
        wall.activate();
        wall.setTargetH(wallH);
        rotateToPos(0.0, 0.0);
        double t = elapsedSecs();
        while(  (wall.getAbsErrorH() > 3 || getAbsoluteGyroError() > 4)
                && elapsedSecs() - t < maxBroadTuning) {
            chaseObject(wall, 0);
        }
        t = elapsedSecs();

        // Start fresh by resetting this
        wall.resetPIDs();

        while (elapsedSecs() - t < minFineTuning || (elapsedSecs() - t < maxFineTuning && driveMotorsRunning())) {
            chaseObject(wall, 0);
        }
        stopDrive();
    }

    // Makes robot move forward and pick up wobble goal
    public void pickUpWobbleGoal(String finalArmPos) {
        wall.activate();
        stopDrive();
        turnArmDownFull();
        toggleClaw();
        double t = elapsedSecs();
        calculateDrivePowers(0, -0.3, 0);
        sendDrivePowers();
        while (/*wall.h < 108 &&*/ elapsedSecs() - t < 1.45) {
            calculateDrivePowers(0, -0.3, getGyroPIDValue());
            sendDrivePowers();
        }
        wall.deactivate();
        stopDrive();
        toggleClaw();
        wait(0.5);
        if (finalArmPos.equals("up")) {
            turnArmUpFull();
        } else {
            turnArmDownDrag();
        }
    }

    // Use when you are in the LEFT_POWERSHOT_POS
    public void shootPowerShots() {
        stopDrive();
        powerLauncher.setLaunchAnglePerfect();
        powerLauncher.toggleOn(0.85);
        wait(0.9);
        indexRings(1);
        move(0.6, 0, 0.7, true);
        rotateToPos(0, 0.5);
        indexRings(1);
        move(0.6, 0, 0.7, true);
        rotateToPos(0, 0.5);
        indexRings(1);
        powerLauncher.toggleOff();
    }
}

// Documentation: https://ftctechnh.github.io/ftc_app/doc/javadoc/index.html