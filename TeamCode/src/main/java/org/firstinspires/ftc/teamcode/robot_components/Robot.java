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
    private double batteryVoltage;
    public double armAngle = 0; // init position
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
        towerXPID = new PIDController(0.0400, 0.0015, 0.0000, 0, 0.26);

        // Having Ki and Kd be zero normally works fine though
        towerWPID = new PIDController(0.0450, 0.0010, 0.0000, 0, 0.15);

        xPID = new PIDController(0.0200, 0.0000, 0.0000, 1); // Could be better
        wPID = new PIDController(0.0250, 0.0000, 0.0000, 1); // Could be better

        CVDetectionPipeline web = camera.webcamPipeline;
        CVDetectionPipeline phone = camera.phoneCamPipeline;

        floor = new FieldFloor(phone, new PIDController(0.0600, 0.0035, 0.0020, 1)); // yPID
        wall = new FieldWall(phone, new PIDController(0.0300, 0.0020, 0.0000, 0, 0.115)); // hPID
        ring = new Ring(phone, xPID, wPID);
        stack = new StarterStack(web);
        tower = new TowerGoal(web, towerXPID, towerWPID);
        wobble = new WobbleGoal(phone, xPID, wPID);
        target = tower;

        batteryVoltage = getBatteryVoltage();
    }





    // ---------------------------------------------------------------------------------------------
    // ---------------------------------------------------------------------------------------------
    // ------------------------------------   HELPER METHODS   -------------------------------------
    // ---------------------------------------------------------------------------------------------
    // ---------------------------------------------------------------------------------------------

    // Call before using CV for field localization
    public void activateFieldLocalization() {
        startUp();
        camera.startMidStream(); // start streaming image input
        camera.webcam.resumeViewport(); // show webcam images on phone in real time
    }

    public void startUp() {
        stopDrive();
        tower.activate();
        wall.activate();
        tower.resetPIDs();
        wall.resetPIDs();
        gyroPID.resetValues();
        CVDetectionPipeline.sleepTimeMS = 0; // increase FPS to maximum
        targetGyroAngle = getReasonableGyroAngle(0);
    }

    // Displays a bunch of useful values on the DS phone
    @Override
    public void addTelemetryData() {

//        telemetry.addLine("Left side: " + tower.x);
//        telemetry.addLine("Right side: " +  (320 - tower.x - tower.w));
        telemetry.addLine("sleepTimeMS: " + CVDetectionPipeline.sleepTimeMS);
        telemetry.addData("CURRENT LAUNCH ANGLE", "" + Math.round(1000 * powerLauncher.launchAngle));
        telemetry.addData("PERFECT LAUNCH ANGLE", "" + Math.round(1000 * powerLauncher.PERFECT_LAUNCH_ANGLE));
        double dist = tower.cvtH2VerticalDist();
        telemetry.addData("Dist", "" + dist);
        telemetry.addData("Launch offset", "" + tower.cvtFt2LaunchOffset(dist));

        telemetry.addLine();

        telemetry.addLine("" + target.toString());
        telemetry.addLine("STREAMING: " + camera.isStreaming());
        telemetry.addLine("CONFIG: " + identifyRingConfig());
        telemetry.addLine("LAUNCHER POWER: " + powerLauncher.leftPower);

        telemetry.addLine();
//        telemetry.addData("phonecam crosshair: ", camera.phoneCamPipeline.crosshairHSV); // TODO
//        telemetry.addData("webcam crosshair: ", camera.webcamPipeline.crosshairHSV); // TODO

        telemetry.addData("gyro angle", "" + gyro.getAngle());
        telemetry.addData("TOWER", "" + tower.toString());
        telemetry.addData("WALL", "" + wall.toString());
        telemetry.addData("(x, y)", "( " + x + ", " + y + " )");
        telemetry.addData("Kp", target.depthPID.k_P);
        telemetry.addData("Ki", target.depthPID.k_I);
        telemetry.addData("Kd", target.depthPID.k_D);
        telemetry.update();
    }

    public double getPhaseTimePassed() {
        return elapsedSecs() - phaseTimeStamp;
    }

    // Call after using CV for field localization in order to reduce lag in teleop
    public void deactivateFieldLocalization() { // TODO
        stopDrive();
//        tower.deactivate();
//        wall.deactivate();
        CVDetectionPipeline.sleepTimeMS = 500;
        camera.webcam.pauseViewport(); // stop displaying webcam stream on phone
    }

    // Classifies the starter stack
    public int identifyRingConfig() {
        return stack.findConfig();
    }

    // To use at the start of each OpMode that uses CV
    public void initWithCV() {
        stopDrive();
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
            powerLauncher.index(0.4);
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
        clawAngle = (clawAngle == 0.55 ? 0.98 : 0.55);
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
    public void adjustPosition() {
        if (getAbsoluteGyroError() > 4) {
            adjustAngle();
        }
        // When robot is too close to front of field
        else if (!tower.isIdentified()) {
            chaseObject(wall);
        }
        else {
            chaseObject(tower);
        }
    }

    public void adjustToTower() {
        if (tower.isIdentified()) {
            // Distance between tower goal and left side of screen: tower.x
            // Distance between tower goal and right side of screen: 320 - tower.x - tower.w
            calculateDrivePowers(0, 0, gyroPID.calcVal(tower.x - (320 - tower.x - tower.w))); // TODO
            sendDrivePowers();
        } else {
            adjustAngle();
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
        return moveInPhases(phase, 1.0, 4.0, 5.0); // TODO
    }

    // Automated move to position function that uses phases and must be called repeatedly
    // This allows us to terminate the function early (because we can just set phase to be 0)
    public int moveInPhases(int phase, double minFineTuning, double maxFineTuning, double maxBroadTuning) {
        if (phase > 4) {
            phase = 4;
        }
        switch (phase) {
            case 4:
                activateFieldLocalization();
                phase--;
                break;
            case 3:
                if (camera.isStreaming() && camera.isWebcamReady()) {
                    phaseTimeStamp = elapsedSecs();
                    phase--;
                }
                break;
            case 2:
                if ((!tower.isIdentified() || !tower.targetInRange(4) || getAbsoluteGyroError() > 4)
                        && elapsedSecs() - phaseTimeStamp < maxBroadTuning) {
                    adjustPosition();
                }
                else {
                    phaseTimeStamp = elapsedSecs();
                    phase--;
                }
                break;
            case 1:
                if (elapsedSecs() - phaseTimeStamp < minFineTuning
                        || (elapsedSecs() - phaseTimeStamp < maxFineTuning && !tower.targetInRange(0))) {
                    adjustPosition();
                }
                else {
                    deactivateFieldLocalization();
                    phase--;
                }
                break;
        }
        return phase;
    }

    public void moveToPos(int[] pos) {
        moveToPos(pos, 0.0, 2.0, 5.0, 2);
    }

    public void moveToPos(int[] pos, int tolerance) {
        moveToPos(pos, 0.0, 2.0, 5.0, tolerance);
    }

    public void moveToPos(int[] pos, double maxFineTuning) {
        moveToPos(pos, 0.0, maxFineTuning, 5.0, 2);
    }

    public void moveToPos(int[] pos, double maxFineTuning, int tolerance) {
        moveToPos(pos, 0.0, maxFineTuning, 5.0, tolerance);
    }

    public void moveToPos(int[] pos, double minFineTuning, double maxFineTuning) {
        moveToPos(pos, minFineTuning, maxFineTuning, 5.0, 2);
    }

    public void moveToPos(int[] pos, double minFineTuning, double maxFineTuning, int tolerance) {
        moveToPos(pos, minFineTuning, maxFineTuning, 5.0, tolerance);
    }

    public void moveToPos(int[] pos, double minFineTuning, double maxFineTuning, double maxBroadTuning) {
        moveToPos(pos, minFineTuning, maxFineTuning, maxBroadTuning, 2);
    }

    // Makes the robot move to a certain position relative to the tower goal
    public void moveToPos(int[] pos, double minFineTuning, double maxFineTuning, double maxBroadTuning, int tolerance) {
        tower.setTargetXW(pos);
        activateFieldLocalization(); // TODO : TEST
        rotateToPos(0.0, 0.0);
        double t = elapsedSecs();

        while((!tower.isIdentified() || !tower.targetInRange(4) || getAbsoluteGyroError() > 4)
                && elapsedSecs() - t < maxBroadTuning) {
            adjustPosition();
        }
        t = elapsedSecs();
        while (elapsedSecs() - t < minFineTuning
                || (elapsedSecs() - t < maxFineTuning && !tower.targetInRange(tolerance))) {
            adjustPosition();
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
            chaseObject(floor);
        }
        t = elapsedSecs();

        // Start fresh by resetting this
        floor.resetPIDs();

        while (elapsedSecs() - t < minFineTuning || (elapsedSecs() - t < maxFineTuning && driveMotorsRunning())) {
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
        double t = elapsedSecs();
        while(  (wall.getAbsErrorH() > 3 || getAbsoluteGyroError() > 4)
                && elapsedSecs() - t < maxBroadTuning) {
            chaseObject(wall);
        }
        t = elapsedSecs();

        // Start fresh by resetting this
        wall.resetPIDs();

        while (elapsedSecs() - t < minFineTuning || (elapsedSecs() - t < maxFineTuning && driveMotorsRunning())) {
            chaseObject(wall);
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
        while (wall.h < 90 && elapsedSecs() - t < 2.5) { // TODO
            calculateDrivePowers(0, -0.3, getGyroPIDValue());
            sendDrivePowers();
        }
        stopDrive();
        toggleClaw();
        wait(0.9);
        if (finalArmPos.equals("up")) {
            turnArmUpFull();
        } else {
            turnArmDownDrag();
        }
    }

    // Automated move to position function that uses phases and must be called repeatedly
    // This allows us to terminate the function early (because we can just set phase to be 0)
    public int rotateToTowerInPhases(int phase) {

        // Distance between tower goal and left side of screen: tower.x
        // Distance between tower goal and right side of screen: 320 - tower.x - tower.w
        double error = tower.x - (334 - tower.x - tower.w); // TODO

        if (phase == 9) {
            activateFieldLocalization();
            tower.setTargetXW(PERFECT_LAUNCH_POS);
            targetGyroAngle = getReasonableGyroAngle(0);
            phase--;
        }
        else if (phase == 8) {
            if (camera.isStreaming() && camera.isWebcamReady()) {
                phaseTimeStamp = elapsedSecs();
                phase--;
            }
        }
        else if (phase == 7) {
            if (!tower.isIdentified() || tower.h > 38 || getAbsoluteGyroError() > 20) {
                adjustPosition();
            }
            else if (Math.abs(error) <= 20) {
                phaseTimeStamp = elapsedSecs();
                phase--;
            }
            else {
                calculateDrivePowers(0, 0, metaGyroPID.calcVal(0.3 * error));
                sendDrivePowers();
            }
        }
        else if (phase == 6) {
            if (Math.abs(error) <= 10 && getPhaseTimePassed() > 0.1) {
                phaseTimeStamp = elapsedSecs();
                phase--;
            }
            else if (tower.isIdentified()) {
                calculateDrivePowers(0, 0, (error > 0 ? 0.14 : -0.14));
                sendDrivePowers();
            }
        }
        else if (phase == 5) {
            if (Math.abs(error) <= 3 && getPhaseTimePassed() > 0.1) {
                deactivateFieldLocalization();
                setAssistedLaunchAngle();
                powerLauncher.toggleOn();
                powerLauncher.resetQueueTimeStamp();
                phase--;
            }
            else if (tower.isIdentified()) {
                calculateDrivePowers(0, 0, (error > 0 ? 0.10 : -0.10));
                sendDrivePowers();
            }
        }
        else if (phase == 4) {
            if (powerLauncher.getTimePassed() > 0.6) {
                powerLauncher.resetQueueTimeStamp();
                phase--;
            }
        }
        else if (phase > 0) {
            phase = powerLauncher.handleIndexQueue(phase);
            if (phase == 0) {
                powerLauncher.toggleOff(); // Turn launcher off after indexing
            }
        }
        return phase;
    }

    public void setAssistedLaunchAngle() {
        powerLauncher.setLaunchAngle(tower.findLaunchAngle(gyro.getAngle()));
    }

    // Use when you are in the LEFT_POWERSHOT_POS
    public void shootPowerShots() {
        stopDrive();
        powerLauncher.setLaunchAnglePerfect();
//        powerLauncher.changeLaunchAngle(0.010);
//        double power = 0.95 - (batteryVoltage - 11.5) * 0.1;
        double power = 0.85;
        powerLauncher.toggleOn(power);
        wait(0.6);
        powerLauncher.index();
        move(0.6, 0, 0.7, true);
        powerLauncher.index();
        move(0.6, 0, 0.7, true);
        powerLauncher.index();
        wait(0.3);
        powerLauncher.toggleOff();
    }

    // Use when you are in the LEFT_MOVING_POWERSHOT_POS
    public void shootPowerShotsMoving() {
        stopDrive();
        powerLauncher.setLaunchAnglePerfect();
        powerLauncher.changeLaunchAngle(0.010);

        double power = 1.0 - (batteryVoltage - 11.5) * 0.08;

        /*
            Voltage         Power
            11.5            1.0
            12              0.9
            12.5            0.85
            13              0.8
            13.5            0.75
         */

        powerLauncher.toggleOn(power);
        wait(0.8);
        powerLauncher.setIndexerForwardPos();
        move(0.6, 0, 0.33, true);
        powerLauncher.setIndexerBackPos();
        move(0.6, 0, 0.33, true);
        powerLauncher.setIndexerForwardPos();
        move(0.6, 0, 0.33, true);
        powerLauncher.setIndexerBackPos();
        move(0.6, 0, 0.33, true);
        powerLauncher.setIndexerForwardPos();
        move(0.6, 0, 0.33, true);
        powerLauncher.setIndexerBackPos();
        move(0.6, 0, 0.33, true);
        powerLauncher.toggleOff();
        stopDrive();
    }
}

// Documentation: https://ftctechnh.github.io/ftc_app/doc/javadoc/index.html