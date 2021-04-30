package org.firstinspires.ftc.teamcode.robot_components;

import com.qualcomm.robotcore.hardware.HardwareMap;

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

public class CVRobot extends Robot implements HSVConstants, FieldPositions {

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
    protected double x = 0;
    protected double y = 0;

    // Stores data for automated functions in TeleOp
    protected double phaseTimeStamp = 0;

    // PID controllers
    public PIDController towerXPID; // For the x-position of the tower goal
    public PIDController towerWPID; // For the width of the tower goal
    public PIDController xPID;
    public PIDController wPID; // For the y-position of the robot

    // Constructs a robot that uses CV
    public CVRobot(HardwareMap hardwareMap, Telemetry telemetry) {

        // Calls the constructor in DriveBase, which handles all the drive base motors
        super(hardwareMap, telemetry);

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
        target = wobble;
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

    // Displays a bunch of useful values on the DS phone
    @Override
    public void addTelemetryData() {

//        addCVTelemetryData();

        telemetry.addData("CURRENT LAUNCH ANGLE", "" + Math.round(1000 * powerLauncher.launchAngle));
        telemetry.addData("PERFECT LAUNCH ANGLE", "" + Math.round(1000 * powerLauncher.PERFECT_LAUNCH_ANGLE));
        telemetry.addData("gyro angle", "" + gyro.getAngle());

        telemetry.addLine("sleepTimeMS: " + CVDetectionPipeline.sleepTimeMS);

        double dist = tower.cvtH2VerticalDist();
        telemetry.addData("Dist", "" + dist);
        telemetry.addData("Launch offset", "" + tower.cvtFt2LaunchOffset(dist));

        telemetry.addLine();

        telemetry.addLine("" + target.toString());

        telemetry.addLine();
//        telemetry.addData("phonecam crosshair: ", camera.phoneCamPipeline.crosshairHSV);
//        telemetry.addData("webcam crosshair: ", camera.webcamPipeline.crosshairHSV);

        telemetry.addData("TOWER", "" + tower.toString());
        telemetry.addData("WALL", "" + wall.toString());
        telemetry.update();
    }

    private void addCVTelemetryData() {
        telemetry.addLine("STREAMING: " + camera.isStreaming());
        telemetry.addLine("CONFIG: " + identifyRingConfig());

        telemetry.addLine("LeftRightError: " + tower.getLeftRightError(0));
        telemetry.addLine("Left side: " + tower.x);
        telemetry.addLine("Right side: " +  (320 - tower.x - tower.w));

        telemetry.addData("(x, y)", "( " + x + ", " + y + " )");
        telemetry.addData("Kp", target.depthPID.k_P);
        telemetry.addData("Ki", target.depthPID.k_I);
        telemetry.addData("Kd", target.depthPID.k_D);
    }

    public double getPhaseTimePassed() {
        return elapsedSecs() - phaseTimeStamp;
    }

    // Call after using CV for field localization in order to reduce lag in teleop
    public void deactivateFieldLocalization() {
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

    // Makes the robot chase the target object (if called repeatedly)
    // Make sure the following things are accounted for before calling this:
    // gyro angle, target values, reset PIDs, activate object, set launcher side as front
    public void chaseObject(CVObject target) {
        x = target.getBreadthPIDValue();
        y = target.getDepthPIDValue();
        calculateDrivePowers(x, y, getGyroPIDValue());
        updateDrive();
    }

    public int moveInPhases(int phase) {
        return moveInPhases(phase, 0.2, 1.2, 4.0);
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
                        || (elapsedSecs() - phaseTimeStamp < maxFineTuning && !tower.targetInRange(1))) {
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

    public void moveToPos(int[] pos, double minFineTuning, double maxFineTuning, double maxBroadTuning, int tolerance) {
        moveToPos(pos, minFineTuning, maxFineTuning, maxBroadTuning, tolerance, false);
    }

    // Makes the robot move to a certain position relative to the tower goal
    public void moveToPos(int[] pos, double minFineTuning, double maxFineTuning, double maxBroadTuning, int tolerance, boolean shooting) {
        tower.setTargetXW(pos);
        activateFieldLocalization();
        rotateToPos(0.0, 0.0);
        double t = elapsedSecs();

        while((!tower.isIdentified() || !tower.targetInRange(4) || getAbsoluteGyroError() > 4)
                && elapsedSecs() - t < maxBroadTuning) {
            adjustPosition();
            if (shooting && tower.h <= 39) {
                setAssistedLaunchAngle();
            }
        }
        t = elapsedSecs();
        while (elapsedSecs() - t < minFineTuning
                || (elapsedSecs() - t < maxFineTuning && !tower.targetInRange(tolerance))) {
            adjustPosition();
            if (shooting) {
                setAssistedLaunchAngle();
            }
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
        stopDrive();
        wall.activate();
        turnArmDownFull();
        toggleClaw();
        double t = elapsedSecs();
        calculateDrivePowers(0, -0.3, 0);
        sendDrivePowers();
        while ( wall.h < 86 &&  elapsedSecs() - t < 2.0) { // TODO
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

    public void rotateAndShoot() {
        int phase = 20;
        while (phase > 0) {
            phase = rotateAndShootInPhases(phase);
        }
    }

    // Automated move to position function that uses phases and must be called repeatedly
    // This allows us to terminate the function early (because we can just set phase to be 0)
    public int rotateAndShootInPhases(int phase) { // TODO : OPTIMIZE

        // Distance between tower goal and left side of screen: tower.x
        // Distance between tower goal and right side of screen: 320 - tower.x - tower.w
        double error = tower.x - (334 - tower.x - tower.w);

        if (phase >= 10) {
            activateFieldLocalization();
            tower.setTargetXW(PERFECT_LAUNCH_POS);
            targetGyroAngle = getReasonableGyroAngle(0);
            phase = 8;
        }

        switch (phase) {
            case 9:
                if (camera.isStreaming() && camera.isWebcamReady()) {
                    phaseTimeStamp = elapsedSecs();
                    phase--;
                }
                break;
            case 8:
                if (!tower.isIdentified() || tower.h > 37 || getAbsoluteGyroError() > 20) {
                    adjustPosition();
                } else if (Math.abs(error) <= 20) {
                    phaseTimeStamp = elapsedSecs();
                    phase--;
                } else {
                    calculateDrivePowers(0, 0, metaGyroPID.calcVal(0.3 * error));
                    sendDrivePowers();
                }
                break;
            case 7:
                if (Math.abs(error) <= 10 && getPhaseTimePassed() > 0.1) {
                    phaseTimeStamp = elapsedSecs();
                    phase--;
                } else if (tower.isIdentified()) {
                    calculateDrivePowers(0, 0, (error > 0 ? 0.14 : -0.14));
                    sendDrivePowers();
                }
                break;
            case 6:
                if (Math.abs(error) <= 3 && getPhaseTimePassed() > 0.1) {
                    powerLauncher.toggleOn();
                    setAssistedLaunchAngle();
                    powerLauncher.resetQueueTimeStamp();
                    phaseTimeStamp = elapsedSecs();
                    phase--;
                } else if (tower.isIdentified()) {
                    calculateDrivePowers(0, 0, (error > 0 ? 0.10 : -0.10));
                    sendDrivePowers();
                }
                break;
            case 5:
                if (elapsedSecs() - phaseTimeStamp > 0.6) {
                    phase--;
                }
        }

        if (0 < phase && phase <= 4) {
            phase = powerLauncher.handleIndexQueue(phase);
            if (phase == 0) {
                powerLauncher.toggleOff(); // Turn launcher off after indexing
            }
        }

        return phase;
    }

    public void setAssistedLaunchAngle() {
        powerLauncher.setLaunchAngle(tower.findLaunchAngle(gyro.getAngle()));
        if (powerLauncher.launchAngle > powerLauncher.PERFECT_LAUNCH_ANGLE + 0.100) {
            powerLauncher.setLaunchAnglePerfect();
        }
    }

    // Rotate in place using the tower goal coordinates
    public void rotateUsingCV(int offSet) {
        targetGyroAngle = getReasonableGyroAngle(0);
        if (getAbsoluteGyroError() > 10) {
            rotateToPos(0, 0.4);
        }

        double error = tower.getLeftRightError(offSet - 5);
        CVDetectionPipeline.sleepTimeMS = 0;

        while (error > -25) {
            CVDetectionPipeline.sleepTimeMS = 0;
            error = tower.getLeftRightError(offSet - 5);
            calculateDrivePowers(0, 0, 0.2);
            sendDrivePowers();
        }
        stopDrive();
        wait(0.1);
        error = tower.getLeftRightError(offSet - 5);

        while (error < 0) {
            CVDetectionPipeline.sleepTimeMS = 0;
            error = tower.getLeftRightError(offSet - 5);
            calculateDrivePowers(0, 0, -0.15);
            sendDrivePowers();
        }
        stopDrive();
    }

    // Go to MID_POWERSHOT_POS before calling this
    public void shootPowerShots() {

        // Setup
        activateFieldLocalization();
        targetGyroAngle = getReasonableGyroAngle(0);

        setAssistedLaunchAngle();
        powerLauncher.changeLaunchAngle(-0.015);
        rotateUsingCV(-148);

        // TODO
        powerLauncher.toggleOn(0.94);
        wait(0.4);
        indexRings(1);
        powerLauncher.toggleOff();

        rotateUsingCV(-104);
        powerLauncher.toggleOn(0.94);
        wait(0.4);
        indexRings(1);
        powerLauncher.toggleOff();

        rotateUsingCV(-66);
        powerLauncher.toggleOn(0.94);
        wait(0.4);
        indexRings(1);
        powerLauncher.toggleOff();
    }

    public void alignToWobble() {
//        wobble.activate();
//        while (wobble.isIdentified()) {
//            CVDetectionPipeline.sleepTimeMS = 0;
//            calculateDrivePowers(-0.3, 0, 0, true);
//            sendDrivePowers();
//        }

        while (tower.x > 135) {
            calculateDrivePowers(0.3, 0, getGyroPIDValue());
            sendDrivePowers();
        }
//        while (tower.x < 135) {
//            calculateDrivePowers(-0.3, 0, getGyroPIDValue());
//            sendDrivePowers();
//        }

        stopDrive();
    }
}

// Documentation: https://ftctechnh.github.io/ftc_app/doc/javadoc/index.html