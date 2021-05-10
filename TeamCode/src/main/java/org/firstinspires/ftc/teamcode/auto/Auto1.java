package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.data.FieldPositions;
import org.firstinspires.ftc.teamcode.robot_components.CVRobot;


@Autonomous
public class Auto1 extends LinearOpMode implements FieldPositions {

    // Declare OpMode members
    CVRobot robot;

    @Override
    public void runOpMode()
    {
        robot = new CVRobot(hardwareMap, telemetry);

        robot.initWithCV();
        robot.stack.activate();
        robot.wobble.activate();
        robot.startUp();

        telemetry.addData("Status", "Initialized");
        robot.telemetry.update();
        robot.powerLauncher.setLaunchAngleVertical();

        // Waiting for driver to hit PLAY
        waitForStart();

        robot.resetElapsedTime();
        robot.powerLauncher.setLaunchAngleLoading();
        robot.turnArmUpFull();

        // Determine how many rings in the starting ring stacks
        int config = robot.identifyRingConfig();

        robot.resetGyroAngle();
        robot.powerLauncher.setLaunchAnglePerfect();


        switch (config) {
            case 0:
                runZeroRingAuto();
                break;
            case 1:
                runOneRingAuto();
                break;
            case 4:
                runFourRingAutoNew();
                break;
        }

        robot.stopDrive();
    }

    private void dropWobble() {
        //once there, place down the wobble goal
        robot.turnArmDownFull();
        robot.toggleClaw();
        robot.wait(0.2);
        robot.turnArmUpFull();
        robot.wait(0.3);
        robot.move(1.0, 0, 0.25);
        robot.toggleClaw();
    }

    private void runZeroRingAuto() {

        robot.wait(0.4);

        // Turn arm down so dropping wobble goal later is quicker
        robot.turnArmDownDrag();

//        robot.towerXPID.setMinMax(-0.7, 0.7);
//        robot.towerWPID.setMinMax(-0.7, 0.7);

        // Shoot power shots with preloaded rings
//        robot.moveToPos(MID_POWERSHOT_POS, 0.5, 1.4, 5.0, 1);
//        robot.shootPowerShots();

        robot.moveToPos(LEFT_POWERSHOT_POS, 0.5, 1.4, 5.0, 1);
        robot.shootPowerShotsStrafeCV();

        // Deliver first wobble goal
//        robot.turnArmDownDrag();
        robot.moveToPos(CONFIG_0_POS_I, 0.5, 1.2);
        dropWobble();

        // Grab 2nd wobble goal
        robot.moveToPos(SECOND_WOBBLE_POS, 0.5, 1.0, 4.0, 1);
        robot.alignToWobble();
        robot.pickUpWobbleGoal("down");

        // Deliver 2nd wobble goal
        robot.moveToPos(PARK_0_POS, 0, 0, 1.0);
        robot.moveToPos(CONFIG_0_POS_II, 0.5, 1.5);
        dropWobble();

        // Move back to park over launch line
        if (robot.elapsedSecs() < 29) {
            robot.moveToPos(PARK_0_POS, 0.1, 1.0);
        }
        robot.stopDrive();
    }

    private void runOneRingAuto() {

        robot.wait(0.4);

        // Turn arm down so dropping wobble goal later is quicker
        robot.turnArmDownDrag();

        // Shoot power shots with preloaded rings
//        robot.moveToPos(MID_POWERSHOT_POS, 0.3, 1.0, 4.0, 1);
//        robot.shootPowerShots();

        robot.moveToPos(LEFT_POWERSHOT_POS, 0.3, 1.0, 5.0, 1);
        robot.shootPowerShotsStrafeCV();

        // Deliver first wobble goal
        robot.moveToPos(PRE_CONFIG_1_POS_I, 0.1, 1.0);
        robot.move(0, 1.0, 0.75);
        dropWobble();
        robot.move(0, -1.0, 0.6);


        // Load rings from starter stack
        robot.moveToPos(PERFECT_LAUNCH_POS, 0.3, 0);
        robot.powerLauncher.setLaunchAngleLoading();
        robot.runIntake(0.9);

        // Grab 2nd wobble goal
        robot.moveToPos(SECOND_WOBBLE_POS, 0.5, 1.0, 4.0, 1);
        robot.alignToWobble();
        robot.pickUpWobbleGoal("down");

        // Shoot one ring
        robot.runIntake(0.0);
        robot.powerLauncher.toggleOn();
        robot.moveToPos(PERFECT_LAUNCH_POS, 0.1, 0.4, 4.0, 2, true);
        robot.setAssistedLaunchAngle();
        robot.indexRings(1);
        robot.powerLauncher.toggleOff();

        // Deliver 2nd wobble goal
        robot.moveToPos(PRE_CONFIG_1_POS_II, 0.3, 1.0, 2.0);
        robot.move(0, 1.0, 0.75);
        dropWobble();

        // Move back to park over launch line
        if (robot.elapsedSecs() <= 29.35) {
            robot.move(0, -1.0, 0.65);
        }
        robot.stopDrive();
    }

    private void runFourRingAutoNew() {

        // Move forward but avoid starter stack
        robot.move(0, 1.0, 1.55);
        robot.powerLauncher.toggleOn();
        robot.move(-1.0, 0, 0.2);
        robot.moveToPos(PERFECT_LAUNCH_POS, 0.1, 0.6, 2.0, 1, true);

        // Shoot 3 preloaded rings
        robot.indexRings(3);
        robot.powerLauncher.setLaunchAngleVertical();
        robot.powerLauncher.toggleOff();

        // Turn arm down so dropping wobble goal later is quicker
        robot.turnArmDownDrag();

        // Deliver first wobble goal
        double t = robot.elapsedSecs();
        while (robot.elapsedSecs() - t < 2.0) {
            robot.calculateDrivePowers(-1.0, -0.5, 0.3, true);
            robot.sendDrivePowers();
        }
        robot.stopDrive();
        dropWobble();

        // Move back to perfect launch pos
        t = robot.elapsedSecs();
        while (robot.elapsedSecs() - t < 1.2) {
            robot.calculateDrivePowers(1.0, 0.7, -0.4, true);
            robot.sendDrivePowers();
        }
        robot.stopDrive();
        robot.powerLauncher.setLaunchAngleLoading();
        robot.moveToPos(PERFECT_LAUNCH_POS, 0.2, 0.8, 2.0, 1);

        // Load 1 or 2 rings from starter stack
        robot.runIntake(0.8);
        robot.move(0, -0.3, 0.7);
        robot.powerLauncher.toggleOn();
        robot.wait(0.4);
        robot.runIntake(0.0);

        // Shoot 1 ring
        robot.moveToPos(PERFECT_LAUNCH_POS, 0.1, 0.5, 5.0, 2, true);
        robot.indexRings(1);
        robot.powerLauncher.toggleOff();

        // Load remaining rings from starter stack and pick up 2nd wobble
        robot.towerXPID.setMinMax(-0.4, 0.4);
        robot.towerWPID.setMinMax(-0.4, 0.4);
        robot.powerLauncher.setLaunchAngleLoading();
        robot.runIntake(0.7);
        robot.move(0, -0.3, 0.5);
        robot.moveToPos(SECOND_WOBBLE_POS, 0.5, 1.0, 4.0, 1);
        robot.alignToWobble();
        robot.pickUpWobbleGoal("down");

        // Shoot remaining rings
        robot.towerXPID.setMinMax(-1, 1);
        robot.towerWPID.setMinMax(-1, 1);
        robot.powerLauncher.setLaunchAnglePerfect();
        robot.powerLauncher.toggleOn();
        robot.moveToPos(PERFECT_LAUNCH_POS, 0.2, 0.8, 4.0, 2, true);
        robot.runIntake(0.0);
        robot.indexRings(3);
        robot.powerLauncher.toggleOff();

        // Deliver 2nd wobble goal
        robot.powerLauncher.setLaunchAngleVertical();
        t = robot.elapsedSecs();
        while (robot.elapsedSecs() - t < 1.9) {
            robot.calculateDrivePowers(-1.0, -0.5, 0.3, true);
            robot.sendDrivePowers();
        }
        robot.stopDrive();
        dropWobble();

        // Move back to park over launch line
        t = robot.elapsedSecs();
        while (robot.elapsedSecs() - t < 1.2) {
            robot.calculateDrivePowers(1.0, 0.7, -0.5, true);
            robot.sendDrivePowers();
            if (robot.elapsedSecs() > 29.95) {
                robot.stopDrive();
            }
        }
        robot.stopDrive();
    }

    private void runFourRingAuto() {

        // Move forward but avoid starter stack
        robot.moveToPos(NEXT_TO_STARTER_STACK_POS, 0, 0, 2.0);
        robot.powerLauncher.setLaunchAnglePerfect();

        // Deliver first wobble goal
        robot.moveToPos(PRE_CONFIG_4_POS_I, 0.2, 1.5, 2.0);
        robot.move(0, 1.0, 0.57);
        robot.rotateToPos(-70, 0.3);
        robot.turnArmDownDrag();
        robot.wait(0.4);
        dropWobble();

        // Shoot 3 preloaded rings
        robot.rotateToPos(15, 0.4);
        robot.move(0, -1.0, 0.7, true);
        robot.powerLauncher.toggleOn();
        robot.moveToPos(PERFECT_LAUNCH_POS, 1.0, 2.5, 5.0, 2, true);
        robot.indexRings(3);

        // Load 1 or 2 rings from starter stack
        robot.powerLauncher.setLaunchAngleLoading();
        robot.runIntake(0.9);
        robot.powerLauncher.toggleOff();
        robot.move(0, -0.3, 0.8);
        robot.wait(1.4);

        // Shoot 1 ring
        robot.powerLauncher.toggleOn();
        robot.powerLauncher.setLaunchAnglePerfect();
        robot.moveToPos(PERFECT_LAUNCH_POS, 1.0, 2.5, 5.0, 2, true);
//        robot.wait(0.3);
        robot.runIntake(0.0);
        robot.indexRings(1);
        robot.powerLauncher.toggleOff();

        // Load remaining rings from starter stack
        robot.powerLauncher.setLaunchAngleLoading();
        robot.runIntake(0.8);
        robot.move(0, -0.3, 2.8);

        // Shoot remaining rings
        robot.powerLauncher.setLaunchAnglePerfect();

        robot.powerLauncher.toggleOn();
        robot.moveToPos(PERFECT_LAUNCH_POS, 0.2, 0.8);
        robot.runIntake(0.0);
        robot.setAssistedLaunchAngle();
        robot.wait(0.3);
        robot.indexRings(3);
        robot.powerLauncher.toggleOff();

        // Deliver 2nd wobble goal
//        robot.moveToPos(PRE_CONFIG_4_POS_I, 1.0, 1.5, 3.0);
//        robot.move(0, 0.6, 0.8);
//        robot.rotateToPos(-45, 0.5);
//        placeWobble();

        // Move back to park over launch line
        robot.move(0, 1.0, 0.45);
    }
}
