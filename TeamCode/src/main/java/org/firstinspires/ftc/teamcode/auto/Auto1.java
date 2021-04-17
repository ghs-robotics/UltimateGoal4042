package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.data.FieldPositions;
import org.firstinspires.ftc.teamcode.robot_components.Robot;


@Autonomous
public class Auto1 extends LinearOpMode implements FieldPositions {

    // Declare OpMode members
    Robot robot;

    @Override
    public void runOpMode()
    {
        robot = new Robot(hardwareMap, telemetry);
//        robot.powerLauncher.setLaunchAngleVertical();

        robot.initWithCV();
        robot.stack.activate();
        robot.activateFieldLocalization();

        telemetry.addData("Status: ", "Initialized");
        robot.telemetry.update();

        // Waiting for driver to hit PLAY
        waitForStart();

        robot.resetElapsedTime();
        robot.powerLauncher.setLaunchAngleLoading();
        robot.wait(0.5); // wait for launcher to go down

        // Determine how many rings in the starting ring stacks
        int config = robot.identifyRingConfig();
        robot.stack.deactivate();

        robot.resetGyroAngle();
        robot.powerLauncher.setLaunchAnglePerfect();

        config = 4; // TODO : DELETE

        if (config == 0) {
            runZeroRingAuto();
        }
        else if (config == 1) {
            runOneRingAuto();
        }
        else if (config == 4) {
            runFourRingAuto();
        }

        robot.stopDrive();
    }

    private void getPowerShots() {
        // Shoot power shots
        robot.powerLauncher.setLaunchAnglePowershot();
        robot.moveToPos(LEFT_POWERSHOT_POS, 1.5, 3.0);
        robot.indexRings(1);
        robot.move(-0.6, 0, 0.6);
        robot.rotateToPos(0, 0.5);
        robot.indexRings(1);
        robot.move(-0.6, 0, 0.6);
        robot.rotateToPos(0, 0.5);
        robot.indexRings(1);
    }

    private void placeWobble() {
        //once there, place down the wobble goal
        robot.turnArm();
        robot.wait(0.25);
        robot.toggleClaw();
        robot.wait(0.2);
        robot.turnArm();
        robot.move(1.0, 0, 0.25);
        robot.toggleClaw();
    }

    private void runZeroRingAuto() {
        // Deliver first wobble goal
        robot.moveToPos(PARK_0_POS, 0, 0);
        robot.moveToPos(CONFIG_0_POS_I, 0.5, 1.0);
        placeWobble();

        // Shoot 3 preloaded rings
        robot.powerLauncher.toggleOn();
        robot.moveToPos(PERFECT_LAUNCH_POS, 0.5, 2.0);
        robot.indexRings(3);

        // Grab 2nd wobble goal
        robot.moveToPos(SECOND_WOBBLE_POS);
        robot.pickUpWobbleGoal();

        // Deliver 2nd wobble goal
        robot.moveToPos(PERFECT_LAUNCH_POS, 0, 0);
        robot.moveToPos(CONFIG_0_POS_II, 0.5, 1.0);
        placeWobble();

        // Move back to park over launch line
        robot.moveToPos(PARK_0_POS);
    }

    private void runOneRingAuto() {

        // Move forward but avoid starter stack
        robot.moveToPos(LEFT_POWERSHOT_POS, 0, 0, 1.0);

        // Deliver first wobble goal
        robot.moveToPos(PRE_CONFIG_1_POS_I, 0.5, 1.5, 2.0);
        robot.move(0, 1.0, 0.6);
        placeWobble();

        // Shoot 3 preloaded rings
        robot.moveToPos(PERFECT_LAUNCH_POS, 1.0, 2.5);
        robot.indexRings(3);

        // Load rings from starter stack
        robot.powerLauncher.setLaunchAngleLoading();
        robot.runIntake(1.0);
        robot.move(0, -0.4, 1.0);


        // Grab 2nd wobble goal
        robot.moveToPos(SECOND_WOBBLE_POS, 0.5, 1.7);
        robot.pickUpWobbleGoal();

        // Shoot one ring
        robot.powerLauncher.setLaunchAnglePerfect();
        robot.runIntake(0.0);

        robot.moveToPos(PERFECT_LAUNCH_POS, 1.0, 2.0);
        robot.indexRings(1);

        // Deliver 2nd wobble goal
        robot.moveToPos(PRE_CONFIG_1_POS_II, 0.5, 1.5, 2.0);
        robot.move(0, 1.0, 0.6);
        placeWobble();

        // Move back to park over launch line
        robot.moveToPos(PARK_1_POS);
    }

    private void runFourRingAuto() {

        // Move forward but avoid starter stack
        robot.moveToPos(NEXT_TO_STARTER_STACK_POS, 0, 0, 1.5);
        robot.powerLauncher.setLaunchAnglePerfect();

        // Shoot 3 preloaded rings
        robot.moveToPos(PERFECT_LAUNCH_POS, 0.5, 1.5);
        robot.indexRings(3);

        // Load 1 or 2 rings from starter stack
        robot.powerLauncher.setLaunchAngleLoading();
        robot.runIntake(1.0);
        robot.move(0, -0.6, 0.8);

        // Shoot 1 ring
        robot.powerLauncher.setLaunchAnglePerfect();
        robot.moveToPos(PERFECT_LAUNCH_POS, 0.5, 1.5);
        robot.indexRings(1);
        robot.runIntake(0.0);
        robot.powerLauncher.setLaunchAngleVertical(); // Avoid running into the wall

        // Deliver first wobble goal
        robot.moveToPos(PRE_CONFIG_4_POS_I, 0.4, 1.2, 2.0);
        robot.move(0, 0.85, 0.8);
        robot.rotateToPos(-70, 0.3);
        placeWobble();

        // Load remaining rings from starter stack
        robot.moveToPos(PERFECT_LAUNCH_POS, 0.6, 2.0);
        robot.powerLauncher.setLaunchAngleLoading();
        robot.runIntake(1.0);
        robot.move(0, -0.3, 2.4);

        // Grab 2nd wobble goal
//        robot.moveToPos(SECOND_WOBBLE_POS, 0.5, 1.7);
//        robot.pickUpWobbleGoal();

        // Shoot remaining rings
        robot.powerLauncher.setLaunchAnglePerfect();

        robot.moveToPos(PERFECT_LAUNCH_POS, 0.5, 1.5);
        robot.runIntake(0.0);
        robot.indexRings(3);

        // Deliver 2nd wobble goal
//        robot.moveToPos(PRE_CONFIG_4_POS_I, 1.0, 1.5, 3.0);
//        robot.move(0, 0.6, 0.8);
//        robot.rotateToPos(-45, 0.5);
//        placeWobble();

        // Move back to park over launch line
        robot.moveToPos(PARK_4_POS);
    }
}
