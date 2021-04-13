package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.data.FieldPosition;
import org.firstinspires.ftc.teamcode.robot_components.Robot;


@Autonomous
public class Auto1 extends LinearOpMode implements FieldPosition {

    // Declare OpMode members
    Robot robot;

    @Override
    public void runOpMode()
    {
        robot = new Robot(hardwareMap, telemetry);
        robot.initWithCV();
        robot.stack.activate();
        robot.stack.updateData();
        int config = robot.identifyRingConfig(); // TODO : Comment out
        telemetry.addData("Status: ", "Initialized");
        telemetry.addData("Config: ", config);
        robot.updateDrive(); // Display telemetry

        waitForStart();

        robot.resetElapsedTime();
        robot.powerLauncher.setLaunchAngleHorizontal();
        robot.wait(0.4); // wait for launcher to go down

        // Determine how many rings in the starting ring stacks
        config = robot.identifyRingConfig();
        robot.stack.deactivate();

        robot.resetGyroAngle();
        robot.powerLauncher.setLaunchAnglePerfect();

        // Move forward but avoid starter stack
        robot.moveToPos(RIGHT_POWERSHOT_POS, 0.5, 0.6, 3.0);


        if (config == 0) {
            robot.moveToPos(CONFIG_0_POS_I, 1.0, 3.0);
            placeWobble();

            robot.moveToPos(PERFECT_LAUNCH_POS, 2.0, 3.0);
            robot.launchRings(3);
            madeIt("shot 3 goals");
        } else {
            if (config == 1) {
                robot.moveToPos(CONFIG_1_POS_I, 1.0, 3.0, 3.0);
                robot.move(0, 0.3, 0.5);
                placeWobble();
                robot.move(0, -1.0, 0.5);
            }

            robot.moveToPos(PERFECT_LAUNCH_POS, 2.0, 3.0);
            robot.launchRings(3);
            madeIt("shot 3 goals");

            robot.powerLauncher.setLaunchAngleHorizontal();
            robot.runIntake(1.0);
            robot.move(0, -0.17, 1.8);
            madeIt("gathered rings");

            robot.wait(2.7);
            robot.powerLauncher.setLaunchAnglePerfect();
            robot.runIntake(0.0);
            robot.moveToPos(PERFECT_LAUNCH_POS, 1.0, 2.0);
            if (config == 1) {
                robot.launchRings(1);
            } else {
                robot.launchRings(3);
            }
        }

        //move forward towards the desired target wobble goal zone
        //Check distance to tower goal and correct if necessary
        //Move left or right depending on target wobble goal

        // Move back to park over launch line
        robot.moveToPos(PARK_POS);
        robot.stopDrive();
    }

    private void getPowerShots() {
        // Shoot power shots
        robot.powerLauncher.setLaunchAnglePowershot();
        robot.moveToPos(RIGHT_POWERSHOT_POS, 1.5, 3.0);
        robot.launchRings(1);
        robot.move(-0.6, 0, 0.6);
        robot.rotateToPos(0, 0.5);
        robot.launchRings(1);
        robot.move(-0.6, 0, 0.6);
        robot.rotateToPos(0, 0.5);
        robot.launchRings(1);
    }

    private void madeIt(String s) {
        robot.telemetry.addData("Made it! Status: ", s);
        robot.telemetry.update();
        robot.wait(0.0);
    }

    private void placeWobble() {
        //once there, place down the wobble goal
        robot.turnArm();
        robot.wait(0.3);
        robot.toggleClaw();
        robot.wait(0.3);
        robot.turnArm();
        robot.wait(0.5);
        robot.toggleClaw();
        robot.move(-0.7, 0, 0.5);
    }
}
