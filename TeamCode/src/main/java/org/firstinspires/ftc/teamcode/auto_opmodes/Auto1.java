package org.firstinspires.ftc.teamcode.auto_opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot_components.Robot;


@Autonomous
public class Auto1 extends LinearOpMode {
    public static final int[] NEXT_TO_STARTER_STACK_POS = new int[]{170, 90}; // x position, width
    public static final int[] SHOOTER_POS = new int[]{100, 80};
    public static final int[] CONFIG_0_POS = new int[]{30, 107};
    public static final int[] CONFIG_1_POS = new int[]{103, 115}; //TO DO: CAN BARELY SEE TOWER GOAL FROM HERE!
    public static final int[] CONFIG_4_POS = new int[]{100, 80}; // TO DO: CAN'T SEE TOWER GOAL FROM THIS POS!
    public static final int[] SECOND_WOBBLE_POS = new int[]{53, 73};
    public static final int[] STARTER_STACK_BEFORE_POS = new int[]{100, 80};
    public static final int[] STARTER_STACK_AFTER_POS = new int[]{100, 70};
    public static final int[] PARK_POS = new int[]{100, 95};

    // Declare OpMode members
    Robot robot;

    @Override
    public void runOpMode() {
        robot = new Robot(hardwareMap, telemetry);
        robot.init();
        robot.setTargetToTower();
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        robot.resetElapsedTime();

        //Determine how many rings in the starting ring stacks
//        robot.identifyRingConfig();
        robot.config = 1; // For testing purposes

        madeIt("config identified");

        //Move forward 6-7 feet until at the edge of launch zone
        robot.moveToPos(NEXT_TO_STARTER_STACK_POS);

        madeIt("next to starter stack");

        //Move sideways until in line with tower goal
        //aim the robot at the goal and make sure that the robot is within the launch zone
        //Shoot 3 rings
        robot.adjustAndShoot(3);

        madeIt("shot three goals");

        //move forward towards the desired target wobble goal zone
        //Check distance to tower goal and correct if necessary
        //Move left or right depending on target wobble goal
        if (robot.config == 0) {
            robot.moveToPos(CONFIG_0_POS);
        } else if (robot.config == 1) {
            robot.moveToPos(CONFIG_1_POS);
        } else {
            robot.moveToPos(CONFIG_4_POS);
        }

        madeIt("next to wobble goal drop zone");

        //once there, place down the wobble goal
        robot.turnArm();
        robot.wait(0.4);
        robot.toggleClaw();
        robot.wait(0.4);
        robot.turnArm();
        robot.wait(0.1);
        robot.toggleClaw();

        madeIt("set down first wobble goal");

        //Head back to location where we shot the rings
        //Move left or right and then backward towards second wobble goal
        robot.moveToPos(SECOND_WOBBLE_POS);

        madeIt("going for the second wobble goal");

        // Turn around
        robot.rotateToPos(180, 5);

        madeIt("turned around");

        //pick up second wobble goal
        robot.pickUpWobbleGoal(1.0);
        robot.targetGyroAngle = 0;

        madeIt("picked up the second wobble goal");

        // if starterStack != 0, pickup the starter stack rings
        if (robot.config == 1) {
            robot.moveToPos(STARTER_STACK_BEFORE_POS);
            robot.moveToPos(STARTER_STACK_AFTER_POS);
        }

        madeIt("gathered rings");


        //Check that we're in shooting position
        //Shoot the 3 rings

        //move forward towards the desired target wobble goal zone
        //Check distance to tower goal and correct if necessary
        //Move left or right depending on target wobble goal
        robot.wait(0.01);
        if (robot.config == 0) {
            robot.moveToPos(CONFIG_0_POS);
        } else if (robot.config == 1) {
            robot.moveToPos(CONFIG_1_POS);
        } else {
            robot.moveToPos(CONFIG_4_POS);
        }

        madeIt("brought second wobble goal to drop zone");

        //once there, place down the wobble goal
        robot.turnArm();
        robot.wait(0.4);
        robot.toggleClaw();
        robot.wait(0.4);
        robot.turnArm();
        robot.wait(0.1);
        robot.toggleClaw();

        madeIt("delivered the second wobble");

        if(robot.getElapsedTimeSeconds() <= 26) {

            robot.wait(0.01);
            robot.adjustAndShoot(3);
            madeIt("shot more rings");
        } else {
            madeIt("Skipped shooting second ring set");
        }



        //Move forward to park over launch line
        robot.moveToPos(PARK_POS);
        robot.stopDrive();


        madeIt("parked");
    }

    public void madeIt(String s) {
        robot.telemetry.addData("Made it! Status: ", s);
        robot.telemetry.update();
        robot.wait(0.0);
    }
}
