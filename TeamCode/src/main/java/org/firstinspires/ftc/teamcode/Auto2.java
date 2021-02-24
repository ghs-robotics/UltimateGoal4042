package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * This will be the main class that we will put all of our autonomous code into
 * This class is obviously not complete, and the structure of the code is not decided yet
 */
@Autonomous
public class Auto2 extends LinearOpMode {
    public static final int[] NEXT_TO_STARTER_STACK_POS = new int[]{170, 90}; // x position, width
    public static final int[] SHOOTER_POS = new int[]{100, 80};
    public static final int[] CONFIG_0_POS = new int[]{30, 107};
    public static final int[] CONFIG_1_POS = new int[]{103, 125};
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

        int stage = 1;
        waitForStart();

        //Determine how many rings in the starting ring stacks
        robot.identifyRingConfig();
        robot.config = 1; // For testing purposes

        //Move forward 6-7 feet until at the edge of launch zone
        robot.moveToPos(NEXT_TO_STARTER_STACK_POS, 15);

        //Move sideways until in line with tower goal
        //aim the robot at the goal and make sure that the robot is within the launch zone
        //Shoot 3 rings
        robot.adjustAndShoot();

        //move forward towards the desired target wobble goal zone
        //Check distance to tower goal and correct if necessary
        //Move left or right depending on target wobble goal
        if (robot.config == 0) {
            robot.moveToPos(CONFIG_0_POS, 10);
        } else if (robot.config == 1) {
            robot.moveToPos(CONFIG_1_POS, 10);
        } else {
            robot.moveToPos(CONFIG_4_POS, 10);
        }

        robot.telemetry.addData("Made it to: ", "Checkpoint 1");
        robot.telemetry.update();

        //once there, place down the wobble goal
        robot.turnArm();
        robot.wait(0.5);
        robot.toggleGrab();
        robot.wait(0.1);
        robot.turnArm();
        robot.wait(0.4);
        robot.resetServos();
    }
}
