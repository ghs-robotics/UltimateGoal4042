/*
 * Copyright (c) 2020 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.test_opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot_components.Controller;
import org.firstinspires.ftc.teamcode.robot_components.Robot;

@TeleOp
public class Tele3 extends LinearOpMode
{
    public static final int[] NEXT_TO_STARTER_STACK_POS = new int[]{198, 65}; // x position, width // DONE
    public static final int[] SHOOTER_POS = new int[]{138, 57}; // DONE Perfect launch position
    public static final int[] CONFIG_0_POS_I = new int[]{78, 78}; // DONE Deliver first wobble goal
    public static final int[] CONFIG_0_POS_II = new int[]{112, 78}; // DONE Deliver second wobble goal
    public static final int[] CONFIG_1_POS_I = new int[]{156, 119}; // DONE
    public static final int[] CONFIG_1_POS_II = new int[]{186, 119}; // DONE
    public static final int[] CONFIG_4_POS_I = new int[]{135, 33}; // DONE Move forward one foot!
    public static final int[] CONFIG_4_POS_II = new int[]{135, 66}; // DONE Move forward one foot!
    public static final int[] SECOND_WOBBLE_POS = new int[]{138, 45}; // DONE
    public static final int[] STARTER_STACK_BEFORE_POS = new int[]{138, 57}; // DONE
    public static final int[] STARTER_STACK_AFTER_POS = new int[]{138, 45}; // DONE
    public static final int[] PARK_POS = new int[]{138, 65}; // DONE

    Robot robot;
    Controller controller1;

    @Override
    public void runOpMode()
    {
        robot = new Robot(hardwareMap, telemetry);
        controller1 = new Controller(gamepad1);
        robot.init();
        robot.setTargetToTower();
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        robot.updateObjectValues();
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
        if (robot.config == 1) {
            robot.adjustAndShoot(1);
        }

        madeIt("shot " + robot.config + "goal(s)");

        // if starterStack != 0, pickup the starter stack rings
        if (robot.config == 1) {
            robot.moveToPos(STARTER_STACK_BEFORE_POS, 2.0);
            robot.toggleIntake("on");
            robot.calculateDrivePowers(0, -0.4, 0);
            robot.sendDrivePowers();
            robot.wait(3.0);
            robot.stopDrive();
            robot.toggleIntake("off");
        }

        madeIt("gathered rings");

        //move forward towards the desired target wobble goal zone
        //Check distance to tower goal and correct if necessary
        //Move left or right depending on target wobble goal
        if (robot.config == 0) {
            robot.moveToPos(CONFIG_0_POS_I);
        } else if (robot.config == 1) {
            robot.moveToPos(CONFIG_1_POS_I, 1.0, true);
        } else {
            robot.moveToPos(CONFIG_4_POS_I, 1.0, true);
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
        robot.moveToPos(NEXT_TO_STARTER_STACK_POS, 0.0);
        robot.moveToPos(SECOND_WOBBLE_POS, 3.0);

        madeIt("going for the second wobble goal");

        //pick up second wobble goal
        robot.pickUpWobbleGoal(1.0);


        madeIt("picked up the second wobble goal");

        //Check that we're in shooting position
        //Shoot the 3 rings

        //move forward towards the desired target wobble goal zone
        //Check distance to tower goal and correct if necessary
        //Move left or right depending on target wobble goal
        robot.wait(0.01);
        if (robot.config == 0) {
            robot.moveToPos(CONFIG_0_POS_II);
        } else if (robot.config == 1) {
            robot.moveToPos(CONFIG_1_POS_II, 1.0, true);
        } else {
            robot.moveToPos(CONFIG_4_POS_II, 1.0, true);
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

        /*
        while (opModeIsActive()) {
            controller1.update();
            if (controller1.a.equals("pressing")) {
                robot.moveToPos(NEXT_TO_STARTER_STACK_POS, 10);
            }
            if (controller1.b.equals("pressing")) {
                robot.adjustAndShoot(3);
            }
            if (controller1.y.equals("pressing")) {
                robot.moveToPos(new int[]{170, 90}, 3);
            }
            if (controller1.x.equals("pressing")) {
                robot.moveToPos(new int[]{100, 105}, 3);
            }

            if (controller1.dpad_down.equals("pressing")) {
                double[] list = Robot.lower.val;
                Robot.lower = new Scalar(list[0], list[1], list[2] - 2);
            }
            if (controller1.dpad_up.equals("pressing")) {
                double[] list = Robot.lower.val;
                Robot.lower = new Scalar(list[0], list[1], list[2] + 2);
            }

            robot.telemetry.addData("HSV MIN, MAX: ", Robot.lower + ", " + Robot.upper);
            robot.telemetry.update();

            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);
        }
        */
    }

    public void madeIt(String s) {
        robot.telemetry.addData("Made it! Status: ", s);
        robot.telemetry.update();
        robot.wait(1.0);
    }
}