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

import org.firstinspires.ftc.teamcode.robot_components.Config;
import org.firstinspires.ftc.teamcode.robot_components.Controller;
import org.firstinspires.ftc.teamcode.robot_components.FieldPosition;
import org.firstinspires.ftc.teamcode.robot_components.Robot;

@TeleOp
public class Tele3 extends LinearOpMode implements FieldPosition {

    Robot robot;
    Controller controller1;

    @Override
    public void runOpMode()
    {
        robot = new Robot(hardwareMap, telemetry);
        controller1 = new Controller(gamepad1);
        robot.init();
        robot.setTargetToStack();
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        robot.updateObjectValues();
        robot.resetElapsedTime();


        /*
        // Determine how many rings in the starting ring stacks
        Config config = robot.identifyRingConfig();

        madeIt("config identified: " + config);

        if (!config.equals(Config.ZERO)) {
            //Move forward 6-7 feet until at the edge of launch zone
            robot.moveToPos(NEXT_TO_STARTER_STACK_POS, 0.1);

            madeIt("next to starter stack");
            robot.moveToPos(new int[]{142, 65});
        }


        //Move sideways until in line with tower goal
        //aim the robot at the goal and make sure that the robot is within the launch zone
        //Shoot 3 rings
        robot.adjustAndShoot(3);
        madeIt("shot 3 goals");

        // if starterStack != 0, pickup the starter stack rings
        if (!config.equals(Config.ZERO)) {
            robot.runIntake(0.8);
            robot.move(0, -0.3, 3.0);
            robot.runIntake(0.0);
            madeIt("gathered rings");

            if (config.equals(Config.ONE)) {
                robot.adjustAndShoot(1);
            } else {
                robot.adjustAndShoot(3);
            }
        }


        //move forward towards the desired target wobble goal zone
        //Check distance to tower goal and correct if necessary
        //Move left or right depending on target wobble goal
        if (config.equals(Config.ZERO)) {
            robot.moveToPos(CONFIG_0_POS_I);
        } else if (config.equals(Config.ONE)) {
            robot.moveToPos(CONFIG_1_POS_I, 0.0, 1.0, 4.0, true);
        } else {
            robot.moveToPos(CONFIG_4_POS_I, 0.0, 1.0, 4.0, true);
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
        robot.moveToPos(PERFECT_LAUNCH_POS, 0.0);

        madeIt("going for the second wobble goal");

        //pick up second wobble goal
        robot.pickUpWobbleGoal();


        madeIt("picked up the second wobble goal");

        //Check that we're in shooting position
        //Shoot the 3 rings

        //move forward towards the desired target wobble goal zone
        //Check distance to tower goal and correct if necessary
        //Move left or right depending on target wobble goal
        robot.wait(0.01);
        if (config.equals(Config.ZERO)) {
            robot.moveToPos(CONFIG_0_POS_II);
        } else if (config.equals(Config.ONE)) {
            robot.moveToPos(CONFIG_1_POS_II, 0.0, 1.0, 4.0, true);
        } else {
            robot.moveToPos(CONFIG_4_POS_II, 0.0, 1.0, 4.0, true);
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

        //Move forward to park over launch line
        robot.moveToPos(PARK_POS);
        robot.stopDrive();


        madeIt("parked");


         */

        int config = Config.ONE;
        while (opModeIsActive()) {
            controller1.update();
            if (controller1.a.equals("pressing")) {
                // Determine how many rings in the starting ring stacks
                config = robot.identifyRingConfig();


                madeIt("config identified: " + config);

                if (config != Config.ZERO) {
                    //Move forward 6-7 feet until at the edge of launch zone
                    robot.moveToPos(NEXT_TO_STARTER_STACK_POS, 0.1);

                    madeIt("next to starter stack");
                    robot.moveToPos(new int[]{142, 65});
                }
            }
            if (controller1.b.equals("pressing")) {
                //Move sideways until in line with tower goal
                //aim the robot at the goal and make sure that the robot is within the launch zone
                //Shoot 3 rings
                robot.adjustAndShoot(3);
                madeIt("shot 3 goals");

                // if starterStack != 0, pickup the starter stack rings
                if (config != Config.ZERO) {
                    robot.runIntake(0.8);
                    robot.move(0, -0.3, 3.0);
                    robot.runIntake(0.0);
                    madeIt("gathered rings");

                    if (config != Config.ONE) {
                        robot.adjustAndShoot(1);
                    } else {
                        robot.adjustAndShoot(3);
                    }
                }
            }
            if (controller1.x.equals("pressing")) {
                //move forward towards the desired target wobble goal zone
                //Check distance to tower goal and correct if necessary
                //Move left or right depending on target wobble goal
                if (config != Config.ZERO) {
                    robot.moveToPos(CONFIG_0_POS_I);
                } else if (config != Config.ONE) {
                    robot.moveToPos(CONFIG_1_POS_I, 4.0);
                } else {
                    robot.moveToPos(CONFIG_4_POS_I, 4.0);
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
            }
            if (controller1.y.equals("pressing")) {
                //Head back to location where we shot the rings
                //Move left or right and then backward towards second wobble goal
                robot.moveToPos(PERFECT_LAUNCH_POS, 0.0);

                madeIt("going for the second wobble goal");

                //pick up second wobble goal
                robot.pickUpWobbleGoal();


                madeIt("picked up the second wobble goal");
            }

            if (controller1.dpad_left.equals("pressing")) {
                //Check that we're in shooting position
                //Shoot the 3 rings

                //move forward towards the desired target wobble goal zone
                //Check distance to tower goal and correct if necessary
                //Move left or right depending on target wobble goal
                robot.wait(0.01);
                if (config == Config.ZERO) {
                    robot.moveToPos(CONFIG_0_POS_II);
                } else if (config == Config.ONE) {
                    robot.moveToPos(CONFIG_1_POS_II, 4.0);
                } else {
                    robot.moveToPos(CONFIG_4_POS_II, 4.0);
                }

                madeIt("brought second wobble goal to drop zone");
            }
            if (controller1.dpad_up.equals("pressing")) {
                madeIt("brought second wobble goal to drop zone");

                //once there, place down the wobble goal
                robot.turnArm();
                robot.wait(0.4);
                robot.toggleClaw();
                robot.wait(0.4);
                robot.turnArm();
                robot.wait(0.1);
                robot.toggleClaw();
            }
            if (controller1.dpad_right.equals("pressing")) {
                //Move forward to park over launch line
                robot.moveToPos(PARK_POS);
                robot.stopDrive();


                madeIt("parked");
            }

            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);
        }


    }

    public void madeIt(String s) {
        robot.telemetry.addData("Made it! Status: ", s);
        robot.telemetry.update();
        robot.wait(1.0);
    }
}