/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.data.FieldPositions;
import org.firstinspires.ftc.teamcode.robot_components.CVDetectionPipeline;
import org.firstinspires.ftc.teamcode.robot_components.Controller;
import org.firstinspires.ftc.teamcode.robot_components.CVRobot;

@TeleOp(name="Tele1", group="Linear Opmode")
public class Tele1 extends LinearOpMode implements FieldPositions {

    // Declare OpMode members
    CVRobot robot;
    Controller controller1;
    Controller controller2;

    @Override
    public void runOpMode() {

        // Code to run ONCE when the driver hits INIT
        robot = new CVRobot(hardwareMap, telemetry);
        controller1 = new Controller(gamepad1); // Whoever presses start + a
        controller2 = new Controller(gamepad2); // Whoever presses start + b

        int queue = 0; // Keeps track of how many rings are "in line" to be shot
        int movePhase = 0; // 0 is normal; not 0 means robot will perform an automated function
        int autoAimPhase = 0; // 0 is normal; not 0 means robot will perform an automated function
        String intakeSetting = "normal"; // "normal," "in," "out"

        robot.initWithCV();
        robot.powerLauncher.setLaunchAngleLoading();
        robot.turnArmUpFull();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
//        robot.camera.webcam.pauseViewport();
        robot.activateFieldLocalization();
//        telemetry.setMsTransmissionInterval(20);
        robot.wobble.activate();

        robot.resetGyroAngle();
        robot.resetElapsedTime();

        while (opModeIsActive()) {

            // Registers controller input
            controller1.update();
            controller2.update();



            // -----------------------------------------------------------------------------------------
            // -----------------------------------------------------------------------------------------
            // ------------------------------   CONTROLLER 1 FUNCTIONS   -------------------------------
            // -----------------------------------------------------------------------------------------
            // -----------------------------------------------------------------------------------------

            // NOTE: TO USE THESE FUNCTIONS PRESS START A


            // Checks if the robot should be performing an automated move function
            if (movePhase > 0) {
                CVDetectionPipeline.sleepTimeMS = 0;
                movePhase = robot.moveInPhases(movePhase);
            }
            else if (autoAimPhase > 0) {
                CVDetectionPipeline.sleepTimeMS = 0;
                autoAimPhase = robot.rotateAndShootInPhases(autoAimPhase);
                robot.setAssistedLaunchAngle();
            }
            else {
                CVDetectionPipeline.sleepTimeMS = 500;
                if (!controller1.right_bumper.equals("pressed")) {
                    // Mecanum wheel drive in meta mode
                    robot.calculateDrivePowers(
                            controller1.left_stick_x,
                            controller1.left_stick_y,
                            controller1.right_stick_x,
                            controller1.right_stick_y
                    );
                }
                else {
                    // Always rotate to face tower goal
//                    robot.calculateDrivePowers(controller1.left_stick_x, controller1.left_stick_y, 1.0, 0);
                    robot.calculateDrivePowers(controller1.left_stick_x, controller1.left_stick_y, 0.9511, 0.3090); // face 16 degrees
                }

                if (controller1.left_trigger > 0.1) {
                    robot.calculateDrivePowers(0, 0, -0.25 * controller1.left_trigger);
                }
                else if (controller1.right_trigger > 0.1) {
                    robot.calculateDrivePowers(0, 0, 0.25 * controller1.right_trigger);
                }

                robot.updateDrive(); // Also updates telemetry
            }

            //
            if (controller1.a.equals("pressing")) {
                if (intakeSetting.equals("in")) {
                    intakeSetting = "normal";
                } else {
                    intakeSetting = "in";
                }
            }

            //
            if (controller1.b.equals("pressing")) {
                if (intakeSetting.equals("out")) {
                    intakeSetting = "normal";
                } else {
                    intakeSetting = "out";
                }
            }

            // Toggle speed
            if (controller1.x.equals("pressing")) {
//                intakeSetting = "normal";
//                robot.rotateToPos(0, 0.7);
//                autoAimPhase = 10;
                intakeSetting = "normal";
                robot.powerLauncher.toggleOn();
                robot.powerLauncher.setLaunchAnglePerfect();
                robot.wait(0.5);
                robot.indexRings(3);
                robot.powerLauncher.toggleOff();
            }

            // Terminate any automated functions and stop streaming
            if (controller1.y.equals("pressing")) {
                movePhase = 0;
                autoAimPhase = 0;
            }

            // Reset gyro in case of emergency
            if (controller1.left_bumper.equals("pressing")) {
                robot.resetGyroAngle();
            }

            // Reset any controls
            if (controller1.left_stick_button.equals("pressing")) {
                robot.speed = 1;
                movePhase = 0;
                autoAimPhase = 0;
                CVDetectionPipeline.sleepTimeMS = 500;
                robot.powerLauncher.toggleOff();
            }

            if (controller1.dpad_right.equals("pressed")) {
//                robot.tower.setTargetXW(LEFT_POWERSHOT_POS);
//                movePhase = 4;
                intakeSetting = "normal";
                robot.rotateToPos(0, 0.7);
                autoAimPhase = 10;
            }
            else if (controller1.dpad_up.equals("pressed")) {
                movePhase = 0;
                autoAimPhase = 0;
                robot.shootPowerShotsStrafe();
            }
            else if (controller1.dpad_left.equals("pressed")) {
                robot.tower.setTargetXW(PERFECT_LAUNCH_POS);
                movePhase = 4;
            }
            else if (controller1.dpad_down.equals("pressed")) {
                robot.cameras.stopStreaming();
                movePhase = 0;
                autoAimPhase = 0;
            }



            // -----------------------------------------------------------------------------------------
            // -----------------------------------------------------------------------------------------
            // ------------------------------   CONTROLLER 2 FUNCTIONS   -------------------------------
            // -----------------------------------------------------------------------------------------
            // -----------------------------------------------------------------------------------------

            // NOTE: TO USE THESE FUNCTIONS, PRESS START B


            // Checks if any rings need to be shot and takes care of indexing
            if (queue > 0) {
                queue = robot.powerLauncher.handleIndexQueue(queue);
                if (queue == 0) { // Turn launcher off after indexing
                    robot.powerLauncher.toggleOff();
                }
            }

            // Intake stuff
            if (intakeSetting.equals("in")) {
                robot.powerLauncher.setLaunchAngleLoading();
                robot.runIntake(0.85);
            }
            else if (intakeSetting.equals("out")) {
                robot.powerLauncher.setLaunchAngleLoading();
                robot.runIntake(-0.85);
            }
            else {
                // Run intake with right joystick
                robot.runIntake(-0.9 * controller2.right_stick_y);
            }

            // Set loading launch angle if the intake is running
            if (controller2.right_stick_y != 0) {
                robot.powerLauncher.setLaunchAngleLoading();
                intakeSetting = "normal";
            }

            // Toggle intake to gather rings
            if (controller2.left_trigger + controller2.right_trigger > 1.8) {
                intakeSetting = "in";
            }

            // Right bumper toggles the claw
            if (controller2.right_bumper.equals("pressing")) {
                robot.toggleClaw();
            }

            // Left bumper turns arm
            if (controller2.left_bumper.equals("pressing")) {
                robot.turnArm();
            }

            // Index 3 rings
            if (controller2.a.equals("pressing")) {
                robot.powerLauncher.resetQueueTimeStamp();
                queue = 3;
            }

            // Toggle launcher and set perfect launch angle
            // Note: You can interrupt queues this way!
            if (controller2.b.equals("pressing")) {
                robot.powerLauncher.setLaunchAnglePerfect();
                robot.powerLauncher.toggle();
                queue = 0;
            }

            // Manually index one ring (e.g. for powershots)
            if (controller2.x.equals("pressing")) {
                robot.indexRings(1);
            }

            // Toggle launcher motors
            if (controller2.y.equals("pressing")) {
                robot.powerLauncher.toggle();
                queue = 0;
            }

            //
            if (controller2.left_trigger_state.equals("pressing")) {
                robot.powerLauncher.setLaunchAngle2ndPerfect();
            }

            //
            if (controller2.right_trigger_state.equals("pressing")) {
                robot.powerLauncher.setLaunchAngle3rdPerfect();
            }

            // Angle the launcher down a tiny bit
            if (controller2.dpad_up.equals("pressing")) {
                robot.powerLauncher.PERFECT_LAUNCH_ANGLE -= 0.005;
            }

            // Angle the launcher up a tiny bit
            if (controller2.dpad_down.equals("pressing")) {
                robot.powerLauncher.PERFECT_LAUNCH_ANGLE += 0.005;
            }

            // Set current angle to default angle
            if (controller2.dpad_right.equals("pressing")) {
                robot.powerLauncher.setCurrentAngleToDefault();
            }

            //
            if (controller2.dpad_left.equals("pressing")) {
                robot.setAssistedLaunchAngle();
            }

            // Change current launch Angle (but keep default the same)
            if (controller2.left_stick_y != 0) {
                double val = -((int) (40 * controller2.left_stick_y)) / 1000.0;
                robot.powerLauncher.changeLaunchAngleGradually(val);
            }

            // Go to default launch angle
            if (controller2.left_stick_button.equals("pressing")) {
                robot.powerLauncher.setLaunchAnglePerfect();
            }
        }
    }
}
