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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot_components.Controller;
import org.firstinspires.ftc.teamcode.robot_components.Robot;

@TeleOp(name="ImantsTesting", group="Iterative Opmode")
public class ImantsTesting extends OpMode
{
    //Declare OpMode members
    Robot robot;
    Controller controller2;
//    Controller controller2;

    //Code to run ONCE when the driver hits INIT
    @Override
    public void init() {
        robot = new Robot(hardwareMap, telemetry);
        controller2 = new Controller(gamepad1);
        robot.resetServos();
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    //Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
    @Override
    public void init_loop() {}

    //Code to run ONCE when the driver hits PLAY
    @Override
    public void start() { robot.resetElapsedTime(); }

    //Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
    @Override
    public void loop() {
        // Registers controller input
        controller2.update();

        // Mecanum wheel drive
        robot.calculateDrivePowers(
                controller2.left_stick_x,
                controller2.left_stick_y,
                controller2.right_stick_x
        );
        robot.updateDrive();

        // Press right stick button to move indexer servo
        if (controller2.right_stick_button.equals("pressing")) {
            robot.launchRing();
        }

        // Press "x" to toggle the intake motor
        if (controller2.x.equals("pressing")) {
            robot.toggleIntake();
        }

        // Press left bumper to turn the arm
        if (controller2.left_bumper.equals("pressing")) {
            robot.turnArm();
        }

        // Press right bumper to toggle the claw
        if (controller2.right_bumper.equals("pressing")) {
            robot.toggleGrab();
        }

        // Diffy motors
        if (controller2.dpad_up.equals("pressing")) {
            robot.powerLauncher.leftPower += 0.05;
            robot.powerLauncher.sendPowers();
        }
        if (controller2.dpad_down.equals("pressing")) {
            robot.powerLauncher.leftPower -= 0.05;
            robot.powerLauncher.sendPowers();
        }
        if (controller2.y.equals("pressing")) {
            robot.powerLauncher.rightPower += 0.05;
            robot.powerLauncher.sendPowers();
        }
        if (controller2.a.equals("pressing")) {
            robot.powerLauncher.rightPower -= 0.05;
            robot.powerLauncher.sendPowers();
        }

        // Press "b" to toggle the diffy motors on/off
        if (controller2.b.equals("pressing")) {
            robot.powerLauncher.toggle();
        }



    }

    //Code to run ONCE after the driver hits STOP
    @Override
    public void stop(){}
}