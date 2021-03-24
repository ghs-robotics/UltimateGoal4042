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

@TeleOp(name="Tele1", group="Iterative Opmode")
public class Tele1 extends OpMode
{
    //Declare OpMode members
    Robot robot;
    Controller controller1;
    Controller controller2;

    //Code to run ONCE when the driver hits INIT
    @Override
    public void init() {
        robot = new Robot(hardwareMap, telemetry);
        controller1 = new Controller(gamepad1);
        controller2 = new Controller(gamepad2);
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
        //Registers controller input
        controller1.update();
        controller2.update();

        //Press "x" to toggle speed between 100% and 30%
        if (controller1.x.equals("pressing")) {
            robot.toggleSpeed();
        }

        //Mecanum wheel drive
        robot.calculateDrivePowers(
                controller1.left_stick_x,
                controller1.left_stick_y,
                controller1.right_stick_x
        );
        robot.updateDrive();

        //Press left bumper to turn on/off the shooter motor
        if (controller2.right_bumper.equals("pressing")) {
            robot.toggleShooter();
            robot.wait(0.5);
            robot.launchRing();
        }

        //Press "y" to turn on/off the intake motor
        if (controller1.y.equals("pressing")) {
            robot.toggleIntake();
        }

        //Press "b" to toggle the wobble gripper, could group together toggle wobble and turn arm?
        if (controller2.b.equals("pressing")) {
            robot.toggleGrab();
        }

        //Press "a" to turn the arm,
        if (controller2.a.equals("pressing")) {
            robot.turnArm();
        }

        if (controller1.dpad_up.equals("pressing")) {
            robot.diffy.leftDiffyPower += 0.05;
            robot.diffy.rightDiffyPower += 0.05;
            robot.diffy.sendPowers();
        }
        if (controller1.dpad_down.equals("pressing")) {
            robot.diffy.leftDiffyPower -= 0.05;
            robot.diffy.rightDiffyPower -= 0.05;
            robot.diffy.sendPowers();
        }
        //adjusts diffy angle
        if (controller2.dpad_up.equals("pressing")) {
            robot.diffy.increaseIncline();
        }
        if (controller2.dpad_down.equals("pressing")) {
            robot.diffy.decreaseIncline();
        }
    }

    //Code to run ONCE after the driver hits STOP
    @Override
    public void stop(){}
}