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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Tele1", group="Iterative Opmode")
public class Tele1 extends OpMode {
    //Declare OpMode members
    Robot robot;
    Controller controller1;
    //Controller controller2;

    //Code to run ONCE when the driver hits INIT
    @Override
    public void init() {
        robot = new Robot(hardwareMap, telemetry);
        controller1 = new Controller(gamepad1);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }
    //kenny codes :) fksdhfkl
    //Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
    @Override
    public void init_loop() {
    }

    //Code to run ONCE when the driver hits PLAY
    @Override
    public void start() {
        robot.resetElapsedTime();
    }

    //Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
    @Override
    public void loop() {
        controller1.update();

        //PRESS THE "x" BUTTON ON THE CONTROLLER TO TOGGLE SPEED
        if (controller1.x.equals("pressing")) {
            robot.toggleSpeed();
        }

        //Mecanum wheel drive
        double r = Math.hypot(controller1.left_stick_x, controller1.left_stick_y);
        double robotAngle = Math.atan2(controller1.left_stick_y, controller1.left_stick_x) - Math.PI / 4;
        double rightX = controller1.right_stick_x;
        robot.leftFrontPower = r * Math.cos(robotAngle) + rightX;
        robot.rightFrontPower = r * Math.sin(robotAngle) - rightX;
        robot.leftRearPower = r * Math.sin(robotAngle) + rightX;
        robot.rightRearPower = r * Math.cos(robotAngle) - rightX;
        robot.updateDrive();

        //@Imants, PRESS THE "a" BUTTON ON THE CONTROLLER TO TOGGLE THE INTAKE MOTOR
        if (controller1.a.equals("pressing")) {
            robot.toggleIntake();
        }

        //grabby thing for wobble goals
        if (controller1.b.equals("pressing")){
            robot.toggleGrab();
        }
    }

    //Code to run ONCE after the driver hits STOP
    @Override
    public void stop () {
    }
}
