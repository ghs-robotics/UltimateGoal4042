package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

/**
 * This will be the main class that we will put all of our autonomous code into
 * This class is obviously not complete, and the structure of the code is not decided yet
 */
@Autonomous
public class Auto1 extends LinearOpMode {
    //Declare OpMode members
    Robot robot;

    @Override
    public void runOpMode() {
        robot = new Robot(hardwareMap, telemetry);
        robot.init();
        telemetry.addData("Status", "Initialized");
        telemetry.update();


        waitForStart();

        robot.identifyRingConfig();

        while(opModeIsActive()){
            robot.identifyRingConfig();
            robot.updateDrive();
        }

        if (opModeIsActive()) {
         /*   //Determine how many rings in the starting ring stacks
            //Move forward 6-7 feet until at the edge of launch zone
                moveRobot(position1);
            //Move sideways until in line with tower goal
            //aim the robot at the goal and make sure that the robot is within the launch zone
            //Shoot 3 rings
                robot.chaseTower2();
                for(int i = 0; i < 3; i++){
                    robot.launchRing();
                }
            //move forward towards the desired target wobble goal zone
            //Check distance to tower goal and correct if necessary
            //Move left or right depending on target wobble goal
                if(robot.config == 0){
                    moveRobot(positionA);
                } else if (robot.config == 1){
                    moveRobot(positionB);
                } else {
                    moveRobot(positionC);
                }
                robot.toggleGrab();
            //once there, place down the wobble goal
            //Head back to location where we shot the rings
            // if starterStack != 0, pickup the starter stack rings
                robot.toggleIntake();
                moveRobot(position2);
                robot.toggleIntake();
            //Move left or right and then backward towards second wobble goal
            //pick up second wobble goal
                moveRobot(position3);
                moveRobot(position4);
                robot.chaseWobble();
                robot.toggleGrab();
                moveRobot(position1);
                if(robot.config != 0){
                    robot.chaseTower2();
                    if(robot.config == 4){
                        for(int i = 0; i < 2; i++){
                            robot.launchRing();
                        }
                    }
                    robot.launchRing();
                }
            if(robot.config == 0){
                moveRobot(positionA);
            } else if (robot.config == 1){
                moveRobot(positionB);
            } else {
                moveRobot(positionC);
            }
            robot.toggleGrab();
            moveRobot(position5);
            //Check that we're in shooting position
            //Shoot the 3 rings
                //(This is repeating the code from dropping off the first wobble goal)
                //Move forward until at the edge of launch zone
                //Move sideways until in line with tower goal
                //move forward towards the desired target wobble goal zone
                //Check distance to tower goal and correct if necessary
                //Move left or right depending on target wobble goal
                //once there, place down the wobble goal
                //Head back to location where we shot the rings
            //Move forward to park over launch line */

            telemetry.addData("opmode is working","");
            telemetry.update();
        }
    }
}
