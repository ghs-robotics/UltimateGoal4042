package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

class Robot {
    double leftFrontPower = 0;
    double rightFrontPower = 0;
    double leftRearPower = 0;
    double rightRearPower = 0;
    double shooterPower = 0;
    double intakePower = 0;
    double armAngle = 0.5;
    double grabAngle = 0.2;
    double shooterAngle = 0.05;
    double speed = 1;

    double previousShooterMotorTicks = 0;
    double previousElapsedTime = 0;

    DcMotor leftFrontDrive;
    DcMotor rightFrontDrive;
    DcMotor leftRearDrive;
    DcMotor rightRearDrive;
    DcMotor shooterMotor;
    DcMotor intakeMotor;
    Servo armServo;
    Servo grabServo;
    Servo shooterServo;

    ElapsedTime elapsedTime;
    Gyro gyro;
    Telemetry telemetry;

    //Creates a robot object with methods that we can use in both Auto and TeleOp
    Robot(HardwareMap hardwareMap, Telemetry telemetry) {

        //These are the names to use in the phone config (in quotes below)
        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFrontDrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFrontDrive");
        leftRearDrive = hardwareMap.get(DcMotor.class, "leftRearDrive");
        rightRearDrive = hardwareMap.get(DcMotor.class, "rightRearDrive");
        shooterMotor = hardwareMap.get(DcMotor.class, "shooterMotor");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        armServo = hardwareMap.get(Servo.class, "armServo");
        grabServo = hardwareMap.get(Servo.class, "grabServo");
        shooterServo = hardwareMap.get(Servo.class, "shooterServo");

        //Defines the forward direction for each of our motors/servos
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftRearDrive.setDirection(DcMotor.Direction.FORWARD);
        rightRearDrive.setDirection(DcMotor.Direction.REVERSE);
        shooterMotor.setDirection(DcMotor.Direction.FORWARD);
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        armServo.setDirection(Servo.Direction.FORWARD);
        grabServo.setDirection(Servo.Direction.FORWARD);
        shooterServo.setDirection(Servo.Direction.FORWARD);

        //Declares some other useful tools for our robot (the gyroscope, the timer, etc.)
        gyro = new Gyro(hardwareMap);
        gyro.resetAngle();
        elapsedTime = new ElapsedTime();
        elapsedTime.reset();
        this.telemetry = telemetry;
    }

    //Sets servos to starting positions
    void resetServos(){
        armServo.setPosition(armAngle);
        grabServo.setPosition(grabAngle);
        shooterServo.setPosition(shooterAngle);
    }

    //Updates the powers being sent to the drive motors
    void updateDrive() {
        //Adjusts powers for speed
        double LF = speed * leftFrontPower;
        double RF = speed * rightFrontPower;
        double LR = speed * leftRearPower;
        double RR = speed * rightRearPower;

        //Displays motor powers on the phone
//        telemetry.addData("leftFrontPower", "" + LF);
//        telemetry.addData("rightFrontPower", "" + RF);
//        telemetry.addData("leftRearPower", "" + LR);
//        telemetry.addData("rightRearPower", "" + RR);
        telemetry.addData("shooterPower", "" + shooterPower);
        telemetry.addData("armServo", "" + armAngle);
        telemetry.addData("grabServo", "" + grabAngle);
        telemetry.addData("shooterAngle", "" + shooterAngle);
        telemetry.update();

        //Sends desired power to drive motors
        leftFrontDrive.setPower(LF);
        rightFrontDrive.setPower(RF);
        leftRearDrive.setPower(LR);
        rightRearDrive.setPower(RR);
    }
    void startMoving(double x, double y){
        startMoving(x,y,0,0,0,0,0,0);
    }

    void startMoving(double x, double y, int ringX, int ringY, int ringWidth, int ringHeight, int targetX, int targetY){
        double r = Math.hypot(x, y);
        double robotAngle = Math.atan2(y, x) - Math.PI / 4;
        double rotate = 0;
        leftFrontPower = r * Math.cos(robotAngle) + rotate;
        rightFrontPower = r * Math.sin(robotAngle) - rotate;
        leftRearPower = r * Math.sin(robotAngle) + rotate;
        rightRearPower = r * Math.cos(robotAngle) - rotate;

        //Adjusts powers for speed
        double LF = speed * leftFrontPower;
        double RF = speed * rightFrontPower;
        double LR = speed * leftRearPower;
        double RR = speed * rightRearPower;

        //Displays motor powers on the phone
//        telemetry.addData("leftFrontPower", "" + LF);
//        telemetry.addData("rightFrontPower", "" + RF);
//        telemetry.addData("leftRearPower", "" + LR);
//        telemetry.addData("rightRearPower", "" + RR);
        telemetry.addData("ringWidth, ringHeight", "( " + ringWidth + ", " + ringHeight + " )");
        telemetry.addData("ringX, targetX", "( " + ringX + ", " + targetX + " )");
        telemetry.addData("ringY, targetY", "( " + ringY + ", " + targetY + " )");
        telemetry.addData("x, y", "( " + x + ", " + y + " )");
        telemetry.update();

        //Sends desired power to drive motors
        leftFrontDrive.setPower(LF);
        rightFrontDrive.setPower(RF);
        leftRearDrive.setPower(LR);
        rightRearDrive.setPower(RR);
    }

    //Sets the drive speed to 30%
    void toggleSpeed() { speed = (speed == 1 ? 0.5 : 1); }

    //Turns the shooter motor on or off
    void toggleShooter() {
        shooterPower = (shooterPower == 0 ? 1.0 : 0);
        shooterMotor.setPower(shooterPower);
    }

    //Turns the intake motor on or off
    void toggleIntake() {
        intakePower = (intakePower == 0 ? 1 : 0);
        intakeMotor.setPower(intakePower);
    }

    //Turns the arm
    void turnArm() {
        armAngle = (armAngle == 0.88 ? 0.5 : 0.88);
        armServo.setPosition(armAngle);
    }

    //Toggles the wobble gripper
    void toggleGrab() {
        grabAngle = (grabAngle == 0.25 ? 0 : 0.25);
        grabServo.setPosition(grabAngle);
    }

    //Launches a ring by moving the shooterServo
    void launchRing() {
        shooterAngle = 0.25;
        shooterServo.setPosition(shooterAngle);
        wait(0.5);
        shooterAngle = 0.05;
        shooterServo.setPosition(shooterAngle);
    }

    void increaseArmAngle(){
        armAngle += 0.1;
        armServo.setPosition(armAngle);
    }

    void decreaseArmAngle(){
        armAngle -= 0.1;
        armServo.setPosition(armAngle);
    }

    //Calculates shooter motor speed in ticks per second
    double findShooterVelocity() {
        double deltaTicks = (shooterMotor.getCurrentPosition() - previousShooterMotorTicks);
        double deltaTime = elapsedTime.seconds() - previousElapsedTime;
        previousShooterMotorTicks = shooterMotor.getCurrentPosition();
        previousElapsedTime = elapsedTime.seconds();
        return (deltaTicks / deltaTime);
    }

    void pickUpWobbleGoal(double dX, double dY) {
        turnArm();
        toggleGrab();
        startMoving(-dX,0);
        wait(1.0);
        startMoving(0,dY);
        wait(1.0);
        toggleGrab();
        turnArm();
    }

    //Resets the timer
    void resetElapsedTime() { elapsedTime.reset(); }

    //Returns how many seconds have passed since the timer was last reset
    double getElapsedTimeSeconds() { return elapsedTime.seconds(); }

    //Makes the robot wait (i.e. do nothing) for a specified number of seconds
    void wait(double seconds) {
        double start = getElapsedTimeSeconds();
        while (getElapsedTimeSeconds() - start < seconds) {}
    }
}

//https://ftctechnh.github.io/ftc_app/doc/javadoc/index.html