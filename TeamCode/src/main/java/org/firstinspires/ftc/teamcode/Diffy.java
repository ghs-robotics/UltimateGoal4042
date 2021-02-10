package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

class Diffy {
    double leftFrontPower = 0;
    double rightFrontPower = 0;
    double leftRearPower = 0;
    double rightRearPower = 0;
    double intakePower = 0;
    double armAngle = 0.5;
    double grabAngle = 0.2;
    double shooterAngle = 0.05;
    double speed = 1;
    double previousShooterMotorTicks = 0;
    double DeltaShooterMotorTicks = 0;
    double CurrentElapsedTime = 0;
    double TargetMotorSpeed = 1600;
    public static boolean ShooterMotorPowered = false;
    double shooterPower1 = 0;
    double shooterPower2 = 0;
    double k = 1;
    //k is a constant for smth ask imants abt diffy

    DcMotor shooterMotor1;
    DcMotor shooterMotor2;
    DcMotor leftFrontDrive;
    DcMotor rightFrontDrive;
    DcMotor leftRearDrive;
    DcMotor rightRearDrive;
    DcMotor intakeMotor;
    Servo armServo;
    Servo grabServo;
    Servo shooterServo;

    ElapsedTime elapsedTime;
    Gyro gyro;
    Telemetry telemetry;

    //Creates a robot object with methods that we can use in both Auto and TeleOp
    Diffy(HardwareMap hardwareMap, Telemetry telemetry) {

        //These are the names to use in the phone config (in quotes below)
        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFrontDrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFrontDrive");
        leftRearDrive = hardwareMap.get(DcMotor.class, "leftRearDrive");
        rightRearDrive = hardwareMap.get(DcMotor.class, "rightRearDrive");
        shooterMotor1 = hardwareMap.get(DcMotor.class, "shooterMotor1");
        shooterMotor2 = hardwareMap.get(DcMotor.class, "shooterMotor2");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        armServo = hardwareMap.get(Servo.class, "armServo");
        grabServo = hardwareMap.get(Servo.class, "grabServo");
        shooterServo = hardwareMap.get(Servo.class, "shooterServo");

        //Defines the forward direction for each of our motors/servos
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftRearDrive.setDirection(DcMotor.Direction.FORWARD);
        rightRearDrive.setDirection(DcMotor.Direction.REVERSE);
        shooterMotor1.setDirection(DcMotor.Direction.FORWARD);
        shooterMotor2.setDirection(DcMotor.Direction.FORWARD);
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

/*    //In case we actually need to use encoders on the drive motors
    void resetDrive() {
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFrontPower = 0;
        rightFrontPower = 0;
        leftRearPower = 0;
        rightRearPower = 0;
        updateDrive();
    } */

    //Sets servos to starting positions
    void resetServos(){
        armServo.setPosition(armAngle);
        grabServo.setPosition(grabAngle);
        shooterServo.setPosition(shooterAngle);
    }
    void resetShooterMotors() {
        shooterMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooterMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
        telemetry.addData("shooterPower1", "" + shooterPower1);
        telemetry.addData("shooterPower2", "" + shooterPower2);
        telemetry.addData("armServo", "" + armAngle);
        telemetry.addData("grabServo", "" + grabAngle);
        telemetry.addData("shooterAngle", "" + shooterAngle);
        //telemetry.addData("shooterVelocity", "" + findShooterVelocity());//
        telemetry.addData("shooterMotorPosition1", "" + shooterMotor1.getCurrentPosition());
        telemetry.addData("shooterMotorPosition2", "" + shooterMotor2.getCurrentPosition());
        telemetry.update();

        //Sends desired power to drive motors
        leftFrontDrive.setPower(LF);
        rightFrontDrive.setPower(RF);
        leftRearDrive.setPower(LR);
        rightRearDrive.setPower(RR);
    }

    //Sets the drive speed to 30%
    void toggleSpeed() { speed = (speed == 1 ? 0.5 : 1); }

    //fires ring
    void toggleShooter() {
        elapsedTime.reset();
        shooterPower1 = (shooterPower1 == 0 ? 1.0 : 0);
        shooterPower2 = (shooterPower2 == 0 ? 1.0 : 0);
        ShooterMotorPowered = !ShooterMotorPowered;
        shooterMotor1.setPower(shooterPower1);
        shooterMotor2.setPower(shooterPower2);
    }

    //Turns the intake motor on or off
    void toggleIntake() {
        intakePower = (intakePower == 0 ? 0.9 : 0);
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

    //Launches a ring by moving the shooterServo pushes ring into shooter thing
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

    void increaseIncline(){
        shooterPower1 = 0.5;
        shooterPower2 = 1.0;
        shooterMotor1.setPower(shooterPower1);
        shooterMotor2.setPower(shooterPower2);
    }

    void decreaseIncline(){
        shooterPower1 = 1.0;
        shooterPower2 = 0.5;
        shooterMotor1.setPower(shooterPower1);
        shooterMotor2.setPower(shooterPower2);
    }

   /* //Calculates the speed of the shooter motor in ticks per second
    double findShooterVelocity(){
        //Finds the number of ticks since the last time we ran the function
        DeltaShooterMotorTicks = (shooterMotor1.getCurrentPosition() - previousShooterMotorTicks);
        previousShooterMotorTicks = shooterMotor1.getCurrentPosition();
        CurrentElapsedTime = elapsedTime.seconds();
        elapsedTime.reset();
        return (DeltaShooterMotorTicks / CurrentElapsedTime);
    }

    void adjustShooterVelocity(){
        if (findShooterVelocity() > TargetMotorSpeed){
            shooterPower -= 0.001;
        } else {
            shooterPower += 0.001;
        }
        shooterPower = Range.clip(shooterPower, 0, 1.0);
        shooterMotor.setPower(shooterPower);
    }
    /*

    */
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
//Velocity== DeltaShooterMotorTicks/Elapsed time when the motor is running
//https://ftctechnh.github.io/ftc_app/doc/javadoc/index.html