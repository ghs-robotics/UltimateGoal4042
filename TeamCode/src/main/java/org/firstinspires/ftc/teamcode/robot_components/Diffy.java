package org.firstinspires.ftc.teamcode.robot_components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Diffy {
    public static boolean shooterMotorPowered = false;
    public double leftDiffyPower = 0;
    public double rightDiffyPower = 0;
    public double k = 1; //k is a constant for smth ask imants abt diffy
    private long prevLeftPos = 0;
    private long prevRightPos = 0;
    private double prevLeftSeconds = 0;
    private double prevRightSeconds = 0;

    public DcMotor leftDiffyMotor;
    public DcMotor rightDiffyMotor;
    public DcMotor encoder;

    private ElapsedTime elapsedTime;

    public Diffy(HardwareMap hardwareMap) {
        leftDiffyMotor = hardwareMap.get(DcMotor.class, "leftDiffyMotor");
        rightDiffyMotor = hardwareMap.get(DcMotor.class, "rightDiffyMotor");

        //Defines the forward direction for diffy motors
        leftDiffyMotor.setDirection(DcMotor.Direction.FORWARD);
        rightDiffyMotor.setDirection(DcMotor.Direction.FORWARD);

        elapsedTime = new ElapsedTime();
        elapsedTime.reset();
    }

    public void sendPowers() { // TODO : Make this private
        leftDiffyMotor.setPower(leftDiffyPower);
        rightDiffyMotor.setPower(rightDiffyPower);
    }

    public void resetDiffyMotors() {
        leftDiffyMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDiffyMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDiffyMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDiffyMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // Fires ring
    public void toggleShooter(double power) {
        elapsedTime.reset(); // TODO : Ask Kenny why we need this line
        leftDiffyPower = power; //TODO: Change back to 1.0
        rightDiffyPower = power;
        shooterMotorPowered = !shooterMotorPowered;
        sendPowers();
    }

    public void increaseIncline(){
        leftDiffyPower = 0.5;
        rightDiffyPower = 1.0;
        sendPowers();
    }

    public void decreaseIncline(){
        leftDiffyPower = 1.0;
        rightDiffyPower = 0.5;
        sendPowers();
    }

    /*
    public double getPosition() {
        return encoder.getCurrentPosition();
    }
     */


    // Calculates left diffy motor speed in ticks per second
    private double getLeftVelocity() {
        long deltaTicks = (leftDiffyMotor.getCurrentPosition() - prevLeftPos);
        double deltaTime = elapsedTime.seconds() - prevLeftSeconds;
        prevLeftPos = leftDiffyMotor.getCurrentPosition();
        prevLeftSeconds = elapsedTime.seconds();
        return (deltaTicks / deltaTime);
    }

    // Calculates right diffy motor speed in ticks per second
    private double getRightVelocity() {
        long deltaTicks = (rightDiffyMotor.getCurrentPosition() - prevRightPos);
        double deltaTime = elapsedTime.seconds() - prevRightSeconds;
        prevRightPos = rightDiffyMotor.getCurrentPosition();
        prevRightSeconds = elapsedTime.seconds();
        return (deltaTicks / deltaTime);
    }

    /*
     void adjustShooterVelocity(){
         if (findShooterVelocity() > TargetMotorSpeed){
             shooterPower -= 0.001;
         } else {
             shooterPower += 0.001;
         }
         shooterPower = Range.clip(shooterPower, 0, 1.0);
         shooterMotor.setPower(shooterPower);
     }
     */
}