package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Diffy {
    public static boolean shooterMotorPowered = false;
    double leftDiffyPower = 0;
    double rightDiffyPower = 0;
    double k = 1; //k is a constant for smth ask imants abt diffy

    DcMotor leftDiffyMotor;
    DcMotor rightDiffyMotor;

    ElapsedTime elapsedTime;
    Gyro gyro;
    Telemetry telemetry;

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

    void setIncline(double newIncline) {
        if (!ShooterMotorPowered) {
            while (incline != newIncline) {
                if (incline < newIncline) {
                    increaseIncline();
                    wait(0.1);
                } else {
                    decreaseIncline();
                    wait(0.1);
                }
            }
        }
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
    // Calculates shooter motor speed in ticks per second
    private double findShooterVelocity() {
        double deltaTicks = (shooterMotor.getCurrentPosition() - previousShooterMotorTicks);
        double deltaTime = elapsedTime.seconds() - previousElapsedTime;
        previousShooterMotorTicks = shooterMotor.getCurrentPosition();
        previousElapsedTime = elapsedTime.seconds();
        return (deltaTicks / deltaTime);
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
     */
}