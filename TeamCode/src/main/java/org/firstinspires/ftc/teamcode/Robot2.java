package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

 class Robot2 {
     double shooterPower1;
     double shooterPower2;

     DcMotor shooterMotor1;
     DcMotor shooterMotor2;

     ElapsedTime elapsedTime;
     Gyro gyro;
     Telemetry telemetry;

     Robot2(HardwareMap hardwareMap, Telemetry telemetry) {
         shooterMotor1 = hardwareMap.get(DcMotor.class, "shooterMotor1");
         shooterMotor2 = hardwareMap.get(DcMotor.class, "shooterMotor2");

         shooterMotor1.setDirection(DcMotor.Direction.FORWARD);
         shooterMotor2.setDirection(DcMotor.Direction.FORWARD);

         gyro = new Gyro(hardwareMap);
         gyro.resetAngle();
         elapsedTime = new ElapsedTime();
         elapsedTime.reset();
         this.telemetry = telemetry;
     }

     void resetShooterMotors() {
         shooterMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         shooterMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

         shooterMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         shooterMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
     }

}
