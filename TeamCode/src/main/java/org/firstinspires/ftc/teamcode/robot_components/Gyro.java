package org.firstinspires.ftc.teamcode.robot_components;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class Gyro {
    BNO055IMU gyro;
    Orientation lastAngles = new Orientation ();
    double globalAngle;

    // Constructs a gyro object which keeps track of the robot's angle
    public Gyro(HardwareMap hardwareMap) {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        gyro = hardwareMap.get(BNO055IMU.class, "imu");
        gyro.initialize(parameters);
    }

    // Sets angle to zero degrees; call at the begin of any opmode
    public void resetAngle () {
        lastAngles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        globalAngle = 0;
    }

    // Returns an angle between -90 and 270 degrees
    public double getAngle () {
        double angle = getOverallAngle();
        while (angle < -110 || angle > 290) {
            if (angle < -110) {
                angle += 360;
            } else {
                angle -= 360;
            }
        }
        return angle;
    }

    // Returns the overall angle in degrees (not taken mod 360)
    public double getOverallAngle() {
        Orientation angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;
        if (deltaAngle < -180) { deltaAngle += 360; }
        if (deltaAngle > 180) { deltaAngle -= 360; }
        globalAngle += deltaAngle;
        lastAngles = angles;
        return globalAngle;
    }
}
