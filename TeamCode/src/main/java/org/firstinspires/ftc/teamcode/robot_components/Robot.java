package org.firstinspires.ftc.teamcode.robot_components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.data.FieldPositions;
import org.firstinspires.ftc.teamcode.data.HSVConstants;

public class Robot extends DriveBase implements HSVConstants, FieldPositions {

    // Robot variables and objects
    protected double intakePower = 0;
    public double armAngle = 0.15; // init position
    public double clawAngle = 0.80; // Closed position

    public DcMotor intakeMotor;
    public Servo armServo;
    public Servo clawServo;

    public PowerLauncher powerLauncher;

    // Constructs a robot with the mechanical functions specific to this year's competition
    public Robot(HardwareMap hardwareMap, Telemetry telemetry) {
        super(hardwareMap, telemetry);

        // These are the names to use in the phone config (in quotes below)
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        armServo = hardwareMap.get(Servo.class, "armServo");
        clawServo = hardwareMap.get(Servo.class, "clawServo");

        // Initializes some other useful tools/objects for our robot
        powerLauncher = new PowerLauncher(hardwareMap);
    }

    // Launches a ring by moving the shooterServo
    public void indexRings(int rings) {
        for (int i = 0; i < rings; i++) {
            powerLauncher.index(0.4);
            wait(0.3);
        }
    }

    // Sets servos to starting positions
    public void resetServos() {
        clawServo.setPosition(clawAngle);
        wait(0.6);
        armServo.setPosition(armAngle);
        powerLauncher.resetServos();
    }

    // Run intake with specified power; negative values make intake run backward
    public void runIntake(double power) {
        intakePower = power;
        intakeMotor.setPower(intakePower);
    }

    // Turn arm to specified position
    private void setArmAngle(double armAngle) {
        this.armAngle = armAngle;
        armServo.setPosition(armAngle);
    }

    // Toggles the wobble gripper/claw
    public void toggleClaw() {
        // Default angle is 0.85 (which means the gripper is closed)
        clawAngle = (clawAngle != 0.80 ? 0.80 : 0.35);
        clawServo.setPosition(clawAngle);
    }

    // Turns the arm
    public void turnArm() {
        // Default angle is 0.42 (which is the up position)
        setArmAngle(armAngle != 0.42 ? 0.42 : 0.88);
    }

    // Good position for grabbing a wobble goal
    public void turnArmDownFull() {
        setArmAngle(0.88);
    }

    // Good position for placing a wobble goal during auto
    public void turnArmDownSlight() {
        setArmAngle(0.80);
    }

    // Good position for holding the wobble goal right above ground
    public void turnArmDownDrag() {
        setArmAngle(0.75);
    }

    // For holding the wobble above the wall height
    public void turnArmUpFull() {
        setArmAngle(0.42);
    }

}
