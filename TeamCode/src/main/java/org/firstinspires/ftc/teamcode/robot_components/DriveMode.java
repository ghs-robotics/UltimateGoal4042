package org.firstinspires.ftc.teamcode.robot_components;

import com.qualcomm.robotcore.util.ElapsedTime;

// For use in TeleOp
public class DriveMode {

    public static boolean smooth = false;
    public static boolean running = false;
    public static double factor = 1.0;
    private static ElapsedTime elapsedTime = new ElapsedTime();
    private static Controller c;

    public static void setController(Controller controller) {
        c = controller;
    }

    public static void switchMode() {
        smooth = !smooth;
    }

    public static double getFactor() {
        if (!smooth) {
            return 1.0;
        }
        return factor;
    }

    public static void update() {
        if (c.left_stick_x != 0 || c.left_stick_y != 0 || c.right_stick_x != 0) { // Any drive motor running
            if (!running) {
                running = true;
                elapsedTime.reset();
            } else {
                factor = Math.min(1.0, elapsedTime.seconds());
            }
        } else { // No drive motors running
            running = false;
        }
    }
}
