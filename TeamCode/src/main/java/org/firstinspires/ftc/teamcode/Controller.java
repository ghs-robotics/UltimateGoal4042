package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;

class Controller {
    Gamepad gamepad;

    String a;
    String b;
    String x;
    String y;
    String dpad_right;
    String dpad_up;
    String dpad_left;
    String dpad_down;
    String back;
    String guide;
    String start;
    String left_stick_button;
    String right_stick_button;
    String left_bumper;
    String right_bumper;

    double left_stick_x;
    double left_stick_y;
    double right_stick_x;
    double right_stick_y;
    double left_trigger;
    double right_trigger;

    Controller(Gamepad gamepad) {
        this.gamepad = gamepad;

        a = "released";
        b = "released";
        x = "released";
        y = "released";
        dpad_right = "released";
        dpad_up = "released";
        dpad_left = "released";
        dpad_down = "released";
        back = "released";
        guide = "released";
        start = "released";
        left_stick_button = "released";
        right_stick_button = "released";
        left_bumper = "released";
        right_bumper = "released";

        left_stick_x = 0;
        left_stick_y = 0;
        right_stick_x = 0;
        right_stick_y = 0;
        left_trigger = 0;
        right_trigger = 0;
    }

    void update() {
        a = check(a, gamepad.a);
        b = check(b, gamepad.b);
        x = check(x, gamepad.x);
        y = check(y, gamepad.y);

        dpad_right = check(dpad_right, gamepad.dpad_right);
        dpad_up    = check(dpad_up,    gamepad.dpad_up);
        dpad_left  = check(dpad_left,  gamepad.dpad_left);
        dpad_down  = check(dpad_down,  gamepad.dpad_down);

        back  = check(back,  gamepad.back);
        guide = check(guide, gamepad.guide);
        start = check(start, gamepad.start);

        left_stick_button  = check(left_stick_button,  gamepad.left_stick_button);
        right_stick_button = check(right_stick_button, gamepad.right_stick_button);

        left_bumper  = check(left_bumper,  gamepad.left_bumper);
        right_bumper = check(right_bumper, gamepad.right_bumper);

        left_stick_x = gamepad.left_stick_x;
        left_stick_y = -gamepad.left_stick_y;

        right_stick_x = gamepad.right_stick_x;
        right_stick_y = -gamepad.right_stick_y;

        left_trigger  = gamepad.left_trigger;
        right_trigger = gamepad.right_trigger;
    }

    String check(String previous, Boolean current) {
        String state;
        if (current) {
            if (previous.equals("released")) {
                state = "pressing";
            } else {
                state = "pressed";
            }
        } else {
            if (previous.equals("pressed")) {
                state = "releasing";
            } else {
                state = "released";
            }
        }
        return state;
    }
}
