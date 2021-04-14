package org.firstinspires.ftc.teamcode.data;

// Contains the coordinates (x position, tower width) for useful locations on the field
public interface FieldPositions {

    // Generally useful positions
    int[] PERFECT_LAUNCH_POS = new int[]{140, 58}; // Perfect launch position
    int[] NEXT_TO_STARTER_STACK_POS = new int[]{190, 60}; // Where to go to avoid first starter stack
    int[] PARK_POS = new int[]{134, 73}; // Where to park during autonomous
    int[] LEFT_POWERSHOT_POS = new int[]{180, 58}; // TODO : CALIBRATE

    // Position pertaining to the wobble goals
    int[] CONFIG_0_POS_I = new int[]{100, 80}; // Where to drop first wobble goal
    int[] CONFIG_0_POS_II = new int[]{110, 80}; // Where to drop second wobble goal

    int[] PRE_CONFIG_1_POS_I = new int[]{186, 110};
    int[] PRE_CONFIG_1_POS_II = new int[]{196, 110};
    int CONFIG_1_WALL_HEIGHT = 28;

    int[] PRE_CONFIG_4_POS_I = new int[]{100, 110};
    int[] PRE_CONFIG_4_POS_II = new int[]{90, 110};
    int CONFIG_4_WALL_HEIGHT = 26;

    int[] SECOND_WOBBLE_POS = new int[]{154, 45}; // Where to go before moving forward and grabbing 2nd wobble
    int SECOND_WOBBLE_WALL_HEIGHT = 75;
}
