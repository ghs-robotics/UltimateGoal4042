package org.firstinspires.ftc.teamcode.data;

// Contains the coordinates (x position, tower width) for useful locations on the field
public interface FieldPositions {

    // Generally useful positions
    int[] PERFECT_LAUNCH_POS = new int[]{140, 58}; // Perfect launch position
    int[] NEXT_TO_STARTER_STACK_POS = new int[]{83, 61}; // Where to go to avoid first starter stack
    int[] PARK_POS = new int[]{132, 74}; // Where to park during autonomous
    int[] LEFT_POWERSHOT_POS = new int[]{83, 58}; // TODO : CALIBRATE

    // Position pertaining to the wobble goals
    int[] CONFIG_0_POS_I = new int[]{194, 90}; // Where to drop first wobble goal
    int[] CONFIG_0_POS_II = new int[]{166, 90}; // Where to drop second wobble goal

    int[] PRE_CONFIG_1_POS_I = new int[]{93, 85};
    int[] PRE_CONFIG_1_POS_II = new int[]{66, 85};
    int CONFIG_1_WALL_HEIGHT = 24;

    int[] PRE_CONFIG_4_POS_I = PRE_CONFIG_1_POS_I;
    int[] PRE_CONFIG_4_POS_II = PRE_CONFIG_1_POS_II;
    int CONFIG_4_WALL_HEIGHT = 24; // TODO : ROTATE

    int[] SECOND_WOBBLE_POS = new int[]{154, 45}; // Where to go before moving forward and grabbing 2nd wobble
    int SECOND_WOBBLE_WALL_HEIGHT = 75;
}
