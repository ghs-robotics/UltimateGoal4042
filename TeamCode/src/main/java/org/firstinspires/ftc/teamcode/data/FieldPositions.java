package org.firstinspires.ftc.teamcode.data;

// Contains the coordinates (x position + tower width) for useful locations on the field
public interface FieldPositions {

    int xOffset = -2;

    // Generally useful positions
    int[] PERFECT_LAUNCH_POS = new int[]{136 + xOffset, 58}; // Perfect launch position
    int[] NEXT_TO_STARTER_STACK_POS = new int[]{94 + xOffset, 67}; // Where to go to avoid first starter stack
    int[] LEFT_POWERSHOT_POS = new int[]{94 + xOffset, 58};

    // Park positions (where to park during autonomous)
    int[] PARK_0_POS = new int[]{96 + xOffset, 74};
    int[] PARK_1_POS = new int[]{80 + xOffset, 74};
    int[] PARK_4_POS = new int[]{96 + xOffset, 74};

    // Position pertaining to the wobble goals
    int[] CONFIG_0_POS_I = new int[]{180 + xOffset, 90}; // Where to drop first wobble goal
    int[] CONFIG_0_POS_II = new int[]{156 + xOffset, 90}; // Where to drop second wobble goal

    int[] PRE_CONFIG_1_POS_I = new int[]{93 + xOffset, 85};
    int[] PRE_CONFIG_1_POS_II = new int[]{66 + xOffset, 85};
    int CONFIG_1_WALL_HEIGHT = 24;

    int[] PRE_CONFIG_4_POS_I = new int[]{180 + xOffset, 90};
    int[] PRE_CONFIG_4_POS_II = new int[]{157 + xOffset, 90};
    int CONFIG_4_WALL_HEIGHT = 24;

    int[] SECOND_WOBBLE_POS = new int[]{135 + xOffset, 45}; // Where to go before moving forward and grabbing 2nd wobble
    int SECOND_WOBBLE_WALL_HEIGHT = 75;
}
