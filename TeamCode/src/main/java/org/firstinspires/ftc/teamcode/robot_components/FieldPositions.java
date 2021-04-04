package org.firstinspires.ftc.teamcode.robot_components;

// Contains the coordinates (x position, tower width) for useful locations on the field
public interface FieldPositions {

    // Generally useful positions
    int[] PERFECT_LAUNCH_POS = new int[]{145, 58}; // Perfect launch position
    int[] NEXT_TO_STARTER_STACK_POS = new int[]{190, 60}; // Where to go to avoid first starter stack
    int[] PARK_POS = new int[]{134, 70}; // Where to park during autonomous

    // Position pertaining to the wobble goals
    int[] CONFIG_0_POS_I = new int[]{100, 80}; // Where to drop first wobble goal
    int[] CONFIG_0_POS_II = new int[]{110, 80}; // Where to drop second wobble goal

    int[] CONFIG_1_POS_I = new int[]{186, 80};
    int[] CONFIG_1_POS_II = new int[]{196, 80};

    int[] CONFIG_4_POS_I = new int[]{80, 110}; // Move forward one foot before dropping wobble goal!
    int[] CONFIG_4_POS_II = new int[]{90, 110}; // Move forward one foot before dropping wobble goal!

    int[] SECOND_WOBBLE_POS = new int[]{154, 45}; // Where to go before moving forward and grabbing 2nd wobble
}
