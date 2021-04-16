package org.firstinspires.ftc.teamcode.cv_objects;

import org.firstinspires.ftc.teamcode.data.MyScalar;
import org.firstinspires.ftc.teamcode.robot_components.CVDetectionPipeline;
import org.firstinspires.ftc.teamcode.robot_components.PIDController;
import org.opencv.core.Mat;

public class TowerGoal extends CVObject {

    Mat dst1;
    Mat dst2;

    public TowerGoal(CVDetectionPipeline pipeline, PIDController xPID, PIDController wPID) {
        super("tower", pipeline, xPID, wPID);
        lowerHSV = LOWER_BLUE_TOWER_HSV;
        upperHSV = UPPER_BLUE_TOWER_HSV;
        cover = 0;
        dst1 = new Mat();
        dst2 = new Mat();
    }

    // Finds HSV values of a point
    public MyScalar findHSV(int row, int column) {
        // Extracting HSV value from center point of the input image
        dst1 = currentHSVMat.row(row);
        dst2 = dst1.col(column);
        String s = dst2.dump();

        int comma1 = 0;
        int comma2 = 0;

        for (int i = 0; i < s.length(); i++) {
            if (s.charAt(i) == ',') {
                if (comma1 == 0) {
                    comma1 = i;
                } else {
                    comma2 = i;
                }
            }
        }

        int val1 = Integer.parseInt(s.substring(1, comma1).trim());
        int val2 = Integer.parseInt(s.substring(comma1 + 1, comma2).trim());
        int val3 = Integer.parseInt(s.substring(comma2 + 1, s.length() - 1).trim());

        return new MyScalar(val1, val2, val3);
    }

    // Tests to make sure there is the goal slot
    private boolean hasHatShape() {
        int row = y + (int) (0.8 * h); // 80% of the way down from top of blue tower goal
        MyScalar scalar;

        // Left flap has to be blue
        scalar = findHSV(row, x + (int) (0.1 * w));
//        if (!scalar.inRange(LOWER_BLUE_TOWER_HSV, UPPER_BLUE_TOWER_HSV)) {
//            return false;
//        }
//
//        // Right flap has to be blue
//        scalar = CVDetectionPipeline.findHSVCrosshair(currentHSVMat, row, x + (int) (0.9 * w));
//        if (!scalar.inRange(LOWER_BLUE_TOWER_HSV, UPPER_BLUE_TOWER_HSV)) {
//            return false;
//        }
//
//        // Middle part shouldn't be blue (since that's the slot for the rings to enter the high goal)
//        scalar = CVDetectionPipeline.findHSVCrosshair(currentHSVMat, row, x + (int) (0.5 * w));
//        if (scalar.inRange(LOWER_BLUE_TOWER_HSV, UPPER_BLUE_TOWER_HSV)) {
//            return false;
//        }
        return true;
    }


    // Testing to make sure the detected object is the tower goal
    @Override
    protected boolean isReasonable(int x, int y, int w, int h) {
        double r = 1.0 * w / h; // ratio is usually about 1.5
        // width 34 is back of the field, closest is 150
        return (30 < w && w < 130 && 18 < h && h < 65 && r > 1.3);
    }
    /*
        Some values for reference:
        w       h
        34      22
        52      34
        77      49
        110     45 (top of goal is out of view)
     */

    @Override
    public void updateData() {
        super.updateData();
        if (!hasHatShape()) {
            setToNotIdentified();
        }
    }
}
