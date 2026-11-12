package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;

// robot data, mysterious name, huh?
public class RobotData {
        String alliance = "None"; // defines the global Alliance variable
        static Double intakeRPM = 0.3;

        private static Gamepad gp1;

        // --------------------+--------------------

        public static void init(Gamepad gamepad1) { gp1 = gamepad1; }

        public static boolean resetIMUKey(){ return gp1 != null && gp1.a; }
        public static boolean activateIntakeKey(){ return gp1 != null && gp1.b; }
        public static boolean robotRelativeKey() { return gp1 != null && gp1.left_bumper; }
} // w
