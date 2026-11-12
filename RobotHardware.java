package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.HashMap;

public class RobotHardware {

    private final LinearOpMode myOpMode;

    public static Double intakeSpeed = RobotData.intakeRPM;
    public static boolean intake_on = false;

    private static final HashMap<String, Double> colorMap = new HashMap<>();

    static {
        colorMap.put("off", 0.0);
        colorMap.put("white", 1.0);
        colorMap.put("red", 0.278);
        colorMap.put("orange", 0.333);
        colorMap.put("yellow", 0.385);
        colorMap.put("sage", 0.444);
        colorMap.put("green", 0.5);
        colorMap.put("azure", 0.555);
        colorMap.put("blue", 0.615);
        colorMap.put("indigo", 0.667);
        colorMap.put("violet", 0.722);
        colorMap.put("purple", 0.722);
    }

    public RobotHardware(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    public void init() {
        myOpMode.telemetry.addData(">", "Hardware Initialized");
        myOpMode.telemetry.update();
    }

    public void setLEDColor(Servo ledServo, String input) {
        try {
            if (input == null) {
                ledServo.setPosition(0.0);
                return;
            }

            String cleanedInput = input.toLowerCase().trim();

            if (colorMap.containsKey(cleanedInput)) {
                ledServo.setPosition(colorMap.get(cleanedInput));
            } else {
                ledServo.setPosition(Double.parseDouble(cleanedInput));
            }
        } catch (Exception e) {
            ledServo.setPosition(1.0);
        }
    }

    public void setLEDColor(Servo ledServo, Double input) {
        try {
            if (input == null) {
                ledServo.setPosition(0.0);
                return;
            }
            ledServo.setPosition(input);
        } catch (Exception e) {
            ledServo.setPosition(1.0);
        }
    }

    public static void activateIntake(DcMotor motor) {
        if (intakeSpeed != null) {
            motor.setPower(intakeSpeed);
        }
    }

    public static void deactivateIntake(DcMotor motor) {
        motor.setPower(0.0);
    }
}