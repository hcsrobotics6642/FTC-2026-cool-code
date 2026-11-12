package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous(name = "Auto Red - AAA", group = "Robot") // :o
public class AutoRed extends LinearOpMode {

    private RobotHardware robot;

    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightBack = null;

    private ElapsedTime runtime = new ElapsedTime();

    private IMU imu;

    public void stopMotors() {
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
    }

    public void driveForward(double power, double seconds) {
        runtime.reset();

        while (opModeIsActive() && (runtime.seconds() < seconds)) {
            leftFront.setPower(power);
            rightFront.setPower(power);
            leftBack.setPower(power);
            rightBack.setPower(power);
        }

        stopMotors();
    }

    public void driveStraightTime(double power, double seconds) {
        runtime.reset();
        double targetYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        while (opModeIsActive() && (runtime.seconds() < seconds)) {
            double yaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            double correction = Range.clip((targetYaw - yaw) * 0.02, -0.3, 0.3);

            leftFront.setPower(power + correction);
            leftBack.setPower(power + correction);
            rightFront.setPower(power - correction);
            rightBack.setPower(power - correction);
        }

        stopMotors();
    }

    // Define your encoder constant at the top of your OpMode
    static final double TICKS_PER_UNIT = 408.98;

    public void driveStraightDistance(double power, double targetDistance) {
        double targetTicks = targetDistance * TICKS_PER_UNIT;

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double targetYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        while (opModeIsActive()) {
            double currentAverageTicks = (
                    Math.abs(leftFront.getCurrentPosition()) +
                    Math.abs(leftBack.getCurrentPosition()) +
                    Math.abs(rightFront.getCurrentPosition()) +
                    Math.abs(rightBack.getCurrentPosition())
            ) / 4.0;

            if (currentAverageTicks >= Math.abs(targetTicks)) {
                break;
            }

            double yaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            double correction = Range.clip((targetYaw - yaw) * 0.02, -0.3, 0.3);

            leftFront.setPower(power + correction);
            leftBack.setPower(power + correction);
            rightFront.setPower(power - correction);
            rightBack.setPower(power - correction);
        }

        stopMotors();
    }

    public void turn(double power, double targetYaw) {
        double error = AngleUnit.normalizeDegrees(targetYaw - imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));

        double kp = 0.02;
        double ki = 0.003;
        double kd = 0.005;

        double integral = 0;
        double lastError = error;
        ElapsedTime timer = new ElapsedTime();

        while (opModeIsActive() && Math.abs(error) > 1.0) {
            double dt = timer.seconds();
            timer.reset();

            integral += error * dt;
            integral = Range.clip(integral, -0.5, 0.5);
            double derivative = (error - lastError) / dt;
            lastError = error;

            double steer = Range.clip(error * kp + integral * ki + derivative * kd, -power, power);

            leftFront.setPower(steer);
            leftBack.setPower(steer);
            rightFront.setPower(-steer);
            rightBack.setPower(-steer);

            error = AngleUnit.normalizeDegrees(targetYaw - imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        }

        stopMotors();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        leftFront  = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightFront = hardwareMap.get(DcMotor.class, "right_front_drive");
        leftBack   = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightBack  = hardwareMap.get(DcMotor.class, "right_back_drive");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot.LogoFacingDirection logo = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usb = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot(logo, usb);

        imu.initialize(new IMU.Parameters(orientation));
        imu.resetYaw();

        waitForStart();
        if (isStopRequested()) return;

        driveStraightTime(0.2, 3);
        turn(0.3, 90);
    }
} // EOF