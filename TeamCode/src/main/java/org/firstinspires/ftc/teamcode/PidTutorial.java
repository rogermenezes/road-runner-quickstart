package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@TeleOp(name = "pidTutorial")
public class PidTutorial extends LinearOpMode {

    private DcMotorEx turret1;
    private DcMotorEx turret2; // unused for PID, but left intact

    // PIDF constants visible & tunable in FTC Dashboard
    public static double Kp = 0.0005;
    public static double Ki = 0.00001;
    public static double Kd = 0.0001;
    public static double Kf = 0.0003;

    // Target ticks/sec for your shooter motor
    public static double TARGET_VELOCITY = 1000;

    // Anti-windup clamp
    public static double INTEGRAL_MAX = 1.0;

    private double integralSum = 0;
    private double lastError = 0;

    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        turret1 = hardwareMap.get(DcMotorEx.class, "left_front_drive");
        turret2 = hardwareMap.get(DcMotorEx.class, "right_front_drive");

        // Combine DS telemetry + Dashboard telemetry
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();
        timer.reset();

        while (opModeIsActive()) {
            double currentVelocity = turret1.getVelocity();

            double power = PIDControl(TARGET_VELOCITY, currentVelocity);
            turret1.setPower(power);

            telemetry.addData("Target Velocity", TARGET_VELOCITY);
            telemetry.addData("Velocity", currentVelocity);
            telemetry.addData("Power", power);
            telemetry.addData("Error", TARGET_VELOCITY - currentVelocity);
            telemetry.update();
        }
    }

    public double PIDControl(double reference, double state) {
        double dt = timer.seconds();
        if (dt == 0) dt = 1e-3;

        double error = reference - state;

        // Integral accumulation with clamp
        integralSum += error * dt;
        if (integralSum > INTEGRAL_MAX) integralSum = INTEGRAL_MAX;
        if (integralSum < -INTEGRAL_MAX) integralSum = -INTEGRAL_MAX;

        // Derivative term
        double derivative = (error - lastError) / dt;
        lastError = error;

        timer.reset();

        // PIDF output
        return (error * Kp) + (derivative * Kd) + (integralSum * Ki) + (reference * Kf);
    }
}
