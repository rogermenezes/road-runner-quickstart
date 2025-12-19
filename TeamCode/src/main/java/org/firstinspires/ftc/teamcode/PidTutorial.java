package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@TeleOp(name = "pidTutorial")
public class PidTutorial extends LinearOpMode {

    private DcMotorEx turret1;
    private DcMotorEx turret2; // unused for PID, but left intact

    // PIDF constants visible & tunable in FTC Dashboard
    public static double Kp = 0.0008;
    public static double Ki = 0.0;
    public static double Kd = 0.0;
    public static double Kf = 0.00046;

    // Target ticks/sec for your shooter motor
    public static double TARGET_VELOCITY = 5000;

    // Anti-windup clamp
    public static double INTEGRAL_MAX = 0.5;

    public static double expectedPower = 0.85;

    private double integralSum = 0;
    private double lastError = 0;

    public static double LOOP_PERIOD = 0.05;

    ElapsedTime timer = new ElapsedTime();
    private FtcDashboard dashboard;



    @Override
    public void runOpMode() throws InterruptedException {
//        turret1 = hardwareMap.get(DcMotorEx.class, "left_front_drive");
//        turret2 = hardwareMap.get(DcMotorEx.class, "right_front_drive");
//        turret1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        turret1 = hardwareMap.get(DcMotorEx.class, "turret1");
        turret1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret2 = hardwareMap.get(DcMotorEx.class, "turret2");
        turret2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        dashboard = FtcDashboard.getInstance();

        // Combine DS telemetry + Dashboard telemetry
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        waitForStart();
        timer.reset();

//        while (opModeIsActive()) {
//            turret1.setPower(1);
//            turret2.setPower(-1);
//
//            double currentVelocity = turret1.getVelocity();
//            double currentVelocity2 = turret2.getVelocity();
//            telemetry.addData("velocity turret1", currentVelocity);
//            telemetry.addData("velocity turret2", currentVelocity2);
//            telemetry.update();
//        }

        while (opModeIsActive()) {



            pid_speed_motor();
//            double dt = timer.seconds();
//            if (dt >= LOOP_PERIOD) {
//                double currentVelocity = turret1.getVelocity();
//
//                double power = estimatePIDControl(TARGET_VELOCITY, currentVelocity);
//                turret1.setPower(power);
//
//                telemetry.addData("Error", TARGET_VELOCITY - currentVelocity);
//                telemetry.update();
//
//                TelemetryPacket packet = new TelemetryPacket();
//                packet.put("velocity", currentVelocity);
//                packet.put("target", TARGET_VELOCITY);
//                packet.put("power", power);
//                dashboard.sendTelemetryPacket(packet);
//
//                timer.reset();
//            }

        }
    }

    public double estimatePIDControl(double reference, double state) {
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

    private void pid_speed_motor() {
        double TARGET_VELOCITY = expectedPower * 2200;
        double dt = timer.seconds();
        if (dt >= LOOP_PERIOD) {
            double currentVelocity = turret1.getVelocity();
            double currentVelocity2 = turret2.getVelocity();
            double power = estimatePIDControl(TARGET_VELOCITY, currentVelocity);
            //double power2 = estimatePIDControl(TARGET_VELOCITY, -currentVelocity2);
            turret1.setPower(power);
            turret2.setPower(-power);

            TelemetryPacket packet = new TelemetryPacket();
            packet.put("velocity turret1", currentVelocity);
            packet.put("velocity turret2", currentVelocity2);
            packet.put("target", TARGET_VELOCITY);
            packet.put("power turret1", power);
            //packet.put("power turret2", power2);
            dashboard.sendTelemetryPacket(packet);

            timer.reset();
        }
    }


}
