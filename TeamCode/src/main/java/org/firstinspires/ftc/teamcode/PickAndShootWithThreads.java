package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;


@Autonomous(name = "Auto: Pick Shoot With Threads", group = "A")
public class PickAndShootWithThreads extends LinearOpMode {

    // Set this to your actual starting pose on the field (units are inches/radians by default in quickstart)
    // Example: Facing down-field from the left tile on BLUE alliance
    private static final Pose2d START_POSE = new Pose2d(0, 0, 0);

    // helper to move "forward" in the current heading frame
    static Vector2d forward(Vector2d p, double headingRad, double inches) {
        double dx = inches * Math.cos(headingRad);
        double dy = inches * Math.sin(headingRad);
        return new Vector2d(p.x + dx, p.y + dy);
    }


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Intake intake = new Intake(hardwareMap);

        // Initialize drive at the known start pose (you already tuned drive params)
        Pose2d START_POSE = new Pose2d(new Vector2d(0, 0), Math.toRadians(0));

        MecanumDrive drive = new MecanumDrive(hardwareMap, START_POSE);

        // Track pose while we "plan" waypoints
        Vector2d p = START_POSE.position;
        double   h = START_POSE.heading.toDouble(); // radians

        // 1) Forward 42
        Vector2d p1 = forward(p, h, 30);

        // 2) Left turn 90°
        double h2 = h + Math.toRadians(90);

        // 3) Forward 42 (from p1, with new heading)
        Vector2d p2 = forward(p1, h2, 35);

        // 4) Come back 42 (i.e., forward -42 along current heading)
        Vector2d p3 = forward(p2, h2, -35);

        // 5) Left turn 90° (from h2)
        double h3 = h2 - Math.toRadians(90);

        // 6) Forward 20
        Vector2d p4 = forward(p3, h3, 20);

        // DcMotorEx shooter = hardwareMap.get(DcMotorEx.class, "shooterMotor");
        Thread drumThread = new Thread(() -> {
            // shooter.setPower(1.0);  // start shooter

            long start = System.currentTimeMillis();
            while (opModeIsActive() &&
                    !isStopRequested()) {

                // optional: telemetry or checks
                // telemetry.addData("shooter", "running");
                // (don’t spam, maybe sleep a bit)
                try {
                    Thread.sleep(10); // small sleep so we don’t busy-wait
                } catch (InterruptedException e) {
                    break;
                }
            }

            // shooter.setPower(0.0);  // stop shooter when done
        });


        drumThread.start();
        Action path = drive.actionBuilder(START_POSE)
                .strafeTo(p1)                 // forward 42 in current heading frame
                .turn(Math.toRadians(90))     // left 90
                .strafeTo(p2)                 // forward 42
                .afterTime(0.0, intake.intakeIn(0.8))
                .waitSeconds(2.0)
                .afterTime(0.0, intake.stop())
                .strafeTo(p3)                 // back 42
                .turn(Math.toRadians(-90))     // left 90
                .strafeTo(p4)                 // forward 20
                .turn(Math.toRadians(45))     // left 90
                .build();


        waitForStart();
        if (isStopRequested()) return;

        // Execute the motion
        Actions.runBlocking(path);

        telemetry.addLine("");
        telemetry.addLine("Auto done ✅");
        telemetry.update();
    }
}