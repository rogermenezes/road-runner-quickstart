package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import com.acmerobotics.roadrunner.ftc.Actions;

@Autonomous(name = "Auto: Simple Park", group = "A")
public class AutoSimplePark extends LinearOpMode {

    // Set this to your actual starting pose on the field (units are inches/radians by default in quickstart)
    // Example: Facing down-field from the left tile on BLUE alliance
    private static final Pose2d START_POSE = new Pose2d( new Vector2d(12, 63), Math.toRadians(-90) );

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Initialize drive at the known start pose (you already tuned drive params)
        MecanumDrive drive = new MecanumDrive(hardwareMap, START_POSE);

        // Build a short autonomous path: forward, turn, strafe to “park”
        Action goScoreAndPark = drive.actionBuilder(START_POSE)
                .lineToY(24)                 // drive forward to Y = 24"
                .turn(Math.toRadians(90))    // rotate 90° CCW
                .strafeTo(new Vector2d(24, 36)) // slide to a “parking” tile
                .build();


        telemetry.addLine("Initialized. Waiting for START…");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // Execute the motion
        Actions.runBlocking(goScoreAndPark);

        telemetry.addLine("Auto done ✅");
        telemetry.update();
    }
}