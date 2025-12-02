package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Auto: Scrimmage 11/30 (Red F6)", group = "A")
public class ScrimmageRedF6Autonomous extends LinearOpMode {

    // Set this to your actual starting pose on the field (units are inches/radians by default in quickstart)
    // Example: Facing down-field from the left tile on BLUE alliance

    public static final Pose2d START_POSE =
            new Pose2d(0.0, 0.0, 0.0);

    public static final double APPROACH_Y_POSITION = 0.0;

    public static final double FINAL_SPIKE_Y_POSITION = -48.0;

    // Shooting at C4, shooter is at the BACK, facing Blue goal
    public static final Pose2d SHOOT_POSE =
            new Pose2d(66.0, 0.0, Math.toRadians(0.0));

    // SPIKE_4 (row 4, E/F seam)
    // heading +90°: FRONT (intake) points +Y into the SPIKE row
    // Using Y as 12.0 otherwise there is a runtime error about maxVel being zero
    public static final Pose2d SPIKE4_APPROACH =
            new Pose2d(72.0, -12.0, Math.toRadians(-90.0));
    public static final Pose2d SPIKE4_POSE =
            new Pose2d(72.0, FINAL_SPIKE_Y_POSITION, Math.toRadians(-90.0));

    private Shooter shooter;   // 🔹 NEW

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Intake intake = new Intake(hardwareMap);

        // Initialize drive at the known start pose (you already tuned drive params)
        Pose2d START_POSE = new Pose2d(new Vector2d(0, 0), Math.toRadians(0));

        MecanumDrive drive = new MecanumDrive(hardwareMap, START_POSE);
        shooter = new Shooter(hardwareMap);

        // trying to move slower while getting to STRIKE_POSE
        TranslationalVelConstraint slowVel = new TranslationalVelConstraint(15.0);   // in/s
        ProfileAccelConstraint slowAccel   = new ProfileAccelConstraint(-10.0, 10.0); // in/s^2

        // 1) START -> SHOOT_POSE
        Action goToShootFirst = drive.actionBuilder(START_POSE)
                .strafeToLinearHeading(
                        new Vector2d(SHOOT_POSE.position.x, SHOOT_POSE.position.y),
                        SHOOT_POSE.heading.toDouble()
                )
                .build();

        // SPIKE_4 cycle: SHOOT -> SPIKE4 -> SHOOT
        Action goToSpike4 = drive.actionBuilder(SHOOT_POSE)
                // move/rotate into approach pose
                .strafeToLinearHeading(
                        new Vector2d(SPIKE4_APPROACH.position.x, SPIKE4_APPROACH.position.y),
                        SPIKE4_APPROACH.heading.toDouble()
                )
                // drive forward into the SPIKE row with front intake
                .strafeToLinearHeading(
                        new Vector2d(SPIKE4_POSE.position.x, SPIKE4_POSE.position.y),
                        SPIKE4_POSE.heading.toDouble(),
                        slowVel,
                        slowAccel
                )
                .build();

        Action backToShootFromSpike4 = drive.actionBuilder(SPIKE4_POSE)
                // drive forward out of the row (back toward approach)
                .strafeToLinearHeading(
                        new Vector2d(SPIKE4_APPROACH.position.x, SPIKE4_APPROACH.position.y),
                        SPIKE4_APPROACH.heading.toDouble()
                )
                // return to C4 shooting pose (shooter/back toward goal)
                .strafeToLinearHeading(
                        new Vector2d(SHOOT_POSE.position.x, SHOOT_POSE.position.y),
                        SHOOT_POSE.heading.toDouble()
                )
                .build();

        waitForStart();
        if (isStopRequested()) return;

        // ---------- SEQUENCE ----------

        // START -> SHOOT -> shoot balls
        Actions.runBlocking(intake.intakeIn(1.0));
        Actions.runBlocking(goToShootFirst);
        shooter.spinUpForAuto();
        sleep(1000); // give flywheels time to get up to speed (tune this)

        shooter.shootThreeBalls(this, telemetry);


        // ===== Cycle 1: SPIKE_4 =====
//        Actions.runBlocking(goToSpike4);
//        shooter.intakeThreeBalls(this, telemetry, true);
//        Actions.runBlocking(backToShootFromSpike4);
//        shooter.shootThreeBalls(this, telemetry);

        // ===== Cycle 2: SPIKE_3 =====
        /*+intake.intakeIn(1.0);                   // start intake before backing in
        Actions.runBlocking(goToSpike3);
        Actions.runBlocking(new SleepAction(0.5));
        intake.stop();

        Actions.runBlocking(backToShootFromSpike3);
        Actions.runBlocking(shootAction);

        // ===== Cycle 3: SPIKE_2 =====
        intake.intakeIn(1.0);                   // start intake before backing in
        Actions.runBlocking(goToSpike2);
        Actions.runBlocking(new SleepAction(0.5));
        intake.stop();

        Actions.runBlocking(backToShootFromSpike2);
        Actions.runBlocking(shootAction);
*/
        /// ////////////////////
        telemetry.addLine("");
        telemetry.addLine("Auto done ✅");
        telemetry.update();
    }
}
