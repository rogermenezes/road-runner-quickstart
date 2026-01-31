package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.io.PrintWriter;
import java.io.StringWriter;

/**
 * The high level logic is this:
 * 1. Robot starts at START_POSE
 * 2. Robot goes to SHOOT_POSE
 * 3. Robot shoots three balls
 * 4. Robot goes to SPIKE_4, intake balls
 * 5. Robot goes back to SHOOT_POSE and shoots three balls
 * 6. Robot goes to SPIKE_3, intake balls
 * 7. Robot goes back to SHOOT_POSE and shoots three balls
 * 8. Robot goes to SPIKE_2
 * 9. Robot goes back to SHOOT_POSE and shoots three balls
 *
 *  START_POSE -> SHOOT_POSE -> Shoot balls -> SPIKE4_APPROACH -> SPIKE4_POSE -> Intake ->
 *      SHOOT_POSE -> Shoot balls -> SPIKE3_APPROACH -> SPIKE3_POSE -> Intake ->
 *      SHOOT_POSE -> Shoot balls -> SPIKE2_APPROACH -> SPIKE_2_POSE -> Intake ->
 *      SHOOT_POSE -> Shoot balls
 *
 * Going to a strike pose, e.g. SPIKE_4, requires it to go in a right angle path,
 * so that it can pick up the balls in SPIKE position in a straight line.
 * The way to do this is:
 * SHOOT_POSE -> SPIKE_4_APPROACH -> SPIKE_4_POSE
 *
 *
 */
@Config
@Autonomous(name = "(Not needed): Qualifier (Draft) 2/1 (Red D1 Start)", group = "A")
public class Qualifier2AutonomousD1 extends LinearOpMode {

    // Set this to your actual starting pose on the field (units are inches/radians by default in quickstart)
    // Example: Facing down-field from the left tile on BLUE alliance

    // ---- Poses (relative to C1 origin) ----
    public static final Pose2d START_POSE =
            new Pose2d(0.0, 0.0, 0.0);

    public static final double APPROACH_Y_POSITION = -12.0;

    public static final double FINAL_SPIKE_Y_POSITION = -42.0;

    public static final Pose2d START_2 =
            new Pose2d(3, 0.0, Math.toRadians(0.0));

    // Shooting at C4, shooter is at the BACK, facing Blue goal
    public static final Pose2d SHOOT_POSE =
            new Pose2d(72, 0.0, Math.toRadians(-45));

    public static final Pose2d SHOOT_POSE_2 =
            new Pose2d(5, 5, Math.toRadians(150.0));


    // SPIKE_4 (row 4, E/F seam)
    // heading +90°: FRONT (intake) points +Y into the SPIKE row
    // Using Y as 12.0 otherwise there is a runtime error about maxVel being zero
    public static final Pose2d SPIKE4_APPROACH =
            new Pose2d(74.0, -12.0, Math.toRadians(-90.0));
    public static final Pose2d SPIKE4_POSE =
            new Pose2d(74.0, -42.0, Math.toRadians(-90.0));


    public static final Pose2d CLASSIFIER_APPROACH =
            new Pose2d(62.0, -12.0, Math.toRadians(-90.0));
    public static final Pose2d CLASSIFIER_POSE =
            new Pose2d(62.0, -54.0, Math.toRadians(-90.0));

    public static final Pose2d TARGET_POSE = new Pose2d(31.1, -40.9, Math.toRadians(-45));


    // SPIKE_3 (row 3, E/F seam)
    public static final Pose2d SPIKE3_APPROACH =
            new Pose2d(48.0, APPROACH_Y_POSITION, Math.toRadians(-90.0));
    public static final Pose2d SPIKE3_POSE =
            new Pose2d(48.0, FINAL_SPIKE_Y_POSITION, Math.toRadians(-90.0));

    // SPIKE_2 (row 2, E/F seam)
    public static final Pose2d SPIKE2_APPROACH =
            new Pose2d(24.0, -12.0, Math.toRadians(-90.0));
    public static final Pose2d SPIKE2_POSE =
            new Pose2d(24.0, -42.0, Math.toRadians(-90.0));

    private ParallelShooter shooter;   // 🔹 NEW

    @Override
    public void runOpMode() throws InterruptedException {
        try {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        MecanumDrive drive = new MecanumDrive(hardwareMap, START_POSE);
        shooter = new ParallelShooter(hardwareMap, telemetry, drive);

        // Initialize drive at the known start pose (you already tuned drive params)
        // TODO: Duplicate variable, remove this after testing
        Pose2d START_POSE = new Pose2d(new Vector2d(0, 0), Math.toRadians(0));

        // trying to move slower while getting to STRIKE_POSE
        TranslationalVelConstraint slowVel = new TranslationalVelConstraint(7.0);   // in/s
        ProfileAccelConstraint slowAccel   = new ProfileAccelConstraint(-5.0, 5.0); // in/s^2

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
//                .strafeToLinearHeading(
//                        new Vector2d(SPIKE4_APPROACH.position.x, SPIKE4_APPROACH.position.y),
//                        SPIKE4_APPROACH.heading.toDouble()
//                )
                // return to C4 shooting pose (shooter/back toward goal)
                .strafeToLinearHeading(
                        new Vector2d(SHOOT_POSE.position.x, SHOOT_POSE.position.y),
                        SHOOT_POSE.heading.toDouble()
                )
                .build();

        //Go to classifier
            Action goToClassifier = drive.actionBuilder(SHOOT_POSE)
                    .strafeToLinearHeading(
                            new Vector2d(CLASSIFIER_APPROACH.position.x, CLASSIFIER_APPROACH.position.y),
                            CLASSIFIER_APPROACH.heading.toDouble()
                    )
                    .strafeToLinearHeading(
                            new Vector2d(CLASSIFIER_POSE.position.x, CLASSIFIER_POSE.position.y),
                            CLASSIFIER_POSE.heading.toDouble()
                    )
                    .build();

            Action lateralMove = drive.actionBuilder(SHOOT_POSE)
                    .strafeToLinearHeading(
                            new Vector2d(TARGET_POSE.position.x, TARGET_POSE.position.y),
                            TARGET_POSE.heading.toDouble()
                    )
                    .build();



            telemetry.addData("actual end pose", drive.localizer.getPose());
            telemetry.update();


            // SPIKE_3 cycle
        Action goToSpike3 = drive.actionBuilder(SHOOT_POSE)
                .strafeToLinearHeading(
                        new Vector2d(SPIKE3_APPROACH.position.x, SPIKE3_APPROACH.position.y),
                        SPIKE3_APPROACH.heading.toDouble()
                )
                .strafeToLinearHeading(
                        new Vector2d(SPIKE3_POSE.position.x, SPIKE3_POSE.position.y),
                        SPIKE3_POSE.heading.toDouble()
                )
                .build();

        Action backToShootFromSpike3 = drive.actionBuilder(SPIKE3_POSE)
                .strafeToLinearHeading(
                        new Vector2d(SPIKE3_APPROACH.position.x, SPIKE3_APPROACH.position.y),
                        SPIKE3_APPROACH.heading.toDouble()
                )
                .strafeToLinearHeading(
                        new Vector2d(SHOOT_POSE.position.x, SHOOT_POSE.position.y),
                        SHOOT_POSE.heading.toDouble()
                )
                .build();

        // SPIKE_2 cycle
        Action goToSpike2 = drive.actionBuilder(SHOOT_POSE)
                .strafeToLinearHeading(
                        new Vector2d(SPIKE2_APPROACH.position.x, SPIKE2_APPROACH.position.y),
                        SPIKE2_APPROACH.heading.toDouble()
                )
                .strafeToLinearHeading(
                        new Vector2d(SPIKE2_POSE.position.x, SPIKE2_POSE.position.y),
                        SPIKE2_POSE.heading.toDouble(),
                        slowVel,
                        slowAccel
                )
                .build();

        Action backToShootFromSpike2 = drive.actionBuilder(SPIKE2_POSE)
//                .strafeToLinearHeading(
//                        new Vector2d(SPIKE2_APPROACH.position.x, SPIKE2_APPROACH.position.y),
//                        SPIKE2_APPROACH.heading.toDouble()
//                )
                .strafeToLinearHeading(
                        new Vector2d(SHOOT_POSE_2.position.x, SHOOT_POSE_2.position.y),
                        SHOOT_POSE_2.heading.toDouble()
                )
                .build();


        waitForStart();
        if (isStopRequested()) return;

        // ---------- SEQUENCE ----------

        // START -> SHOOT -> shoot balls
        shooter.intakeOn();
        Actions.runBlocking(goToShootFirst);
        //shooter.autoalign();

//        shooter.shootThreeBallsV2(0.2, 0.6, 1);
        shooter.count = 0;
        // Go to SPIKE_4 =====
        //Actions.runBlocking(goToSpike4);
        //shooter.intake(this, telemetry);

        telemetry.addData("ball count", shooter.count);
        telemetry.update();

        //OR
//        shooter.intakeOn();
        //Actions.runBlocking(goToSpike4);
        //Actions.runBlocking(backToShootFromSpike4);
        Actions.runBlocking(lateralMove);

//        shooter.drum.setPosition(0.0);
//        Actions.runBlocking(
//                new ParallelAction(
//                        goToSpike2,
//                        new ParallelIntakeAction(shooter, telemetry)
//                )
//        );
//
//        Actions.runBlocking(backToShootFromSpike2);
//        //shooter.autoalign();
//        shooter.shootThreeBallsV2(0.2, 0.6, 0.99);

        // ===== Cycle 2: SPIKE_3 =====
        // Actions.runBlocking(goToSpike3);
        //shooter.intake(this, telemetry);
        //Actions.runBlocking(backToShootFromSpike3);
        //shooter.shootThreeBalls(0.2, 0.6, 0.99);

        // ===== Cycle 3: SPIKE_2 =====
        //Actions.runBlocking(goToSpike2);
        //shooter.intake(this, telemetry);
        //Actions.runBlocking(backToShootFromSpike2);
        //shooter.shootThreeBalls(0.2, 0.6, 0.99);

        /// ////////////////////
        telemetry.addLine("");
        telemetry.addLine("Auto done ✅");        telemetry.update();

        } catch (Exception e) {
            StringWriter sw = new StringWriter();
            PrintWriter pw = new PrintWriter(sw);
            e.printStackTrace(pw);


            telemetry.addLine("EXCEPTION STACK TRACE:");
            telemetry.addLine(sw.toString());
            telemetry.update();
            e.printStackTrace();
        }
    }
}
