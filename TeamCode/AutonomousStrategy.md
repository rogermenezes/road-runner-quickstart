
### Pseudocode
```java
Pose2d posRow1, posRow2, posRow3; // centers of the 3 artifact rows
Pose2d posStarting;               // starting pose of robot
Pose2d posShooting;               // shooting pose (facing drum)
Pose2d posFinal;                  // final resting pose after auto
Pose2d posTarget;                 // known calibration pose we log
```

Calculate posStarting either manually or using aprilTags. And then calculate the following positions.

```java
// Pre-calculated based on posStarting
posRow1 = computeRowPose(1, posStarting)
posRow2 = computeRowPose(2, posStarting)
posRow3 = computeRowPose(3, posStarting)
posShooting = computeShootingPose(posStarting)
```
Write the following shooting sub routine that will be helpful in TeleOp as well:

```javascript
function shootArtifacts():
    goTo(posShooting, headingTowardDrum)
    shooter.spinUp()
    waitUntilAllBallsShot()
    shooter.stop()
```

The following will outline the  autonomous code.

```java
setPoseEstimate(posStarting)
shootArtifacts()

intake.start()
goTo(posRow1, headingIntoRow)
shootArtifacts()

goTo(posRow2, headingIntoRow)
shootArtifacts()

goTo(posRow3, headingIntoRow)
shootArtifacts()
intake.stop()

goTo(posTarget, someKnownHeading)
writeCurrentPoseToFile()   // for later analysis / TeleOp init
```

### Sample final code

Say, if we are the blue alliance, starting with C1 position, this assumes the ball is behind.

```java
// Starting pose: tile C1, facing “forward” (toward goals)
Pose2d START_POSE = new Pose2d(
                0.0,
                0.0,
                0.0       // 0 rad = forward
        );

// Starting on C1, facing upfield (toward goals)
public static final Pose2d START_POSE =
        new Pose2d(0.0, 0.0, 0.0);

// Shooting position: C4, 45° to the left, facing Blue goal
public static final Pose2d SHOOT_POSE =
        new Pose2d(72.0, 0.0, Math.toRadians(45.0));

// Approach point before backing into SPIKE_4
public static final Pose2d SPIKE4_APPROACH =
        new Pose2d(72.0, 48.0, Math.toRadians(-90.0));

// Final SPIKE_4 pose: back (intake) pointing into the SPIKE row
public static final Pose2d SPIKE4_POSE =
        new Pose2d(72.0, 60.0, Math.toRadians(-90.0));



```

Final code will look like this:
```java
package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

// import your drive, shooter, intake, etc.

@Autonomous(name = "Auto: 3 Spike Cycle", group = "A")
public class ThreeSpikeCycleAuto extends LinearOpMode {

    // ---- Poses (relative to C1 origin) ----
    public static final Pose2d START_POSE =
            new Pose2d(0.0, 0.0, 0.0);

    // Shooting at C4, facing 45° left toward Blue goal
    public static final Pose2d SHOOT_POSE =
            new Pose2d(72.0, 0.0, Math.toRadians(45.0));

    // SPIKE_4 (row 4, E/F seam)
    public static final Pose2d SPIKE4_APPROACH =
            new Pose2d(72.0, 48.0, Math.toRadians(-90.0));
    public static final Pose2d SPIKE4_POSE =
            new Pose2d(72.0, 60.0, Math.toRadians(-90.0));

    // SPIKE_3 (row 3, E/F seam)
    public static final Pose2d SPIKE3_APPROACH =
            new Pose2d(48.0, 48.0, Math.toRadians(-90.0));
    public static final Pose2d SPIKE3_POSE =
            new Pose2d(48.0, 60.0, Math.toRadians(-90.0));

    // SPIKE_2 (row 2, E/F seam)
    public static final Pose2d SPIKE2_APPROACH =
            new Pose2d(24.0, 48.0, Math.toRadians(-90.0));
    public static final Pose2d SPIKE2_POSE =
            new Pose2d(24.0, 60.0, Math.toRadians(-90.0));

    @Override
    public void runOpMode() throws InterruptedException {
        // ---- Init hardware ----
        MecanumDrive drive = new MecanumDrive(hardwareMap, START_POSE);
        Shooter shooter = new Shooter(hardwareMap);   // your class
        Intake intake = new Intake(hardwareMap);      // your class

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addLine("Ready for 3-spike auto");
        telemetry.update();

        // ---- Pre-build driving Actions ----

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
                // back into SPIKE row (reverse motion, rear intake leading)
                .setReversed(true)
                .lineToLinearHeading(SPIKE4_POSE)
                .setReversed(false)
                .build();

        Action backToShootFromSpike4 = drive.actionBuilder(SPIKE4_POSE)
                // drive forward out of the row
                .lineToLinearHeading(SPIKE4_APPROACH)
                // return to C4 shooting pose
                .strafeToLinearHeading(
                        new Vector2d(SHOOT_POSE.position.x, SHOOT_POSE.position.y),
                        SHOOT_POSE.heading.toDouble()
                )
                .build();

        // SPIKE_3 cycle
        Action goToSpike3 = drive.actionBuilder(SHOOT_POSE)
                .strafeToLinearHeading(
                        new Vector2d(SPIKE3_APPROACH.position.x, SPIKE3_APPROACH.position.y),
                        SPIKE3_APPROACH.heading.toDouble()
                )
                .setReversed(true)
                .lineToLinearHeading(SPIKE3_POSE)
                .setReversed(false)
                .build();

        Action backToShootFromSpike3 = drive.actionBuilder(SPIKE3_POSE)
                .lineToLinearHeading(SPIKE3_APPROACH)
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
                .setReversed(true)
                .lineToLinearHeading(SPIKE2_POSE)
                .setReversed(false)
                .build();

        Action backToShootFromSpike2 = drive.actionBuilder(SPIKE2_POSE)
                .lineToLinearHeading(SPIKE2_APPROACH)
                .strafeToLinearHeading(
                        new Vector2d(SHOOT_POSE.position.x, SHOOT_POSE.position.y),
                        SHOOT_POSE.heading.toDouble()
                )
                .build();

        // Example shooter Action (replace with your own, or use markers)
        Action shootAction = shooter.shootThree();  // placeholder
        // You could also expose: shooter.spinUpAndFire(), etc.

        waitForStart();

        if (isStopRequested()) return;

        // ---------- SEQUENCE ----------

        // START -> SHOOT -> shoot balls
        Actions.runBlocking(goToShootFirst);
        Actions.runBlocking(shootAction);

        // ===== Cycle 1: SPIKE_4 =====
        intake.setPower(1.0);                   // start intake before backing in
        Actions.runBlocking(goToSpike4);
        Actions.runBlocking(new SleepAction(0.5)); // small dwell to finish intake
        intake.setPower(0.0);

        Actions.runBlocking(backToShootFromSpike4);
        Actions.runBlocking(shootAction);

        // ===== Cycle 2: SPIKE_3 =====
        intake.setPower(1.0);
        Actions.runBlocking(goToSpike3);
        Actions.runBlocking(new SleepAction(0.5));
        intake.setPower(0.0);

        Actions.runBlocking(backToShootFromSpike3);
        Actions.runBlocking(shootAction);

        // ===== Cycle 3: SPIKE_2 =====
        intake.setPower(1.0);
        Actions.runBlocking(goToSpike2);
        Actions.runBlocking(new SleepAction(0.5));
        intake.setPower(0.0);

        Actions.runBlocking(backToShootFromSpike2);
        Actions.runBlocking(shootAction);

        telemetry.addLine("Auto complete");
        telemetry.update();
    }
}

```


#### Notes:
Few things to keep in mind:
 * C1 will be the starting mode and we'll end at C1 as well.
 * 


