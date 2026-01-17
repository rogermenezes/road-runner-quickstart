package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepQualifierD1 {

    public static final Pose2d START_POSE =
            //new Pose2d(0.0, 0.0, 0.0);
            new Pose2d(60, 0, Math.toRadians(180));

    public static final double APPROACH_Y_POSITION = -12.0;

    public static final double FINAL_SPIKE_Y_POSITION = -42.0;

    public static final Pose2d START_2 =
            new Pose2d(62.0, 0, Math.toRadians(90.0));

    // Shooting at C4, shooter is at the BACK, facing Blue goal
    public static final Pose2d SHOOT_POSE =
            new Pose2d(5, 0.0, Math.toRadians(157.0));

    public static final Pose2d SHOOT_POSE_2 =
            new Pose2d(5, 5, Math.toRadians(157.0));


    // SPIKE_4 (row 4, E/F seam)
    // heading +90°: FRONT (intake) points +Y into the SPIKE row
    // Using Y as 12.0 otherwise there is a runtime error about maxVel being zero
    public static final Pose2d SPIKE4_APPROACH =
            new Pose2d(72.0, -12.0, Math.toRadians(-90.0));
    public static final Pose2d SPIKE4_POSE =
            new Pose2d(72.0, -42.0, Math.toRadians(-90.0));

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



    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        var drive = myBot.getDrive();

        TranslationalVelConstraint slowVel = new TranslationalVelConstraint(7.0);   // in/s
        ProfileAccelConstraint slowAccel   = new ProfileAccelConstraint(-5.0, 5.0); // in/s^2

        // 1) START -> SHOOT_POSE
        Action goToShootFirst = drive.actionBuilder(START_POSE)
                .strafeToLinearHeading(
                        new Vector2d(START_2.position.x, START_2.position.y),
                        START_2.heading.toDouble()
                )
//                .strafeToLinearHeading(
//                        new Vector2d(SHOOT_POSE.position.x, SHOOT_POSE.position.y),
//                        SHOOT_POSE.heading.toDouble()
//                )
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


        Action seq = new com.acmerobotics.roadrunner.SequentialAction(
                goToShootFirst
                //goToSpike4
        );

        myBot.runAction(seq);

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_LIGHT)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();

    }
}
