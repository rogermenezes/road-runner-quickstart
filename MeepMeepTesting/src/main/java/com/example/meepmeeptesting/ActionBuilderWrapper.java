package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;

public interface ActionBuilderWrapper {
    ActionBuilderWrapper strafeToLinearHeading(Vector2d pos, double heading);
    ActionBuilderWrapper strafeToLinearHeading(
            Vector2d pos,
            double heading,
            TranslationalVelConstraint vel,
            ProfileAccelConstraint accel
    );
    Action build();
}
