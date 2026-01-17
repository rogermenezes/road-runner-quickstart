package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;

public interface DriveActionBuilder {
    ActionBuilderWrapper actionBuilder(Pose2d startPose);
}
