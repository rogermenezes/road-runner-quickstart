package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;

import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Arrays;

@Autonomous(name = "Basket Test")
public class BasketSide extends LinearOpMode {
    ElapsedTime runtime = new ElapsedTime();
    ElapsedTime currentTime = new ElapsedTime();

    MecanumDrive drive;

    DcMotor intake, cascade, cascade_secondary, horizontal_cascade;
    Servo delivery_arm_rotation_left, delivery_rotation, delivery_claw_spin, delivery_claw;
    Servo intake_rotation;
    ColorRangeSensor intake_color;

    public static final Vector2d dump_pos = new Vector2d(6.46, 24.332); // 5.2, 21.5 ;5.46, 25.332

    public static class IntoTheDeep_teleOP {
        public static double delivery_claw_close = 0.3;
        public static double delivery_claw_open = 0.54;
        public static double intake_box_flip_out_down = 0.25;
        public static double intake_box_flip_back_in = 0.04;
        public static double intake_claw_spin_normal_position = 0.18;
    }


    @Override
    public void runOpMode() {
        drive = new MecanumDrive(hardwareMap, new Pose2d(0.8, 0, 0));

        Servo light = hardwareMap.get(Servo.class, "light");

        delivery_arm_rotation_left = hardwareMap.get(Servo.class, "delivery_arm_rotation_left");
//        delivery_arm_rotation_right = hardwareMap.get(Servo.class, "delivery_arm_rotation_right");
        delivery_rotation = hardwareMap.get(Servo.class, "delivery_rotation");
        delivery_claw_spin = hardwareMap.get(Servo.class, "delivery_claw_spin");
        delivery_claw = hardwareMap.get(Servo.class, "delivery_claw");

        intake_rotation = hardwareMap.get(Servo.class, "intake_rotation");
        intake = hardwareMap.get(DcMotor.class, "intake");
        intake_color = hardwareMap.get(ColorRangeSensor.class, "color");

        cascade = hardwareMap.get(DcMotor.class, "cascade_left");
        cascade_secondary = hardwareMap.get(DcMotor.class, "cascade_right");
        horizontal_cascade = hardwareMap.get(DcMotor.class, "horizontal_cascade");

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        cascade.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        cascade_secondary.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        cascade.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        cascade_secondary.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        cascade_secondary.setDirection(DcMotorSimple.Direction.REVERSE);

        //horizontal_cascade.setDirection(DcMotorSimple.Direction.REVERSE);
        horizontal_cascade.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontal_cascade.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        light.setPosition(0.368);

        waitForStart();

        light.setPosition(0);

        Thread telemetryThread = new Thread(() -> {
            currentTime.reset();
            while (opModeIsActive()) {
                telemetry.addData("x", drive.localizer.getPose().position.x);
                telemetry.addData("y", drive.localizer.getPose().position.y);
                telemetry.addData("heading (deg)", Math.toDegrees(drive.localizer.getPose().heading.toDouble()));
                telemetry.addLine();
                telemetry.addData("time", currentTime.seconds());
                telemetry.update();
            }
        });

        telemetryThread.start();

        beginCycle();

        oneCycle(-9.1, -2, 0);

        oneCycle(-17.5, 0.5, 0);

//        oneCycle(-0.4, 2, Math.toRadians(62)); // 4, 0.5, 38
        oneCycle(-10, 2.64, Math.toRadians(30.4)); // -8.1, 2.64, 30.4 deg

        Actions.runBlocking(drive.actionBuilder(drive.localizer.getPose()).strafeTo(new Vector2d(52, -8)).build()); // deg -45
    }

    public void beginCycle() {
        delivery_claw.setPosition(IntoTheDeep_teleOP.delivery_claw_close);

        Thread cascadeThread = new Thread(() -> {
            sleep(200);
            cascade_move(cascade, cascade_secondary, 1, -3100, 3, true);
            delivery_arm_rotation_left.setPosition(0.77);
            delivery_rotation.setPosition(0.14);
        });

        cascadeThread.start();

        Actions.runBlocking(drive.actionBuilder(drive.localizer.getPose())
//               .strafeToLinearHeading(new Vector2d(9, 17), Math.toRadians(-65))//y=35.5
                        .strafeToLinearHeading(dump_pos, Math.toRadians(-45), new MinVelConstraint(Arrays.asList(
                                new TranslationalVelConstraint(18.5)
                        )))//x = 6.12 y=54.29
                        .build()
        );

        try {
            cascadeThread.join();
        } catch (InterruptedException e) {}

        sleep(1100);
        delivery_claw.setPosition(IntoTheDeep_teleOP.delivery_claw_open);
        sleep(500);

        delivery_arm_rotation_left.setPosition(0.2);
        delivery_rotation.setPosition(0.3);

        Actions.runBlocking(drive.actionBuilder(drive.localizer.getPose())
                .strafeTo(dump_pos.minus(new Vector2d(-1.6, 0.7)))
                .build()
        );

        Thread cascadeDownThread = new Thread(() -> cascade_move(cascade, cascade_secondary, 1, 1, 3, true));
        cascadeDownThread.start(); // drop the piece and prepare to cycle
    }

    public void oneCycle(double strafeDiff, double sxdiff, double heading) {

        Actions.runBlocking(drive.actionBuilder(drive.localizer.getPose()) //move forward to close to 1st sample
                .strafeToLinearHeading(new Vector2d(13 + sxdiff, 30 + strafeDiff), heading)
                .build()
        );

        if (heading == 0) gamepadA(); // horizontal cascade out and pick up pieces
        else finalGamepadA();

        Thread watch = new Thread(() -> {
            intake.setPower(1);
            ElapsedTime test = new ElapsedTime();
            test.reset();
            while (intake_color.getDistance(DistanceUnit.MM) > 17 && opModeIsActive() && test.milliseconds() < 1750);
            intake.setPower(0);
        });

        watch.start();

        Actions.runBlocking(drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(new Vector2d(15.5 + sxdiff, 30 + strafeDiff), heading)
                .build()
        );

//        Thread pickup2 = new Thread(() -> {
//            Actions.runBlocking(drive.actionBuilder(drive.localizer.getPose())
//                    .turn(Math.toRadians(8))
//                    .build()
//            );
//            while (opModeIsActive()) {
//                Actions.runBlocking(drive.actionBuilder(drive.localizer.getPose())
//                        .turn(Math.toRadians(-16))
//                        .turn(Math.toRadians(16))
//                        .build()
//                );
//            }
//        });
//
//        pickup2.start();

        try {
            watch.join();
        } catch (InterruptedException ignored) {
        } finally {
            intake.setPower(0);
//            pickup2.interrupt();
        }

        gamepadY();// after successfully sucked in a sample, intake comes back in and delivery claw grab the sample in the holder

        cascade_move(cascade, cascade_secondary, 1, -2000, 2, true);
        Thread cascade_thread = new Thread(() -> cascade_move(cascade, cascade_secondary, 1, -3100, 1, true));
        cascade_thread.start();

        Actions.runBlocking(drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(dump_pos, Math.toRadians(-45))
                .build()
        );

        drop(0);
        intake_rotation.setPosition(IntoTheDeep_teleOP.intake_box_flip_back_in);
    }

    public void drop(int delay){
        delivery_arm_rotation_left.setPosition(0.77);
        delivery_rotation.setPosition(0.14);
        sleep(delay);
        delivery_claw.setPosition(IntoTheDeep_teleOP.delivery_claw_open);
        sleep(500);

        delivery_arm_rotation_left.setPosition(0.2);
        delivery_rotation.setPosition(0.3);

        Actions.runBlocking(drive.actionBuilder(drive.localizer.getPose())
                .strafeTo(dump_pos.minus(new Vector2d(-1.6, 0.7)))
                .build()
        );

        Thread cascadeDownThread = new Thread(() -> cascade_move(cascade, cascade_secondary, 1, 1, 3, true));
        cascadeDownThread.start();
    }
    public void gamepadA(){
        intake_rotation.setPosition(IntoTheDeep_teleOP.intake_box_flip_out_down + .01);
        intake.setPower(1);
        //gamepad a
        cascade_move_horizontal(horizontal_cascade, 1,1800,2,false);
        // horizontal cascade out
        delivery_arm_rotation_left.setPosition(0.2);
        delivery_rotation.setPosition(0.3);
        delivery_claw.setPosition(IntoTheDeep_teleOP.delivery_claw_open);

    }
    public void finalGamepadA() {
        //gamepad a
        cascade_move_horizontal(horizontal_cascade, 1,1800,2,false);
        intake_rotation.setPosition(IntoTheDeep_teleOP.intake_box_flip_out_down + .01);
        intake.setPower(1);
        // horizontal cascade out
        delivery_arm_rotation_left.setPosition(0.2);
        delivery_rotation.setPosition(0.3);
        delivery_claw.setPosition(IntoTheDeep_teleOP.delivery_claw_open);
    }
    public void gamepadY(){ //This is the serial action from after intake sucked in a sample successfully to "delivery claw to grab the sample in the holder.
//        intake.setPower(0);
        //gamepad y
        delivery_claw_spin.setPosition(IntoTheDeep_teleOP.intake_claw_spin_normal_position); //intake claw spin to normal position = 0.18 ; flip 180 degree position =0.85
        intake_rotation.setPosition(IntoTheDeep_teleOP.intake_box_flip_back_in); //intake box flip back in = 0.04; intake box flip down out = 0.25
        cascade_move_horizontal(horizontal_cascade, 1,-100,1,false); // horizontal cascade retract back
        intake.setPower(1);// intake roller spin to push sample into the holder
        ElapsedTime test = new ElapsedTime();
        test.reset();
        while (intake_color.getDistance(DistanceUnit.MM) < 17 && opModeIsActive() && test.milliseconds() < 1750);
        sleep(200); //wait for 0.5 seconds for sample to be in the holder
        intake.setPower(0);
        intake_rotation.setPosition(IntoTheDeep_teleOP.intake_box_flip_out_down + .01); //intake box flip to down out = 0.25 to avoid to claw crash to it;intake box flip back in = 0.04;
        sleep(100); // wait 0.5 seconds for

        delivery_claw.setPosition(IntoTheDeep_teleOP.delivery_claw_open); // delivery claw open == 0.54
        delivery_rotation.setPosition(0.55);
        sleep(200);
        delivery_arm_rotation_left.setPosition(0); //delivery black servo down to position to grab sample
        sleep(200);
        delivery_claw.setPosition(IntoTheDeep_teleOP.delivery_claw_close); // delivery_claw_close =0.3
        sleep(250);
        cascade_move(cascade, cascade_secondary, 1, -3100, 2, true);
        delivery_arm_rotation_left.setPosition(0.77);
        delivery_rotation.setPosition(0.14);


        //cascade_move(cascade, cascade_secondary, 1, -3100, 2, true);
//        delivery_arm_rotation_left.setPosition(0.77);
//        delivery_rotation.setPosition(0.14);
//        sleep(500);

    }

    public void cascade_move(DcMotor motor, DcMotor motor_secondary, double speed, int target,
                             int timeoutS, boolean hold) {
        telemetry.update();
        motor.setTargetPosition(target);
        if (motor_secondary != null) motor_secondary.setTargetPosition(target);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if (motor_secondary != null) motor_secondary.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        runtime.reset();
        motor.setPower(Math.abs(speed));
        if (motor_secondary != null) motor_secondary.setPower(Math.abs(speed));

        while (opModeIsActive() && (runtime.seconds() < timeoutS) && motor.isBusy()
                && (target < 0? (-motor.getCurrentPosition() < Math.abs(Math.floor(target * 0.9))) : (motor.getCurrentPosition() < Math.floor(target * 0.9)))) {
            telemetry.addLine("---------- Vertical Cascade ---------");
            telemetry.addData("Target :", "encoder counts %7d: timeout       = %3d", target, timeoutS);
            if (motor_secondary != null) telemetry.addData("Target :", "encoder_secondary counts %7d: timeout       = %3d", target, timeoutS);
            telemetry.addData("Current:", "encoder counts %7d: current_time= %3d", motor.getCurrentPosition(), (int) runtime.seconds());
            if (motor_secondary != null) telemetry.addData("Current:", "encoder_secondary counts %7d: current_time= %3d", motor_secondary.getCurrentPosition(), (int) runtime.seconds());
            telemetry.update();
        }
        if (! hold) motor.setPower(0);
        if (motor_secondary != null) motor_secondary.setPower(0);
        telemetry.addData("Stopped :",  "encoder counts %7d: timeout       = %3d", target, timeoutS);
        telemetry.update();
    }

    public void cascade_move_horizontal(DcMotor motor, double speed, int target,
                                        int timeoutS, boolean hold) {
        telemetry.update();
        motor.setTargetPosition(target);

        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        runtime.reset();
        motor.setPower(Math.abs(speed));

        while (opModeIsActive() && (runtime.seconds() < timeoutS) // && motor.isBusy()
                && (target < 0? (-motor.getCurrentPosition() < Math.abs(Math.floor(target * 0.9))) : (motor.getCurrentPosition() < Math.floor(target * 0.9)))) {
            telemetry.addLine("---------- Vertical Cascade ---------");
            telemetry.addData("Target :", "encoder counts %7d: timeout       = %3d", target, timeoutS);

            telemetry.addData("Current:", "encoder counts %7d: current_time= %3d", motor.getCurrentPosition(), (int) runtime.seconds());

            telemetry.update();

        }
        if (! hold) motor.setPower(0);

        telemetry.addData("Stopped :",  "encoder counts %7d: timeout       = %3d", target, timeoutS);
        telemetry.update();
    }

}
