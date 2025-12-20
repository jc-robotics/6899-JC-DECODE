package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.trajectory.Trajectory;

import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import java.util.Arrays;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Libs.PlayOpMode;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import com.acmerobotics.roadrunner.geometry.Vector2d;
@Autonomous (name = "PreProgrammedAuto", group = "Autonomous")
public class Auto extends PlayOpMode {
    Vector2d LEFT = new Vector2d(0,0);
    Vector2d Right = new Vector2d(0,0);

    enum Stage {
        MOVE_TO_CENTER,
        PICKUP_SAMPLE,
        MOVE_TO_DROPOFF,
        DROP_SAMPlE,
        WAIT,
        PICKUP_SPECIMEN,
        MOVE_TO_BAR,
        DROP_SPECIMEN,
        Reset,
        IDLE
    }
    Stage currentStage = Stage.IDLE;
    @Override
    protected void preinitilize() {
        isTeleOp = false;
    }

    @Override
    protected void initialize() {

    }

    @Override
    protected void run(double dt) throws InterruptedException {

    }
    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(0, 0, 0);
        drive.setPoseEstimate(startPose);
        Trajectory MoveToCenter = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(10, 108), Math.toRadians(0))
                .build();
//        Trajectory PickUpSample =  drive.trajectoryBuilder(MoveToCenter.end())
//
//                .addDisplacementMarker(() ->{
//
//                    //Activate intake
//                    //check color sensor for status
//                    //if color is acceptable intake goes into position for drivetrain movement
//                })
//                .lineTo(LEFT)
//                .lineTo(Right)
//
//                .build();
//        Trajectory MoveToDropOff = drive.trajectoryBuilder(PickUpSample.end())
//                .splineTo(new Vector2d(startPose.getX(),startPose.getY()),Math.toRadians(90))
//                .build();
//        Trajectory DropSample;
//        Trajectory Wait;
//        Trajectory PickUpSpecimen;
//        Trajectory MoveToBar= drive.trajectoryBuilder(PickUpSample.end())
//                .splineTo(new Vector2d(12,24),Math.toRadians(-90))
//                .build();
//        Trajectory DropSpecimen;

        waitForStart();

        if (isStopRequested()) return;
        currentStage = Stage.MOVE_TO_CENTER;
        drive.followTrajectoryAsync(MoveToCenter);

        int cycleCount = 0;

        while (opModeIsActive() && cycleCount < 3 && !isStopRequested()) {
            switch (currentStage) {
                case MOVE_TO_CENTER:

                    if (!drive.isBusy()) {
                        currentStage = Stage.PICKUP_SAMPLE;
                       // drive.followTrajectoryAsync(Pick)
                        break;}

                case PICKUP_SAMPLE:
                    currentStage = Stage.MOVE_TO_DROPOFF;
                    break;

                case MOVE_TO_DROPOFF:

                    currentStage = Stage.DROP_SAMPlE;
                    break;

                case DROP_SAMPlE:

                    currentStage = Stage.WAIT;
                    break;

                case WAIT:

                    currentStage = Stage.PICKUP_SPECIMEN;
                    break;



                case PICKUP_SPECIMEN:

                    currentStage = Stage.MOVE_TO_BAR;
                    break;

                case MOVE_TO_BAR:

                    currentStage = Stage.DROP_SPECIMEN;
                    break;

                case DROP_SPECIMEN:
                    currentStage = Stage.Reset;
                    break;

                case Reset:

                    return;
            }

        }


    }




}
