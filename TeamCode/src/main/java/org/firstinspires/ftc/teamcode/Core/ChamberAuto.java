package org.firstinspires.ftc.teamcode;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import  com.acmerobotics.roadrunner.trajectory.SimpleTrajectoryBuilder;

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
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import com.acmerobotics.roadrunner.geometry.Vector2d;
@Autonomous (name = "BasicAuto", group = "Autonomous")
public class ChamberAuto  extends  PlayOpMode{
    Vector2d SAMPLE_1 = new Vector2d(0,0);
    Vector2d SAMPLE_2  = new Vector2d(0,0);
    Vector2d SAMPLE_3   = new Vector2d(0,0);

    enum Stage {
        MOVE_TO_BASKET,
        MOVE_TO_SAMPLE_1,
        MOVE_TO_BASKET_1,
        MOVE_TO_SAMPLE_2,
        MOVE_TO_BASKET_2,
        MOVE_TO_SAMPLE_3,
        MOVE_TO_BASKET_3,
        SEARCHING,
        IDLE

    }
    Stage currentStage = Stage.IDLE;
    lift L = new lift();

    @Override
    protected void preinitilize() {
        isTeleOp = false;
    }

    @Override
    protected void initialize() {
     L.init(hardwareMap);
     L.initializeEncoderMotor(L.DC1);
     L.initializeEncoderMotor(L.DC2);

    }
    void Build(){


    }
    @Override
    protected void run(double dt) throws InterruptedException {

    }

    @Override
    public void runOpMode(){
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(0, 0, 0);
        drive.setPoseEstimate(startPose);
        waitForStart();

        if (isStopRequested()) return;
        TrajectorySequence MoveToBasket = drive.trajectorySequenceBuilder(startPose)
                .splineTo(new Vector2d(50, 108), Math.toRadians(90))
                .addDisplacementMarker(() ->{
                    L.Arm(3400,1,3000,1);
                    //Intake
                })
                .build();
        TrajectorySequence MoveToSample1 = drive.trajectorySequenceBuilder(startPose)
                .addDisplacementMarker(() ->{
                    L.Arm(0,1,0,1);
                    //Intake
                })
                .waitSeconds(3)
                .splineTo(new Vector2d(SAMPLE_1.getX(),SAMPLE_1.getY()), Math.toRadians(90))
                .addDisplacementMarker(() ->{
                    L.Arm(3400,1,3000,1);
                    //Intake
                })
                .build();
        TrajectorySequence MoveToBasket1 = drive.trajectorySequenceBuilder(startPose)
                .splineTo(new Vector2d(50, 108), Math.toRadians(90))
                .addDisplacementMarker(() ->{
                    L.Arm(3400,1,3000,1);
                    //Intake
                })
                .build();
        TrajectorySequence MoveToSample2 = drive.trajectorySequenceBuilder(startPose)
                .splineTo(new Vector2d(SAMPLE_2.getX(), SAMPLE_2.getY()), Math.toRadians(90))
                .addDisplacementMarker(() ->{
                    L.Arm(3400,1,3000,1);
                    //Intake
                })
                .build();
        TrajectorySequence MoveToBasket2 = drive.trajectorySequenceBuilder(startPose)
                .splineTo(new Vector2d(50, 108), Math.toRadians(90))
                .addDisplacementMarker(() ->{
                    L.Arm(3400,1,3000,1);
                    //Intake
                })
                .build();
        TrajectorySequence MoveToSample3 = drive.trajectorySequenceBuilder(startPose)
                .addDisplacementMarker(() ->{
                    L.Arm(0,1,0,1);
                    //Intake
                })
                .splineTo(new Vector2d(SAMPLE_3.getX(), SAMPLE_3.getY()), Math.toRadians(90))
                .addDisplacementMarker(() ->{

                    //Intake
                })

                .build();
        TrajectorySequence MoveToBasket3 = drive.trajectorySequenceBuilder(startPose)
                .splineTo(new Vector2d(50, 108), Math.toRadians(90))
                .addDisplacementMarker(() ->{
                    L.Arm(3400,1,3000,1);
                    //Intake
                })
                .build();
        TrajectorySequence Idle = drive.trajectorySequenceBuilder(startPose)
                .splineTo(new Vector2d(0, 0), Math.toRadians(0))
                .addDisplacementMarker(1,() ->{
                    L.Arm(0,1,0,1);
                    //Intake
                })
                .build();
        currentStage = Stage.MOVE_TO_BASKET;
        drive.followTrajectorySequence(MoveToBasket);
        while(opModeIsActive() && !isStopRequested()){

            switch (currentStage)
            {
                case IDLE:
                    break;
                case SEARCHING:
                    break;
                case MOVE_TO_BASKET:
                    if (!drive.isBusy()) {
                        currentStage = Stage.MOVE_TO_SAMPLE_1;
                        drive.followTrajectorySequence(MoveToSample1);
                    }
                    break;
                case MOVE_TO_SAMPLE_1:
                    break;
                case MOVE_TO_BASKET_1:
                    break;
                case MOVE_TO_SAMPLE_2:
                    break;
                case MOVE_TO_BASKET_2:
                    break;
                case MOVE_TO_SAMPLE_3:
                    break;
                case MOVE_TO_BASKET_3:
                    break;
            }

        }
    }
}
