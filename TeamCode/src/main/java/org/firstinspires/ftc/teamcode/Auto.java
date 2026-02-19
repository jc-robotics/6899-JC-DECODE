package org.firstinspires.ftc.teamcode;

import java.util.Arrays;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Libs.PlayOpMode;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import java.util.List;

import static org.firstinspires.ftc.teamcode.Libs.JCLibs.lerp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Libs.Classes.Vector3;
import org.firstinspires.ftc.teamcode.Libs.PlayOpMode;

import java.util.Objects;




@Autonomous (name = "PreProgrammedAuto", group = "Autonomous")
@Configurable // Panels
public class PedroAutonomous extends PlayOpMode {
  private TelemetryManager panelsTelemetry; // Panels Telemetry instance
  public Follower follower; // Pedro Pathing follower instance
  private int pathState; // Current autonomous path state (state machine)
  private Paths paths; // Paths defined in the Paths class
  public double odoInches;
  AprilTagProcessor tagProcessor;
  VisionPortal visionPortal;
  double cameraOffsetx = -5, cameraOffsety = 4;
  boolean Team, Start_Pos; // Red & Triangle = true; Blue & Goal = false
  enum Stage {
    MOVE_TO_CENTER,
    PICKUP_SAMPLE,
    MOVE_TO_COLLECT,
    MOVE_TO_SHOOT,
    RETURN_TO_START,
    WAIT,
    RESET,
    IDLE
  }
  //CORE VARIABLES
  DcMotor LF, LB, RF, RB;
  DcMotor GunR, GunL;
  DcMotor Intake;
  DcMotor Lift;
  WebcamName Camera;
  public double liftZero;

  public static class aTags{
    public int ID;
    public double x,y;
    public aTags(int ID, double x, double y) {
      this.ID = ID;
      this.x = x;
      this.y = y;
    }
  }
  public aTags aTagRed = new aTags(24, 55.63, 58.34);
  public aTags aTagBlue = new aTags(20, -55.63, 58.34);
  public static class Paths {
    public PathChain Centre;
    public PathChain Collect;
    public PathChain Shoot;
    public PathChain ReturnStart;

    public Paths(Follower follower) {
      Centre = follower.pathBuilder()
        .addPath(
            new BezierLine(
              follower.getPose(), 
              new Pose(0.0, 0.0)
              )
            )
        .setLinearHeadingInterpolation(follower.getPose().getHeading(), Math.toRadians(90)) // face forward
        .build();


      Collect = follower.pathBuilder()
        .addPath(
            new BezierLine(
              follower.getPose(),
              new Pose(72.000, 84.000)
              )
            )
        .setLinearHeadingInterpolation(follower.getPose().getHeading(), Math.toRadians(0))
        .build();

      Shoot = follower.pathBuilder()
        .addPath(
            new BezierLine(
              follower.getPose(),
              new Pose(18.000, 84.000)
              )
            )
        .setLinearHeadingInterpolation(follower.getPose().getHeading(), Math.toRadians(0))
        .build();

      ReturnStart = follower.pathBuilder()
        .addPath(
            new BezierLine(
              follower.getPose(),
              new Pose(72.000, 72.000)
              )
            )
        .setTangentHeadingInterpolation()
        .build();
    }
  }

  //Motor Setup
  void initializeMotor(DcMotor motor,  DcMotor.Direction direction) {
    motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    motor.setDirection(direction);
  }
  void initializeEncoderMotor(DcMotor motor,  DcMotor.Direction direction) {
    motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    motor.setDirection(direction);
  }
  void moveLiftBackToInitialPosition(DcMotor motor) {
    motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    motor.setTargetPosition(liftZero);

    motor.setPower(0.5);
    while (motor.isBusy()) {
      telemetry.addData("Lift position: ",motor.getCurrentPosition());
    }
    motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);  // Set motor mode to move to target position
  }

  //Moves the 4 mechanum wheels in a car formation
  public void DTMove(double x, double y, double turn) {
    LF.setPower(y+x+turn);
    LB.setPower(y-x-turn);
    RF.setPower(y-x+turn);
    RB.setPower(y+x-turn);
  }

  @Override
  public void initHardware(HardwareMap map) {
    LF = map.get(DcMotor.class, "FrontLeft");
    RF = map.get(DcMotor.class, "FrontRight");
    LB = map.get(DcMotor.class, "BackLeft");
    RB = map.get(DcMotor.class, "BackRight");
    Odo = map.get(DcMotor.class, "OdoWheel");
    GunR = map.get(DcMotor.class, "GunRight");
    GunL = map.get(DcMotor.class, "GunLeft");
    Intake = map.get(DcMotor.class, "Intake");
    Lift = map.get(DcMotor.class, "Lift");
    Camera = map.get(WebcamName.class, "Webcam");
  }
  void initializeOdoMotor(DcMotor motor) {
    motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
  }
  void initializeCamera(WebcamName webcam) {
    tagProcessor = new AprilTagProcessor.Builder().build();
    visionPortal = new VisionPortal.Builder().setCamera(webcam).addProcessor(tagProcessor).build();
  }

  public double lerpIt(double value, double factor, double threshold) {
    return lerp(value * factor, value, threshold);
  }

  public Pose computeFieldPoseFromTag(AprilTagDetection tag) {
    aTags fieldTag;
    if (tag.id == aTagRed.ID) {
      fieldTag = aTagRed;
    } else if (tag.id == aTagBlue.ID) {
      fieldTag = aTagBlue;
    } else {
      return null; // Unknown tag
    }

    Pose relative = computeCurrentFieldPose(tag);

    // Convert to field coordinates
    double fieldX = relative.getX() + fieldTag.x;
    double fieldY = relative.getY() + fieldTag.y;
    double fieldHeading = relative.getHeading();

    return new Pose(fieldX, fieldY, fieldHeading);
  }
  public void checkAprilTagCorrection() {
    List<AprilTagDetection> detections = tagProcessor.getDetections();

    if (detections.size() > 0 && follower.isBusy() == false) {

      AprilTagDetection tag = detections.get(0);
      Pose visionPose = computeCurrentFieldPose(tag);

      follower.setPose(visionPose);
    }
  }

  public void moveToCenter() {
    follower.follow(paths.Centre);
  }
  public void moveToCollect() {
    follower.follow(paths.Collect);
  }
  public void moveToShoot() {
    follower.follow(paths.Shoot);
  }
  public void pickupSample() {
    return;
  }
  public void returnToStart() {
    follower.follow(paths.ReturnStart);
  }
  @Override
  public void init() {
    telemetry = PanelsTelemetry.INSTANCE.getTelemetry();

    initHardware(hardwareMap);
    initializeMotor(LB, DcMotor.Direction.REVERSE);
    initializeMotor(LF, DcMotor.Direction.FORWARD);
    initializeMotor(RF, DcMotor.Direction.FORWARD);
    initializeMotor(RB, DcMotor.Direction.REVERSE);
    initializeMotor(GunR, DcMotor.Direction.REVERSE);
    initializeMotor(GunL, DcMotor.Direction.FORWARD);
    initializeMotor(Intake, DcMotor.Direction.FORWARD);
    initializeEncoderMotor(Lift, DcMotor.Direction.FORWARD);
    initializeCamera(Camera);
    follower = Constants.createFollower(hardwareMap);
    List<AprilTagDetection> detections = tagProcessor.getDetections();
    if (!detections.isEmpty()) {
      AprilTagDetection tag = detections.get(0);
      Pose visionPose = computeCurrentFieldPose(tag);
      follower.setStartingPose(visionPose);
      telemetry.addData("Starting Pose", "X: %.2f, Y: %.2f, Heading: %.2f",
          visionPose.getX(), visionPose.getY(), visionPose.getHeading());
    } else {
      Pose startingPose;

      if (Start_Pos) {
        if (Team) {
          startingPose = new Pose(84, 8, Math.toRadians(90));
        } else {
          startingPose = new Pose(60, 8, Math.toRadians(90));
        }
      } else {
        if (Team) {
          startingPose = new Pose(121, 121, Math.toRadians(225));
        } else {
          startingPose = new Pose(24, 120, Math.toRadians(315));
        }
      }

      follower.setStartingPose(startingPose);

      telemetry.addData("Status", "No AprilTag detected. Using default pose guestimate");
    }

    paths = new Paths(follower); // Build paths

    telemetry.addData("Status", "Initialized");
    telemetry.update();
  }

  @Override
  public void loop() {
    follower.update(); // Update Pedro Pathing

    pathState = autonomousPathUpdate(); // Update autonomous state machine
    List<AprilTagDetection> detections = tagProcessor.getDetections();
    if (!detections.isEmpty()) {
      currentPose = computeFieldPoseFromTag(detections.get(0));
      follower.setPose(currentPose);
    }

    // Log values to Panels and Driver Station
    telemetry.addData("Path State: ", pathState);
    telemetry.addData("X: ", currentPose.getX());
    telemetry.addData("Y: ", currentPose.getY());
    telemetry.addData("Heading: ", currentPose.getHeading());
    telemetry.update();
  }
  Stage currentStage = Stage.IDLE;
  @Override
  protected void preInitialize() {
    isTeleOp = false;
  }
  @Override
  protected void run(double dt) throws InterruptedException {
    if (follower.isFollowing()) {
      telemetry.addData("Bot is moving...", "");
      telemetry.update();
    } else {
      switch(currentStage) {
        case MOVE_TO_CENTER:
          moveToCenter();
          currentStage = Stage.PICKUP_SAMPLE;
          break;
        case MOVE_TO_COLLECT:
          moveToCollect();
          currentStage = Stage.MOVE_TO_SHOOT;
          break;
        case PICKUP_SAMPLE:
          pickupSample();
          currentStage = Stage.MOVE_TO_COLLECT;
          break;
        case MOVE_TO_SHOOT:
          moveToShoot();
          currentStage = Stage.RETURN_TO_START;
          break;
        case RETURN_TO_START:
          returnToStart();
          currentStage = Stage.IDLE;
          break;
        case WAIT:
          break;
        case RESET:
          currentStage = Stage.IDLE;
          break;
        case IDLE:
          telemetry.addData("Bot Run:", "Complete");
          currentStage = Stage.PICKUP_SAMPLE;
          break;
      }
    }

  }

  public int autonomousPathUpdate() {
    // Add your state machine Here
    // Access paths with paths.pathName
    // Refer to the Pedro Pathing Docs (Auto Example) for an example state machine
    return 0;
  }
}
