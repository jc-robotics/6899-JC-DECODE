package org.firstinspires.ftc.teamcode.Auto;

import java.util.Arrays;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Libs.PlayOpMode;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

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
import org.firstinspires.ftc.teamcode.Core;

@Autonomous (name = "PreProgrammedAuto", group = "Autonomous")
@Configurable // Panels
public class PedroAutonomous extends PlayOpMode {
  private TelemetryManager panelsTelemetry; // Panels Telemetry instance
  public Follower follower; // Pedro Pathing follower instance
  private int pathState; // Current autonomous path state (state machine)
  private Paths paths; // Paths defined in the Paths class
  public double odoInches;
  Pose currentPose;
  AprilTagProcessor tagProcessor;
  VisionPortal visionPortal;
  double cameraOffsetx = -5, cameraOffsety = 4;
  DcMotor LF, LB, RF, RB;
  DcMotor Odo;
  DcMotor GunR, GunL;
  DcMotor Intake;
  DcMotor Lift;
  WebcamName Camera;
  boolean Team, Start_Pos; // Red & Triangle = true; Blue & Goal = false
  enum Stage {
    MOVE_TO_CENTER,
    PICKUP_SAMPLE,
    MOVE_TO_COLLECT,
    DROP_SAMPlE,
    MOVE_TO_SHOOT,
    RETURN_TO_START,
    WAIT,
    RESET,
    IDLE
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

  public Pose computeCurrentFieldPose(AprilTagDetection tag) {
    double tagFieldX = tag.fieldX;
    double tagFieldY = tag.fieldY;
    double tagFieldHeading = Math.toRadians(tag.fieldYaw);

    double camX = tag.relativeX;
    double camY = tag.relativeY;

    double cos = Math.cos(tagFieldHeading);
    double sin = Math.sin(tagFieldHeading);

    double fieldRelX = camX * cos - camY * sin;
    double fieldRelY = camX * sin + camY * cos;

    double robotX = tagFieldX - fieldRelX;
    double robotY = tagFieldY - fieldRelY;

    double robotHeading = tagFieldHeading - Math.toRadians(tag.relativeYaw);

    double offsetX = cameraOffsetx * Math.cos(robotHeading) - cameraOffsety * Math.sin(robotHeading);

    double offsetY = cameraOffsetx * Math.sin(robotHeading) + cameraOffsety * Math.cos(robotHeading);

    robotX -= offsetX;
    robotY -= offsetY;

    return new Pose(robotX, robotY, robotHeading);
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
  }
  public void pickupSample() {
  }
  public void moveToCollect() {
  }
  public void dropSample() {
  }
  public void moveToShoot() {
  }
  public void returnToStart() {
  }
  public void wait() {
  }
  public void reset() {
  }
  public void idle() {
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
    if (!detections.isEmpty()) {
        AprilTagDetection tag = detections.get(0);
        Pose visionPose = computeCurrentFieldPose(tag);
        follower.setStartingPose(visionPose);
        telemetry.addData("Starting Pose", "X: %.2f, Y: %.2f, Heading: %.2f",
                           visionPose.getX(), visionPose.getY(), visionPose.getHeading());
    } else {
        Pose defaultPose = new Pose(72, 8, Math.toRadians(90)); // Example default pose
        follower.setStartingPose(defaultPose);

        telemetry.addData("Status", "No AprilTag detected. Using default pose.");
    }

    paths = new Paths(follower); // Build paths

    telemetry.addData("Status", "Initialized");
    telemetry.update();
  }

  @Override
  public void loop() {
    follower.update(); // Update Pedro Pathing

    pathState = autonomousPathUpdate(); // Update autonomous state machine

    // Log values to Panels and Driver Station
    telemetry.addData("Path State: ", pathState);
    telemetry.addData("X: ", follower.getPose().getX());
    telemetry.addData("Y: ", follower.getPose().getY());
    telemetry.addData("Heading: ", follower.getPose().getHeading());
    telemetry.update();
  }

  public static class Paths {
    public PathChain Path1;
    public PathChain Path2;
    public PathChain Path3;
    public PathChain Path4;
    public PathChain Path5;

    public Paths(Follower follower) {
      Path1 = follower.pathBuilder()
        .addPath(
            new BezierLine(
              new Pose(71.500, 8.000),
              new Pose(72.000, 72.000)
              )
            )
        .setLinearHeadingInterpolation(Math.toRadians(null), Math.toRadians(90))
        .build();

      Path2 = follower.pathBuilder()
        .addPath(
            new BezierLine(
              new Pose(72.000, 72.000),
              new Pose(72.000, 84.000)
              )
            )
        .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))
        .build();

      Path3 = follower.pathBuilder()
        .addPath(
            new BezierLine(
              new Pose(72.000, 84.000),
              new Pose(18.000, 84.000)
              )
            )
        .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
        .build();

      Path4 = follower.pathBuilder()
        .addPath(
            new BezierLine(
              new Pose(18.000, 84.000),
              new Pose(72.000, 72.000)
              )
            )
        .setTangentHeadingInterpolation()
        .build();

      Path5 = follower.pathBuilder()
        .addPath(
            new BezierLine(
              new Pose(72.000, 72.000),
              new Pose(108.000, 108.000)
              )
            )
        .setTangentHeadingInterpolation()
        .build();
    }
  }
  Stage currentStage = Stage.IDLE;
  @Override
  protected void preInitilize() {
    isTeleOp = false;
  }
  @Override
  protected void run(double dt) throws InterruptedException {
    switch (currentStage) {
      case MOVE_TO_CENTER:
        break;
      case PICKUP_SAMPLE:
        break;
      case MOVE_TO_COLLECT:
        break;
      case DROP_SAMPlE:
        break;
      case MOVE_TO_SHOOT:
        break;
      case RETURN_TO_START:
        break;
      case WAIT:
        break;
      case RESET:
        break;
      case IDLE:
        break;
      case RESET:
        return;
    }

  }

  public int autonomousPathUpdate() {
    // Add your state machine Here
    // Access paths with paths.pathName
    // Refer to the Pedro Pathing Docs (Auto Example) for an example state machine
    return 0;
  }
}
