package org.firstinspires.ftc.teamcode;

import java.util.ArrayList;
import java.util.Arrays;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Libs.PlayOpMode;

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




@Autonomous (name = "DemoAuto", group = "Autonomous")
@Configurable // Panels
public class PedroAutonomous extends OpMode {
  private TelemetryManager telemetry; // Panels Telemetry instance
  public Follower follower; // Pedro Pathing follower instance
  private Paths paths; // Paths defined in the Paths class
  public Pose startingPose;
  AprilTagProcessor tagProcessor;
  VisionPortal visionPortal;
  List<AprilTagDetection> detections = new ArrayList<>();
  double cameraOffsetx = -5, cameraOffsety = 4;
  boolean Team, Start_Pos; // Red & Triangle = true; Blue & Goal = false
  double myPitch, myRoll, myYaw;
  double fieldHeading;
  Pose currentPose;
  aTags fieldTag;

  enum Stage {
    MOVE_TO_CENTER,
    MOVE_TO_COLLECT,
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
  public int liftZero;

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
    public PathChain Center;
    public PathChain Collect;
    public PathChain Collect2;
    public PathChain ReturnStart;

    public Paths(Follower follower) {
      Center = follower.pathBuilder()
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
            new Pose(4.000, 45.423)
            )
          )
          .setTangentHeadingInterpolation()
          .setReversed()
          .build();

      Collect2 = follower.pathBuilder()
          .addPath(
            new BezierLine(
              new Pose(4.000, 45.423),
            new Pose(4.000, 45.423)
            )
          )
          .setLinearHeadingInterpolation(Math.toRadians(21), Math.toRadians(0))
          .build();

      ReturnStart = follower.pathBuilder()
          .addPath(
            new BezierLine(
              new Pose(4.000, 45.423),
            new Pose(38.658, 33.342)
            )
          )
          .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
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


  public void initHardware(HardwareMap map) {
    LF = map.get(DcMotor.class, "lf");
    RF = map.get(DcMotor.class, "rf");
    LB = map.get(DcMotor.class, "lr");
    RB = map.get(DcMotor.class, "rr");
    GunR = map.get(DcMotor.class, "FireR");
    GunL = map.get(DcMotor.class, "FireL");
    Intake = map.get(DcMotor.class, "intake");
    Lift = map.get(DcMotor.class, "screw");
    Camera = map.get(WebcamName.class, "Webcam");
  }
  void initializeOdoMotor(DcMotor motor) {
    motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
  }
  void initializeCamera(WebcamName webcam) {
    tagProcessor = new AprilTagProcessor.Builder().setDrawTagID(true).setDrawTagOutline(true).setDrawAxes(true).setDrawCubeProjection(true).setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES).build();
    visionPortal = new VisionPortal.Builder().setCamera(webcam).addProcessor(tagProcessor).build();
  }

  public double lerpIt(double value, double factor, double threshold) {
    return lerp(value * factor, value, threshold);
  }

  public Pose computeFieldPoseFromTag(AprilTagDetection tag) {

    if (tag.id == aTagRed.ID) {
      fieldTag = new aTags(aTagRed.ID, aTagRed.x, aTagRed.y);
    } else if (tag.id == aTagBlue.ID) {
      fieldTag = new aTags(aTagBlue.ID, aTagBlue.x, aTagBlue.y);
    }
    Pose relative = computeCurrentFieldPose(tag);

    // Convert to field coordinates
    double fieldX = relative.getX() + fieldTag.x;
    double fieldY = relative.getY() + fieldTag.y;
    fieldHeading = relative.getHeading();

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

  public AprilTagDetection GetTagBySpecificID(int ID){

    for(AprilTagDetection detection : detections){
      if(detection.id ==  ID){
        return detection;
      }
    }
    return null;
  }
  public  List<AprilTagDetection> getDectectedTags(){
    return detections;
  }




    public Pose computeCurrentFieldPose(AprilTagDetection tag) {
      double X = tag.robotPose.getPosition().x;
      double Y = tag.robotPose.getPosition().y;

      myPitch = tag.robotPose.getOrientation().getPitch(AngleUnit.DEGREES);
      myRoll = tag.robotPose.getOrientation().getRoll(AngleUnit.DEGREES);
      myYaw = tag.robotPose.getOrientation().getYaw(AngleUnit.DEGREES);
      return  new Pose(X, Y, fieldHeading);
    }




  public void moveToCenter() {
    follower.followPath(paths.Center);

  }
  public void moveToCollect() {
    follower.followPath(paths.Collect);
    follower.followPath(paths.Collect2);
  }
  public void pickupSample() {
    return;
  }
  public void returnToStart() {
    follower.followPath(paths.ReturnStart);
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
      startingPose = visionPose;
      //telemetry.addData("Starting Pose", "X: %.2f, Y: %.2f, Heading: %.2f",visionPose.getX(), visionPose.getY(), visionPose.getHeading());
    } else {

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
    currentStage = Stage.MOVE_TO_COLLECT;

    telemetry.addData("Status", "Initialized");
    telemetry.update();
  }

  @Override
  public void loop() {
    follower.update(); // Update Pedro Pathing

    detections = tagProcessor.getDetections();

    if (!detections.isEmpty()) {
      currentPose = computeFieldPoseFromTag(detections.get(0));
      follower.setPose(currentPose);
    }

    // Log values to Panels and Driver Station
    telemetry.addData("Path State: ", currentStage);
    telemetry.addData("X: ", currentPose.getX());
    telemetry.addData("Y: ", currentPose.getY());
    telemetry.addData("Heading: ", currentPose.getHeading());
    telemetry.update();
    run();
  }
  Stage currentStage = Stage.IDLE;



  protected void run()  {
    if (follower.isBusy()) {
      telemetry.addData("Bot is moving...", "");
      telemetry.update();
    } else {
      switch(currentStage) {
        case MOVE_TO_CENTER:
          moveToCenter();
          currentStage = Stage.MOVE_TO_COLLECT;
          break;
        case MOVE_TO_COLLECT:
          moveToCollect();
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
          break;
      }
    }

  }
}
