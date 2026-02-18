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
package org.firstinspires.ftc.teamcode.Core;

@Autonomous (name = "PreProgrammedAuto", group = "Autonomous")
@Configurable // Panels
public class PedroAutonomous extends OpMode {
  private TelemetryManager panelsTelemetry; // Panels Telemetry instance
  public Follower follower; // Pedro Pathing follower instance
  private int pathState; // Current autonomous path state (state machine)
  private Paths paths; // Paths defined in the Paths class
  public double odoInches;
  double wheelR;
  int tpr;
  double cameraOffsetx, cameraOffsety;
  DcMotor LF, LB, RF, RB;
  DcMotor GunR, GunL;
  DcMotor Intake;
  DcMotor Lift;
  WebcamName Camera;

  public void initHardware(HardwareMap map) {
    LF = map.get(DcMotor.class, "FrontLeft");
    RF = map.get(DcMotor.class, "FrontRight");
    LB = map.get(DcMotor.class, "BackLeft");
    RB = map.get(DcMotor.class, "BackRight");
    GunR = map.get(DcMotor.class, "GunRight");
    GunL = map.get(DcMotor.class, "GunLeft");
    Intake = map.get(DcMotor.class, "Intake");
    Lift = map.get(DcMotor.class, "Lift");
    Camera = map.get(WebcamName.class, "Webcam");
  }
  void initializeCamera(WebcamName webcam) {
    tagProcessor = new AprilTagProcessor.Builder().build();
    visionPortal = new VisionPortal.Builder().setCamera(webcam).addProcessor(tagProcessor).build();
  }

  double tick2Inch(int ticks){
    return (wheelR * 2 * Math.PI) * (ticks / tpr)
  }
  public void odoWheel() {
    double odoInches = tick2Inch(backOdo.getCurrentPosition());
    follower.updateOdometry(odoInches);
    checkAprilTagCorrection();
  }
  public Pose computeFieldPose(AprilTagDetection tag) {
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

    double offsetX = CAMERA_OFFSET_X * Math.cos(robotHeading)
      - CAMERA_OFFSET_Y * Math.sin(robotHeading);

    double offsetY = CAMERA_OFFSET_X * Math.sin(robotHeading)
      + CAMERA_OFFSET_Y * Math.cos(robotHeading);

    robotX -= offsetX;
    robotY -= offsetY;

    return new Pose(robotX, robotY, robotHeading);
  }

  @Override
  public void init() {
    telemetry = PanelsTelemetry.INSTANCE.getTelemetry();

    follower = Constants.createFollower(hardwareMap);
    follower.setStartingPose(new Pose(72, 8, Math.toRadians(90)));

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

    public Paths(Follower follower) {
      Path1 = follower.pathBuilder()
        .addPath(
            new BezierLine(
              new Pose(71.500, 8.000),
              new Pose(133.280, 89.443)
              )
            )
        .setTangentHeadingInterpolation()
        .build();

      Path2 = follower.pathBuilder()
        .addPath(
            new BezierLine(
              new Pose(133.280, 89.443),
              new Pose(34.773, 65.134)
              )
            )
        .setTangentHeadingInterpolation()
        .build();
    }
  }
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
    switch (currentStage) {
      case MOVE_TO_CENTER:

        break;

      case PICKUP_SAMPLE:
        break;

      case MOVE_TO_DROPOFF:
        break;

      case DROP_SAMPlE:
        break;

      case WAIT:
        break;

      case PICKUP_SPECIMEN:
        break;

      case MOVE_TO_BAR:
        break;

      case DROP_SPECIMEN:
        break;

      case Reset:

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
