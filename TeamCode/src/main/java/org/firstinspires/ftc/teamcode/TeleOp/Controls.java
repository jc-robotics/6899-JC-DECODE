package org.firstinspires.ftc.teamcode.TeleOp;

import org.firstinspires.ftc.teamcode.Core.DriveTrain;
import org.firstinspires.ftc.teamcode.Core.Core;

public class Controls extends DriveTrain {

  public void Controlling() {

    LSx = Math.abs(gamepad1.left_stick_x) > 0.05 ? gamepad1.left_stick_x : 0;
    LSy = Math.abs(gamepad1.left_stick_y) > 0.05 ? gamepad1.left_stick_y : 0;
    RSx = Math.abs(gamepad1.right_stick_x) > 0.05 ? gamepad1.right_stick_x : 0;
    RSy = Math.abs(gamepad1.right_stick_y) > 0.05 ? gamepad1.right_stick_y : 0;
    DTMove(LSx_lerped, LSy_lerped, RSx_lerped);

    Intake = LSy_lerped;

    if (current.a && !previous.a) {
      GunL = -0.3;
      GunR = -0.3;
    }
    if (RTrigger != 0) {
      GunR = GradualGearShift(RTrigger);
      GunL = GradualGearShift(RTrigger);
    }
    if (!current.a && previous.a) {
      Intake = .01;

    }
  }
}
