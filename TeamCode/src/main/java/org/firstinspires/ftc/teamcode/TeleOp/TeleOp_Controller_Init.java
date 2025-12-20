package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.robotcore.hardware.DcMotor;

package org.firstinspires.ftc.Initialization;


void Controller(Gamepad gamepaD){

        //Thump sticks
        LSy = gamepaD.left_stick_y;
        LSx = gamepaD.left_stick_x;
        RSx = -gamepaD.right_stick_x;
        RSy = -gamepaD.right_stick_y;

        //LERP the Thumb stick input
        LSx_lerped = lerp(LSx*.588, LSx, 0.8)
        LSy_lerped = lerp(LSy*.588, LSy, 0.8)
        RSx_lerped = lerp(RSx*.588, RSx, 0.8)
        RSy_lerped = lerp(RSy*.588, RSy, 0.8)

        //Dpad
        up_d = gamepaD.dpad_up;
        down_d = gamepaD.dpad_down;
        left_d = gamepaD.dpad_left;
        right_d = gamepaD.dpad_right;

        //Face buttons
        face_a = gamepaD.a;
        face_b = gamepaD.b;
        face_x = gamepaD.x;
        face_y = gamepaD.y;

        //Top buttons
        RTrigger = gamepaD.right_trigger;
        LTrigger = gamepaD.left_trigger;
        RBumper = gamepaD.right_bumper;
        LBumper = gamepaD.left_bumper;


        previous.copy(current);


        // Store the gamepad values from this loop iteration in
        // currentGamepad1/2 to be used for the entirety of this loop iteration.
        // This prevents the gamepad values from changing between being
        // used and stored in previousGamepad1/2.
        current.copy(gamepaD);

    }
