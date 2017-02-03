/* Copyright (c) 2014 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "New Bot TeleOp", group = "TeleOp")

 /* TeleOp Mode
 * <p>
 * Enables control of the robot via the gamepad
 */
public class NewBotTeleOp extends OpMode {
    //Gyro thread stuff

    GyroSensor gyro;
    DcMotor FL;
    DcMotor FR;
    DcMotor BL;
    DcMotor BR;
    DcMotor Shooter;
    DcMotor Harvester;
    DcMotor lights;
    Servo buttonPusher;
    double rightPos = .3;

    //sets default Powers
    double shooterPower = 1;
    double harvesterPower = 1;
    int shooterRot = 1000;

    boolean left_stick_pressed = false;
    boolean robotCentric = false;
    /**
     * Constructor
     */
    public NewBotTeleOp()
    {

    }

    /*
     * Code to run when the op mode is initialized goes here
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#init()
     */
    @Override
    public void init()
    {
        Shooter = hardwareMap.dcMotor.get("Sh");
        Harvester = hardwareMap.dcMotor.get("Ha");
        gyro = hardwareMap.gyroSensor.get("gyro");
        FL = hardwareMap.dcMotor.get("FL");
        FR = hardwareMap.dcMotor.get("FR");
        BL = hardwareMap.dcMotor.get("BL");
        BR = hardwareMap.dcMotor.get("BR");
        lights = hardwareMap.dcMotor.get("lights");
        buttonPusher = hardwareMap.servo.get("beaconPush");
        FR.setDirection(DcMotor.Direction.REVERSE);
        BR.setDirection(DcMotor.Direction.REVERSE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Harvester.setDirection(DcMotor.Direction.REVERSE);
        Shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        buttonPusher.setPosition(rightPos);
        gyro.calibrate();
        while(gyro.isCalibrating()){}
    }


    @Override
    public void loop()
    {
        double heading = Math.toRadians(gyro.getHeading());
        telemetry.addData("heading",heading);
        telemetry.update();
        double rawDirectionY = -scaleInput(gamepad1.left_stick_y);
        double rawDirectionX = scaleInput(gamepad1.left_stick_x);
        double turnX = gamepad1.right_stick_x;
        double directionX = rawDirectionX * Math.cos(heading) - rawDirectionY * Math.sin(heading);
        double directionY = rawDirectionX * Math.sin(heading) + rawDirectionY * Math.cos(heading);

        // clip the right/left values so that the values never exceed +/- 1
        directionY = Range.clip(directionY, -1, 1);
        directionX = Range.clip(directionX, -1, 1);
        turnX = Range.clip(turnX, -1, 1);
        double FRP;
        double FLP;
        double BRP;
        double BLP;
        double lightsPower;
        //Mecanum algorithm
        if(!robotCentric) {
            FRP = directionY - directionX - turnX;
            FLP = directionY + directionX + turnX;
            BRP = directionY + directionX - turnX;
            BLP = directionY - directionX + turnX;
            lightsPower = 0;
        }
        else{
            FRP = rawDirectionY - rawDirectionX - turnX;
            FLP = rawDirectionY + rawDirectionX + turnX;
            BRP = rawDirectionY + rawDirectionX - turnX;
            BLP = rawDirectionY - rawDirectionX + turnX;
            lightsPower = 1;
        }

        // clip the right/left values so that the values never exceed +/- 1
        FRP = scaleInput(Range.clip(FRP,-1,1));
        FLP = scaleInput(Range.clip(FLP,-1,1));
        BRP = scaleInput(Range.clip(BRP,-1,1));
        BLP = scaleInput(Range.clip(BLP,-1,1));

        // write the values to the motors
        FL.setPower(FLP);
        FR.setPower(FRP);
        BR.setPower(BRP);
        BL.setPower(BLP);
        lights.setPower(lightsPower);

        //toggle field centric
        if(gamepad1.left_stick_button && !left_stick_pressed){
            left_stick_pressed = true;
            robotCentric = !robotCentric;
        }
        else if(!gamepad1.left_stick_button){
            left_stick_pressed = false;
        }

        //resets gyro value
        if(gamepad1.y)
            gyro.resetZAxisIntegrator();
        //shooter does about full rotation
        if(gamepad1.right_trigger > .5) {
            int rotationTarget = Shooter.getCurrentPosition() + shooterRot;
            while ((Shooter.getCurrentPosition() < rotationTarget)) {
                Harvester.setPower(0);
                Shooter.setPower(shooterPower);
            }
            Shooter.setPower(0);
        }
        //shooter rotates as button is held down
        else if (gamepad1.x)
            Shooter.setPower(-1);
        else if (gamepad1.b)
            Shooter.setPower(1);
        else
            Shooter.setPower(0);
        //harvester rotates as button is held down
        if(gamepad2.a)
            Harvester.setPower(harvesterPower);
        else if(gamepad2.y)
            Harvester.setPower(-harvesterPower);
        else
            Harvester.setPower(0);
    }

    /*
     * Code to run when the op mode is first disabled goes here
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#stop()
     */
    @Override
    public void stop()
    {
    }

    /*
     * This method scales the joystick input so for low joystick values, the
     * scaled value is less than linear.  This is to make it easier to drive
     * the robot more precisely at slower speeds.
     */
    double scaleInput(double dVal)
    {
        double[] scaleArray = { 0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00 };

        // get the corresponding index for the scaleInput array.
        int index = (int) (dVal * 16.0);
        if (index < 0)
        {
            index = -index;
        }
        else if (index > 16)
        {
            index = 16;
        }

        double dScale = 0.0;
        if (dVal < 0)
        {
            dScale = -scaleArray[index];
        } else
        {
            dScale = scaleArray[index];
        }

        return dScale;
    }

}
