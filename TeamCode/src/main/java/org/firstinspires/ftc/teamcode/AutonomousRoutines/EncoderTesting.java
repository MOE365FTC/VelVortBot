/* Copyright (c) 2015 Qualcomm Technologies Inc

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

package org.firstinspires.ftc.teamcode.AutonomousRoutines;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Encoder Tests", group = "Auton")
//@Disabled                            // Comment this out to add to the opmode list
public class EncoderTesting extends LinearOpMode {
    ElapsedTime runtime = new ElapsedTime();
    ModernRoboticsI2cGyro gyro;
    DcMotor FL;
    DcMotor FR;
    DcMotor BL;
    DcMotor BR;
    DcMotor Shooter;

    double ticsPerInch = 38.5;
    double strafeTicsPerInch = 53;

    double leftPos = .8;
    double rightPos = .3;

    //sets default Powers
    double shooterPower = 1;
    double harvesterPower = 1;
    int shooterRot = 1625;
    @Override
    public void runOpMode() throws InterruptedException {
        gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyro");
        FL = hardwareMap.dcMotor.get("FL");
        FR = hardwareMap.dcMotor.get("FR");
        BL = hardwareMap.dcMotor.get("BL");
        BR = hardwareMap.dcMotor.get("BR");
        Shooter = hardwareMap.dcMotor.get("Sh");
        Shooter.setDirection(DcMotor.Direction.REVERSE);
        FR.setDirection(DcMotor.Direction.REVERSE);
        BR.setDirection(DcMotor.Direction.REVERSE);
        waitForStart();
        startForward(0.5);
        while(opModeIsActive()){
            telemetry.addData("FR",FR.getCurrentPosition());
            telemetry.addData("FL",FL.getCurrentPosition());
            telemetry.addData("BR",BR.getCurrentPosition());
            telemetry.addData("BL",BL.getCurrentPosition());
            telemetry.addData("SH",Shooter.getCurrentPosition());
            telemetry.update();
        }
    }

    public void moveRightInches(double inches, double speed){
        double turnSpeed = 0.05;
        double target = inches * strafeTicsPerInch;
        int startHeading = gyro.getIntegratedZValue();
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        startRight(speed);
        while(opModeIsActive() && FL.getCurrentPosition() < target){
            if(gyro.getIntegratedZValue() > startHeading + 5) {//If we've turned > 5 degrees CCW, turn CW
                FR.setPower(FR.getPower() - turnSpeed);
                BR.setPower(BR.getPower() - turnSpeed);
                FL.setPower(FL.getPower() + turnSpeed);
                BL.setPower(BL.getPower() + turnSpeed);
            }
            else if(gyro.getIntegratedZValue() < startHeading - 5){//If we've turned > 5 degrees CW, turn CCW
                FR.setPower(FR.getPower() + turnSpeed);
                BR.setPower(BR.getPower() + turnSpeed);
                FL.setPower(FL.getPower() - turnSpeed);
                BL.setPower(BL.getPower() - turnSpeed);
            }
            else
                startRight(speed);
            idle();
        }
        stopDrive();
    }

    public void pointToOneEighty() throws InterruptedException{//We sit at negative 180 as our "zero" heading
        double angle = gyro.getIntegratedZValue();
        if (angle > -180) {//if pointed left, do a right turn
            telemetry.addData("ANGLE",angle);
            rightTurn(Math.abs(angle + 180), 0.25);
            Thread.sleep(100);
        }
        else {//if pointed right, do a left turn
            leftTurn(Math.abs(angle + 180), 0.25);
            Thread.sleep(100);
        }
    }

    public void startForward(double speed){
        FR.setPower(speed);
        FL.setPower(speed);
        BR.setPower(speed);
        BL.setPower(speed);
    }

    public void startBackward(double speed){
        FR.setPower(-speed);
        FL.setPower(-speed);
        BR.setPower(-speed);
        BL.setPower(-speed);
    }

    public void startRight(double speed){
        FR.setPower(-speed);
        FL.setPower(speed);
        BR.setPower(speed);
        BL.setPower(-speed);
    }

    public void startLeft(double speed){
        FR.setPower(speed);
        FL.setPower(-speed);
        BR.setPower(-speed);
        BL.setPower(speed);
    }

    public void stopDrive(){
        FR.setPower(0);
        FL.setPower(0);
        BR.setPower(0);
        BL.setPower(0);
    }

    public void rightTurn(double angle, double speed){
        double target = gyro.getIntegratedZValue() - angle;
        FR.setPower(-speed);
        FL.setPower(speed);
        BR.setPower(-speed);
        BL.setPower(speed);
        while(opModeIsActive() && gyro.getIntegratedZValue() > target){
            idle();
        }
        stopDrive();
    }

    public void leftTurn(double angle, double speed){
        double start = gyro.getHeading();
        double target = gyro.getIntegratedZValue() + angle;
        FR.setPower(speed);
        FL.setPower(-speed);
        BR.setPower(speed);
        BL.setPower(-speed);
        while(opModeIsActive() && gyro.getIntegratedZValue() < target){
            idle();
        }
        stopDrive();
    }

    public void moveLeftInches(double inches, double speed){
        double target = -inches * strafeTicsPerInch;
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        startLeft(speed);
        while(opModeIsActive() && FL.getCurrentPosition() > target){
            idle();
        }
        stopDrive();
    }

    public void moveForwardInches(double inches, double speed){
        double target = inches * ticsPerInch;
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        startForward(speed);
        while(opModeIsActive() && FL.getCurrentPosition() < target){
            idle();
        }
        stopDrive();
    }

    public void moveBackwardInches(double inches, double speed){
        double target = inches*ticsPerInch;
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        startBackward(speed);
        while(opModeIsActive() && FL.getCurrentPosition() > -target){
            idle();
        }
        stopDrive();
    }

}