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

package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "AutonomousMove", group = "Auton")
//@Disabled                            // Comment this out to add to the opmode list
public class AutonomousMove extends LinearOpMode {
    ElapsedTime runtime = new ElapsedTime();
    DcMotor FR,FL,BR,BL,Shooter,Harvester;
    ColorSensor beaconSensor;
    //ColorSensor lineSensor
    float lineHsvValues[] = {0F,0F,0F};
    float beaconHsvValues[] = {0F,0F,0F};

    double shooterPower = 0.8;
    double harvesterPower = 0.6;
    @Override
    public void runOpMode() throws InterruptedException {

        // get a reference to our DeviceInterfaceModule object.
        FR = hardwareMap.dcMotor.get("FR");
        FL = hardwareMap.dcMotor.get("FL");
        BR = hardwareMap.dcMotor.get("BR");
        BL = hardwareMap.dcMotor.get("BL");
        Shooter = hardwareMap.dcMotor.get("Sh");
        Harvester = hardwareMap.dcMotor.get("Ha");
        //lineSensor = hardwareMap.colorSensor.get("lineSense");
        beaconSensor = hardwareMap.colorSensor.get("beaconSense");
        //lineSensor.setI2cAddress(I2cAddr.create8bit(0x4c));
        beaconSensor.setI2cAddress(I2cAddr.create8bit(0x3c));

        Shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        Harvester.setDirection(DcMotorSimple.Direction.REVERSE);
        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        BR.setDirection(DcMotorSimple.Direction.REVERSE);
        Shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();
        Shooter.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // set the digital channel to output mode.
        while (!isStarted()) {
            beaconSensor.enableLed(true);
            //lineSensor.enableLed(true);
        }
        /**
         while(opModeIsActive()) {
         //telemetry.addData("line", lineSensor.argb());
         telemetry.addData("beacon", beaconSensor.argb());
         telemetry.update();
         }**/
        Shooter.setTargetPosition(1650);
        Shooter.setPower(shooterPower);
        while(opModeIsActive() && Shooter.isBusy()){
            telemetry.addData("pos",Shooter.getCurrentPosition());
            telemetry.update();
            idle();
        }
        Shooter.setPower(0);
        runtime.reset();
        while(opModeIsActive() && (runtime.seconds() < 2.0)){
            Harvester.setPower(harvesterPower);
            idle();
        }
        Harvester.setPower(0);
        Shooter.setTargetPosition(3300);
        Shooter.setPower(shooterPower);
        while(opModeIsActive() && Shooter.isBusy()){
            telemetry.addData("pos",Shooter.getCurrentPosition());
            telemetry.update();
            idle();
        }
        Shooter.setPower(0);
        runtime.reset();
        while(opModeIsActive() && runtime.seconds() < 5){
            idle();
        }
        runtime.reset();
        //change                                     this one
        while(opModeIsActive() && runtime.seconds() < 4.5){
            startRight(0.6);
            idle();
        }
        stopDrive();
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

    public void stopDrive(){
        FR.setPower(0);
        FL.setPower(0);
        BR.setPower(0);
        BL.setPower(0);
    }

}