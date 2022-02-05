// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.controller;
import frc.robot.subsystems.DriveTrain;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;

/** An example command that uses an example subsystem. */
public class AutoDriveForward extends CommandBase 
{
  private final DriveTrain _DriveTrain;
  PIDController pid = new PIDController(kP, kI, kD);
  private double direction;
  private double distance;
  private double absDistance;
  private double speed;
  private double error;
  private double kP = 0.8; //test constant later

  public AutoDriveForward(DriveTrain dt, double dist) 
  {
    distance = dist;
    absDistance = Math.abs(distance);
    _DriveTrain = dt;
    addRequirements(_DriveTrain);

     if(dist<0){
      direction = -1;
    }else{
      direction = 1;
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    encoders.resetEncoder();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    SmartDashboard.putNumber("Ticks", _DriveTrain.getTicks());
    error = absDistance - Math.abs(_DriveTrain.getPosition());
    error = (error / absDistance)*2;
    speed = error * kP;

    if(speed > .7)
    {
      speed = .7;
    }

    if(speed < .2)
    {
      speed = .2;
    }
    speed = speed * direction
    SmartDashboard.putNumber("Current Speed", speed);
    SmartDashboard.putNumber("Current Distance", _DriveTrain.getPosition());
    _DriveTrain.tankDrive(speed,speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    _DriveTrain.tankDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
    return Math.abs(_DriveTrain.getPosition()) >= absDistance;
  }
}
