// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.DriveTrain;

/** An example command that uses an example subsystem. */
public class AutoDriveForward extends CommandBase 
{
  private final DriveTrain _DriveTrain;

  private double distance;
  private double speed;
  private double error;
  private double kP = 0.8; //test constant later

  public AutoDriveForward(DriveTrain dt, double dist) 
  {
    distance = dist;
    _DriveTrain = dt;
    addRequirements(_DriveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    SmartDashboard.putNumber("Ticks", _DriveTrain.getTicks());
    error = distance - _DriveTrain.getPosition();
    error = (error / distance)*2;
    speed = error * kP;

    if(speed > .7)
    {
      speed = .7;
    }

    if(speed < .2)
    {
      speed = .2;
    }

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
    return _DriveTrain.getPosition() >= distance;
  }
}
