// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.*;
//import frc.robot.commands.buttons.*;
import frc.robot.commands.*;
import edu.wpi.first.wpilibj2.command.Command;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  private RonMK1 m_Ron;
 // private Climber m_Climber;
  //private Intake m_Intake;
  private Limelight m_Limelight;
  private RobotContainer robotContainer;
  private SwerveDrivetrain m_SwerveDrivetrain;
 

  public static RobotContainer m_RobotContainer;
  private Command autoCommand;

  
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    robotContainer = new RobotContainer();
    m_Ron = RonMK1.getInstance();
   // m_Intake = Intake.getInstance();
    m_Limelight = Limelight.getInstance();
    m_RobotContainer = RobotContainer.getInstance();
   // m_SwerveDrivetrain = SwerveDrivetrain.getInstance();
    //m_Climber = Climber.getInstance();

  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    autoCommand = robotContainer.getAutonomousCommand();

    if (autoCommand != null) {
        autoCommand.schedule();
    }

  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {

  }



  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    double RonPercent = 0.0;
    double IntakePercent = 0.0;
    double ClimberPercent = 0.0;
  
 
    if(m_RobotContainer.getRonState()){
      RonPercent = 3.0;
      //IntakePercent = -0.4;
    }

    if(m_RobotContainer.getIntakeState()){
      IntakePercent = -1.0;

    }
    if (m_RobotContainer.reverseIntakeState()){
      IntakePercent = 1.0;
    }
  //  if(m_RobotContainer.getCharlieState()){
    //  m_SwerveDrivetrain.CharlieMode(true);
    if(m_RobotContainer.getClimberState()){
      ClimberPercent = 1;
    }
    if(m_RobotContainer.getReverseClimberState()){
      ClimberPercent = -1;
    
      
    }
    
    m_Ron.FireRon(RonPercent);
  //  m_Intake.RunIntake(IntakePercent);
 //   m_Climber.RunClimber(ClimberPercent);

    







  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
