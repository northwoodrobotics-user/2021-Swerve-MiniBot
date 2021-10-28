package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.SwerveModuleMK3;
import lib.util.ArcadeDriveHelper;

public class SwerveDriveCommand extends CommandBase {

  private final SwerveDrivetrain drivetrain;
  private final XboxController controller;

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter xspeedLimiter = new SlewRateLimiter(1);
  private final SlewRateLimiter yspeedLimiter = new SlewRateLimiter(1);
  private final SlewRateLimiter rotLimiter = new SlewRateLimiter(1);

  public SwerveDriveCommand(SwerveDrivetrain drivetrain, XboxController controller) {
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);

    this.controller = controller;
  }

  @Override 
  public void execute() {
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    final var xSpeed =
    -xspeedLimiter.calculate(SwerveDrivetrain.deadBand(controller.getY(GenericHID.Hand.kLeft)))
        * SwerveDrivetrain.kMaxSpeed;

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    final var ySpeed =
    yspeedLimiter.calculate(SwerveDrivetrain.deadBand(controller.getX(GenericHID.Hand.kLeft)))
        * SwerveDrivetrain.kMaxSpeed;

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    final var rot =
    -rotLimiter.calculate(SwerveDrivetrain.deadBand(controller.getX(GenericHID.Hand.kRight)))
        * SwerveDrivetrain.kMaxAngularSpeed*.6;

    boolean calibrate = controller.getBumper(GenericHID.Hand.kLeft);




    

    
    
  } 
  
  public SwerveDrivetrain swerveRun(xSpeed, ySpeed, rot){
    return swerveRun(xSpeed, ySpeed, rot)
    


  }

}
