package frc.robot.Commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.SwerveDriveTrain;
import frc.util.lib.ArcadeJoystickUtil;

public class SwerveTeleop extends Command{
    SwerveDriveTrain dt;
    CommandXboxController joy;

    public SwerveTeleop(SwerveDriveTrain dt, CommandXboxController joy){
        this.dt = dt;
        this.joy = joy;

        addRequirements(dt);
    }
    
    @Override
    public void execute(){
        double xVal = -joy.getLeftY();
        double yVal = -joy.getLeftX();

        double rotVal = joy.getRightX();

        xVal = MathUtil.applyDeadband(xVal, Constants.SwerveConstants.deadBand);
        yVal = MathUtil.applyDeadband(yVal, Constants.SwerveConstants.deadBand);
        rotVal = MathUtil.applyDeadband(rotVal, Constants.SwerveConstants.deadBand);

        double[] polarCoords = ArcadeJoystickUtil.regularGamePadControls(xVal, yVal, 1.3);

        xVal = Math.cos(polarCoords[1]) * polarCoords[0];
        yVal = Math.sin(polarCoords[1]) * polarCoords[0];

        dt.drive(new Translation2d(xVal, yVal), rotVal, false);
    }

    @Override
    public void end(boolean interrupted){
        dt.drive(new Translation2d(0, 0), 0, false);
        dt.stopMotors();
    }
}
