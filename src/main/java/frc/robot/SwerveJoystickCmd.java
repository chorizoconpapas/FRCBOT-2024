package frc.robot;
import java.util.function.Supplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ModuleConstants.DriveConstants;
import frc.robot.ModuleConstants.OIConstants;


public class SwerveJoystickCmd extends Command {
    private final swerveSubsystemBase swerveSubsystemBase;
    private final Supplier<Double> xSpdFunction,ySpdFunction,turningSpdFunction;
    private final Supplier<Boolean> fieldOrientedFunction;
    private final SlewRateLimiter xLimiter,yLimiter, turningLimiter;

    public SwerveJoystickCmd (swerveSubsystemBase swerveSubsystemBase,
    Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction, Supplier<Double> turningSpdFunction,
    Supplier<Boolean> fieldorientedFunction) {
    this.swerveSubsystemBase = swerveSubsystemBase;
    this.xSpdFunction = xSpdFunction;
    this.ySpdFunction = ySpdFunction;
    this.turningSpdFunction = turningSpdFunction;
    this.fieldOrientedFunction = fieldorientedFunction;
    addRequirements (swerveSubsystemBase);
    this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
    this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
    this.turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);

    }

    @Override
    public void execute() {
        double xSpeed = xSpdFunction.get();
        double ySpeed = ySpdFunction.get();
        double turningSpeed = turningSpdFunction.get();

        xSpeed = Math.abs(xSpeed) > OIConstants.kDeadband ? xSpeed:0.0;
        ySpeed = Math.abs(ySpeed) > OIConstants.kDeadband ? ySpeed:0.0;
        turningSpeed = Math.abs(turningSpeed) > OIConstants.kDeadband ? turningSpeed : 0.0;

        xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        turningSpeed = turningLimiter.calculate(turningSpeed) * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;

        ChassisSpeeds chassisSpeeds;
        if(fieldOrientedFunction.get()){
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turningSpeed,swerveSubsystemBase.getRotation2d());
        }
        else{
            chassisSpeeds = new ChassisSpeeds(xSpeed,ySpeed,turningSpeed);
        }
        SwerveModuleState[] modulStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
        swerveSubsystemBase.setModulesState(modulStates);
    }
    @Override
    public void end(boolean interrupted) {
        swerveSubsystemBase.stopModels();
    }

    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return false;
    }
}