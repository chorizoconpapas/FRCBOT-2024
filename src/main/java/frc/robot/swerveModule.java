package frc.robot;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.ModuleConstants.DriveConstants;


public class swerveModule {
    private final CANSparkMax driveMotor;
    private final CANSparkMax turningMotor;

    private final CANEncoder driverEncoder;
    private final CANEncoder turningEncoder;

    private final AnalogInput absoluteEncoder;
    private final boolean  absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad;

    private final PIDController turningPidController;

    public swerveModule(int driveMotorId,int turningMotorId,boolean driveMotorReversed,boolean turningMotorReversed,int absoluteEncoderId
    ,double absoluteEncoderOffset,boolean absoluteEncoderReversed){

        this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;
        absoluteEncoder = new AnalogInput(absoluteEncoderId);

        driveMotor = new CANSparkMax (driveMotorId,MotorType.kBrushless);
        turningMotor = new CANSparkMax(turningMotorId, MotorType.kBrushless);

        driveMotor.setInverted(driveMotorReversed);
        turningMotor.setInverted(turningMotorReversed);

        driverEncoder = driveMotor.getEncoder();
        turningEncoder = turningMotor.getEncoder();

        driverEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
        driverEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
        turningEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
        turningEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);

        turningPidController =new PIDController(ModuleConstants.kPTurning, 0,0);
        turningPidController.enableContinuousInput(-Math.PI,Math.PI);
        
        resetEncoders();
    }
 
    public double getDrivePosition(){
        return driverEncoder.getPosition();
    }

    public double getTurningPosition(){
        return turningEncoder.getPosition();
    }

    public double getDriveVelocity(){
        return driverEncoder.getVelocity();
    }

    public double getTurningVelocity(){
        return turningEncoder.getVelocity();
    }

    public double getAbsoluteEncoderRad(){
        double angel = absoluteEncoder.getVoltage()/RobotController.getVoltage5V();
        angel *= 2.0*Math.PI;
        angel -= absoluteEncoderOffsetRad;
        return angel * (absoluteEncoderReversed? -1.0: 1.0);
    }

    public void resetEncoders(){
        driverEncoder.setPosition(0);
        turningEncoder.setPosition(getAbsoluteEncoderRad());
    }

    public SwerveModuleState getState (){
        return new SwerveModuleState(getDriveVelocity(),new Rotation2d(getTurningPosition()));
    }

    public void  setDesiredState(SwerveModuleState state){

        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop ();
            return;
        }

        state = SwerveModuleState.optimize(state,getState().angle);
        driveMotor.set(state.speedMetersPerSecond/DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        turningMotor.set(absoluteEncoderOffsetRad);
        SmartDashboard.putString("Swerve["+ absoluteEncoder.getChannel() +"] state",state.toString());
     }

    public void stop() {
        driveMotor.set(0);
        turningMotor.set(0);
    }
    
}