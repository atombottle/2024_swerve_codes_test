package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveModule extends SubsystemBase{
    private CANSparkMax drive;
    private CANSparkMax turn;
    private CANcoder canCoder;
    private SparkMaxPIDController drivePID, turnPID;
    private RelativeEncoder driveEncoder, turnEncoder;
    private SimpleMotorFeedforward Arbfeedforward;
    //private boolean field_oriented;
    //private PIDController controller = new PIDController(0.003455, 0.000009, 0);

    public SwerveModule(int driveMotor_ID, int turn_ID, int cancoder_ID, double angleOffset, boolean EncoderReversed, boolean driveMotorReversed) {
        drive = new CANSparkMax(driveMotor_ID, MotorType.kBrushless);
        turn = new CANSparkMax(turn_ID, MotorType.kBrushless);
        canCoder = new CANcoder(cancoder_ID);

        drive.restoreFactoryDefaults();
        turn.restoreFactoryDefaults();

        drivePID = drive.getPIDController();
        turnPID = turn.getPIDController();

        driveEncoder = drive.getEncoder();
        turnEncoder = turn.getEncoder();

        //set the range(0.5 to -0.5)  of sensor
        CANcoderConfiguration CANConfigs = new CANcoderConfiguration();
        CANConfigs.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        if(EncoderReversed){
            CANConfigs.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        }else{
            CANConfigs.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        }
        
        canCoder.getConfigurator().apply(CANConfigs);

        //Magnet Sensor Offset
        MagnetSensorConfigs MagnetCFG = new MagnetSensorConfigs();
        MagnetCFG.MagnetOffset = angleOffset;
        canCoder.getConfigurator().apply(MagnetCFG);

        drive.setInverted(driveMotorReversed);
        turn.setInverted(true);

        //drivePID.setSmartMotionMaxVelocity(Constants.PhysicalConstants.MAX_VELOCITY_RPM, 0);
        drive.setSmartCurrentLimit(Constants.PhysicalConstants.DRIVE_CURRENT_LIMIT);
        turn.setSmartCurrentLimit(Constants.PhysicalConstants.TURN_CURRENT_LIMIT);

        drive.setClosedLoopRampRate(0.1);
        turn.setClosedLoopRampRate(0.1);

        driveEncoder.setPositionConversionFactor(Units.inchesToMeters(1 / Constants.PhysicalConstants.DRIVE_GEAR_RATIO * Math.PI * 4));
        driveEncoder.setVelocityConversionFactor(Units.inchesToMeters(1 / Constants.PhysicalConstants.DRIVE_GEAR_RATIO * Math.PI * 4) / 60);
        turnEncoder.setPositionConversionFactor((1 / Constants.PhysicalConstants.TURN_GEAR_RATIO) * Math.PI * 2);
        turnEncoder.setVelocityConversionFactor(((1 / Constants.PhysicalConstants.TURN_GEAR_RATIO) * Math.PI * 2) / 60);

        //canCoder.setPositionToAbsolute();

        driveEncoder.setPosition(0.0);
        turnEncoder.setPosition(Math.toRadians(canCoder.getAbsolutePosition().getValueAsDouble()));

        drivePID.setP(Constants.TunedConstants.PIDF0_DRIVE_P, 0);
        drivePID.setI(Constants.TunedConstants.PIDF0_DRIVE_I, 0);
        drivePID.setD(Constants.TunedConstants.PIDF0_DRIVE_D, 0);
        drivePID.setFF(Constants.TunedConstants.PIDF0_DRIVE_F, 0);

        Arbfeedforward = new SimpleMotorFeedforward(
            Constants.TunedConstants.FEED_DRIVE_KS, 
            Constants.TunedConstants.FEED_DRIVE_KV,
            Constants.TunedConstants.FEED_DRIVE_KA
        );

        turnPID.setP(Constants.TunedConstants.PIDF0_TURN_P, 0);
        turnPID.setI(Constants.TunedConstants.PIDF0_TURN_I, 0);
        turnPID.setD(Constants.TunedConstants.PIDF0_TURN_D, 0);
        turnPID.setFF(Constants.TunedConstants.PIDF0_TURN_F, 0);
        //turnPID.setOutputRange(-Math.PI, Math.PI, 0);
        //turnPID.setIZone(1);

        drivePID.setOutputRange(-1, 1);

        drivePID.setFeedbackDevice(driveEncoder);
        turnPID.setFeedbackDevice(turnEncoder);

        turn.enableVoltageCompensation(Constants.PhysicalConstants.NOMINAL_VOLTAGE);
        drive.enableVoltageCompensation(Constants.PhysicalConstants.NOMINAL_VOLTAGE);

        setBrakeMode(true);

        drive.burnFlash();
        turn.burnFlash();
    }

    public void setdrivePID(double p, double i, double d, double f, int slotID) {
        drivePID.setP(p, slotID);
        drivePID.setI(i, slotID);
        drivePID.setD(d, slotID);
        drivePID.setFF(f, slotID);

        drive.burnFlash();
    }    
    
    public void setTurnPID(double p, double i, double d, double f, int slotID) {
        turnPID.setP(p, slotID);
        turnPID.setI(i, slotID);
        turnPID.setD(d, slotID);
        turnPID.setFF(f, slotID);
        //turnPID.setIZone(1);
        //turnPID.setOutputRange(-180, 180);

        turn.burnFlash();
    }

    public void setBrakeMode(Boolean mode){
        if(mode){
            drive.setIdleMode(IdleMode.kBrake);
            turn.setIdleMode(IdleMode.kBrake);
        }else{
            drive.setIdleMode(IdleMode.kCoast);
            turn.setIdleMode(IdleMode.kCoast);        
        }
    }

    public void resetEncoder(){
        driveEncoder.setPosition(0);
        canCoder.setPosition(getAbsoluteAngle());
    }

    public void checkAngle() {
        if(getAngle() > Math.PI) {
            turnEncoder.setPosition(getAngle() - (2 * Math.PI));
        } else if(getAngle() < -Math.PI) {
            turnEncoder.setPosition(getAngle() + (2 * Math.PI));
        }
    }

    public void checkEncoder() {
        if(Math.abs(turnEncoder.getVelocity()) < 0.005 && Math.abs(getVelocity()) < 0.001) {
            turnEncoder.setPosition(Math.toRadians(canCoder.getPosition().getValueAsDouble()));
        }
    }

    public double closestAngle(double angle) {
        double dir = (angle % (2 * Math.PI)) - (getAngle() % (2 * Math.PI));

        if(Math.abs(dir) > Math.PI){
            dir = -(Math.signum(dir) * (2 * Math.PI)) + dir;
        }

        return dir;
    }

    public void setAngle(double angle) {
        double setpointAngle = closestAngle(angle);
        double setpointAngleInvert = closestAngle(angle + Math.PI);

        if(Math.abs(setpointAngle) <= Math.abs(setpointAngleInvert)){
            turnPID.setReference(getAngle() + setpointAngle, ControlType.kPosition);
        }else{
            turnPID.setReference(getAngle() + setpointAngleInvert, ControlType.kPosition);
        }
    }

    public void setState(SwerveModuleState state){
        SwerveModuleState desiredState = SwerveModuleState.optimize(state, getRotation());
        double velocity = desiredState.speedMetersPerSecond;
        double moduleangle = desiredState.angle.getRadians();
        
        drivePID.setReference(
            velocity, 
            ControlType.kVelocity,
            0,
            Arbfeedforward.calculate(velocity),
            ArbFFUnits.kVoltage
        );
         //drive.set(velocity / Constants.PhysicalConstants.MAX_WHEEL_SPEED_METERS);
        
         //turnPID.setReference(moduleangle, ControlType.kPosition);
        setAngle(moduleangle);

        //SmartDashboard.getNumber("TARGET ANGLE " + canCoder.getDeviceID() / 3, moduleangle);
        //SmartDashboard.getNumber("ANGLE " + canCoder.getDeviceID() / 3, moduleangle);
    }

    public double getVelocity(){
        return driveEncoder.getVelocity();
    }

    public double getAngle() {
        return turnEncoder.getPosition();
    }

    public double getAngleCancoder() {
        return canCoder.getPosition().getValueAsDouble();
    }

    public double getAbsoluteAngle() {
       return canCoder.getAbsolutePosition().getValueAsDouble();
    }

    public Rotation2d getRotation() {
        return Rotation2d.fromRadians(getAngle());
    }

    public double getDistance() {
        return driveEncoder.getPosition();
    }

    public SwerveModuleState getModuleState() {
        return new SwerveModuleState(
           getVelocity(), 
           getRotation()
        );
    }

    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition( 
            getDistance(), 
            getRotation()
        );
    }
    
    @Override
    public void periodic(){
        checkEncoder();
    }
}