package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;

public class Constants {
    public static class Controller {
        public static int DRIVER_ID = 0;
        public static int OPERATOR_ID = 1;

        public static double GAMEPAD_THRESHOLD = 0.1;

        public static double DRIVER_RUMBLE_STRENGTH = 0.3; // Cannot be larger than 1. The lower the number the less feedback strength the driver would feel.
    }

    public static class IDs{
        // -----------------------------------------------
        // DRIVE ID'S
        // -----------------------------------------------

        public static final int FRONT_LEFT_FORWARD_ID = 1;
        public static final int FRONT_LEFT_ROTATION_ID = 2;
        public static final int FRONT_LEFT_CANCODER_ID = 3;
    
        public static final int FRONT_RIGHT_FORWARD_ID = 4;
        public static final int FRONT_RIGHT_ROTATION_ID = 5;
        public static final int FRONT_RIGHT_CANCODER_ID = 6;
    
        public static final int REAR_LEFT_FORWARD_ID = 7;
        public static final int REAR_LEFT_ROTATION_ID = 8;
        public static final int REAR_LEFT_CANCODER_ID = 9;
    
        public static final int REAR_RIGHT_FORWARD_ID = 10;
        public static final int REAR_RIGHT_ROTATION_ID = 11;
        public static final int REAR_RIGHT_CANCODER_ID = 12;    

        // -----------------------------------------------

        // -----------------------------------------------
        // ARM ID'S
        // -----------------------------------------------

        public static final int ARM = 15;
        public static final int ARM_FOLLOWER = 16;
        
        public static final int ARM_INTAKE = 17;
        public static final Port INTAKE_COLOR_SENSOR = I2C.Port.kOnboard;

        public static final int SHOULDER = 18;

        // ------------------------------------------------

        public static final int LIMELIGHT_SERVO = 0;

        public static final int LED_PORT = 3;
    }

    public static class TunedConstants {
        // ------------------------------------------------
        // DRIVE
        // ------------------------------------------------

        public static final double PIDF0_DRIVE_P = 4.497E-06; // Previously 0.003405
        public static final double PIDF0_DRIVE_I = 0;
        public static final double PIDF0_DRIVE_D = 0;
        public static final double PIDF0_DRIVE_F = 0; // Previously 0.315

        public static final double FEED_DRIVE_KV = 2.3854; // 2.3854
        public static final double FEED_DRIVE_KS = 0.15778;
        public static final double FEED_DRIVE_KA = 0.41036;

        public static final double PIDF0_TURN_P = 0.292;
        public static final double PIDF0_TURN_I = 0;
        public static final double PIDF0_TURN_D = 0;
        public static final double PIDF0_TURN_F = 0.0008;

        // ------------------------------------------------

        // ------------------------------------------------
        // ARM
        // ------------------------------------------------

        public static final double PIDF0_ARM_P = 0.2;
        public static final double PIDF0_ARM_I = 0;
        public static final double PIDF0_ARM_D = 0;
        public static final double PIDF0_ARM_F = 0.0015;

        public static final double PIDF1_ARM_P = 8.1607E-05;
        public static final double PIDF1_ARM_I = 0;
        public static final double PIDF1_ARM_D = 5.3545E-05;
        public static final double PIDF1_ARM_F = 0;

        public static final double FEED_ARM_KV = 3.5181;
        public static final double FEED_ARM_KS = -0.055858;
        public static final double FEED_ARM_KA = 0.49846;
        public static final double FEED_ARM_KG = 0.15946;

        // -------------------------------------------------

        // -------------------------------------------------
        // AUTOBALANCE PID
        // -------------------------------------------------
        
        public static final double STAGE1_AUTOBALANCE_P = 0.05;
        public static final double STAGE1_AUTOBALANCE_I = 0;
        public static final double STAGE1_AUTOBALANCE_D = 0;

        public static final double STAGE2_AUTOBALANCE_P = 0.003;
        public static final double STAGE2_AUTOBALANCE_I = 0;
        public static final double STAGE2_AUTOBALANCE_D = 0;

        // -------------------------------------------------

        // -------------------------------------------------
        // AUTO CONSTANTS
        // -------------------------------------------------

        public static final double AUTO_PID_TRANSLATION_P = 0.5;
        public static final double AUTO_PID_TRANSLATION_I = 0;
        public static final double AUTO_PID_TRANSLATION_D = 0;

        public static final double AUTO_PID_THETA_P = 0.05;
        public static final double AUTO_PID_THETA_I = 0;
        public static final double AUTO_PID_THETA_D = 0;

        // -------------------------------------------------

        public static final double THETA_PID_P = 0.03;
        public static final double THETA_PID_I = 0;
        public static final double THETA_PID_D = 0;
        public static final double THETA_MAX_VEL = 135;
        public static final double THETA_MAX_ACCEL = 90;
    }

    public static class PhysicalConstants {
        public static final double TRACK_WIDTH = 24 - 2.625; // Square Base, so no track length needed
        public static final double TRACK_WIDTH_METERS = Units.inchesToMeters(TRACK_WIDTH);

        public final static SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
            new Translation2d((TRACK_WIDTH_METERS / 2), (TRACK_WIDTH_METERS / 2)),
            new Translation2d((TRACK_WIDTH_METERS / 2), -(TRACK_WIDTH_METERS / 2)),
            new Translation2d(-(TRACK_WIDTH_METERS / 2), (TRACK_WIDTH_METERS / 2)),
            new Translation2d(-(TRACK_WIDTH_METERS / 2), -(TRACK_WIDTH_METERS / 2))
        );

        public static final double DRIVE_WHEEL_DIAMETER_INCHES = 4;
        public static final double DRIVE_WHEEL_DIAMTER_METERS = Units.inchesToMeters(4);

        public static final double DRIVE_GEAR_RATIO = 6.12;
        public static final double TURN_GEAR_RATIO = 150 / 7;

        public static final double ARM_GEAR_RATIO = 180; // Larger Tooth: 480, Smaller Tooth: 240

        public static final double SHUFFLER_GEAR_RATIO = 1.25;

        public static final double MAX_TRANSLATION_SPEED_METERS = 4.8;
        public static final double MAX_ANGULAR_SPEED_METERS = 4.5;
        public static final double MAX_WHEEL_SPEED_METERS = 5;

        public static final double MAX_ARM_VELOCITY = 4.25;
        public static final double MAX_ARM_ACCELERATION = 2.25;

        // -------------------------------------------------
        // CURRENT
        // -------------------------------------------------

        public static final int DRIVE_CURRENT_LIMIT = 40;
        public static final int TURN_CURRENT_LIMIT = 40;

        public static final int ARM_CURRENT_LIMIT = 40;

        public static final int SHUFFLER_ARM_CURRENT_LIMIT = 30;

        public static final int INTAKE_CURRENT_LIMIT = 20;
        public static final int INTAKE_HOLD_CURRENT_LIMIT = 5;

        public static final int SHOULDER_CURRENT_LIMIT = 50;

        // --------------------------------------------------
    
        public static final double NOMINAL_VOLTAGE = 12; // For Comp saturation

        public static final boolean isGyroReversed = true;
        public static final double GYRO_OFFSET = 0;

        // --------------------------------------------------
        // ENCODER OFFSETS
        // --------------------------------------------------
        
        public static final double FRONT_LEFT_OFFSET = 155.3027; //157.1;
        public static final double FRONT_RIGHT_OFFSET = 65.03906; //63;
        public static final double REAR_LEFT_OFFSET = -65.83; //-66.2;
        public static final double REAR_RIGHT_OFFSET = 121.99; //121.9;

        // --------------------------------------------------

        // --------------------------------------------------
        // ARM POSITIONS
        // --------------------------------------------------
        
        public static double ARM_STOWED = 3.16;
        public static double ARM_HOME = 3.9;
        public static double ARM_HIGH = 0.79;
        public static double ARM_MID = 0.41;
        public static double ARM_LOW = -0.43;
        public static double ARM_DOUBLE_SUB = 0.4;

        // --------------------------------------------------

        public static final int INTAKE_PROXIMITY_THRESHOLD = 400;
        
    }

    public static final double LOOP_TIME_S = 0.02;
    public static final int LOOP_TIME_MS = 20;

    public static final int LED_BUFFER_LENGTH = 57;
}