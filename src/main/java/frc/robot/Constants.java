package frc.robot;

import com.ctre.phoenix6.configs.Slot0Configs;

public class Constants {
    public static final class LiftConstants
    {
        public static final int lift_motor1_id = 40; 
        public static final int lift_motor2_id = 41; 
        public static final int bot_limit_switch_id = 0;

        public static final boolean lift_motor1_inverted = false;
        public static final boolean lift_motor2_inverted = false;

        public static final double lift_manual_speed = 0.75;

        private static final Slot0Configs lift_gains = new Slot0Configs()
        .withKP(0.1).withKI(0).withKD(0)
        .withKS(0).withKV(0);

        public static final double height_tolerance = 5; // +-amount 
        public static final double pickup_height = 34.5;
        public static final double L1_height = 45.8;
        public static final double L2_height = 27.1;
        public static final double L3_height = 89;
        public static final double L4_height = 180;
    }

    public static final class ManipulatorConstants
    {
        public static final int chute_motor_id          = 42;
        public static final int algae_intake_motor_id   = 43;
        public static final int algae_arm_motor_id      = 44;

        public static final boolean chute_inverted = false;
        public static final boolean algae_intake_inverted = false;
        public static final boolean algae_arm_inverted = false;

        public static final double chute_speed = .2;
        public static final double algae_intake_speed = 1;
        public static final double algae_arm_speed = .15;
    }

    public static final class ElevatorSetpoints {
        public static final double kFeederStation = 0.00;
        public static final double kLevel1 = 0.15;
        public static final double kLevel2 = 0.32;
        public static final double kLevel3 = 0.3;
        public static final double kLevel4 = 0.4;
      }
}
