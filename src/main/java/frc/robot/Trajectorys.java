package frc.robot;

import java.util.Arrays;
import java.util.List;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;

// X 5.334 Y .762

// B Red is Orange 
// B Blue is yellow

public interface Trajectorys {
        public String ABlue = "output/A-Blue.wpilib.json";
        public String ARed = "output/A-Red.wpilib.json";
        public String BBlue = "output/B-Blue.wpilib.json";
        public String BRed = "output/B-Red.wpilib.json";

        public List<Pose2d> PathARed = Arrays.asList(new Pose2d(), new Pose2d(2.286, 0, new Rotation2d()),
                        new Pose2d(4.572, 1.524, new Rotation2d()), new Pose2d(8.763, 1.524, new Rotation2d()));

        public List<Pose2d> PathABlue = Arrays.asList(new Pose2d(), new Pose2d(2.286, -.762, new Rotation2d()),
                        new Pose2d(4.572, -1.524, new Rotation2d()), new Pose2d(5.334, .762, new Rotation2d()),
                        new Pose2d(6.858, 0, new Rotation2d()), new Pose2d(8.763, 0, new Rotation2d()));

        public List<Pose2d> PathBRed = Arrays.asList(new Pose2d(), new Pose2d(2.286, -1.524, new Rotation2d()),
                        new Pose2d(2.286, .762, new Rotation2d()), new Pose2d(3.81, -.762, new Rotation2d()),
                        new Pose2d(5.334, .762, new Rotation2d()), new Pose2d(5.334, .762, new Rotation2d()),
                        new Pose2d(8.763, .762, new Rotation2d()));

        public List<Pose2d> PathBBlue = Arrays.asList(new Pose2d(), new Pose2d(4.572, -.762, new Rotation2d()),
                        new Pose2d(6.096, .762, new Rotation2d()), new Pose2d(7.62, -.762, new Rotation2d()),
                        new Pose2d(8.763, -.762, new Rotation2d()));
}
