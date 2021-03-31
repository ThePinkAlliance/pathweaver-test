package frc.robot.PathweaverDash;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import frc.robot.PathweaverDash.Server.Server;

class Position {
  double x, y = 0.0;

  public Position(double x, double y) {
    this.x = x;
    this.y = y;
  }
}

public class PathweaverDash {
  private Server serve;

  public PathweaverDash() {
    try {
      this.serve = new Server(5456);
    } catch (Exception err) {
      err.printStackTrace();
    }
  }

  public void SendInitalPath(Trajectory path) {
    Object[] states = path.getStates().toArray();

    for (int i = 0; i != states.length; i++) {
      Trajectory.State _s = (Trajectory.State) states[i];

      double x = _s.poseMeters.getX();
      double y = _s.poseMeters.getY();

      serve.WriteData("inital-path", "{x: " + x + ", y: " + y + "}");
    }
  }

  public void SendRobotPosition(Pose2d pose, double heading) {
    serve.WriteData("robo-pos", "{x: " + pose.getX() + ", y: " + pose.getY() + "}");
  }
}
