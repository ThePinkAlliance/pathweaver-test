package frc.robot.PathweaverDash.Server;

import java.io.IOException;
import java.net.ServerSocket;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Server extends ServerSocket {
  private NetworkTableEntry t;
  private NetworkTable table;

  public Server(int port) throws IOException {
    NetworkTableInstance inst = NetworkTableInstance.getDefault();

    table = inst.getTable("datatable");

    // Get the entries within that table that correspond to the X and Y values
    // for some operation in your program.
    t = table.getEntry("inital-path");

  }

  public void WriteData(String label, Object data) {
    t = table.getEntry(label);

    t.setValue(data);
  }
}
