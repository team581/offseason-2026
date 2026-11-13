package com.team581.vision.spot;

import com.team581.vision.spot.data.Profile;
import com.team581.vision.spot.structs.*;
import edu.wpi.first.networktables.*;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.ArrayList;
import java.util.List;

/** Desktop peer publishing Spot's camera metadata and readiness. */
final class TestPeer implements AutoCloseable {
  static byte[] frame(FrameResult value) {
    var bytes = ByteBuffer.allocate(FrameResult.struct.getSize()).order(ByteOrder.LITTLE_ENDIAN);
    FrameResult.struct.pack(bytes, value);
    return bytes.array();
  }

  private final NetworkTableInstance nt;

  private final List<AutoCloseable> resources = new ArrayList<>();

  TestPeer(NetworkTableInstance nt) {
    this.nt = nt;
  }

  @Override
  public void close() throws Exception {
    for (var resource : resources) resource.close();
  }

  void ready(Profile profile) {
    for (var camera : profile.cameras()) {
      var settings =
          nt.getStructTopic("/Spot/" + camera.name() + "/settings", CameraSettings.struct)
              .publish();
      settings.set(new CameraSettings(camera.enabled(), camera.required()));
      resources.add(settings);
    }
    var names = nt.getStringArrayTopic("/Spot/cameras").publish();
    names.set(profile.cameras().stream().map(c -> c.name()).toArray(String[]::new));
    resources.add(names);
    var health =
        nt.getStructTopic("/Spot/health", Heartbeat.struct)
            .publish(PubSubOption.keepDuplicates(true));
    health.set(new Heartbeat(1, 2, true));
    resources.add(health);
  }
}
