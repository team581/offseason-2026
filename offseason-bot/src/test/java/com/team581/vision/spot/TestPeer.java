package com.team581.vision.spot;

import com.team581.vision.spot.data.*;
import com.team581.vision.spot.structs.*;
import edu.wpi.first.networktables.*;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.*;

/** Desktop test peer standing in for Spot; production Java only subscribes to results. */
final class TestPeer implements AutoCloseable {
  private static String snake(String name) {
    return name.replaceAll("([a-z])([A-Z])", "$1_$2").toLowerCase(Locale.ROOT);
  }

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

  private void put(String path, Object value) {
    String key = "/Spot/config/acknowledgement/" + path;
    if (value instanceof String s) {
      var p = nt.getStringTopic(key).publish();
      p.set(s);
      resources.add(p);
    } else if (value instanceof Boolean b) {
      var p = nt.getBooleanTopic(key).publish();
      p.set(b);
      resources.add(p);
    } else {
      var p = nt.getIntegerTopic(key).publish();
      p.set(((Number) value).longValue());
      resources.add(p);
    }
  }

  void acknowledgement(Acknowledgement ack) throws ReflectiveOperationException {
    for (var field : Acknowledgement.class.getRecordComponents()) {
      if (!field.getName().equals("cameras"))
        put(snake(field.getName()), field.getAccessor().invoke(ack));
    }
    for (var camera : ack.cameras()) {
      for (var field : CameraGeneration.class.getRecordComponents()) {
        put(
            "cameras/" + camera.cameraName() + "/" + snake(field.getName()),
            field.getAccessor().invoke(camera));
      }
    }
    var names = nt.getStringArrayTopic("/Spot/config/acknowledgement/camera_names").publish();
    names.set(ack.cameras().stream().map(CameraGeneration::cameraName).toArray(String[]::new));
    resources.add(names);
    var header = nt.getStringTopic("/Spot/config/ack").publish();
    header.set(SpotProtocol.hash(ack));
    resources.add(header);
  }
}
