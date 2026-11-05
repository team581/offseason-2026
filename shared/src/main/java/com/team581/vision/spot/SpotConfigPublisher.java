package com.team581.vision.spot;

import com.team581.vision.spot.data.*;
import com.team581.vision.spot.structs.FieldTagValue;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.util.ArrayList;
import java.util.List;

/** Spot parses camera defaults; the robot only forwards the deployed file. */
final class SpotConfigPublisher implements AutoCloseable {
  private static final String ROOT = "/Spot/Config/";
  private final NetworkTableInstance nt;
  private final List<AutoCloseable> publishers = new ArrayList<>();

  SpotConfigPublisher(NetworkTableInstance nt, Profile profile, String cameraConfig) {
    this.nt = nt;
    text("cameraConfig", cameraConfig);
    flag("fusionEnabled", profile.fusionEnabled());
    flag("headingEnabled", profile.headingEnabled());
    number("maxAgeS", profile.maxAgeS());
    number("futureToleranceS", profile.futureToleranceS());
    number("reorderWindowS", profile.reorderWindowS());
    text("Field/origin", profile.field().origin());
    number("Field/lengthM", profile.field().lengthM());
    number("Field/widthM", profile.field().widthM());
    var tags = nt.getStructArrayTopic(ROOT + "Field/tags", FieldTagValue.struct).publish();
    publishers.add(tags);
    tags.set(
        profile.field().tags().stream()
            .map(tag ->
                new FieldTagValue(tag.id(), SpotProtocol.pose(tag.fieldToTag()), tag.sizeM()))
            .toArray(FieldTagValue[]::new));
  }

  @Override
  public void close() {
    for (var publisher : publishers) {
      try {
        publisher.close();
      } catch (Exception exception) {
        throw new IllegalStateException(exception);
      }
    }
  }

  private void flag(String key, boolean value) {
    var publisher = nt.getBooleanTopic(ROOT + key).publish();
    publishers.add(publisher);
    publisher.set(value);
  }

  private void number(String key, double value) {
    var publisher = nt.getDoubleTopic(ROOT + key).publish();
    publishers.add(publisher);
    publisher.set(value);
  }

  private void text(String key, String value) {
    var publisher = nt.getStringTopic(ROOT + key).publish();
    publishers.add(publisher);
    publisher.set(value);
  }
}
