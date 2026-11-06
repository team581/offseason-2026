package com.team581.vision.spot;

import com.team581.vision.spot.data.*;
import com.team581.vision.spot.structs.FieldTagValue;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.util.ArrayList;
import java.util.List;

/**
 * Publishes what Spot reads: the deployed spot-config.json, forwarded unparsed, and the settings
 * robot code shares with Spot. Robot-only settings such as fusion are never sent.
 */
final class SpotConfigPublisher implements AutoCloseable {
  private static final String ROOT = "/Spot/Config/";
  private final NetworkTableInstance nt;
  private final List<AutoCloseable> publishers = new ArrayList<>();

  SpotConfigPublisher(NetworkTableInstance nt, Profile profile, String cameraConfig) {
    this.nt = nt;
    text("cameraConfig", cameraConfig);
    number("maxAgeS", profile.maxAgeS());
    number("futureToleranceS", profile.futureToleranceS());
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
