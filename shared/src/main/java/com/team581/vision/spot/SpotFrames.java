package com.team581.vision.spot;

import com.google.common.collect.ImmutableList;
import com.team581.vision.spot.data.*;
import com.team581.vision.spot.structs.*;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.*;

/** Resolves a numeric frame against the already acknowledged configuration. */
final class SpotFrames {
  static final String[] SOURCES = {
    "", "read-completion-local-nt", "driver-exposure-start-local-nt", "driver-exposure-end-local-nt"
  };
  static final String[] SYNC_STATES = {"Waiting", "Synchronized", "Invalid"};

  static Health health(Heartbeat value, Acknowledgement ack) {
    var service = SpotProtocol.token(ack.serviceSession());
    var runtime = SpotProtocol.token(ack.runtimeGeneration());
    if (value.serviceHigh() != service[0]
        || value.serviceLow() != service[1]
        || value.runtimeHigh() != runtime[0]
        || value.runtimeLow() != runtime[1]
        || value.syncState() < 0
        || value.syncState() >= SYNC_STATES.length)
      throw new IllegalArgumentException("Heartbeat context mismatch");
    return new Health(
        ack.serviceSession(),
        value.sequence(),
        value.uptime(),
        SYNC_STATES[value.syncState()],
        value.offsetUs(),
        value.syncAge(),
        value.connectionEpoch(),
        value.syncEpoch(),
        value.discontinuities(),
        ack.runtimeGeneration(),
        "",
        "",
        ImmutableList.of());
  }

  static Observation observation(
      FrameResult frame, Acknowledgement ack, CameraGeneration camera, Profile profile) {
    if (frame.candidateCount() < 0
        || frame.candidateCount() > 2
        || frame.timestampSource() < 1
        || frame.timestampSource() >= SOURCES.length)
      throw new IllegalArgumentException("Invalid frame count or timestamp source");
    var candidates = new ArrayList<PoseCandidate>();
    var estimates = new PoseEstimate[] {frame.primary(), frame.alternate()};
    long usedTags = 0;
    for (int i = 0; i < frame.candidateCount(); i++) {
      var estimate = estimates[i];
      if (estimate.method() < 1 || estimate.method() > 2)
        throw new IllegalArgumentException("Invalid pose method");
      candidates.add(
          new PoseCandidate(
              SpotProtocol.pose(estimate.pose()),
              PoseMethod.values()[estimate.method()],
              SpotProtocol.tagIds(estimate.tagMask(), profile.field()),
              estimate.reprojectionError(),
              estimate.averageDistance(),
              estimate.imageArea(),
              estimate.imageSpread(),
              estimate.distortion()));
      usedTags |= estimate.tagMask();
    }
    var tags = new ArrayList<TagObservation>();
    for (var id : SpotProtocol.tagIds(frame.observedTags(), profile.field())) {
      boolean used = (SpotProtocol.tagMask(ImmutableList.of(id), profile.field()) & usedTags) != 0;
      tags.add(new TagObservation(id, ImmutableList.of(), 0, 0., used, "", null));
    }
    String epoch =
        HexFormat.of()
            .formatHex(
                ByteBuffer.allocate(16)
                    .order(ByteOrder.LITTLE_ENDIAN)
                    .putLong(frame.captureHigh())
                    .putLong(frame.captureLow())
                    .array());
    return new Observation(
        ack.robotId(),
        ack.robotSession(),
        ack.serviceSession(),
        epoch,
        ack.connectionEpoch(),
        ack.syncEpoch(),
        frame.sequence(),
        camera.cameraName(),
        camera.bindingId(),
        camera.bindingGeneration(),
        ack.manifestHash(),
        ack.fieldHash(),
        camera.calibrationHash(),
        camera.calibrationValid(),
        ack.runtimeGeneration(),
        camera.modeId(),
        camera.opticalSignature(),
        frame.captureLocalNtUs(),
        SOURCES[frame.timestampSource()],
        frame.timestampUncertainty(),
        candidates,
        tags,
        new ProcessingTiming(
            frame.captureToDetectMs(), frame.detectMs(), frame.solveMs(), frame.totalMs()),
        frame.tagSelectionGeneration());
  }

  private SpotFrames() {}
}
