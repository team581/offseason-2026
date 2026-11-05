package com.team581.vision.spot;

import com.google.common.collect.ImmutableList;
import com.team581.vision.spot.data.*;
import com.team581.vision.spot.structs.*;

/** Resolves a numeric frame against the already acknowledged configuration. */
final class SpotFrames {
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

  static void validate(FrameResult frame, Profile profile) {
    if (frame.candidateCount() < 0
        || frame.candidateCount() > 2
        || frame.timestampSource() < 1
        || frame.timestampSource() > 3)
      throw new IllegalArgumentException("Invalid frame count or timestamp source");
    int tags = profile.field().tags().size();
    long mask = tags == 64 ? -1L : (1L << tags) - 1;
    if ((frame.observedTags() & ~mask) != 0)
      throw new IllegalArgumentException("Observed tags exceed field layout");
    for (int i = 0; i < frame.candidateCount(); i++) {
      var candidate = i == 0 ? frame.primary() : frame.alternate();
      if (candidate.method() < 1 || candidate.method() > 2 || (candidate.tagMask() & ~mask) != 0)
        throw new IllegalArgumentException("Invalid pose method or tag mask");
    }
  }

  private SpotFrames() {}
}
