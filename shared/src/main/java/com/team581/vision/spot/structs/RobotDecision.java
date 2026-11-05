package com.team581.vision.spot.structs;

import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructGenerator.SchemaBuilder;
import edu.wpi.first.util.struct.StructSerializable;
import java.nio.ByteBuffer;

public record RobotDecision(
    long captureHigh,
    long captureLow,
    long sequence,
    double captureFpgaS,
    int reason,
    boolean fused,
    boolean fusionEnabled)
    implements StructSerializable {
  private static final String SCHEMA =
      new SchemaBuilder()
          .addField("captureHigh", "int64")
          .addField("captureLow", "int64")
          .addField("sequence", "int64")
          .addField("captureFpgaS", "double")
          .addField("reason", "int32")
          .addField("fused", "bool")
          .addField("fusionEnabled", "bool")
          .build();

  @SuppressWarnings("ConstantNaming") // WPILib discovers this field by name.
  public static final Struct<RobotDecision> struct =
      new Struct<>() {
        @Override
        public Struct<?>[] getNested() {
          return new Struct<?>[] {};
        }

        @Override
        public String getSchema() {
          return SCHEMA;
        }

        @Override
        public int getSize() {
          return 38;
        }

        @Override
        public Class<RobotDecision> getTypeClass() {
          return RobotDecision.class;
        }

        @Override
        public String getTypeName() {
          return "SpotRobotDecision";
        }

        @Override
        public boolean isImmutable() {
          return true;
        }

        @Override
        public void pack(ByteBuffer buffer, RobotDecision value) {
          buffer.putLong(value.captureHigh());
          buffer.putLong(value.captureLow());
          buffer.putLong(value.sequence());
          buffer.putDouble(value.captureFpgaS());
          buffer.putInt(value.reason());
          buffer.put((byte) (value.fused() ? 1 : 0));
          buffer.put((byte) (value.fusionEnabled() ? 1 : 0));
        }

        @Override
        public RobotDecision unpack(ByteBuffer buffer) {
          return new RobotDecision(
              buffer.getLong(),
              buffer.getLong(),
              buffer.getLong(),
              buffer.getDouble(),
              buffer.getInt(),
              buffer.get() != 0,
              buffer.get() != 0);
        }
      };
}
