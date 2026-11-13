package com.team581.vision.spot.structs;

import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructGenerator.SchemaBuilder;
import edu.wpi.first.util.struct.StructSerializable;
import java.nio.ByteBuffer;

public record Heartbeat(long revisionHigh, long revisionLow, boolean ready)
    implements StructSerializable {
  private static final String SCHEMA =
      new SchemaBuilder()
          .addField("revisionHigh", "int64")
          .addField("revisionLow", "int64")
          .addField("ready", "bool")
          .build();

  @SuppressWarnings("ConstantNaming") // WPILib discovers this field by name.
  public static final Struct<Heartbeat> struct =
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
          return 17;
        }

        @Override
        public Class<Heartbeat> getTypeClass() {
          return Heartbeat.class;
        }

        @Override
        public String getTypeName() {
          return "SpotHeartbeat";
        }

        @Override
        public boolean isImmutable() {
          return true;
        }

        @Override
        public void pack(ByteBuffer buffer, Heartbeat value) {
          buffer.putLong(value.revisionHigh());
          buffer.putLong(value.revisionLow());
          buffer.put((byte) (value.ready() ? 1 : 0));
        }

        @Override
        public Heartbeat unpack(ByteBuffer buffer) {
          return new Heartbeat(buffer.getLong(), buffer.getLong(), buffer.get() != 0);
        }
      };
}
