package com.team581.vision.spot.structs;

import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructGenerator.SchemaBuilder;
import edu.wpi.first.util.struct.StructSerializable;
import java.nio.ByteBuffer;

public record CameraSettings(boolean enabled, boolean required) implements StructSerializable {
  private static final String SCHEMA =
      new SchemaBuilder().addField("enabled", "bool").addField("required", "bool").build();

  @SuppressWarnings("ConstantNaming") // WPILib discovers this field by name.
  public static final Struct<CameraSettings> struct =
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
          return 2;
        }

        @Override
        public Class<CameraSettings> getTypeClass() {
          return CameraSettings.class;
        }

        @Override
        public String getTypeName() {
          return "SpotCameraSettings";
        }

        @Override
        public boolean isImmutable() {
          return true;
        }

        @Override
        public void pack(ByteBuffer buffer, CameraSettings value) {
          buffer.put((byte) (value.enabled() ? 1 : 0));
          buffer.put((byte) (value.required() ? 1 : 0));
        }

        @Override
        public CameraSettings unpack(ByteBuffer buffer) {
          return new CameraSettings(buffer.get() != 0, buffer.get() != 0);
        }
      };
}
