package com.team581.vision.spot.data;

import java.util.List;

public record FieldLayout(String origin, double lengthM, double widthM, List<FieldTag> tags) {
  public FieldLayout {
    tags = List.copyOf(tags);
  }
}
