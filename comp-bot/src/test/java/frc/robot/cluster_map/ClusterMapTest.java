package frc.robot.cluster_map;

import static org.assertj.core.api.Assertions.assertThat;

import edu.wpi.first.math.geometry.Translation2d;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Random;
import org.junit.jupiter.api.Test;

final class ClusterMapTest {
  private static ClusterMapElement element(double expiry, double x, double y) {
    return new ClusterMapElement(expiry, new Translation2d(x, y), 1.0, 50.0, 1.0);
  }

  @Test
  void closestClusterMatchesReferenceForRandomizedAndExpiredInputs() {
    Random random = new Random(581);
    for (int trial = 0; trial < 10_000; trial++) {
      double now = 10.0;
      double newExpiry = now + 2.0;
      Translation2d observed =
          new Translation2d(random.nextDouble() * 20.0, random.nextDouble() * 10.0);
      List<ClusterMapElement> clusters = new ArrayList<>();
      for (int index = 0; index < random.nextInt(51); index++) {
        clusters.add(
            element(
                random.nextBoolean() ? now - 0.01 : now + random.nextDouble() * 4.0,
                random.nextDouble() * 20.0,
                random.nextDouble() * 10.0));
      }

      // updateMap removes expired entries before searching.
      clusters.removeIf(element -> element.expiresAt() < now);
      ClusterMapElement expected = null;
      double closest = Double.POSITIVE_INFINITY;
      for (ClusterMapElement element : clusters) {
        double distance = element.clusterTranslation().getDistance(observed);
        if (element.expiresAt() != newExpiry && distance < 1.0 && distance < closest) {
          expected = element;
          closest = distance;
        }
      }
      assertThat(ClusterMap.findClosestCluster(clusters, observed, newExpiry)).isSameAs(expected);
    }
  }

  @Test
  void closestClusterReturnsEmptyAndPreservesEqualDistanceEncounterOrder() {
    Translation2d observed = new Translation2d(1.0, 1.0);
    assertThat(ClusterMap.findClosestCluster(List.of(), observed, 10.0)).isNull();

    var first = element(9.0, 0.5, 1.0);
    var second = element(9.0, 1.5, 1.0);
    assertThat(ClusterMap.findClosestCluster(List.of(first, second), observed, 10.0))
        .isSameAs(first);
  }

  @Test
  void emptyLaneCountsReturnNone() {
    assertThat(
            ClusterMap.chooseBestLane(
                new int[5],
                new int[] {
                  Integer.MAX_VALUE,
                  Integer.MAX_VALUE,
                  Integer.MAX_VALUE,
                  Integer.MAX_VALUE,
                  Integer.MAX_VALUE
                }))
        .isEqualTo(Lane.NONE);
  }

  @Test
  void equalCountsPreserveFirstEncounteredLane() {
    assertThat(
            ClusterMap.chooseBestLane(
                new int[] {10, 10, 0, 0, 0},
                new int[] {8, 2, Integer.MAX_VALUE, Integer.MAX_VALUE, Integer.MAX_VALUE}))
        .isEqualTo(Lane.LANE_1);
  }

  @Test
  void optimizedSelectionMatchesEncounterOrderedReference() {
    Random random = new Random(581);
    Lane[] lanes = {Lane.LANE_0, Lane.LANE_1, Lane.LANE_2, Lane.LANE_3, Lane.LANE_4};

    for (int trial = 0; trial < 10_000; trial++) {
      int[] counts = new int[5];
      int[] firstSeen = {
        Integer.MAX_VALUE,
        Integer.MAX_VALUE,
        Integer.MAX_VALUE,
        Integer.MAX_VALUE,
        Integer.MAX_VALUE
      };
      Map<Lane, Integer> reference = new LinkedHashMap<>();
      int observations = random.nextInt(40);
      for (int observation = 0; observation < observations; observation++) {
        int laneIndex = random.nextInt(lanes.length);
        int size = random.nextInt(51);
        counts[laneIndex] += size;
        firstSeen[laneIndex] = Math.min(firstSeen[laneIndex], observation);
        reference.merge(lanes[laneIndex], size, Integer::sum);
      }

      Lane expected = Lane.NONE;
      int largest = Integer.MIN_VALUE;
      for (var entry : reference.entrySet()) {
        if (entry.getValue() > largest) {
          expected = entry.getKey();
          largest = entry.getValue();
        }
      }
      assertThat(ClusterMap.chooseBestLane(counts, firstSeen)).isEqualTo(expected);
    }
  }
}
