package com.team581.trailblazer3.preview;

import static java.util.Objects.requireNonNull;

import com.team581.trailblazer3.preview.PreviewSchema.SegmentDescriptor;
import com.team581.trailblazer3.preview.PreviewSchema.SourceSpan;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.Set;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import org.jspecify.annotations.Nullable;

/**
 * Extracts 1-based line ranges for {@code AutoSegment} declarations, segment builder helper
 * methods, and {@code State} switch branches from Java autonomous source files.
 */
public final class SourceSpanAnalyzer {
  /** Parsed segment declaration and its raw source block. */
  public record RawSegmentSpan(
      String fieldName,
      @Nullable String literalSegmentId,
      int startLine,
      int endLine,
      String bodyText,
      List<Integer> waypointLines) {}

  private record HelperMethodSpan(
      String name,
      boolean returnsBuilder,
      int startLine,
      int endLine,
      String bodyText,
      List<Integer> waypointLines) {}

  private record SwitchCaseSpan(List<String> stateNames, int startLine, int endLine, String text) {}

  record WaypointSpan(int startLine, int endLine) {}

  private static final Pattern PACKAGE_PATTERN =
      Pattern.compile("(?m)^\\s*package\\s+([a-zA-Z0-9_.]+)\\s*;");
  private static final Pattern CLASS_PATTERN =
      Pattern.compile(
          "(?m)^\\s*(?:public\\s+|final\\s+|abstract\\s+)*class\\s+([A-Za-z_][A-Za-z0-9_]*)");
  private static final Pattern SEGMENT_FIELD_START =
      Pattern.compile(
          "^\\s*(?:(?:private|protected|public|static|final)\\s+)*AutoSegment\\s+([A-Za-z_][A-Za-z0-9_]*)\\s*=");
  private static final Pattern HELPER_METHOD_START =
      Pattern.compile(
          "^\\s*(?:(?:private|protected|public|static|final)\\s+)*AutoSegment(\\.Builder)?\\s+([A-Za-z_][A-Za-z0-9_]*)\\s*\\(");
  private static final Pattern CASE_START =
      Pattern.compile("^\\s*case\\s+([A-Za-z0-9_,\\s]+?)(?:->|:)");
  private static final Pattern DRIVE_CALL =
      Pattern.compile("\\bdrive(?:WithoutClearance)?\\s*\\(\\s*([A-Za-z_][A-Za-z0-9_]*)\\s*\\)");
  private static final Pattern LITERAL_ID =
      Pattern.compile("(?:Trailblazer\\.segment|returnToTrench|segment)\\s*\\(\\s*\"([^\"]+)\"");

  private static final Pattern WAYPOINT_CALL =
      Pattern.compile("\\.(?:through|stopAt|finishThrough|addGoal)\\s*\\(");

  /**
   * Builds {@link SegmentDescriptor} entries with declaration, helper, and state-branch line spans.
   *
   * @param source Java source text (empty if unavailable)
   * @param knownSegments ordered list of {@code (fieldName, segmentId)} pairs from the loaded class
   */
  public static List<SegmentDescriptor> analyze(
      String source, List<Map.Entry<String, String>> knownSegments) {
    requireNonNull(source);
    requireNonNull(knownSegments);
    String[] lines = source.isEmpty() ? new String[0] : source.split("\\R", -1);
    Map<String, RawSegmentSpan> rawByField = parseSegmentDeclarations(lines);
    List<HelperMethodSpan> helpers = parseHelperMethods(lines);
    List<SwitchCaseSpan> cases = parseSwitchCases(lines);

    // Map state name -> field name based on drive(FIELD_NAME) inside any case block
    Map<String, Set<String>> stateToFields = new LinkedHashMap<>();
    for (SwitchCaseSpan c : cases) {
      Matcher driveMatcher = DRIVE_CALL.matcher(c.text());
      while (driveMatcher.find()) {
        String drivenField = driveMatcher.group(1);
        for (String state : c.stateNames()) {
          stateToFields.computeIfAbsent(state, k -> new LinkedHashSet<>()).add(drivenField);
        }
      }
    }

    List<SegmentDescriptor> descriptors = new ArrayList<>();
    int index = 0;
    for (Map.Entry<String, String> entry : knownSegments) {
      String fieldName = entry.getKey();
      String segmentId = entry.getValue();
      RawSegmentSpan raw = rawByField.get(fieldName);
      if (raw == null) {
        // Fallback match by literal segment ID
        for (RawSegmentSpan candidate : rawByField.values()) {
          if (segmentId.equals(candidate.literalSegmentId())) {
            raw = candidate;
            break;
          }
        }
      }
      int startLine = raw != null ? raw.startLine() : 0;
      int endLine = raw != null ? raw.endLine() : 0;
      String bodyText = raw != null ? raw.bodyText() : "";

      List<SourceSpan> related = new ArrayList<>();
      if (startLine > 0 && endLine >= startLine) {
        related.add(new SourceSpan("DECLARATION", fieldName, startLine, endLine));
      }

      // Attach helper methods called by this segment declaration
      for (HelperMethodSpan helper : helpers) {
        Pattern callPattern = Pattern.compile("\\b" + Pattern.quote(helper.name()) + "\\s*\\(");
        if (callPattern.matcher(bodyText).find()) {
          related.add(
              new SourceSpan("HELPER_METHOD", helper.name(), helper.startLine(), helper.endLine()));
        }
      }

      // Attach switch case blocks whose state drives this segment
      for (SwitchCaseSpan caseSpan : cases) {
        boolean matchesSegment = false;
        String matchedState = "";
        for (String stateName : caseSpan.stateNames()) {
          Set<String> driven = stateToFields.get(stateName);
          if (driven != null && driven.contains(fieldName)) {
            matchesSegment = true;
            matchedState = stateName;
            break;
          }
        }
        if (matchesSegment) {
          related.add(
              new SourceSpan(
                  "STATE_BRANCH", matchedState, caseSpan.startLine(), caseSpan.endLine()));
        }
      }

      descriptors.add(
          new SegmentDescriptor(segmentId, fieldName, index++, startLine, endLine, related));
    }
    return descriptors;
  }

  /** Extracts the fully-qualified class name from Java source text, if present. */
  public static Optional<String> parseClassName(String source) {
    requireNonNull(source);
    Matcher classMatcher = CLASS_PATTERN.matcher(source);
    if (!classMatcher.find()) {
      return Optional.empty();
    }
    String simpleName = classMatcher.group(1);
    Matcher pkgMatcher = PACKAGE_PATTERN.matcher(source);
    if (pkgMatcher.find()) {
      return Optional.of(pkgMatcher.group(1) + "." + simpleName);
    }
    return Optional.of(simpleName);
  }

  /**
   * Parses raw {@code AutoSegment} declarations directly from source text when class is unloaded.
   */
  public static List<RawSegmentSpan> parseRawSegments(String source) {
    requireNonNull(source);
    String[] lines = source.isEmpty() ? new String[0] : source.split("\\R", -1);
    Map<String, RawSegmentSpan> rawByField = parseSegmentDeclarations(lines);
    List<HelperMethodSpan> helpers = parseHelperMethods(lines);
    List<RawSegmentSpan> enriched = new ArrayList<>(rawByField.size());
    for (RawSegmentSpan raw : rawByField.values()) {
      List<Integer> combinedWpLines = new ArrayList<>();
      for (HelperMethodSpan helper : helpers) {
        if (helper.returnsBuilder()
            && Pattern.compile("\\b" + Pattern.quote(helper.name()) + "\\s*\\(")
                .matcher(raw.bodyText())
                .find()) {
          combinedWpLines.addAll(helper.waypointLines());
        }
      }
      combinedWpLines.addAll(raw.waypointLines());
      for (HelperMethodSpan helper : helpers) {
        if (!helper.returnsBuilder()
            && Pattern.compile("\\b" + Pattern.quote(helper.name()) + "\\s*\\(")
                .matcher(raw.bodyText())
                .find()) {
          combinedWpLines.addAll(helper.waypointLines());
        }
      }
      enriched.add(
          new RawSegmentSpan(
              raw.fieldName(),
              raw.literalSegmentId(),
              raw.startLine(),
              raw.endLine(),
              raw.bodyText(),
              List.copyOf(combinedWpLines)));
    }
    return List.copyOf(enriched);
  }

  /**
   * Resolves the active segment ID for a 1-based editor cursor line, or {@link Optional#empty()}
   * when the cursor is outside any segment-specific region (defaulting to {@code WHOLE_AUTO}).
   */
  public static Optional<String> resolveSegmentAtCursor(
      List<SegmentDescriptor> segments, int cursorLine, @Nullable String previousSegmentId) {
    requireNonNull(segments);
    if (cursorLine <= 0 || segments.isEmpty()) {
      return Optional.empty();
    }

    // 1. Direct match inside a segment declaration
    for (SegmentDescriptor seg : segments) {
      if (seg.declarationStartLine() > 0
          && cursorLine >= seg.declarationStartLine()
          && cursorLine <= seg.declarationEndLine()) {
        return Optional.of(seg.id());
      }
    }

    // 2. Match inside state branches first (more specific than shared helpers)
    List<SegmentDescriptor> stateMatches = new ArrayList<>();
    for (SegmentDescriptor seg : segments) {
      for (SourceSpan span : seg.relatedSpans()) {
        if ("STATE_BRANCH".equals(span.kind())
            && cursorLine >= span.startLine()
            && cursorLine <= span.endLine()) {
          stateMatches.add(seg);
          break;
        }
      }
    }
    if (stateMatches.size() == 1) {
      return Optional.of(stateMatches.get(0).id());
    }
    if (stateMatches.size() > 1) {
      if (previousSegmentId != null) {
        for (SegmentDescriptor seg : stateMatches) {
          if (seg.id().equals(previousSegmentId) || seg.fieldName().equals(previousSegmentId)) {
            return Optional.of(seg.id());
          }
        }
      }
      return Optional.of(stateMatches.get(0).id());
    }

    // 3. Match inside helper methods
    List<SegmentDescriptor> helperMatches = new ArrayList<>();
    for (SegmentDescriptor seg : segments) {
      for (SourceSpan span : seg.relatedSpans()) {
        if ("HELPER_METHOD".equals(span.kind())
            && cursorLine >= span.startLine()
            && cursorLine <= span.endLine()) {
          helperMatches.add(seg);
          break;
        }
      }
    }
    if (helperMatches.size() == 1) {
      return Optional.of(helperMatches.get(0).id());
    }
    if (helperMatches.size() > 1) {
      if (previousSegmentId != null) {
        for (SegmentDescriptor seg : helperMatches) {
          if (seg.id().equals(previousSegmentId) || seg.fieldName().equals(previousSegmentId)) {
            return Optional.of(seg.id());
          }
        }
      }
      return Optional.of(helperMatches.get(0).id());
    }

    return Optional.empty();
  }

  private static List<HelperMethodSpan> parseHelperMethods(String[] lines) {
    List<HelperMethodSpan> helpers = new ArrayList<>();
    for (int i = 0; i < lines.length; i++) {
      Matcher m = HELPER_METHOD_START.matcher(lines[i]);
      if (!m.find()) {
        continue;
      }
      boolean returnsBuilder = m.group(1) != null;
      String methodName = m.group(2);
      int startLine = i + 1;
      int braceDepth = 0;
      boolean seenOpenBrace = false;
      int endLine = startLine;
      StringBuilder body = new StringBuilder();
      List<Integer> wpLines = new ArrayList<>();
      for (int j = i; j < lines.length; j++) {
        String line = stripLineComment(lines[j]);
        body.append(line).append('\n');
        Matcher wp = WAYPOINT_CALL.matcher(line);
        while (wp.find()) {
          wpLines.add(j + 1);
        }
        for (int c = 0; c < line.length(); c++) {
          char ch = line.charAt(c);
          if (ch == '{') {
            braceDepth++;
            seenOpenBrace = true;
          } else if (ch == '}') {
            braceDepth--;
          }
        }
        if (seenOpenBrace && braceDepth <= 0) {
          endLine = j + 1;
          i = j;
          break;
        }
      }
      helpers.add(
          new HelperMethodSpan(
              methodName,
              returnsBuilder,
              startLine,
              endLine,
              body.toString(),
              List.copyOf(wpLines)));
    }
    return helpers;
  }

  private static Map<String, RawSegmentSpan> parseSegmentDeclarations(String[] lines) {
    Map<String, RawSegmentSpan> result = new LinkedHashMap<>();
    for (int i = 0; i < lines.length; i++) {
      Matcher m = SEGMENT_FIELD_START.matcher(lines[i]);
      if (!m.find()) {
        continue;
      }
      String fieldName = m.group(1);
      int startLine = i + 1;
      int parenDepth = 0;
      int braceDepth = 0;
      int endLine = startLine;
      StringBuilder body = new StringBuilder();
      List<Integer> waypointLines = new ArrayList<>();

      for (int j = i; j < lines.length; j++) {
        String rawLine = lines[j];
        String line = stripLineComment(rawLine);
        body.append(line).append('\n');
        Matcher wp = WAYPOINT_CALL.matcher(line);
        while (wp.find()) {
          waypointLines.add(j + 1);
        }
        boolean endedStatement = false;
        for (int c = 0; c < line.length(); c++) {
          char ch = line.charAt(c);
          if (ch == '(') {
            parenDepth++;
          } else if (ch == ')') {
            parenDepth = Math.max(0, parenDepth - 1);
          } else if (ch == '{') {
            braceDepth++;
          } else if (ch == '}') {
            braceDepth = Math.max(0, braceDepth - 1);
          } else if (ch == ';' && parenDepth == 0 && braceDepth == 0) {
            endedStatement = true;
            break;
          }
        }
        if (endedStatement) {
          endLine = j + 1;
          i = j;
          break;
        }
      }

      String bodyText = body.toString();
      Matcher idMatcher = LITERAL_ID.matcher(bodyText);
      String literalId = idMatcher.find() ? idMatcher.group(1) : null;
      result.put(
          fieldName,
          new RawSegmentSpan(
              fieldName, literalId, startLine, endLine, bodyText, List.copyOf(waypointLines)));
    }
    return result;
  }

  private static List<SwitchCaseSpan> parseSwitchCases(String[] lines) {
    List<SwitchCaseSpan> cases = new ArrayList<>();
    for (int i = 0; i < lines.length; i++) {
      String stripped = stripLineComment(lines[i]);
      Matcher m = CASE_START.matcher(stripped);
      if (!m.find()) {
        continue;
      }
      String rawStates = m.group(1);
      List<String> stateNames = new ArrayList<>();
      for (String part : rawStates.split(",")) {
        String trimmed = part.trim();
        int dot = trimmed.lastIndexOf('.');
        if (dot >= 0) {
          trimmed = trimmed.substring(dot + 1).trim();
        }
        if (!trimmed.isEmpty()) {
          stateNames.add(trimmed);
        }
      }
      if (stateNames.isEmpty()) {
        continue;
      }
      int startLine = i + 1;
      int endLine = startLine;
      int braceDepth = 0;
      boolean seenOpenBrace = false;
      StringBuilder text = new StringBuilder();

      int arrowIdx = stripped.indexOf("->");
      String afterCase = arrowIdx >= 0 ? stripped.substring(arrowIdx + 2) : stripped;
      if (afterCase.contains("{")) {
        for (int j = i; j < lines.length; j++) {
          String line = stripLineComment(lines[j]);
          text.append(line).append('\n');
          for (int c = 0; c < line.length(); c++) {
            char ch = line.charAt(c);
            if (ch == '{') {
              braceDepth++;
              seenOpenBrace = true;
            } else if (ch == '}') {
              braceDepth--;
            }
          }
          if (seenOpenBrace && braceDepth <= 0) {
            endLine = j + 1;
            break;
          }
        }
      } else {
        for (int j = i; j < lines.length; j++) {
          String line = stripLineComment(lines[j]);
          text.append(line).append('\n');
          if (line.contains(";")) {
            endLine = j + 1;
            break;
          }
        }
      }
      cases.add(new SwitchCaseSpan(List.copyOf(stateNames), startLine, endLine, text.toString()));
    }
    return cases;
  }

  private static String stripLineComment(String line) {
    int idx = line.indexOf("//");
    return idx >= 0 ? line.substring(0, idx) : line;
  }

  static @Nullable WaypointSpan findWaypointSpan(List<WaypointSpan> spans, int line) {
    WaypointSpan match = null;
    for (WaypointSpan span : spans) {
      if (span.startLine() <= line && line <= span.endLine()) {
        if (match != null) {
          return null;
        }
        match = span;
      }
    }
    return match;
  }

  static List<WaypointSpan> waypointSpans(String source) {
    // Preserve offsets and line breaks while ignoring parentheses in comments and literals.
    String code =
        Pattern.compile(
                "//[^\\r\\n]*|/\\*[\\s\\S]*?\\*/|\"(?:\\\\.|[^\"\\\\])*\"|'(?:\\\\.|[^'\\\\])*'")
            .matcher(source)
            .replaceAll(match -> match.group().replaceAll("[^\\r\\n]", " "));
    int[] linesAt = new int[code.length()];
    int line = 1;
    for (int i = 0; i < code.length(); i++) {
      linesAt[i] = line;
      char ch = code.charAt(i);
      if (ch == '\n' || (ch == '\r' && (i + 1 == code.length() || code.charAt(i + 1) != '\n'))) {
        line++;
      }
    }
    List<WaypointSpan> spans = new ArrayList<>();
    Matcher calls = WAYPOINT_CALL.matcher(code);
    while (calls.find()) {
      int depth = 1;
      for (int i = calls.end(); i < code.length(); i++) {
        char ch = code.charAt(i);
        if (ch == '(') {
          depth++;
        } else if (ch == ')' && --depth == 0) {
          spans.add(new WaypointSpan(linesAt[calls.start()], linesAt[i]));
          break;
        }
      }
    }
    return spans;
  }

  private SourceSpanAnalyzer() {}
}
