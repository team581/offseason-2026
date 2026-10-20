#!/usr/bin/env bash
# GUI Git clients may not inherit the Java environment configured for terminals.
if [ -z "${JAVA_HOME:-}" ] && ! java -version >/dev/null 2>&1; then
  WPILIB_JAVA_HOME="$HOME/wpilib/2026/jdk"
  if [ -x "$WPILIB_JAVA_HOME/bin/java" ]; then
    export JAVA_HOME="$WPILIB_JAVA_HOME"
  fi
fi

DIRTY_BEFORE=$(git diff --name-only)
set -e
./gradlew spotlessApply
set +e
DIRTY_AFTER=$(git diff --name-only)
# Only stage files that became dirty after spotless ran (i.e. spotless modified them)
CHANGED=$(comm -13 <(echo "$DIRTY_BEFORE" | sort) <(echo "$DIRTY_AFTER" | sort))
if [ -n "$CHANGED" ]; then
  echo "$CHANGED" | xargs git add
fi
