#!/usr/bin/env bash
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
