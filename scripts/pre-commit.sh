#!/usr/bin/env bash
STAGED_FILES=$(git diff --cached --name-only --diff-filter=d)
set -e
./gradlew spotlessApply
set +e
if [ -n "$STAGED_FILES" ]; then
  echo "$STAGED_FILES" | xargs git add
fi
