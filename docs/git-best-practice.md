# Git best practices

## Writing commits

- Commit messages should be concise and descriptive
  - Bad: "Fix swerve"
  - Good: "Fix swerve snaps bug"
  - Bad: "Intake changes"
  - Good: "Add intake logs and voltage logic"
- When it's possible, ensure each commit contains a single change
  - Bad: One commit is "Fix swerve snaps bug and tune shooter PID"
  - Good: One commit is "Fix swerve snaps bug" and another is "Tune shooter PID"

## Branches and PRs

- Keep changes on the `main` branch as much as possible
  - Only create PRs when changes can't be finished in a single meeting
    - We want to push code every day, but we can't push broken code to `main`
- Pull changes from the `main` branch regularly during development in shop
  - You can use `git config --global rebase.autoStash true` to automatically stash changes when pulling changes
