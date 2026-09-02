# Git Instructions

Use these instructions for Git operations and when drafting commits or MR/PR
content.

## Safety

- Inspect the working tree before making changes.
- Preserve unrelated user changes and untracked files.
- Do not create commits, branches, tags, or stashes unless explicitly asked.
- Do not amend, rebase, reset, force-push, or discard changes unless explicitly
  asked.
- Do not modify `.git/` directly.

## Commit Preparation

- Follow the commit format in `CONTRIBUTING.md`.
- Base the subject on the actual diff and select the primary affected component
  as the scope.
- Keep each requested commit focused and do not include unrelated changes.
- Before committing, inspect the staged diff and report the validation performed.

## MR/PR Preparation

- Follow the title and description formats in `CONTRIBUTING.md`.
- Summarize the complete proposed change, not only the latest commit.
- Derive the `Changes` section from the actual diff.
- Report only validation that was actually performed. Include failures,
  skipped checks, and environment limitations.
- Call out public API, configuration, ROS 2 topic/message, model-service, and
  hardware compatibility impact when applicable.
- Do not include credentials, private URLs, personal filesystem paths, or other
  non-public information.
