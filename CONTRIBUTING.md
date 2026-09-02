# Contributing to RoboOrchard

Thank you for contributing to RoboOrchard. GitHub pull requests and GitLab
merge requests are referred to collectively as MR/PRs in this document.

## Commit Format

Write each commit subject using:

```text
<type>(<scope>): <Description>
```

Allowed types are:

- `feat`: Add user-visible functionality.
- `fix`: Correct a defect.
- `bugfix`: Correct a defect when preserving the existing repository wording.
- `docs`: Change documentation only.
- `style`: Change formatting without changing behavior.
- `refactor`: Restructure code without intentionally changing behavior.
- `perf`: Improve performance.
- `test`: Add or change tests.
- `chore`: Perform maintenance not covered by another type.
- `scm`: Change repository automation, CI, or development tooling.

Use a short, lowercase scope naming the primary affected component, such as
`deploy`, `data`, `teleop`, `marvin`, `inference-app`, `holobrain`, or `scm`.
Use `repo` for repository-wide changes and avoid combining multiple scopes.

Write the description in imperative form, start it with a capital letter,
keep it concise, and do not end it with a period.

Example:

```text
fix(deploy): Reject malformed model responses
```

## MR/PR Title

Use the same `<type>(<scope>): <Description>` format for the MR/PR title. The
title must summarize the complete change rather than repeat the subject of the
latest commit.

The accepted title types are enforced by `scm/qac/check_mr_title.py`.

## MR/PR Description

Use the following sections when applicable:

```md
## Summary

<!-- Explain the purpose and user-visible result. -->

## Changes

<!-- List the important implementation and documentation changes. -->

## Validation

<!-- List commands actually run and their results. State what was not run. -->

## Compatibility and Risks

<!-- Describe compatibility impact and known risks, or write "None". -->
```

For robot or ROS 2 changes, include relevant topic, message, configuration,
model-service, hardware, and runtime implications. Clearly identify validation
that requires unavailable hardware, ROS 2 setup, model services, or other
external resources.
