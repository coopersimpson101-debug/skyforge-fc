# Security policy

## Supported versions

Only the latest commit on `main` is supported. This is a research/portfolio project that has
**not flown yet**; do not rely on it for anything critical.

## Scope

Security issues in a flight controller are mostly **safety** issues. Please report privately:

- anything that can make the motors spin while disarmed, or bypass an arming check or the failsafe;
- parser bugs in the CRSF or CLI input paths (out-of-bounds reads/writes, hangs or crashes from malformed frames);
- config handling that could load corrupted parameters without the CRC32 check catching it;
- anything that lets the CLI or the `bl` (bootloader) command be triggered while armed.

## Reporting a vulnerability

Please use **GitHub private vulnerability reporting**: open the repository's **Security** tab and click
**Report a vulnerability** (https://github.com/coopersimpson101-debug/skyforge-fc/security/advisories/new).
Do not open a public issue, pull request or discussion for an unfixed safety or security problem.

Please include:

- a description of the issue and its impact;
- steps to reproduce, ideally a failing host test or simulator scenario;
- the commit hash you tested, and the hardware if any.

## What to expect

- an acknowledgement within **7 days**;
- an assessment and, if confirmed, a fix plan within **30 days**;
- coordinated disclosure once a fix is on `main`;
- credit in the advisory and release notes, if you want it.

Thank you for helping keep the project and its users safe.
