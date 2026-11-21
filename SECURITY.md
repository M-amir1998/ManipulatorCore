# Security policy

## Scope

ManipulatorCore is an algorithm library for research, education and prototyping. It is **not**
certified for safety-critical use: do not rely on it alone to command physical robots without
independent safety layers (hardware limits, safety-rated monitoring, emergency stops).

Relevant security concerns include memory-safety defects (out-of-bounds access, use after free),
crashes or hangs triggered by untrusted input — in particular the URDF/XML parser
(`mc::fromURDF`, `mc::loadURDF`) — and supply-chain issues in the build (fetched dependencies are
pinned by version and SHA-256).

## Reporting a vulnerability

Please report suspected vulnerabilities privately through GitHub's
[private vulnerability reporting](https://github.com/M-amir1998/ManipulatorCore/security/advisories/new)
instead of opening a public issue. Include a description, the affected version or commit, and a
minimal reproducer if possible. You can expect an acknowledgement within a week.

## Supported versions

Only the latest release on the `main` branch receives fixes.
