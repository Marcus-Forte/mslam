---
description: Use when editing mslam C++ source or header files, especially SLAM, map, config, and registration code.
applyTo: '**/*.cc, **/*.hh'
---

<!-- Tip: Use /create-instructions in chat to generate content with agent assistance -->

Use these instructions when working on C++ source or header files in this repository.

Project context:
- This is a small C++ SLAM codebase centered on a runtime pipeline of sensor input, preprocessing, registration, map updates, and optional server/export publication.
- The code favors direct data flow and small translation units over framework-heavy abstractions.
- Configuration already flows through `SlamConfiguration`, logging through `ILog`, and map/sensor dependencies through interfaces such as `IMap`, `ILidar`, and `IImu`.

Coding guidelines:
- Keep changes local to the owning component. Prefer extending the existing SLAM, map, config, or registration class over introducing a new abstraction layer.
- Match the repository naming style: types and most methods use PascalCase, local variables use snake_case, and data members use trailing underscores.
- Prefer file-local helpers in an anonymous namespace for translation-unit-only behavior.
- Use early returns and `continue` to keep control flow flat, especially in sensor-processing loops.
- Prefer existing logging infrastructure (`logger->log(...)`) for runtime diagnostics. Reserve `std::cout` for CLI-facing entrypoint behavior such as usage or explicit startup output in `main.cc`.
- Preserve the current include organization: project headers first, then standard-library headers, separated by a blank line.
- Prefer standard smart-pointer ownership already used in the codebase: `std::unique_ptr` for exclusive ownership, `std::shared_ptr` for shared interfaces. Do not introduce raw owning pointers.
- Reuse existing configuration values instead of hardcoding behavior when a matching field already exists. If a new constant is genuinely local to one `.cc` file, define it as a `constexpr` in the anonymous namespace.
- When adding or changing configuration fields, keep `config/mslam.json`, `JsonConfig`, and validation logic in sync instead of patching only one layer.
- Keep hot-path code straightforward. Avoid unnecessary copies, virtual layering, or container churn inside per-scan and per-point processing.
- When changing protocol or published data shapes, update the owning contract first and then any generated or downstream client artifacts that depend on it.
- Preserve existing logging and timing style when touching pipeline stages, and avoid mixing unrelated behavioral changes into the same edit.

Testing and review guidance:
- When adding behavior, prefer a focused `gtest` that exercises the touched class directly.
- Small local fakes or stubs inside the test file are preferred over broad test scaffolding when validating interfaces such as `IMap`.
- In reviews, prioritize behavioral regressions in pose updates, scan preprocessing, map mutation, playback termination, and server/export side effects.