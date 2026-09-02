# Repository Guidelines

## Project Structure & Module Organization

This repository is currently a minimal scaffold. The only established directory is `reference/`, reserved for supporting specifications, design notes, sample data, or other material that informs implementation but is not shipped as application code.

As the project grows, keep production code under `src/`, automated tests under `tests/`, and static resources under `assets/`. Mirror source paths in the test tree—for example, test `src/control/motor.py` in `tests/control/test_motor.py`. Avoid committing generated output; place it in a clearly named directory such as `build/` and add that directory to `.gitignore`.

## Build, Test, and Development Commands

No build system, dependency manifest, or test runner is configured yet. Until tooling is added, useful repository checks include:

- `find . -maxdepth 3 -type f` — review the current file layout.
- `git diff --check` — detect trailing whitespace and malformed conflict markers once the repository is initialized with Git.

When adding a toolchain, provide one documented entry point (preferably `make build`, `make test`, and `make lint`) and update this section in the same change.

## Coding Style & Naming Conventions

Use the standard formatter for the chosen language and commit its configuration. Default to spaces, UTF-8, LF line endings, and a final newline. Name files and modules descriptively: `snake_case` for Python, `kebab-case` for documentation and assets, and `PascalCase` for types where the language convention supports it. Keep functions focused and comments centered on intent rather than restating code.

## Testing Guidelines

Add tests with every behavioral change or bug fix. Use the ecosystem-standard test framework selected by the first implementation, and document installation and execution commands here. Prefer deterministic unit tests; isolate hardware, network, time, and filesystem dependencies behind fixtures or fakes. Name tests after observable behavior, such as `test_stops_motor_on_timeout`.

## Commit & Pull Request Guidelines

No Git history is available to infer an existing convention. Use short, imperative commit subjects, optionally with Conventional Commit prefixes (for example, `feat: add motor timeout handling`). Keep commits scoped and include related tests and documentation.

Pull requests should explain the problem, summarize the solution, list verification performed, and link relevant issues. Include screenshots or logs when behavior or output changes, and call out new dependencies or configuration requirements.
