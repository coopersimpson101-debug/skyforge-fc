## What does this change?

<!-- One or two sentences. Link the issue it fixes, e.g. "Fixes #12". -->

## How was it tested?

- [ ] `python tools/build.py test && python tools/build.py sim && python -m pytest tests/test_sim_py.py` passes locally
- [ ] New or updated tests cover the change
- [ ] Simulated results are labelled as simulated; hardware results say what hardware was used
- [ ] Touches arming / failsafe / mixer / motor output: bench-tested with **props off** (describe below)
- [ ] README figures regenerated with `docs/assets/make_figures.py` if results changed

## Notes for the reviewer

<!-- Anything non-obvious: trade-offs, follow-ups, measurements. -->
