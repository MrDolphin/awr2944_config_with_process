# V0.2-A seed-101 acceptance baseline

> Historical note: this accepted V0.2-A run predates the project decision to
> cap state-3 Hs at 1.00 m. Its 1.20 m stress case is preserved as historical
> evidence only and is no longer valid input for the current configuration.

This directory preserves the small, reviewable evidence from the manually accepted MATLAB-to-Python V0.2-A run. The full HDF5 files and figures remain under the ignored `results/` tree.

## Source runs

- MATLAB producer: `results/matlab/v02_quick_seed101`
- Python analyzer: `results/python/v02_quick_seed101_analysis_v2`
- Random seed: 101
- Cases: `ss0_flat`, `ss1_rippled`, `ss2_normal`, `ss3_nominal`, `ss3_upper`
- Radar installation height: 1 m above the mean sea plane

## Acceptance interpretation

- Target and achieved significant wave heights pass for all five cases.
- Wave-induced grazing-angle temporal variability rises monotonically from effectively 0 degrees for flat water to about 7.95 degrees for the sea-state-3 upper case.
- The sea-state-3 nominal case has only 0.006814 m minimum sampled radar clearance and is a near-contact boundary at this installation height.
- The sea-state-3 upper case has -0.402145 m clearance and invalid geometry. It is retained as a failure-boundary test, not a normal radar operating condition.
- The sea-state-3 height fields are amplitude-scaled from the same seeded spectral realization to meet the target Hs values. They are controlled stress cases, not independently equilibrated measured seas.
- This baseline validates dynamic sea-surface truth and derived geometry only. It does not yet validate 77 GHz sea-clutter power, complex IQ, Doppler spectra, CFAR detections, or measured AWR2944P performance.

See `summary.json` for numerical evidence and `validation.md` for the recorded manual acceptance.
