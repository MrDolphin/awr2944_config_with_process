# V0.2-B vessel-forward seed-101 Hs<=1 m baseline

This baseline preserves the reviewable evidence from the first vessel-forward V0.2-B kinematics run.

## Source runs

- MATLAB: `results/matlab/v02b_forward_seed101_hs1m`
- Python: `results/python/v02b_forward_seed101_hs1m_analysis`
- MATLAB `WindDirection`: 90 deg
- Configured vessel wave direction: 0 deg (`+y`, forward)
- Random seed: 101
- Radar height: 1 m above mean sea level
- Hs cases: 0, 0.05, 0.30, 0.85 and 1.00 m

## Accepted observations

- MATLAB runtime tests passed: 2 passed, 0 failed, 0 incomplete in 3.5685 s.
- Target/achieved Hs checks passed for all five cases.
- Dominant single spectral-peak direction errors for non-flat cases are 19.8559, 4.0050, 11.0916 and 11.0916 deg.
- The state-3 upper case has -0.163628 m minimum clearance and invalid geometry at a 1 m radar height.
- Grazing-angle temporal variability and Eulerian slant-range-rate magnitude increase with Hs.

## Scope boundary

This is a single-seed, target-Hs-scaled spectral-surface baseline. It validates vessel-axis mapping, dynamic geometry and Eulerian slant-range rate. It does not validate water-particle orbital velocity, electromagnetic sea-clutter power, complex IQ, Doppler spectra, CFAR detections or measured AWR2944P performance. Multi-seed and multi-direction acceptance remains V0.2-B work.
