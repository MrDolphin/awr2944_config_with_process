# Validation record

- [x] MATLAB R2025a `test_run_v02.m`: 2 passed, 0 failed, 0 incomplete (3.5685 s).
- [x] MATLAB generated all five Hs<=1.00 m cases with `WindDirection=90 deg` and seed 101.
- [x] Python accepted the MATLAB wind-direction metadata and analyzed all five HDF5 files.
- [x] Hs checks passed: 5/5.
- [x] One non-positive radar-clearance warning reviewed and classified as an intentional failure boundary.
- [x] Comparison and state-3 upper-case figures manually reviewed.
- [x] Vessel-forward configured direction and dominant spectral-peak errors recorded.
- [ ] Multi-seed direction statistics completed.
- [ ] Multiple configured vessel directions completed.

This baseline accepts the first vessel-forward V0.2-B kinematics loop only. It does not authorize a claim that full V0.2-B, sea-clutter IQ or Doppler validation is complete.
