# Quad Center Fast Path Plan

1. Add a diagnostic search API used by tests to report whether exhaustive
   constraint enumeration was required.
2. Add RED tests for a regular column (no exhaustive search) and a warped
   fallback column (exhaustive search, valid result).
3. Add the validated arithmetic-mean early return.
4. Run focused transition tests, Release build, and the 2dot5 20-layer case.
