#!/bin/bash
# Data of Fig. 4 for ARENA (default hyperparameters): its voting weights follow the risks with Eq. 11 (risk sweep tests),
# 10 runs per risk level. Writes the risks report (report_3d_arena_risk_sweep_*.csv), same format as moar3d_risk_sweep.sh.
source "$(dirname "$(readlink -f "$0")")/baseline_settings.sh"

run_risk_sweep arena "" arena_risk_sweep 10
