#!/bin/bash
# Data of Fig. 4 for MOAR-3D: its weights follow the risks with Eq. 11 (risk sweep tests), one A* search per planning.
# Deterministic, 3 runs per risk level for the planning time. Writes the risks report (report_3d_moar3d_risk_sweep_*.csv).
source "$(dirname "$(readlink -f "$0")")/baseline_settings.sh"

run_risk_sweep moar_3d "$BENCHMARK_CONFIGS_DIR/moar_3d_params.yaml" moar3d_risk_sweep 3
