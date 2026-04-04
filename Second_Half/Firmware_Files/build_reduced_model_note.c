/*
Reference note extracted from model-side process:
  build_reduced_model.m computes recommended leg-length feedforward using:

    ff.force_per_leg_ground_N = 0.5*(M + mp + mw)*g/cos(theta_nom)
    ff.force_per_leg_air_N    = 0.5*(mp + mw)*g/cos(theta_nom)

This file exists as firmware-side documentation trace for tuning records.
*/
