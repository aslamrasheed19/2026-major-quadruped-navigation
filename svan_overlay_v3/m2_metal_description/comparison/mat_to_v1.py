#!/usr/bin/env python3
"""mat_to_v1.py -- convert a svan_simulate result (results/<scenario>.mat, saved by run_all / run_scenario_to_csv) to a svan_log v1 CSV
without starting MATLAB.   usage:  mat_to_v1.py results/trot.mat trot out.csv     (scenario name: stand | trot)"""
import os, sys
import numpy as np
import scipy.io as sio
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "scripts"))
import svan_log as L

mat, scn, dst = sys.argv[1:4]
o = sio.loadmat(mat, squeeze_me=True, struct_as_record=False)["out"]
opts = o.opts
L.write_matrix(dst, "matlab", scn, o.t, o.p.T, o.rpy.T, o.cmd.T, o.Fn.T, o.q.T, o.qd.T, o.q_ref.T, o.tau.T,
               meta=dict(T=opts.T, vx=opts.vx, t_walk=opts.t_walk, ctrl_hz=500, latency_ticks=opts.latency, physics_dt=opts.dt,
                         converted_from=os.path.basename(mat)))
print("wrote", dst)
