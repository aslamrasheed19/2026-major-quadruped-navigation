#!/usr/bin/env python3
"""svan_compare.py -- check and compare svan_log v1 files (Gazebo / MATLAB / Simscape / ...).

  svan_compare.py LOG [LOG ...]                    metrics table (+ checks), one column per log
  svan_compare.py LOG [LOG ...] --plot out.png     also an overlay figure (window --win t0 t1, default 8 10 s)

The label of a log is its '# source=' line (gazebo, matlab, simscape, mujoco ...), or the file name when two logs share a source.
Metrics are computed over the steady part of the scenario (trot: 6-12 s; stand: 4-6 s) and are meant to be compared between simulators:
what matters is whether they are of the same size, not whether every sample agrees (the gait is open loop and contact models differ)."""
import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "scripts"))
import svan_log as L

JT = ("hip", "thigh", "calf")
LIM = np.array([11.92, 11.92, 23.84])


def jcols(a):                                            # N x 12 -> N x 4 x 3
    return a.reshape(len(a), 4, 3)


def gait_hz(t, x):
    x = x - x.mean()
    f = np.fft.rfftfreq(len(x), t[1] - t[0]); P = np.abs(np.fft.rfft(x * np.hanning(len(x))))
    m = (f > 0.5) & (f < 5.0)
    return f[m][np.argmax(P[m])] if m.any() else float("nan")


def metrics(lg):
    scn = lg.meta.get("scenario", "trot")
    t0 = 6.0 if scn == "trot" else 4.0
    w = lg.t >= t0 - 1e-9
    t = lg.t[w]
    M = {}
    M["rows / duration [s]"] = "%d / %.2f" % (len(lg.t), lg.t[-1])
    if scn == "trot":
        k = np.polyfit(t, lg.pos[w, 0], 1)
        M["forward speed 6-12 s [m/s]"] = k[0]
        M["x at end [m]"] = lg.pos[-1, 0]
        M["y at end (drift) [m]"] = lg.pos[-1, 1]
        M["yaw at end [rad]"] = lg.rpy[-1, 2]
    M["base height mean [m]"] = lg.pos[w, 2].mean()
    M["roll RMS [rad]"] = np.sqrt(np.mean((lg.rpy[w, 0] - lg.rpy[w, 0].mean()) ** 2))
    M["pitch RMS [rad]"] = np.sqrt(np.mean((lg.rpy[w, 1] - lg.rpy[w, 1].mean()) ** 2))
    if scn == "trot":
        M["gait frequency (FR thigh) [Hz]"] = gait_hz(t, lg["q_FR_thigh"][w])
    tau, q, qr = jcols(lg.tau[w]), jcols(lg.q[w]), jcols(lg.qref[w])
    for j, n in enumerate(JT):
        M["tau RMS %s (4 legs) [Nm]" % n] = np.sqrt(np.mean(tau[:, :, j] ** 2))
        M["tau peak %s [Nm]" % n] = np.max(np.abs(tau[:, :, j]))
    for j, n in enumerate(JT):
        M["track err RMS %s [rad]" % n] = np.sqrt(np.mean((qr[:, :, j] - q[:, :, j]) ** 2))
    M["saturated samples [%]"] = 100.0 * np.mean(np.any(np.abs(jcols(lg.tau[w])) >= LIM - 1e-6, axis=1))
    if scn == "stand":
        ff = np.array([1.476, 1.849, 5.413, -1.476, 1.849, 5.413, 1.296, 1.719, 5.006, -1.296, 1.719, 5.006])
        M["max |mean tau - FF_STAND| [Nm]"] = np.max(np.abs(lg.tau[w].mean(0) - ff))
    return M


def checks(lg):
    msg = []
    d = np.diff(lg.t)
    if not np.allclose(d, L.LOG_DT, atol=2e-3): msg.append("time step is not 10 ms (min %.4f, max %.4f)" % (d.min(), d.max()))
    if abs(lg.t[0]) > 1e-6: msg.append("t does not start at 0 (%.4f)" % lg.t[0])
    bad = np.isnan(np.delete(lg.data, [10, 11, 12, 13], axis=1)).any(axis=0)
    if bad.any(): msg.append("NaN in: " + ", ".join(L.HEADER[i] for i in np.where(np.insert(bad, 10, [False] * 4))[0][:5]))
    if np.any(np.abs(lg.q) > 3.2): msg.append("a joint angle is outside +-3.2 rad (wrong unit or offset?)")
    if np.abs(lg.q[len(lg.q) // 2, [1, 4, 7, 10]]).min() < 0.3: msg.append("thigh angles are near 0 at mid-run: zero-offset joints? (expected ~0.68 rad)")
    if np.max(np.abs(lg.tau)) > 30: msg.append("torque above 30 Nm: wrong unit?")
    return msg


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("logs", nargs="+"); ap.add_argument("--plot"); ap.add_argument("--win", nargs=2, type=float, default=[8.0, 10.0])
    a = ap.parse_args()
    logs = [L.Log(p) for p in a.logs]
    labels = [lg.meta.get("source", "?") for lg in logs]
    if len(set(labels)) < len(labels): labels = [os.path.basename(p) for p in a.logs]
    scn = {lg.meta.get("scenario") for lg in logs}
    if len(scn) > 1: print("WARNING: logs are from different scenarios: %s" % scn)
    for lb, lg in zip(labels, logs):
        for m in checks(lg): print("CHECK [%s] %s" % (lb, m))
    Ms = [metrics(lg) for lg in logs]
    keys = list(Ms[0].keys())
    w = max(len(k) for k in keys)
    print("%-*s " % (w, "") + "".join("%14s" % lb[:13] for lb in labels))
    for k in keys:
        print("%-*s " % (w, k) + "".join("%14s" % (v if isinstance(v, str) else "%.4g" % v) for v in (M.get(k, float("nan")) for M in Ms)))
    if a.plot:
        import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
        fig, ax = plt.subplots(5, 1, figsize=(9, 11), sharex=True)
        t0, t1 = a.win
        for lb, lg, c in zip(labels, logs, ("C0", "C1", "C2", "C3")):
            m = (lg.t >= t0) & (lg.t <= t1)
            ax[0].plot(lg.t[m], lg["q_FR_thigh"][m], c, label=lb)
            ax[1].plot(lg.t[m], lg["q_FR_calf"][m], c)
            ax[2].plot(lg.t[m], lg["tau_FR_thigh"][m], c)
            ax[3].plot(lg.t[m], lg["tau_FR_calf"][m], c)
            ax[4].plot(lg.t[m], lg.pos[m, 2], c)
        for i, y in enumerate(("FR thigh q [rad]", "FR calf q [rad]", "FR thigh tau [Nm]", "FR calf tau [Nm]", "base height [m]")):
            ax[i].set_ylabel(y); ax[i].grid(alpha=0.3)
        ax[0].legend(ncol=len(logs)); ax[-1].set_xlabel("time [s]")
        fig.suptitle("svan_log v1 comparison, window %.1f-%.1f s (trot is open loop: compare shape, size and period, not phase)" % (t0, t1), fontsize=9)
        fig.tight_layout(); fig.savefig(a.plot, dpi=130); print("wrote", a.plot)


if __name__ == "__main__":
    main()
