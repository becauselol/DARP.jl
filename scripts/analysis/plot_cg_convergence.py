"""
scripts/analysis/plot_cg_convergence.py

Plots CG convergence curves from _iters.csv files produced by DemandCG runs.

Usage:
    python3 scripts/analysis/plot_cg_convergence.py [run_dir1] [run_dir2] ...

    If no args, searches experiments/cg_vs_ip_small/run_*/ and experiments/cg_large/run_*/

    Plots are written to experiments/plots/ (next to the experiment dirs, not in scripts/).

Outputs:
    convergence_by_size.png       — LP obj vs iter, one panel per (n, variant)
    convergence_normalized.png    — (LP obj - LP_final) / (LP_iter1 - LP_final) vs iter
    cols_added.png                — columns added per iteration
    gap_summary.png               — integrality gap (IP obj vs LP bound) by instance
"""

import sys
import os
import glob
import re
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import numpy as np

# ── Locate run directories ────────────────────────────────────────────────────

repo = os.path.join(os.path.dirname(__file__), "..", "..")
if len(sys.argv) > 1:
    run_dirs = sys.argv[1:]
else:
    run_dirs = sorted(glob.glob(os.path.join(repo, "experiments/cg_vs_ip_small/run_*/"))) + \
               sorted(glob.glob(os.path.join(repo, "experiments/cg_large/run_*/")))

if not run_dirs:
    print("No run directories found.")
    sys.exit(1)

# ── Load data ─────────────────────────────────────────────────────────────────

iters_frames = []
results_frames = []

for run_dir in run_dirs:
    for iters_path in glob.glob(os.path.join(run_dir, "DemandCG/*_iters.csv")):
        df = pd.read_csv(iters_path)
        iters_frames.append(df)

    for res_path in glob.glob(os.path.join(run_dir, "DemandCG/*.csv")):
        if res_path.endswith("_iters.csv"):
            continue
        df = pd.read_csv(res_path)
        results_frames.append(df)

if not iters_frames:
    print("No _iters.csv files found in run directories:")
    for d in run_dirs: print(" ", d)
    sys.exit(1)

iters  = pd.concat(iters_frames,   ignore_index=True)
results = pd.concat(results_frames, ignore_index=True)

# ── Parse instance metadata ───────────────────────────────────────────────────

def parse_instance(name):
    m = re.match(r"n(\d+)_k(\d+)_q(\d+)_(multi|split)_(\d+)", name)
    if m:
        return int(m.group(1)), int(m.group(2)), int(m.group(3)), m.group(4), int(m.group(5))
    return None, None, None, None, None

for df in (iters, results):
    df[["n", "k", "q", "variant", "seed"]] = pd.DataFrame(
        df["instance"].map(lambda x: parse_instance(x)).tolist(),
        index=df.index
    )

results = results.rename(columns={"objective": "ip_obj", "lp_bound": "lp_final"})

# Merge final IP obj and LP bound into iters
iters = iters.merge(
    results[["instance", "ip_obj", "lp_final", "status"]],
    on="instance", how="left"
)

out_dir = os.path.join(repo, "experiments", "plots")
os.makedirs(out_dir, exist_ok=True)

SEED_COLORS = {42: "#e41a1c", 123: "#377eb8", 999: "#4daf4a"}
VARIANT_STYLE = {"multi": "-", "split": "--"}

sizes_variants = sorted(iters[["n", "variant"]].drop_duplicates().values.tolist())

# ── Plot 1: LP objective vs iteration, one panel per (n, variant) ─────────────

ncols = 3
nrows = -(-len(sizes_variants) // ncols)  # ceil div
fig, axes = plt.subplots(nrows, ncols, figsize=(5 * ncols, 4 * nrows), squeeze=False)
axes_flat = axes.flatten()

for ax_i, (n, variant) in enumerate(sizes_variants):
    ax = axes_flat[ax_i]
    subset = iters[(iters.n == n) & (iters.variant == variant)]
    for seed, grp in subset.groupby("seed"):
        grp = grp.sort_values("iter")
        ip_obj   = grp["ip_obj"].iloc[0]
        lp_final = grp["lp_final"].iloc[0]
        status   = grp["status"].iloc[0]
        color    = SEED_COLORS.get(seed, "gray")
        ax.plot(grp["iter"], grp["lp_obj"], color=color, lw=1.8,
                label=f"seed {seed} (IP={ip_obj:.0f}, {status})")
        # Mark where LP stabilises (first iter with 0 cols added that stays 0)
        zero_mask = grp["cols_added"] == 0
        if zero_mask.any():
            first_zero = grp[zero_mask].iloc[0]
            ax.axvline(first_zero["iter"], color=color, lw=0.8, ls=":")
        # Draw horizontal IP obj line
        ax.axhline(ip_obj, color=color, lw=0.8, ls="--", alpha=0.5)

    ax.set_title(f"n={n}, {variant}", fontsize=11)
    ax.set_xlabel("CG iteration")
    ax.set_ylabel("LP objective")
    ax.legend(fontsize=7, loc="upper right")
    ax.grid(True, alpha=0.3)

for ax in axes_flat[len(sizes_variants):]:
    ax.set_visible(False)

fig.suptitle("DemandCG: LP bound convergence by instance\n"
             "(dotted vertical = first zero-column iter; dashed horizontal = IP obj)",
             fontsize=12, y=1.01)
fig.tight_layout()
out = os.path.join(out_dir, "convergence_by_size.png")
fig.savefig(out, dpi=150, bbox_inches="tight")
plt.close(fig)
print(f"Written: {out}")

# ── Plot 2: Normalised convergence (0 = start, 1 = LP converged) ─────────────

fig, axes = plt.subplots(nrows, ncols, figsize=(5 * ncols, 4 * nrows), squeeze=False)
axes_flat = axes.flatten()

for ax_i, (n, variant) in enumerate(sizes_variants):
    ax = axes_flat[ax_i]
    subset = iters[(iters.n == n) & (iters.variant == variant)]
    for seed, grp in subset.groupby("seed"):
        grp = grp.sort_values("iter")
        lp0     = grp["lp_obj"].iloc[0]
        lp_min  = grp["lp_obj"].min()
        span    = lp0 - lp_min
        if span < 1e-6:
            continue
        normed  = (grp["lp_obj"] - lp_min) / span  # 1 at start, 0 at end
        color   = SEED_COLORS.get(seed, "gray")
        ax.plot(grp["iter"], normed, color=color, lw=1.8, label=f"seed {seed}")
        zero_mask = grp["cols_added"] == 0
        if zero_mask.any():
            ax.axvline(grp[zero_mask].iloc[0]["iter"], color=color, lw=0.8, ls=":")

    ax.set_title(f"n={n}, {variant}", fontsize=11)
    ax.set_xlabel("CG iteration")
    ax.set_ylabel("Normalised LP gap\n(1=start, 0=converged)")
    ax.set_ylim(-0.05, 1.05)
    ax.legend(fontsize=7)
    ax.grid(True, alpha=0.3)

for ax in axes_flat[len(sizes_variants):]:
    ax.set_visible(False)

fig.suptitle("DemandCG: Normalised LP convergence\n"
             "(dotted = first zero-column iter)", fontsize=12, y=1.01)
fig.tight_layout()
out = os.path.join(out_dir, "convergence_normalized.png")
fig.savefig(out, dpi=150, bbox_inches="tight")
plt.close(fig)
print(f"Written: {out}")

# ── Plot 3: Columns added per iteration ───────────────────────────────────────

fig, axes = plt.subplots(nrows, ncols, figsize=(5 * ncols, 4 * nrows), squeeze=False)
axes_flat = axes.flatten()

for ax_i, (n, variant) in enumerate(sizes_variants):
    ax = axes_flat[ax_i]
    subset = iters[(iters.n == n) & (iters.variant == variant)]
    for seed, grp in subset.groupby("seed"):
        grp = grp.sort_values("iter")
        color = SEED_COLORS.get(seed, "gray")
        ax.bar(grp["iter"] + (list(SEED_COLORS.keys()).index(seed) - 1) * 0.25,
               grp["cols_added"], width=0.25, color=color, alpha=0.7, label=f"seed {seed}")

    ax.set_title(f"n={n}, {variant}", fontsize=11)
    ax.set_xlabel("CG iteration")
    ax.set_ylabel("Columns added")
    ax.legend(fontsize=7)
    ax.grid(True, alpha=0.3, axis="y")

for ax in axes_flat[len(sizes_variants):]:
    ax.set_visible(False)

fig.suptitle("DemandCG: Columns added per iteration", fontsize=12, y=1.01)
fig.tight_layout()
out = os.path.join(out_dir, "cols_added.png")
fig.savefig(out, dpi=150, bbox_inches="tight")
plt.close(fig)
print(f"Written: {out}")

# ── Plot 4: Integrality gap summary ──────────────────────────────────────────

res = results.copy()
res["gap_pct"] = np.where(
    (res["ip_obj"] > 0) & (res["lp_final"] > 0),
    100 * (res["ip_obj"] - res["lp_final"]) / res["ip_obj"],
    np.nan
)
res = res.dropna(subset=["gap_pct"]).sort_values(["variant", "n", "seed"])

fig, ax = plt.subplots(figsize=(14, 5))
x = np.arange(len(res))
colors = ["#e41a1c" if v == "multi" else "#377eb8" for v in res["variant"]]
bars = ax.bar(x, res["gap_pct"], color=colors, alpha=0.8, edgecolor="white", lw=0.5)

ax.set_xticks(x)
ax.set_xticklabels(
    [f"n{r.n}\n{r.variant[:3]}\ns{r.seed}" for r in res.itertuples()],
    fontsize=7, rotation=0
)
ax.set_ylabel("Integrality gap  (IP obj − LP bound) / IP obj  [%]")
ax.set_title("DemandCG: Integrality gap by instance\n(red = multi, blue = split)")
ax.grid(True, alpha=0.3, axis="y")
ax.axhline(0, color="black", lw=0.8)

from matplotlib.patches import Patch
ax.legend(handles=[Patch(color="#e41a1c", label="multi"),
                   Patch(color="#377eb8", label="split")], fontsize=9)

fig.tight_layout()
out = os.path.join(out_dir, "gap_summary.png")
fig.savefig(out, dpi=150, bbox_inches="tight")
plt.close(fig)
print(f"Written: {out}")

print("\nDone. All plots written to:", out_dir)
