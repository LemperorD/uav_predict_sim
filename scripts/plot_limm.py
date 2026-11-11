#!/usr/bin/env python3
"""Plot scientific diagnostics from a uav-limm train output directory."""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--run", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    training = json.loads((args.run/"training.json").read_text())
    metrics = json.loads((args.run/"metrics.json").read_text())
    dt = training["metadata"]["config"]["dt"]
    figure, axes = plt.subplots(1, 3, figsize=(15, 4.3))
    for ax, stage in zip(axes, ("upstream", "fusion")):
        rows = [row for row in training["epochs"] if row["stage"] == stage]
        for key, label in (("train_mse", "Training"), ("val_mse", "Validation")):
            ax.plot([row["epoch"] for row in rows], [row[key] for row in rows], "o-", label=label)
        ax.set(title=f"{stage.capitalize()} normalized feature loss", xlabel="Epoch", ylabel="MSE")
        ax.legend()
    for name in ("position", "velocity", "acceleration", "limm", "cv", "ca"):
        values = metrics[name]["de_by_step_m"]
        axes[2].plot(np.arange(1, len(values)+1)*dt, values,
                     label=name.upper(), linewidth=2.5 if name == "limm" else 1.3)
    axes[2].set(title="Held-out synthetic flights", xlabel="Prediction horizon (s)", ylabel="Displacement error (m)")
    axes[2].legend(ncol=2)
    for ax in axes:
        ax.grid(alpha=.25)
    figure.suptitle("LIMM engineering validation (synthetic data; not the paper benchmark)", fontsize=12)
    figure.tight_layout()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(args.output, dpi=180)
    plt.close(figure)


if __name__ == "__main__":
    main()
