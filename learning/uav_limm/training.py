"""Two-stage optimization, portable weight checkpoints and physical metrics."""
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import platform
import random

import numpy as np
import torch
from torch.utils.data import DataLoader

from .config import ModelConfig
from .limm import LIMM


def seed_everything(seed, threads=1):
    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)
    torch.set_num_threads(threads)
    torch.use_deterministic_algorithms(True)


def load_checkpoint(path, device="cpu"):
    payload = torch.load(path, map_location=device, weights_only=True)
    if payload.get("format") != 1:
        raise ValueError("Unsupported checkpoint format")
    model = LIMM(ModelConfig(**payload["config"])).to(device)
    model.load_state_dict(payload["state_dict"])
    model.eval()
    return model, payload


def save_checkpoint(path, model, metadata):
    torch.save({"format": 1, "config": asdict(model.config),
                "state_dict": model.state_dict(), "metadata": metadata}, path)


def source_manifest(paths):
    files = sorted({p.resolve() for root in map(Path, paths) for p in
                    (root.rglob("*.csv") if root.is_dir() else [root])})
    return [{"path": str(p), "sha256": hashlib.sha256(p.read_bytes()).hexdigest()}
            for p in files]


def optimize(model, training, validation, epochs, batch_size, device, checkpoint,
             metadata, stage, patience=10):
    if epochs < 1 or batch_size < 1 or patience < 1:
        raise ValueError("epochs, batch_size, patience must be positive")
    loader = DataLoader(training, batch_size=batch_size, shuffle=True)
    valid = DataLoader(validation, batch_size=batch_size)
    optimizer = torch.optim.AdamW([p for p in model.parameters() if p.requires_grad],
                                 lr=.001, weight_decay=.001)
    scheduler = torch.optim.lr_scheduler.LambdaLR(optimizer, lambda step: .9**(step//6000))
    best, stale, records = float("inf"), 0, []
    for epoch in range(epochs):
        model.train()
        total = 0.
        for history, future in loader:
            history, future = history.to(device), future.to(device)
            optimizer.zero_grad(set_to_none=True)
            loss = model.loss(history, future)
            if not torch.isfinite(loss):
                raise FloatingPointError(f"Nonfinite {stage} loss")
            loss.backward()
            # Reproduction choice: prevent exploding recurrent gradients.
            torch.nn.utils.clip_grad_norm_(model.parameters(), 10., error_if_nonfinite=True)
            optimizer.step()
            scheduler.step()
            total += loss.item()*len(history)
        model.eval()
        score = 0.
        with torch.inference_mode():
            for history, future in valid:
                score += model.loss(history.to(device), future.to(device)).item()*len(history)
        score /= len(validation)
        record = {"stage": stage, "epoch": epoch+1,
                  "train_mse": total/len(training), "val_mse": score}
        records.append(record)
        print(json.dumps(record), flush=True)
        if not np.isfinite(score):
            raise FloatingPointError("Nonfinite validation loss")
        if score < best:
            best, stale = score, 0
            # The upstream stage optimizes only its submodule; final file uses LIMM.
            torch.save(model.state_dict(), checkpoint)
        else:
            stale += 1
            if stale >= patience:
                break
    model.load_state_dict(torch.load(checkpoint, map_location=device, weights_only=True))
    return records


@torch.inference_mode()
def evaluate(model, dataset, batch_size=128, device="cpu"):
    model.eval()
    errors = {name: [] for name in ("position", "velocity", "acceleration", "limm", "cv", "ca")}
    dt, horizon = model.config.dt, model.config.horizon
    for history, future in DataLoader(dataset, batch_size=batch_size):
        history, future = history.to(device), future.to(device)
        prediction = model(history)
        baselines = prediction["upstream_positions"]
        curves = {name: baselines[:, :, i] for i, name in enumerate(errors) if i < 3}
        curves["limm"] = prediction["positions"]
        v = (history[:, -1:] - history[:, -2:-1])/dt
        a = (history[:, -1:]-2*history[:, -2:-1]+history[:, -3:-2])/dt**2
        k = torch.arange(1, horizon+1, device=device)[None, :, None]
        curves["cv"] = history[:, -1:]+v*k*dt
        # Same discrete acceleration convention as Eq.(1), not continuous .5*k^2.
        curves["ca"] = curves["cv"]+a*k*(k+1)*dt**2/2
        for name, value in curves.items():
            errors[name].append((value-future).cpu())
    result = {}
    for name, chunks in errors.items():
        error = torch.cat(chunks).double()
        displacement = error.norm(dim=-1)
        result[name] = {"samples": len(error), "ade_m": displacement.mean().item(),
                        "fde_m": displacement[:, -1].mean().item(),
                        "de_by_step_m": displacement.mean(0).tolist(),
                        "final_axis_mae_m": error[:, -1].abs().mean(0).tolist(),
                        "final_axis_rmse_m": error[:, -1].square().mean(0).sqrt().tolist()}
    return result


def train(config, datasets, output, paths, upstream_epochs=50, fusion_epochs=50,
          batch_size=128, seed=42, device="cpu", threads=1, patience=10):
    seed_everything(seed, threads)
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    training, validation, testing = datasets
    model = LIMM(config).to(device)
    model.upstream.fit_scalers(training)
    metadata = {"seed": seed, "torch": str(torch.__version__), "numpy": np.__version__,
                "python": platform.python_version(), "device": device,
                "windows": dict(zip(("train", "val", "test"), map(len, datasets))),
                "sources": source_manifest(paths), "config": asdict(config),
                "batch_size": batch_size, "upstream_epochs": upstream_epochs,
                "fusion_epochs": fusion_epochs, "patience": patience, "threads": threads,
                "optimizer": "AdamW(lr=.001,weight_decay=.001)",
                "lr_schedule": "0.9**floor(step/6000)"}
    records = optimize(model.upstream, training, validation, upstream_epochs, batch_size,
                       device, output/"upstream_best.pt", metadata, "upstream", patience)
    model.freeze_upstream()
    records += optimize(model, training, validation, fusion_epochs, batch_size,
                        device, output/"fusion_best.pt", metadata, "fusion", patience)
    save_checkpoint(output/"model.pt", model, metadata)
    metrics = evaluate(model, testing, batch_size, device)
    (output/"training.json").write_text(json.dumps({"metadata": metadata, "epochs": records}, indent=2)+"\n")
    (output/"metrics.json").write_text(json.dumps(metrics, indent=2)+"\n")
    return model, metrics
