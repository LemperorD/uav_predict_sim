import argparse
import csv
from dataclasses import fields
import json
from pathlib import Path

import torch

from .config import ModelConfig
from .data import Windows, load_splits, resample_csv
from .synthetic import generate
from .training import evaluate, load_checkpoint, seed_everything, train


def main():
    parser = argparse.ArgumentParser(description="Tang et al. 2025 differential Transformer-LSTM + LIMM")
    commands = parser.add_subparsers(dest="command", required=True)
    synthetic = commands.add_parser("generate", help="Generate disjoint synthetic flights (not paper data)")
    synthetic.add_argument("--output", required=True)
    synthetic.add_argument("--seed", type=int, default=42)
    synthetic.add_argument("--samples", type=int, default=320)
    synthetic.add_argument("--dt", type=float, default=1.)
    training = commands.add_parser("train", help="Two-stage training, chronological or explicit splits")
    training.add_argument("--data", nargs="+", required=True)
    training.add_argument("--output", required=True)
    training.add_argument("--config", help="JSON object of ModelConfig fields")
    for field in fields(ModelConfig):
        training.add_argument("--"+field.name.replace("_", "-"), type=type(field.default), default=None)
    training.add_argument("--upstream-epochs", type=int, default=50)
    training.add_argument("--fusion-epochs", type=int, default=50)
    training.add_argument("--patience", type=int, default=10)
    training.add_argument("--seed", type=int, default=42)
    training.add_argument("--batch-size", type=int, default=128)
    training.add_argument("--stride", type=int, default=1)
    training.add_argument("--device", default="cpu")
    training.add_argument("--threads", type=int, default=1)
    for name in ("evaluate", "predict"):
        sub = commands.add_parser(name, help="Use all windows of the supplied files (no re-split)")
        sub.add_argument("--checkpoint", required=True)
        sub.add_argument("--data", nargs="+", required=True)
        sub.add_argument("--output", required=True)
        sub.add_argument("--device", default="cpu")
        sub.add_argument("--stride", type=int, default=1)
        sub.add_argument("--threads", type=int, default=1)
    args = parser.parse_args()
    if args.command == "generate":
        generate(args.output, args.seed, args.samples, args.dt)
        return
    if args.command == "train":
        options = json.loads(Path(args.config).read_text()) if args.config else {}
        options.update({f.name: getattr(args, f.name) for f in fields(ModelConfig)
                        if getattr(args, f.name) is not None})
        config = ModelConfig(**options)
        splits = load_splits(args.data, config, args.stride)
        _, metrics = train(config, splits, args.output, args.data, args.upstream_epochs,
                           args.fusion_epochs, args.batch_size, args.seed, args.device,
                           args.threads, args.patience)
        print(json.dumps(metrics, indent=2))
        return
    seed_everything(42, args.threads)
    model, _ = load_checkpoint(args.checkpoint, args.device)
    paths = sorted({p for root in map(Path, args.data) for p in
                    (root.glob("*.csv") if root.is_dir() else [root])})
    flights = [resample_csv(p, model.config.dt) for p in paths]
    dataset = Windows(flights, model.config.observation, model.config.horizon, args.stride)
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    if args.command == "evaluate":
        with output.open("x") as stream:
            json.dump(evaluate(model, dataset, device=args.device), stream, indent=2)
        return
    with output.open("x", newline="") as stream, torch.inference_mode():
        writer = csv.writer(stream)
        writer.writerow(["flight", "origin_t", "target_t", "pred_x", "pred_y", "pred_z",
                         "truth_x", "truth_y", "truth_z", "error"])
        for index, (f, start) in enumerate(dataset.indices):
            history, future = dataset[index]
            prediction = model(history[None].to(args.device))["positions"][0].cpu()
            origin = flights[f][0][start+model.config.observation-1]
            for k, (predicted, truth) in enumerate(zip(prediction, future)):
                writer.writerow([str(paths[f]), origin, origin+(k+1)*model.config.dt,
                                 *predicted.tolist(), *truth.tolist(), (predicted-truth).norm().item()])


if __name__ == "__main__":
    main()
