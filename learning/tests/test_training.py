import json

import numpy as np
import torch

from uav_limm.config import ModelConfig
from uav_limm.data import load_splits
from uav_limm.synthetic import generate
from uav_limm.training import evaluate, load_checkpoint, train


def test_real_two_stage_training_and_reload(tmp_path):
    config = ModelConfig(observation=8, horizon=3, hidden=8, heads=2, layers=1,
                         feedforward=16, dropout=0)
    generate(tmp_path/"flights", samples=60)
    datasets = load_splits([tmp_path/"flights"], config, stride=12)
    output = tmp_path/"run"
    model, metrics = train(config, datasets, output, [tmp_path/"flights"],
                           upstream_epochs=2, fusion_epochs=2, batch_size=32)
    restored, _ = load_checkpoint(output/"model.pt")
    history, _ = datasets[2][0]
    for key, value in model.state_dict().items():
        torch.testing.assert_close(value, restored.state_dict()[key], rtol=0, atol=0)
    # Frozen and trainable parameter flags may select different Transformer CPU
    # kernels; check float32 numerical equivalence as well as exact saved weights.
    torch.testing.assert_close(model(history[None])["positions"], restored(history[None])["positions"], rtol=1e-5, atol=1e-5)
    rescored = evaluate(restored, datasets[2])
    for name in metrics:
        for key in metrics[name]:
            np.testing.assert_allclose(rescored[name][key], metrics[name][key], rtol=1e-5, atol=1e-5)
    training = json.loads((output/"training.json").read_text())
    assert len(training["epochs"]) == 4
    assert training["metadata"]["window_stride"] == 12
    assert len(training["metadata"]["sources"]) == 11
    assert set(metrics) == {"position", "velocity", "acceleration", "limm", "cv", "ca"}
