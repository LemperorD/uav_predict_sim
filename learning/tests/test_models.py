import numpy as np
import pytest
import torch
from torch import nn

from uav_limm.config import ModelConfig
from uav_limm.limm import LIMM
from uav_limm.model import DifferentialForecaster
from uav_limm.runtime import StreamingPredictor
from uav_limm.training import load_checkpoint, save_checkpoint


def config(**kwargs):
    return ModelConfig(observation=8, horizon=3, hidden=8, heads=2, layers=1,
                       feedforward=16, dropout=0, **kwargs)


@pytest.mark.parametrize("backbone", ["transformer_lstm", "transformer", "lstm"])
def test_all_backbones_train_without_future_inputs(backbone):
    torch.manual_seed(42)
    model = LIMM(config(backbone=backbone))
    history = torch.randn(2, 8, 3)
    future = torch.randn(2, 3, 3)
    loss = model.upstream.loss(history, future)
    loss.backward()
    assert all(p.grad is not None and torch.isfinite(p.grad).all() for p in model.upstream.predictors.parameters())
    model.zero_grad(set_to_none=True)
    model.freeze_upstream()
    model.train()
    assert not model.upstream.training
    model.loss(history, future).backward()
    for name, parameter in model.fusion.named_parameters():
        assert parameter.grad is not None, name
        assert torch.isfinite(parameter.grad).all(), name
        assert parameter.grad.abs().sum() > 0, name
    assert all(p.grad is None for p in model.upstream.parameters())
    model.eval()
    result = model(history)
    torch.testing.assert_close(result["probabilities"].sum(-1), torch.ones(2, 3))
    torch.testing.assert_close(result["transition"].sum(-1), torch.ones(3, 3))
    assert torch.linalg.eigvalsh(result["velocity_covariance"]).min() > 0


def test_eq1_three_routes_recover_known_accelerating_trajectory():
    dt = .5
    model = DifferentialForecaster(ModelConfig(observation=8, horizon=3, dt=dt,
                                               hidden=8, heads=2, layers=1))
    t = torch.arange(11)*dt
    path = (t*t)[None, :, None].expand(-1, -1, 3)
    history, future = path[:, :8], path[:, 8:]
    from uav_limm.data import future_features, features
    labels = future_features(history, future, dt)
    class Exact(nn.Module):
        def __init__(self, value):
            super().__init__()
            self.value = value
        def forward(self, x):
            return self.value
    # Inject an oracle for the learned part; test physical conversion separately.
    model.predictors = nn.ModuleList([Exact(y/scaler.scope(x)) for x, y, scaler in
                                     zip(features(history, dt), labels, model.scalers)])
    for mode in range(3):
        torch.testing.assert_close(model(history)["positions"][:, :, mode], future)


def test_checkpoint_and_streaming_contract(tmp_path):
    torch.manual_seed(1)
    model = LIMM(config()).eval()
    history = torch.randn(1, 8, 3)
    path = tmp_path/"model.pt"
    save_checkpoint(path, model, {"test": True})
    loaded, payload = load_checkpoint(path)
    assert payload["metadata"]["test"]
    torch.testing.assert_close(model(history)["positions"], loaded(history)["positions"], rtol=0, atol=0)
    stream = StreamingPredictor(loaded)
    origin = np.array([1000., -2000., 3000.])
    for i, p in enumerate(history[0].numpy()):
        stream.update(float(i), p+origin)
    # Runtime uses flight origin (first observed point), matching data preprocessing.
    relative = history-history[:, :1]
    expected = loaded(relative)["positions"][0].detach().numpy()+origin+history[0, 0].numpy()
    np.testing.assert_allclose(stream.predict(), expected, rtol=1e-5, atol=1e-5)
    assert stream.predict(count=2).shape == (2, 3)
    with pytest.raises(ValueError):
        stream.predict(step=.5)
    with pytest.raises(ValueError):
        stream.update(10., [0., 0., 0.])
    with pytest.raises(ValueError):
        stream.update(7., [0., 0., 0.])
    assert stream.timestamp == 7.
    stream.reset()
    with pytest.raises(RuntimeError):
        stream.predict()
