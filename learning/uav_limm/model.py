import torch
from torch import nn

from .backbones import LSTMForecaster, TransformerForecaster
from .data import features, future_features
from .normalization import ExtendedMaxAbs


class DifferentialForecaster(nn.Module):
    """Independent P/V/A forecasters and Eq.(1) physical conversions."""
    def __init__(self, config):
        super().__init__()
        self.config = config
        self.scalers = nn.ModuleList([ExtendedMaxAbs() for _ in range(3)])
        self.predictors = nn.ModuleList([
            LSTMForecaster(config) if config.backbone == "lstm" else
            TransformerForecaster(config, config.backbone == "transformer_lstm")
            for _ in range(3)])

    @torch.no_grad()
    def fit_scalers(self, dataset):
        # Raw training flight points only; never validation/test points or targets.
        for mode, scaler in enumerate(self.scalers):
            values = [features(torch.tensor(x, dtype=torch.float32), self.config.dt)[mode]
                      for _, x, _ in dataset.flights]
            scaler.fit(torch.cat(values))

    def normalized_predictions(self, history):
        result, scales = [], []
        for series, scaler, predictor in zip(features(history, self.config.dt), self.scalers, self.predictors):
            normalized, scale = scaler(series)
            result.append(predictor(normalized))
            scales.append(scale)
        return result, scales

    def loss(self, history, future):
        predictions, scales = self.normalized_predictions(history)
        labels = future_features(history, future, self.config.dt)
        return sum((pred-label/scale).square().mean()
                   for pred, label, scale in zip(predictions, labels, scales))

    def forward(self, history):
        predictions, scales = self.normalized_predictions(history)
        p, v, a = [value*scale for value, scale in zip(predictions, scales)]
        dt = self.config.dt
        v_from_p = torch.cat([history[:, -1:], p], 1).diff(dim=1)/dt
        v_from_a = (history[:, -1:]-history[:, -2:-1])/dt + a.cumsum(1)*dt
        # [batch, horizon, mode=P/V/A, xyz], common physical velocity state.
        velocities = torch.stack([v_from_p, v, v_from_a], dim=2)
        positions = history[:, -1:, None, :] + velocities.cumsum(1)*dt
        return {"velocities": velocities, "positions": positions}
