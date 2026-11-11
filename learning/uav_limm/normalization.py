"""Eq.(13)-(15): training-only extended MaxAbs and causal scope adjustment."""
import torch
from torch import nn


class ExtendedMaxAbs(nn.Module):
    def __init__(self):
        super().__init__()
        self.register_buffer("scale", torch.ones(3))

    @torch.no_grad()
    def fit(self, values):
        values = values.reshape(-1, 3)
        if not values.numel() or not torch.isfinite(values).all():
            raise ValueError("Scaler needs nonempty finite training features")
        iqr = torch.quantile(values, .75, dim=0) - torch.quantile(values, .25, dim=0)
        self.scale.copy_((values.abs().amax(0) + 1.5*iqr).clamp_min(1e-6))

    def scope(self, history):
        normalized = history / self.scale
        maximum = normalized.abs().amax(dim=-2)
        iqr = torch.quantile(normalized, .75, dim=-2) - torch.quantile(normalized, .25, dim=-2)
        extra = torch.where(maximum > 1, maximum + 1.5*iqr, torch.ones_like(maximum))
        return (self.scale * extra).unsqueeze(-2)

    def forward(self, history):
        scale = self.scope(history)
        return history/scale, scale
