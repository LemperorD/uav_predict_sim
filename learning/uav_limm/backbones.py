"""Reusable sequence regressors using official PyTorch Transformer/LSTM layers.

No ROS, flight data, normalization or IMM dependencies in this module.
"""
import math

import torch
from torch import nn


class SinusoidalPosition(nn.Module):
    def forward(self, x, offset=0):
        length, width = x.shape[-2:]
        t = torch.arange(offset, offset+length, device=x.device, dtype=x.dtype)[:, None]
        frequency = torch.exp(torch.arange(0, width, 2, device=x.device, dtype=x.dtype)
                              * (-math.log(10000.0)/width))
        code = torch.zeros(length, width, device=x.device, dtype=x.dtype)
        code[:, 0::2] = torch.sin(t*frequency)
        code[:, 1::2] = torch.cos(t*frequency[:width//2])
        return x + code


class TransformerForecaster(nn.Module):
    """Fig.3: history suffix + zeros, causal decoder, optional LSTM output."""
    def __init__(self, config, recurrent_output=True):
        super().__init__()
        self.horizon = config.horizon
        self.embedding = nn.Linear(3, config.hidden)
        self.position = SinusoidalPosition()
        self.transformer = nn.Transformer(
            d_model=config.hidden, nhead=config.heads,
            num_encoder_layers=config.layers, num_decoder_layers=config.layers,
            dim_feedforward=config.feedforward, dropout=config.dropout, batch_first=True)
        self.recurrent = nn.LSTM(config.hidden, config.hidden, batch_first=True) if recurrent_output else None
        self.output = nn.Linear(config.hidden, 3)

    def forward(self, history):
        h = self.horizon
        if history.shape[1] < h:
            raise ValueError("Decoder prefix requires at least horizon historical samples")
        decoder = torch.cat([history[:, -h:], history.new_zeros(history.shape[0], h, 3)], 1)
        source = self.position(self.embedding(history))
        target = self.position(self.embedding(decoder), history.shape[1]-h)
        mask = torch.triu(torch.ones(2*h, 2*h, device=history.device, dtype=torch.bool), diagonal=1)
        decoded = self.transformer(source, target, tgt_mask=mask)
        if self.recurrent is not None:
            decoded, _ = self.recurrent(decoded)
        return self.output(decoded[:, -h:])


class LSTMForecaster(nn.Module):
    """Paper's LSTM_F baseline: stacked LSTM encoder, flattened dense output."""
    def __init__(self, config):
        super().__init__()
        self.horizon = config.horizon
        self.encoder = nn.LSTM(3, config.hidden, num_layers=config.layers,
                               dropout=config.dropout if config.layers > 1 else 0,
                               batch_first=True)
        self.output = nn.Linear(config.hidden, 3*config.horizon)

    def forward(self, history):
        _, (h, _) = self.encoder(history)
        return self.output(h[-1]).reshape(-1, self.horizon, 3)
