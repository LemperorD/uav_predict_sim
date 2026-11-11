from dataclasses import dataclass
import math


@dataclass
class ModelConfig:
    observation: int = 40
    horizon: int = 20
    dt: float = 1.0
    hidden: int = 64
    heads: int = 4
    layers: int = 3
    feedforward: int = 256
    dropout: float = 0.1
    backbone: str = "transformer_lstm"

    def __post_init__(self):
        if self.observation < 4 or not 1 <= self.horizon <= self.observation - 2:
            raise ValueError("Require observation>=4 and 1<=horizon<=observation-2")
        if not math.isfinite(self.dt) or self.dt <= 0:
            raise ValueError("dt must be positive and finite")
        if self.hidden <= 0 or self.heads <= 0 or self.hidden % self.heads:
            raise ValueError("hidden must be a positive multiple of heads")
        if self.layers < 1 or self.feedforward < 1 or not 0 <= self.dropout < 1:
            raise ValueError("Invalid network dimensions or dropout")
        if self.backbone not in ("transformer_lstm", "transformer", "lstm"):
            raise ValueError("Unknown backbone")
