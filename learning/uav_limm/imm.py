"""Differentiable IMM equations (2)-(12), in a common velocity state space."""
import math

import torch
from torch import nn
from torch.nn import functional as F


def positive_covariance(parameters):
    """Six unconstrained outputs -> SPD 3x3 covariance (implementation choice)."""
    row, col = torch.tril_indices(3, 3, device=parameters.device)
    factor = parameters.new_zeros(*parameters.shape[:-1], 3, 3)
    factor[..., row, col] = parameters
    diagonal = torch.diagonal(factor, dim1=-2, dim2=-1)
    factor = factor - torch.diag_embed(diagonal) + torch.diag_embed(F.softplus(diagonal)+1e-3)
    return factor @ factor.transpose(-1, -2) + torch.eye(3, device=factor.device, dtype=factor.dtype)*1e-5


def interact(states, covariances, weights, transition):
    """Eq.(3)-(4); transition[..., source i, destination j], rows sum to 1."""
    joint = weights.unsqueeze(-1)*transition
    prior = joint.sum(-2).clamp_min(torch.finfo(states.dtype).tiny)
    mixing = joint/prior.unsqueeze(-2)
    mixed = torch.einsum("bij,bid->bjd", mixing, states)
    delta = states[:, :, None, :] - mixed[:, None, :, :]
    spread = delta.unsqueeze(-1)*delta.unsqueeze(-2)
    covariance = torch.einsum("bij,bijde->bjde", mixing, covariances[:, :, None]+spread)
    return mixed, covariance, prior


def kalman_update(predicted, covariance, measurement, measurement_covariance):
    """H=I, Eq.(6)-(8). Cholesky solve, log likelihood, Joseph covariance."""
    residual = measurement - predicted
    innovation = covariance + measurement_covariance
    chol = torch.linalg.cholesky(innovation)
    gain = torch.cholesky_solve(covariance.transpose(-1, -2), chol).transpose(-1, -2)
    filtered = predicted + (gain @ residual.unsqueeze(-1)).squeeze(-1)
    identity = torch.eye(3, device=predicted.device, dtype=predicted.dtype)
    remaining = identity - gain
    updated = (remaining @ covariance @ remaining.transpose(-1, -2)
               + gain @ measurement_covariance @ gain.transpose(-1, -2))
    updated = (updated+updated.transpose(-1, -2))*.5
    mahalanobis = (residual.unsqueeze(-2) @ torch.cholesky_solve(residual.unsqueeze(-1), chol)).squeeze(-1).squeeze(-1)
    logdet = 2*torch.diagonal(chol, dim1=-2, dim2=-1).log().sum(-1)
    log_likelihood = -.5*(mahalanobis+logdet+3*math.log(2*math.pi))
    return filtered, updated, log_likelihood


def combine(states, covariances, weights):
    mean = (states*weights.unsqueeze(-1)).sum(-2)
    delta = states-mean.unsqueeze(-2)
    cov = ((covariances+delta.unsqueeze(-1)*delta.unsqueeze(-2))*weights[..., None, None]).sum(-3)
    return mean, cov


class RecurrentHead(nn.Module):
    def __init__(self, hidden, output):
        super().__init__()
        self.cell = nn.LSTMCell(3, hidden)
        self.output = nn.Linear(hidden, output)

    def forward(self, value, memory):
        memory = self.cell(value, memory)
        return self.output(memory[0]), memory


class LearningIMM(nn.Module):
    def __init__(self, config):
        super().__init__()
        self.config = config
        # Aligned P/V/A normalized history (last observation-2 timestamps).
        self.encoder = nn.LSTM(9, config.hidden, num_layers=2, batch_first=True)
        self.state_heads = nn.ModuleList([RecurrentHead(config.hidden, 3) for _ in range(3)])
        self.process_heads = nn.ModuleList([RecurrentHead(config.hidden, 6) for _ in range(3)])
        self.measurement_heads = nn.ModuleList([RecurrentHead(config.hidden, 6) for _ in range(3)])
        self.transition_logits = nn.Parameter(torch.zeros(config.horizon, 3, 3))
        self.initial_covariance = nn.Parameter(torch.zeros(3, 6))

    def forward(self, history_features, last_velocity, measurements):
        _, (h, c) = self.encoder(history_features)
        memories = [[(h[-1], c[-1]) for _ in range(3)] for _ in range(3)]
        states = last_velocity[:, None, :].expand(-1, 3, -1)
        covariance = positive_covariance(self.initial_covariance).expand(states.shape[0], -1, -1, -1)
        weights = states.new_full(states.shape[:2], 1/3)
        transition = self.transition_logits.softmax(-1)
        outputs, probabilities, covariances = [], [], []
        for step in range(self.config.horizon):
            mixed, mixed_cov, prior = interact(states, covariance, weights, transition[step])
            predicted, q, r = [], [], []
            for mode in range(3):
                state, memories[0][mode] = self.state_heads[mode](mixed[:, mode], memories[0][mode])
                qp, memories[1][mode] = self.process_heads[mode](state, memories[1][mode])
                rp, memories[2][mode] = self.measurement_heads[mode](measurements[:, step, mode], memories[2][mode])
                predicted.append(state)
                q.append(positive_covariance(qp))
                r.append(positive_covariance(rp))
            predicted = torch.stack(predicted, 1)
            # Eq.(11) deliberately omits F/Jacobian as in the paper.
            predicted_cov = mixed_cov + torch.stack(q, 1)
            states, covariance, log_likelihood = kalman_update(
                predicted, predicted_cov, measurements[:, step], torch.stack(r, 1))
            weights = (log_likelihood + prior.log()).softmax(-1)
            mean, combined_cov = combine(states, covariance, weights)
            outputs.append(mean)
            probabilities.append(weights)
            covariances.append(combined_cov)
        return {"normalized_velocity": torch.stack(outputs, 1),
                "probabilities": torch.stack(probabilities, 1),
                "normalized_covariance": torch.stack(covariances, 1),
                "transition": transition}
