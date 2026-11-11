import math

import torch

from uav_limm.imm import combine, interact, kalman_update, positive_covariance


def test_interaction_against_analytical_mixture():
    states = torch.tensor([[[0., 0., 0.], [2., 0., 0.]]], dtype=torch.double)
    covariance = torch.eye(3, dtype=torch.double).expand(1, 2, 3, 3)
    weights = torch.tensor([[.25, .75]], dtype=torch.double)
    transition = torch.tensor([[.8, .2], [.4, .6]], dtype=torch.double)
    mean, cov, prior = interact(states, covariance, weights, transition)
    torch.testing.assert_close(prior, torch.tensor([[.5, .5]], dtype=torch.double))
    torch.testing.assert_close(mean[0, :, 0], torch.tensor([1.2, 1.8], dtype=torch.double))
    torch.testing.assert_close(cov[0, :, 0, 0], torch.tensor([1.96, 1.36], dtype=torch.double))
    torch.testing.assert_close(cov[0, :, 1, 1], torch.ones(2, dtype=torch.double))
    fused, fused_cov = combine(states, covariance, weights)
    torch.testing.assert_close(fused[0], torch.tensor([1.5, 0., 0.], dtype=torch.double))
    torch.testing.assert_close(fused_cov[0].diag(), torch.tensor([1.75, 1., 1.], dtype=torch.double))


def test_kalman_against_closed_form_gaussian():
    predicted = torch.zeros(1, 1, 3, dtype=torch.double)
    measurement = torch.ones_like(predicted)
    identity = torch.eye(3, dtype=torch.double)[None, None]
    state, covariance, likelihood = kalman_update(predicted, 2*identity, measurement, identity)
    torch.testing.assert_close(state, torch.full_like(state, 2/3))
    torch.testing.assert_close(covariance, identity*2/3)
    expected = -.5*(1+3*math.log(3)+3*math.log(2*math.pi))
    torch.testing.assert_close(likelihood, torch.full_like(likelihood, expected))


def test_kalman_gradients_and_covariance_positive():
    torch.manual_seed(3)
    parameters = torch.randn(1, 2, 6, dtype=torch.double, requires_grad=True)
    covariance = positive_covariance(parameters)
    assert torch.linalg.eigvalsh(covariance).min() > 0
    predicted = torch.randn(1, 2, 3, dtype=torch.double, requires_grad=True)
    measurement = torch.randn_like(predicted)
    def function(x, p):
        state, cov, likelihood = kalman_update(x, positive_covariance(p), measurement, torch.eye(3, dtype=x.dtype))
        return state, cov, likelihood
    assert torch.autograd.gradcheck(function, (predicted, parameters), atol=1e-4)


def test_extreme_likelihoods_do_not_underflow_probabilities():
    predicted = torch.zeros(1, 3, 3, dtype=torch.double)
    measurement = torch.full_like(predicted, 1e4)
    covariance = torch.eye(3, dtype=torch.double).expand(1, 3, 3, 3)
    _, _, likelihood = kalman_update(predicted, covariance, measurement, covariance)
    weights = (likelihood + math.log(1/3)).softmax(-1)
    torch.testing.assert_close(weights.sum(-1), torch.ones(1, dtype=torch.double))
    assert torch.isfinite(weights).all()
