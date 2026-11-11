import numpy as np
import pytest
import torch

from uav_limm.config import ModelConfig
from uav_limm.data import features, future_features, load_splits, resample_csv
from uav_limm.normalization import ExtendedMaxAbs


def test_finite_differences_and_boundary_targets():
    # Constant discrete acceleration 2 m/s² with non-unit dt.
    dt = .5
    t = torch.arange(12)*dt
    path = (t*t)[:, None].expand(-1, 3)[None]
    history, future = path[:, :8], path[:, 8:]
    _, velocity, acceleration = features(history, dt)
    _, future_v, future_a = future_features(history, future, dt)
    torch.testing.assert_close(acceleration, torch.full_like(acceleration, 2))
    torch.testing.assert_close(future_a, torch.full_like(future_a, 2))
    reconstructed = history[:, -1:] + future_v.cumsum(1)*dt
    torch.testing.assert_close(reconstructed, future)
    from_a = velocity[:, -1:] + future_a.cumsum(1)*dt
    torch.testing.assert_close(from_a, future_v)


def test_split_before_windows_and_training_only_scale(tmp_path):
    t = np.arange(1000)
    path = tmp_path/"flight.csv"
    np.savetxt(path, np.column_stack([t, t, t, t]), delimiter=",", comments="", header="t,px,py,pz")
    train, val, test = load_splits([path], ModelConfig(), stride=1)
    assert (len(train), len(val), len(test)) == (641, 41, 141)
    assert train[-1][1][-1, 0] == 699
    assert val[0][0][0, 0] == 700
    assert test[0][0][0, 0] == 800
    scaler = ExtendedMaxAbs()
    scaler.fit(torch.tensor(train.flights[0][1], dtype=torch.float32))
    # Extended MaxAbs with IQR on [0,699], not on test ending at 999.
    torch.testing.assert_close(scaler.scale, torch.full((3,), 699+1.5*349.5))


def test_renormalization_is_per_axis_causal_and_invertible():
    scaler = ExtendedMaxAbs()
    history = torch.tensor([[[1., 0., .1], [3., 0., .2], [7., 0., .3]]])
    value, scale = scaler(history)
    assert value.abs().max() <= 1
    assert scale[0, 0, 1] == 1
    assert scale[0, 0, 2] == 1
    torch.testing.assert_close(value*scale, history)
    value2, scale2 = scaler(torch.cat([history, history*100], 0))
    torch.testing.assert_close(scale, scale2[:1])
    torch.testing.assert_close(value, value2[:1])
    scaler.fit(torch.zeros(20, 3))
    assert torch.isfinite(scaler(torch.zeros(1, 5, 3))[0]).all()


@pytest.mark.parametrize("timestamps", [[0, 0, 1], [0, 2, 1], [0, 1, 5], [0, float('nan'), 2]])
def test_reject_invalid_or_gapped_records(tmp_path, timestamps):
    path = tmp_path/"bad.csv"
    np.savetxt(path, np.column_stack([timestamps, np.zeros((3, 3))]),
               delimiter=",", comments="", header="t,px,py,pz")
    with pytest.raises(ValueError):
        resample_csv(path)


def test_simulator_truth_is_only_a_label(tmp_path):
    path = tmp_path/"samples.csv"
    values = np.column_stack([np.arange(100), np.zeros((100, 3)), np.full((100, 3), 200)])
    np.savetxt(path, values, delimiter=",", comments="",
               header="t,measurement_x,measurement_y,measurement_z,truth_x,truth_y,truth_z")
    _, obs, labels = resample_csv(path)
    assert np.all(obs == 0)
    assert np.all(labels == 200)
