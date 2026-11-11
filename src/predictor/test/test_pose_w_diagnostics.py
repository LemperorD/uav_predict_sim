#!/usr/bin/python3
"""Physics identities and causality checks for the offline w diagnostic."""
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[3]/'scripts'))
import numpy as np
from experiment_pose_w import GRAVITY, DT, decompose, proxy_from_velocity, correlation


class WDiagnostics(unittest.TestCase):
    def test_level_lateral_thrust_and_residual(self):
        a=np.array([[0.,0.,0.],[3.,0.,0.]])
        force=a+[0,0,GRAVITY]
        axis=force/np.linalg.norm(force,axis=1)[:,None]
        w,residual,perpendicular=decompose(a,axis)
        np.testing.assert_allclose(w,[0,np.sqrt(GRAVITY**2+9)-GRAVITY],atol=1e-12)
        np.testing.assert_allclose(residual,w[:,None]*axis,atol=1e-12)
        np.testing.assert_allclose(perpendicular,0,atol=1e-12)

    def test_vertical_w_is_signed_acceleration(self):
        a=np.array([[0.,0.,2.],[0.,0.,-2.]])
        axis=np.array([[0.,0.,1.],[0.,0.,1.]])
        w,_,_=decompose(a,axis)
        np.testing.assert_allclose(w,[2,-2],atol=1e-12)

    def test_scalar_w_cannot_fix_axis_error(self):
        a=np.zeros((1,3));axis=np.array([[np.sin(.2),0,np.cos(.2)]])
        w,residual,perpendicular=decompose(a,axis)
        np.testing.assert_allclose(residual-w[:,None]*axis,perpendicular,atol=1e-12)
        self.assertGreater(np.linalg.norm(perpendicular),1.)

    def test_velocity_proxy_uses_no_future(self):
        t=np.arange(200)*DT
        velocity=np.column_stack([np.sin(t),np.zeros_like(t),np.cos(t)])
        axis=np.tile([0.,0.,1.],(len(t),1))
        raw,window=proxy_from_velocity(t,velocity,axis,26)
        changed=velocity.copy();changed[100:]+=100
        new_raw,new_window=proxy_from_velocity(t,changed,axis,26)
        np.testing.assert_allclose(raw[:100],new_raw[:100],equal_nan=True)
        np.testing.assert_allclose(window[:100],new_window[:100],equal_nan=True)
        self.assertIsNone(correlation(np.ones(100),1))


if __name__=='__main__':unittest.main()
