# Software License Agreement (BSD)
#
# @author    Luis Camero <lcamero@clearpathrobotics.com>
# @copyright (c) 2026, Clearpath Robotics, Inc., All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# * Redistributions of source code must retain the above copyright notice,
#   this list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
# * Neither the name of Clearpath Robotics nor the names of its contributors
#   may be used to endorse or promote products derived from this software
#   without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# Redistribution and use in source and binary forms, with or without
# modification, is not permitted without the express permission
# of Clearpath Robotics.
"""
Base class and registry for per-manipulator launch composition.

`BaseManipulatorLaunch` defines the contract that each concrete arm launch
subclass implements; `ManipulatorLaunch` is the registry that maps arm
`MANIPULATOR_MODEL` strings to their corresponding launch subclasses.
Subclasses live under `clearpath_generator_robot.launch.manipulators` and
self-register at import time. Vendor-specific node factories live as private
methods on each subclass rather than in a shared module, because they are
only meaningful for their particular arm vendor.
"""
from __future__ import annotations

from typing import Iterable


class BaseManipulatorLaunch:
    """
    Base class for per-manipulator launch composition.

    Subclasses set `MANIPULATOR_MODELS` to the list of `MANIPULATOR_MODEL`
    strings they handle (most subclasses register against a single model;
    `KinovaManipulatorLaunch` covers all three Kinova arms) and override
    `get_components` to emit any vendor-specific launch components beyond
    the shared `manipulators.launch.py` include that the generator always
    adds when at least one manipulator is present.
    """

    MANIPULATOR_MODELS: tuple = ()

    def __init__(self, arm, namespace: str) -> None:
        """Store the arm config slice and the ROS namespace."""
        self.arm = arm
        self.namespace = namespace

    @property
    def name(self) -> str:
        """Arm instance name, sourced from the arm config."""
        return self.arm.name

    def get_components(self) -> list:
        """
        Return vendor-specific launch components for this arm.

        Default is no extras; subclasses override.
        """
        return []


class ManipulatorLaunch:
    """Registry mapping `MANIPULATOR_MODEL` strings to their concrete launch subclass."""

    _REGISTRY: dict = {}

    @classmethod
    def register(cls, manipulator_launch_cls, keys: Iterable[str] | None = None) -> None:
        """
        Register a concrete `BaseManipulatorLaunch` subclass.

        If `keys` is not provided, registers against every entry in the
        subclass's `MANIPULATOR_MODELS` tuple. Pass `keys` explicitly to
        override (e.g. when a subclass should be registered against a
        subset of its declared models).
        """
        if keys is None:
            keys = manipulator_launch_cls.MANIPULATOR_MODELS
        for key in keys:
            cls._REGISTRY[key] = manipulator_launch_cls

    @classmethod
    def get(cls, name: str):
        """Return the registered subclass for the given `MANIPULATOR_MODEL`."""
        if name not in cls._REGISTRY:
            raise KeyError(
                f'No manipulator launch registered for "{name}". '
                f'Available: {list(cls._REGISTRY.keys())}'
            )
        return cls._REGISTRY[name]

    @classmethod
    def all_names(cls) -> list:
        """Return list of all registered `MANIPULATOR_MODEL` names."""
        return list(cls._REGISTRY.keys())
