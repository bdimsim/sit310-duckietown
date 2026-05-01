"""Straight line script for moving target sign"""
""" Modified for SIT 310 - DLAI 16/3/26 """

import numpy as np

from packages.duckiematrix_engine.entities.matrix_entity import (
    MatrixEntityBehavior,
)


class StraightLineScript(MatrixEntityBehavior):
    """Straight line script."""

    _distance: float
    _distance_on_leg: float
    _speed: float
    _direction: float

    def __init__(
        self,
        matrix_key: str,
        world_key: str | None,
        distance: float = 1.0,
        speed: float = 0.1,
        direction: float = 1.0, 
    ) -> None:
        """Initialize straight line script."""
        super().__init__(matrix_key, world_key)
        self._distance = distance
        self._distance_on_leg = 0
        self._direction = 1.0
        self._speed = speed

    def update(self, delta_time: float) -> None:
        """Update."""
        if self.state:
            distance = self._speed * delta_time
            self.state.x -= self._direction * distance * np.sin(self.state.yaw)
            self.state.y += self._direction * distance * np.cos(self.state.yaw)
            self._distance_on_leg += distance
            if self._distance_on_leg > self._distance:
                #self.state.yaw += np.deg2rad(180)
                self._direction = self._direction * -1
                self._distance_on_leg = 0
            self.state.commit()
