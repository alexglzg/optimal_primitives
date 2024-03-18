from dataclasses import dataclass

from helper import angle_difference, normalize_angle

import numpy as np

@dataclass(frozen=True)
class TrajectoryParameters:
    """
    A dataclass that holds the data needed to create the path for a trajectory.

    turning_radius: The radius of the circle used to generate
        the arc of the path
    end_point: The end coordinate of the path
    start_angle: The starting angle of the path
        - given in radians from -pi to pi where 0 radians is along
            the positive x axis
    end_angle: The end angle of the path
        - given in radians from -pi to pi where 0 radians is along
            the positive x axis
    left_turn: Whether the arc in the path turns to the left
    """

    turning_radius: float
    end_point: np.array
    start_angle: float
    end_angle: float
    left_turn: bool
    total_length: float
    straight_length: float

    @property
    def arc_length(self):
        """Arc length of the trajectory."""
        return self.turning_radius * angle_difference(
            self.start_angle, self.end_angle, self.left_turn
        )


@dataclass(frozen=True)
class Path:
    """
    A dataclass that holds the generated poses for a given trajectory.

    xs: X coordinates of poses along trajectory
    ys: Y coordinates of poses along trajectory
    yaws: Yaws of poses along trajectory
    """

    xs: np.array
    ys: np.array
    yaws: np.array

    def __add__(self, rhs):
        """Add two paths together by concatenating them."""
        if self.xs is None:
            return rhs

        xs = np.concatenate((self.xs, rhs.xs))
        ys = np.concatenate((self.ys, rhs.ys))
        yaws = np.concatenate((self.yaws, rhs.yaws))

        return Path(xs, ys, yaws)

    def to_output_format(self):
        """Return the path data in a format suitable for outputting."""
        output_xs = self.xs.round(5)
        output_ys = self.ys.round(5)

        # A bit of a hack but it removes any -0.0
        output_xs = output_xs + 0.0
        output_ys = output_ys + 0.0
        output_yaws = self.yaws + 0.0

        vectorized_normalize_angle = np.vectorize(normalize_angle)
        output_yaws = vectorized_normalize_angle(output_yaws)

        stacked = np.vstack([output_xs, output_ys, output_yaws]).transpose()

        return stacked.tolist()


@dataclass(frozen=True)
class Trajectory:
    """
    A dataclass that holds the path and parameters for a trajectory.

    path: The Path that represents the trajectory
    parameters: The TrajectoryParameters that represent the trajectory
    """

    path: Path
    parameters: TrajectoryParameters