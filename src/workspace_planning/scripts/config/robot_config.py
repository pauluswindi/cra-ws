#!/usr/bin/env python3

from pathlib import Path

import yaml


class RobotConfig:
    """Load robot configuration from YAML."""

    def __init__(self, config_path):
        self.config_path = Path(
            config_path
        ).resolve()

        if not self.config_path.exists():
            raise FileNotFoundError(
                f"Configuration file not found:\n"
                f"{self.config_path}"
            )

        with open(
            self.config_path,
            "r",
            encoding="utf-8",
        ) as file:
            self.config = yaml.safe_load(file)

        if not self.config:
            raise ValueError(
                "Configuration file is empty."
            )

        self._initialize_paths()
        self._initialize_robot()
        self._initialize_workspace()
        self._initialize_gng()

    def _initialize_paths(self):
        self.package_dir = (
            self.config_path.parent.parent
        )

    def _initialize_robot(self):
        robot = self.config.get(
            "robot",
            {},
        )

        self.robot_name = robot.get(
            "name",
            "robot",
        )

        model = robot.get(
            "model",
            {},
        )

        model_path = model.get("path")

        if not model_path:
            raise ValueError(
                "robot.model.path is not defined."
            )

        self.model_path = (
            self.package_dir / model_path
        ).resolve()

        frames = robot.get(
            "frames",
            {},
        )

        self.base_frame = frames.get(
            "base",
            "base_link",
        )

        self.end_effector_frame = frames.get(
            "end_effector",
            "end_effector",
        )

    def _initialize_workspace(self):
        workspace = self.config.get(
            "workspace",
            {},
        )

        self.workspace_num_samples = int(
            workspace.get(
                "num_samples",
                10000,
            )
        )

        self.workspace_random_seed = int(
            workspace.get(
                "random_seed",
                42,
            )
        )

    def _initialize_gng(self):
        gng = self.config.get(
            "gng",
            {},
        )

        self.gng_max_nodes = int(
            gng.get(
                "max_nodes",
                500,
            )
        )

        self.gng_epochs = int(
            gng.get(
                "epochs",
                20,
            )
        )

        self.gng_epsilon_b = float(
            gng.get(
                "epsilon_b",
                0.05,
            )
        )

        self.gng_epsilon_n = float(
            gng.get(
                "epsilon_n",
                0.006,
            )
        )

        self.gng_max_edge_age = int(
            gng.get(
                "max_edge_age",
                50,
            )
        )

        self.gng_lambda = int(
            gng.get(
                "lambda",
                100,
            )
        )

        self.gng_error_decay = float(
            gng.get(
                "error_decay",
                0.995,
            )
        )

    def get_model_path(self):
        """Return the MuJoCo model path."""

        return self.model_path

    def get_robot_name(self):
        """Return the robot name."""

        return self.robot_name

    def get_base_frame(self):
        """Return the robot base frame."""

        return self.base_frame

    def get_end_effector_frame(self):
        """Return the end-effector frame."""

        return self.end_effector_frame


if __name__ == "__main__":
    script_dir = Path(
        __file__
    ).resolve().parent

    package_dir = script_dir.parent.parent

    config_path = (
        package_dir
        / "config"
        / "robot.yaml"
    )

    config = RobotConfig(
        config_path
    )

    print("Robot Configuration")
    print()
    print(
        f"Robot          : "
        f"{config.robot_name}"
    )
    print(
        f"Model          : "
        f"{config.model_path}"
    )
    print(
        f"Base frame     : "
        f"{config.base_frame}"
    )
    print(
        f"End effector   : "
        f"{config.end_effector_frame}"
    )
    print()
    print("Workspace")
    print(
        f"Samples        : "
        f"{config.workspace_num_samples}"
    )
    print(
        f"Random seed    : "
        f"{config.workspace_random_seed}"
    )
    print()
    print("GNG")
    print(
        f"Max nodes      : "
        f"{config.gng_max_nodes}"
    )
    print(
        f"Epochs         : "
        f"{config.gng_epochs}"
    )
    print(
        f"Epsilon b      : "
        f"{config.gng_epsilon_b}"
    )
    print(
        f"Epsilon n      : "
        f"{config.gng_epsilon_n}"
    )
    print(
        f"Max edge age   : "
        f"{config.gng_max_edge_age}"
    )
    print(
        f"Lambda         : "
        f"{config.gng_lambda}"
    )
    print(
        f"Error decay    : "
        f"{config.gng_error_decay}"
    )