"""
Experiment configuration definitions for timing-the-chaos.
Defines parameter sweeps with default values and ranges.
"""

import os
from itertools import product
from dataclasses import dataclass, field, asdict, replace
from typing import Dict, List, Optional

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "../.."))

@dataclass
class DhakaSimConfig:
    simulation_end_time: int = 120
    random_seed: int = 0
    strip_width: float = 0.5
    maximum_speed: float = 100.0
    error_mode: bool = True
    ft_method: int = 4
    attack_scenario: int = 0
    delay_probability: float = 0.3
    jitter_magnitude: float = 0.5
    delay_mean: float = 3.0
    delay_spread: float = 1.0
    packet_loss_rate: float = 0.2
    attack_strip_mod: int = -1
    attack_start_time: int = 1
    attack_end_time: int = -1
    jitter_only_magnitude: int = 2
    single_shot_delay: float = 5.0
    single_shot_time: int = 600
    position_buffer_size: int = 10
    cf_model: int = 0
    ttc_threshold: float = 0.6
    brake_hard: bool = False

    # Demand/config independent
    pixel_per_meter: int = 15
    encounter_per_accident: float = 0.01
    footpath_strip_width: float = 0.5
    gui_mode: str = "Off"
    debug_mode: str = "Off"
    object_mode: str = "On"
    trace_mode: str = "Off"
    signal_change_duration: int = 1
    default_translate_x: int = -1340
    default_translate_y: int = -580
    centered_view: str = "On"
    slow_vehicle: int = 50
    medium_vehicle: int = 25
    fast_vehicle: int = 25
    vehicle_generation_rate: int = 1
    dlc_model: int = 0
    across_pedestrian_mode: str = "On"
    along_pedestrian_mode: str = "On"
    across_pedestrian_limit: int = 1
    across_pedestrian_per_hour: int = 400
    along_pedestrian_per_hour: int = 50
    along_pedestrian_percentage: int = 99
    density_percentage: int = 50
    pedestrian_weight: int = 2
    pedestrian_random_lane_change_percentage: int = 20
    pedestrian_left_bias_percentage: int = 40
    penalty_wait: str = "On"
    consider_minimum: str = "On"
    no_of_routes: int = 12

    def to_param_lines(self) -> List[str]:
        mapping = {
            "RandomSeed": str(self.random_seed),
            "SimulationSpeed": "1",
            "SimulationEndTime": str(self.simulation_end_time),
            "PixelPerMeter": str(self.pixel_per_meter),
            "EncounterPerAccident": f"{self.encounter_per_accident:.2f}",
            "StripWidth": f"{self.strip_width:.1f}",
            "FootpathStripWidth": f"{self.footpath_strip_width:.1f}",
            "MaximumSpeed": f"{self.maximum_speed:.1f}",
            "GUIMode": self.gui_mode,
            "DebugMode": self.debug_mode,
            "ObjectMode": self.object_mode,
            "TraceMode": self.trace_mode,
            "SignalChangeDuration": str(self.signal_change_duration),
            "DefaultTranslateX": str(self.default_translate_x),
            "DefaultTranslateY": str(self.default_translate_y),
            "CenteredView": self.centered_view,
            "SlowVehicle": str(self.slow_vehicle),
            "MediumVehicle": str(self.medium_vehicle),
            "FastVehicle": str(self.fast_vehicle),
            "DemandType": "2",
            "TTC_Threshold": f"{self.ttc_threshold:.1f}",
            "LowRate": "200",
            "MediumRate": "600",
            "HighRate": "1000",
            "VehicleGenerationRate": str(self.vehicle_generation_rate),
            "ErrorMode": "On" if self.error_mode else "Off",
            "FTMethod": str(self.ft_method),
            "NoOfReadings": "2",
            "MFactor": "0.0",
            "ALPHA": "0.92",
            "BETA": "0.1",
            "ETA": "0",
            "DLC_model": str(self.dlc_model),
            "CF_model": str(self.cf_model),
            "AcrossPedestrianMode": self.across_pedestrian_mode,
            "AlongPedestrianMode": self.along_pedestrian_mode,
            "AcrossPedestrianLimit": str(self.across_pedestrian_limit),
            "AcrossPedestrianPerHour": str(self.across_pedestrian_per_hour),
            "AlongPedestrianPerHour": str(self.along_pedestrian_per_hour),
            "AlongPedestrianPercentage": str(self.along_pedestrian_percentage),
            "DensityPercentage": str(self.density_percentage),
            "PedestrianWeight": str(self.pedestrian_weight),
            "PedestrianRandomLaneChangePercentage": str(self.pedestrian_random_lane_change_percentage),
            "PedestrianLeftBiasPercentage": str(self.pedestrian_left_bias_percentage),
            "PenaltyWait": self.penalty_wait,
            "ConsiderMinimum": self.consider_minimum,
            "NoOfRoutes": str(self.no_of_routes),
            "BrakeHard": "On" if self.brake_hard else "Off",
            "AttackScenario": str(self.attack_scenario),
            "DelayProbability": f"{self.delay_probability:.2f}",
            "JitterMagnitude": f"{self.jitter_magnitude:.2f}",
            "DelayMean": f"{self.delay_mean:.2f}",
            "DelaySpread": f"{self.delay_spread:.2f}",
            "PacketLossRate": f"{self.packet_loss_rate:.2f}",
            "AttackStripMod": str(self.attack_strip_mod),
            "AttackStartTime": str(self.attack_start_time),
            "AttackEndTime": str(self.attack_end_time),
            "JitterOnlyMagnitude": str(self.jitter_only_magnitude),
            "SingleShotDelay": f"{self.single_shot_delay:.2f}",
            "SingleShotTime": str(self.single_shot_time),
            "PositionBufferSize": str(self.position_buffer_size),
        }
        return [f"{k} {v}" for k, v in mapping.items()]


def generate_fractional_factorial(
    base_config: DhakaSimConfig,
) -> List[DhakaSimConfig]:
    """Generate a fractional factorial experiment set focusing on key variables."""
    scenarios = [0, 1, 2, 3, 4, 5]
    delays = [1.0, 3.0, 5.0, 7.0]
    jitters = [0.2, 0.5, 1.0]
    probs = [0.1, 0.3, 0.5]
    configs = []

    for sc in scenarios:
        for d in delays:
            cfg = DhakaSimConfig(
                attack_scenario=sc,
                delay_mean=d,
                delay_probability=1.0 if sc == 0 else (0.5 if sc == 1 else 0.3),
                jitter_magnitude=1.0 if sc in (3, 5) else 0.5,
                packet_loss_rate=0.3 if sc in (4, 5) else 0.2,
                single_shot_time=60 if sc == 2 else 600,
                single_shot_delay=d if sc == 2 else 5.0,
            )
            configs.append(cfg)

    return configs


def generate_full_factorial(
    base_config: DhakaSimConfig,
) -> List[DhakaSimConfig]:
    """Generate full factorial experiment across attack parameters."""
    scenarios = [0, 1, 2, 3, 4, 5]
    delays = [0.5, 1.0, 2.0, 3.0, 5.0, 7.0, 10.0]
    jitters = [0.1, 0.2, 0.5, 1.0, 2.0]
    probs = [0.05, 0.1, 0.2, 0.3, 0.5, 0.7]
    configs = []

    for sc in scenarios:
        for d in delays:
            cfg = DhakaSimConfig(attack_scenario=sc, delay_mean=d)
            configs.append(cfg)

    for j in jitters:
        cfg = DhakaSimConfig(attack_scenario=3, jitter_magnitude=j)
        configs.append(cfg)

    for p in probs:
        for d in delays[:4]:
            cfg = DhakaSimConfig(attack_scenario=1, delay_probability=p, delay_mean=d)
            configs.append(cfg)

    return configs


def generate_baseline_configs(
    base_config: DhakaSimConfig = None,
    simulation_end_time: int = 120,
) -> List[DhakaSimConfig]:
    """No-attack baseline runs (error_mode=False). Same traffic, no temporal desync."""
    cfg = replace(base_config, error_mode=False, ft_method=4,
                  simulation_end_time=simulation_end_time, attack_scenario=-1) if base_config \
        else DhakaSimConfig(error_mode=False, ft_method=4,
                            simulation_end_time=simulation_end_time, attack_scenario=-1)
    return [cfg]


def generate_extended_matrix(
    base_config: DhakaSimConfig = None,
) -> List[DhakaSimConfig]:
    """Extended sweep: extra delay levels + probability/jitter sweeps."""
    base = base_config or DhakaSimConfig()
    scenarios = [0, 1, 2, 3, 4, 5]
    extra_delays = [0.5, 2.0, 4.0, 6.0, 10.0]
    probs = [0.1, 0.3, 0.5, 0.7]
    jitters = [0.5, 1.0, 2.0]
    configs = []

    for sc in scenarios:
        for d in extra_delays:
            configs.append(DhakaSimConfig(
                attack_scenario=sc,
                delay_mean=d,
                delay_probability=1.0 if sc == 0 else (0.5 if sc == 1 else 0.3),
                jitter_magnitude=1.0 if sc in (3, 5) else 0.5,
                packet_loss_rate=0.3 if sc in (4, 5) else 0.2,
                single_shot_time=60 if sc == 2 else 600,
                single_shot_delay=d if sc == 2 else 5.0,
            ))

    for p in probs:
        configs.append(DhakaSimConfig(attack_scenario=1, delay_mean=3.0, delay_probability=p))
        configs.append(DhakaSimConfig(attack_scenario=4, delay_mean=3.0, packet_loss_rate=p))

    for j in jitters:
        configs.append(DhakaSimConfig(attack_scenario=3, jitter_magnitude=j))

    return configs
