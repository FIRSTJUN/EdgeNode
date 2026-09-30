"""GPS 위치와 IMU yaw/gyro를 사용하는 4-state EKF: [x, y, yaw, v]."""

import math

import numpy as np


def normalize_angle(angle):
    """각도를 [-pi, pi) 범위로 정규화한다."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


class EKF:
    """평면 운동 EKF. 거리/속도는 m, m/s이고 모든 내부 각도는 rad이다.

    process_noise는 표준편차가 아닌 상태별 초당 분산 증가량이다.
    Q(dt) = diag(process_noise) * dt로 IMU 주기에 비례해 적용한다.
    각 원소의 단위는 [m^2/s, m^2/s, rad^2/s, (m/s)^2/s]이다.
    기본값은 rosbag 검증 시작용 baseline이며 최종 튜닝값이 아니다.
    """

    def __init__(
        self,
        *,
        process_noise=(0.05, 0.05, 0.02, 0.50),
        gps_position_std_m=0.50,
        gps_velocity_std_mps=0.50,
        imu_yaw_std_rad=math.radians(2.0),
        initial_position_std_m=1.0,
        initial_yaw_std_rad=math.radians(5.0),
        initial_velocity_std_mps=2.0,
    ):
        noise = np.asarray(process_noise, dtype=float)
        if noise.shape != (4,) or not np.all(np.isfinite(noise)) or np.any(noise < 0.0):
            raise ValueError('process_noise는 유한한 비음수 4개여야 합니다.')
        stds = np.array([
            gps_position_std_m, gps_velocity_std_mps, imu_yaw_std_rad, initial_position_std_m,
            initial_yaw_std_rad, initial_velocity_std_mps,
        ], dtype=float)
        if not np.all(np.isfinite(stds)) or np.any(stds <= 0.0):
            raise ValueError('측정/초기 표준편차는 유한한 양수여야 합니다.')

        self._process_noise = np.diag(noise)
        self._position_noise = np.eye(2) * gps_position_std_m ** 2
        # GPS 위치에서 파생된 pseudo-measurement이므로 독립 센서처럼 R을 작게 잡지 않는다.
        # 0.50 m/s는 conservative baseline이며 최종 튜닝값이 아니다.
        self._velocity_noise = np.array([[gps_velocity_std_mps ** 2]])
        self._yaw_noise = np.array([[imu_yaw_std_rad ** 2]])
        self._initial_covariance = np.diag([
            initial_position_std_m ** 2, initial_position_std_m ** 2,
            initial_yaw_std_rad ** 2, initial_velocity_std_mps ** 2,
        ])
        self._state = np.zeros(4)
        self._covariance = self._initial_covariance.copy()
        self.initialized = False

    def initialize(self, x, y, yaw, v=0.0):
        state = np.array([x, y, yaw, v], dtype=float)
        if not np.all(np.isfinite(state)):
            raise ValueError('초기 상태는 유한한 값이어야 합니다.')
        state[2] = normalize_angle(state[2])
        self._state = state
        self._covariance = self._initial_covariance.copy()
        self.initialized = True

    def predict(self, gyro_z, dt):
        self._require_initialized()
        if not math.isfinite(gyro_z) or not math.isfinite(dt) or dt <= 0.0:
            raise ValueError('gyro_z는 유한해야 하고 dt는 유한한 양수여야 합니다.')

        # Jacobian과 위치 예측 모두 갱신 전 yaw/v를 사용한다.
        yaw, velocity = self._state[2], self._state[3]
        cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
        transition = np.array([
            [1.0, 0.0, -velocity * sin_yaw * dt, cos_yaw * dt],
            [0.0, 1.0, velocity * cos_yaw * dt, sin_yaw * dt],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ])
        self._state[0] += velocity * cos_yaw * dt
        self._state[1] += velocity * sin_yaw * dt
        self._state[2] = normalize_angle(yaw + gyro_z * dt)
        self._covariance = (
            transition @ self._covariance @ transition.T
            + self._process_noise * dt
        )
        self._covariance = 0.5 * (self._covariance + self._covariance.T)

    def update_position(self, x, y):
        self._require_initialized()
        measurement = np.array([x, y], dtype=float)
        if not np.all(np.isfinite(measurement)):
            raise ValueError('GPS 위치는 유한한 값이어야 합니다.')
        observation = np.array([
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
        ])
        self._update(measurement - self._state[:2], observation, self._position_noise)

    def update_yaw(self, yaw):
        self._require_initialized()
        if not math.isfinite(yaw):
            raise ValueError('IMU yaw는 유한한 값이어야 합니다.')
        innovation = np.array([normalize_angle(yaw - self._state[2])])
        observation = np.array([[0.0, 0.0, 1.0, 0.0]])
        self._update(innovation, observation, self._yaw_noise)

    def update_velocity(self, v_mps):
        """GNSS 거리 기반 scalar ground speed pseudo-measurement를 반영한다."""
        self._require_initialized()
        if not math.isfinite(v_mps) or v_mps < 0.0:
            raise ValueError('GPS 속도는 유한한 비음수여야 합니다.')
        innovation = np.array([v_mps - self._state[3]])
        observation = np.array([[0.0, 0.0, 0.0, 1.0]])
        self._update(innovation, observation, self._velocity_noise)

    def _update(self, innovation, observation, noise):
        covariance_measurement = self._covariance @ observation.T
        innovation_covariance = observation @ covariance_measurement + noise
        # S는 대칭행렬이다. inverse 대신 S * K.T = (P * H.T).T를 푼다.
        gain = np.linalg.solve(innovation_covariance, covariance_measurement.T).T
        self._state += gain @ innovation
        self._state[2] = normalize_angle(self._state[2])

        # Joseph form으로 covariance의 대칭성과 양의 준정부호 성질을 보존한다.
        residual = np.eye(4) - gain @ observation
        self._covariance = (
            residual @ self._covariance @ residual.T + gain @ noise @ gain.T
        )
        self._covariance = 0.5 * (self._covariance + self._covariance.T)

    def get_state(self):
        """상태 [x, y, yaw, v]의 독립적인 (4,) 배열을 반환한다."""
        self._require_initialized()
        return self._state.copy()

    def get_covariance(self):
        """상태 covariance의 독립적인 (4, 4) 배열을 반환한다."""
        self._require_initialized()
        return self._covariance.copy()

    def _require_initialized(self):
        if not self.initialized:
            raise RuntimeError('EKF.initialize()를 먼저 호출해야 합니다.')
