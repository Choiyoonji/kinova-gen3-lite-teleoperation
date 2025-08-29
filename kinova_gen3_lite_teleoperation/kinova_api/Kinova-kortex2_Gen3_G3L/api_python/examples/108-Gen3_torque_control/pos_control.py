#! /usr/bin/env python3
import sys
import os
import time
from typing import List, Optional
from collections import deque
# import queue

from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient
from kortex_api.autogen.client_stubs.DeviceManagerClientRpc import DeviceManagerClient
from kortex_api.autogen.messages import Base_pb2, BaseCyclic_pb2, Common_pb2
from kortex_api.RouterClient import RouterClientSendOptions
from kortex_api.autogen.client_stubs.ActuatorConfigClientRpc import ActuatorConfigClient
from kortex_api.autogen.messages import ActuatorConfig_pb2

class PositionController:
    """
    Kinova Kortex: 단순 위치 제어 루프 (저수준 서보 모드, 무스레드)
    - current_positions: 최신 관절값(rad)
    - target_positions : 목표 관절값(rad) — 외부에서 set_target()으로 갱신
    - run() 루프 내에서 현재값을 저장하고, 목표를 읽어 position 명령을 보냄
    """

    def __init__(self, router, router_rt, alpha: float = 1.0):
        """
        alpha: 0<alpha<=1.0, 목표 위치를 부드럽게 만들고 싶을 때 0.1~0.5 권장
               1.0이면 바로 목표로 명령
        """
        self.base = BaseClient(router)
        self.base_cyclic = BaseCyclicClient(router_rt)
        self.device_manager = DeviceManagerClient(router)
        self.actuator_config = ActuatorConfigClient(router)

        self.send_opt = RouterClientSendOptions()
        self.send_opt.andForget = False
        self.send_opt.delay_ms = 0
        self.send_opt.timeout_ms = 3

        self.base_command = BaseCyclic_pb2.Command()
        self.base_feedback = BaseCyclic_pb2.Feedback()

        # 장치/관절 개수 파악 및 프레임 초기화
        self.actuator_count = self.base.GetActuatorCount().count
        self.base_command.ClearField("actuators")
        self.base_feedback.ClearField("actuators")
        for _ in range(self.actuator_count):
            self.base_command.actuators.add()
            self.base_feedback.actuators.add()

        # 상태 버퍼
        self.current_positions: List[float] = [0.0] * self.actuator_count
        self.target_positions: List[float] = [0.0] * self.actuator_count
        self.alpha = float(alpha)

        # 이동 평균 필터
        self.action_history = deque(maxlen=10)
        self.window_size = int(100/40)

    # ---------- 유틸 ----------
    @staticmethod
    def _retry(call, retry, *args):
        i = 0
        arg_out = []
        while i < retry:
            try:
                arg_out = call(*args)
                break
            except:
                i = i + 1
                continue
        if i == retry:
            print("Failed to communicate")
        return arg_out

    def move_to_home(self, timeout_s: float = 20.0) -> bool:
        """Single level servoing에서 Home 액션 실행."""
        self._set_servo_mode(Base_pb2.SINGLE_LEVEL_SERVOING)

        req = Base_pb2.RequestedActionType()
        req.action_type = Base_pb2.REACH_JOINT_ANGLES
        actions = self.base.ReadAllActions(req)

        home_handle = None
        for act in actions.action_list:
            if act.name == "kinova_home":
                home_handle = act.handle
                break
        if home_handle is None:
            print("[WARN] Home 액션을 찾을 수 없습니다.")
            return False

        e = {"done": False}

        def _cb(note, _e=e):
            if note.action_event in (Base_pb2.ACTION_END, Base_pb2.ACTION_ABORT):
                _e["done"] = True

        handle = self.base.OnNotificationActionTopic(_cb, Base_pb2.NotificationOptions())
        self.base.ExecuteActionFromReference(home_handle)

        t0 = time.time()
        while not e["done"] and (time.time() - t0) < timeout_s:
            time.sleep(0.05)
        self.base.Unsubscribe(handle)
        return e["done"]

    # ---------- 외부 API ----------
    def set_target(self, joint_positions: List[float]):
        if len(joint_positions) != self.actuator_count:
            raise ValueError(...)
        self.target_positions = list(joint_positions)   # 즉시 반영

        # 필요하면 이동평균을 별도 옵션으로:
        self.action_history.append(joint_positions)
        if len(self.action_history) >= self.window_size:
            smoothed = [0.0] * self.actuator_count
            for pos in self.action_history:
                for i in range(self.actuator_count):
                    smoothed[i] += pos[i]
            smoothed = [x / len(self.action_history) for x in smoothed]
            self.target_positions = smoothed

    def get_current(self) -> List[float]:
        """현재 관절각(rad) 반환 (최근 피드백 기반)."""
        return list(self.current_positions)

    # ---------- 메인 초기화 ----------
    def init(self, do_home: bool = True) -> None:
        # 1) 홈으로 (SINGLE_LEVEL_SERVOING)
        if do_home:
            ok = self.move_to_home()
            if not ok:
                print("[WARN] Home 이동 실패 또는 건너뜀")

        # 2) 첫 피드백 (여전히 SINGLE_LEVEL 상태)
        fb = self._retry(self.base_cyclic.RefreshFeedback, 3)
        if fb:
            self.base_feedback = fb
            for i in range(self.actuator_count):
                self.current_positions[i] = fb.actuators[i].position
                # 첫 명령은 현재값으로 동기화 + servoing on
                self.base_command.actuators[i].flags = 1
                self.base_command.actuators[i].position = fb.actuators[i].position

            base_servo_mode = Base_pb2.ServoingModeInformation()
            base_servo_mode.servoing_mode = Base_pb2.LOW_LEVEL_SERVOING
            self.base.SetServoingMode(base_servo_mode)
            time.sleep(0.05)  # 소폭 대기(권장)

            # 4) 첫 프레임을 현재값으로 한 번 보내 안정화
            self.base_feedback = self.base_cyclic.Refresh(self.base_command, 0, self.send_opt)

            # 5) 초기 타깃 설정(없다면 현재값)
            if all(abs(tp) < 1e-9 for tp in self.target_positions):
                self.target_positions = list(self.current_positions)


    # ---------- 제어 루프 ----------
    def run(self, period_s: float = 0.001, duration_s: Optional[float] = None, print_rate: float = 0.5):
        if period_s <= 0:
            raise ValueError("period_s must be > 0")

        print("[INFO] Position control loop started.")
        t0 = time.time()
        t_next = t0
        t_print = t0

        try:
            while True:
                now = time.time()
                if now >= t_next:
                    t_next += period_s

                    # (A) 직전 피드백 기반으로 현재값 갱신
                    fb_prev = self.base_feedback
                    for i in range(self.actuator_count):
                        self.current_positions[i] = fb_prev.actuators[i].position

                    # (B) 목표값 읽어 명령 구성 (필요하면 필터/슬루 추가)
                    for i in range(self.actuator_count):
                        curr = self.current_positions[i]
                        tgt  = self.target_positions[i]
                        cmd_pos = tgt  # 또는 alpha로 부드럽게: self.alpha*tgt + (1-self.alpha)*curr
                        self.base_command.actuators[i].flags = 1
                        self.base_command.actuators[i].position = cmd_pos

                    # frame_id / command_id
                    self.base_command.frame_id += 1
                    if self.base_command.frame_id > 65535:
                        self.base_command.frame_id = 0
                    for i in range(self.actuator_count):
                        self.base_command.actuators[i].command_id = self.base_command.frame_id
                        
                    # (C) Refresh를 호출하며 "지금 만든 명령"을 즉시 전송 + 새로운 피드백 수신
                    try:
                        self.base_feedback = self.base_cyclic.Refresh(self.base_command, 0, self.send_opt)
                    except Exception:
                        # 통신 에러 시 다음 사이클
                        continue

                # 상태 출력
                if print_rate and (now - t_print) >= print_rate:
                    t_print = now
                    print("[posctl] curr:", [round(x, 4) for x in self.current_positions],
                        "-> tgt:", [round(x, 4) for x in self.target_positions],
                        "| send j0:", round(self.base_command.actuators[0].position, 3))

                if (duration_s is not None) and ((now - t0) >= duration_s):
                    break

                time.sleep(0.0002)
        except KeyboardInterrupt:
            print("\n[INFO] Interrupted by user.")
        finally:
            self._set_servo_mode(Base_pb2.SINGLE_LEVEL_SERVOING)
            print("[INFO] Position control loop ended.")

    def _set_servo_mode(self, mode: Base_pb2.ServoingMode):
        info = Base_pb2.ServoingModeInformation()
        info.servoing_mode = mode
        self._retry(self.base.SetServoingMode, 3, info)

# ---------------- 예시 실행 ----------------
def main():
    import argparse
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
    import utilities

    parser = argparse.ArgumentParser()
    parser.add_argument("--period", type=float, default=0.002, help="control period seconds")
    parser.add_argument("--duration", type=float, default=10.0, help="run duration seconds (None for infinite)")
    parser.add_argument("--alpha", type=float, default=1.0, help="smoothing factor (0<alpha<=1)")
    args = utilities.parseConnectionArguments(parser)

    with utilities.DeviceConnection.createTcpConnection(args) as router:
        with utilities.DeviceConnection.createUdpConnection(args) as router_rt:
            ctl = PositionController(router, router_rt, alpha=args.alpha)
            ctl.init(do_home=True)

            # 초기 타깃: 현재 자세 유지
            ctl.set_target(ctl.get_current())

            # 예시: 2초 뒤 첫 번째 관절을 +0.2rad로 살짝 회전
            time.sleep(2.0)
            tgt = ctl.get_current()
            tgt[0] += 20
            ctl.set_target(tgt)

            ctl.run(period_s=args.period, duration_s=args.duration)

    return 0

if __name__ == "__main__":
    sys.exit(main())
