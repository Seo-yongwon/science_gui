"""로버(UMPC)에서 실행: 원격에서 rover.launch.py 를 한 번 실행할 수 있는 트리거 서비스.

전체 스택(rover.launch)이 아직 안 돌 때는 별도 데몬으로 본 노드만 켠 뒤,
기지국 GUI 등에서 Trigger 서비스를 호출하면 워크스페이스를 source 한 뒤
``ros2 launch science_gui rover.launch.py`` 를 백그라운드로 실행한다.

중복 실행은 pgrep 패턴으로 감지한다(pgrep 미설치 시 생략).
"""
from __future__ import annotations

import os
import shlex
import subprocess
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


class RoverLaunchTriggerNode(Node):
    def __init__(self) -> None:
        super().__init__('rover_launch_trigger')

        self.declare_parameter('service_name', '/rover/trigger_launch')
        self.declare_parameter('launch_package', 'science_gui')
        self.declare_parameter('launch_file', 'rover.launch.py')
        # 예: basestation_ip:=192.168.1.30
        self.declare_parameter('launch_extra_args', '')
        self.declare_parameter('ros_setup', '')
        self.declare_parameter(
            'workspace_setup',
            '~/science_ws/install/setup.bash',
        )
        # pgrep -af 가 이 문자열을 포함하는 프로세스가 있으면 이미 실행 중으로 간주
        self.declare_parameter(
            'duplicate_grep_pattern',
            'ros2 launch science_gui rover.launch.py',
        )
        self.declare_parameter(
            'log_file',
            '~/camera_captures/rover_launch_background.log',
        )

        srv_name = self._p_str('service_name')
        self.create_service(Trigger, srv_name, self._on_trigger)

        self.get_logger().info(
            f'rover_launch_trigger: 서비스 {srv_name}\n'
            '  대기 중 — 기지국 GUI에서 로버 스택 시작 트리거 가능'
        )

    def _p_str(self, name: str) -> str:
        return self.get_parameter(name).get_parameter_value().string_value

    def _stack_running(self) -> bool:
        pat = self._p_str('duplicate_grep_pattern').strip()
        if not pat:
            return False
        try:
            r = subprocess.run(
                ['/usr/bin/pgrep', '-af', pat],
                capture_output=True,
                text=True,
                timeout=3.0,
                check=False,
            )
            if r.returncode != 0 or not r.stdout.strip():
                return False
            return True
        except (FileNotFoundError, subprocess.TimeoutExpired):
            self.get_logger().warn('pgrep 사용 불가 — 중복 실행 검사 생략')
            return False

    def _on_trigger(self, _req: Trigger.Request, res: Trigger.Response) -> Trigger.Response:
        pkg = self._p_str('launch_package').strip()
        launch_py = self._p_str('launch_file').strip()
        extra = self._p_str('launch_extra_args').strip()

        if not pkg or not launch_py:
            res.success = False
            res.message = 'launch_package 또는 launch_file 파라미터가 비었습니다.'
            return res

        if self._stack_running():
            res.success = False
            res.message = '이미 rover.launch 가 실행 중입니다.'
            return res

        ros_setup = self._p_str('ros_setup').strip()
        if not ros_setup:
            distro = os.environ.get('ROS_DISTRO', 'humble')
            ros_setup = f'/opt/ros/{distro}/setup.bash'

        ws_setup = os.path.expanduser(self._p_str('workspace_setup').strip())
        if not ws_setup:
            res.success = False
            res.message = 'workspace_setup 경로가 비었습니다.'
            return res

        log_path = self._p_str('log_file').strip()
        log_f = None
        if log_path:
            log_path = os.path.expanduser(log_path)
            Path(log_path).parent.mkdir(parents=True, exist_ok=True)
            log_f = open(log_path, 'a', encoding='utf-8')

        argv_launch = ['ros2', 'launch', pkg, launch_py]
        if extra:
            argv_launch.extend(shlex.split(extra))

        inner = ' '.join(shlex.quote(a) for a in argv_launch)
        bash_src = (
            f'source {shlex.quote(ros_setup)} && '
            f'source {shlex.quote(ws_setup)} && '
            f'exec {inner}'
        )

        try:
            subprocess.Popen(
                ['/bin/bash', '-lc', bash_src],
                stdin=subprocess.DEVNULL,
                stdout=log_f or subprocess.DEVNULL,
                stderr=subprocess.STDOUT if log_f else subprocess.DEVNULL,
                start_new_session=True,
            )
        except OSError as e:
            if log_f:
                log_f.close()
            res.success = False
            res.message = f'실행 실패: {e}'
            return res

        if log_f:
            log_f.close()

        res.success = True
        res.message = f'백그라운드 시작: {" ".join(argv_launch)} (로그: {log_path or "없음"})'
        self.get_logger().info(res.message)
        return res


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RoverLaunchTriggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
