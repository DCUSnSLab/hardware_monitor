"""bag 재생 담당 (ROS/WS 비의존).

재생은 ros2 bag play 서브프로세스로 처리한다(배포판 서비스에 의존하지 않는 respawn 모델).
 - open   = bag 로드(일시정지 상태로 대기, 재생 X) + 토픽 목록을 metadata에서 고정
 - play   = 보존 위치에서 --start-offset 으로 시작/재개
 - pause  = 프로세스 종료 + 현재 위치 보존
 - seek   = 원하는 위치에서 재시작(일시정지 중이면 위치만 갱신)
 - rate   = 현재 위치에서 --rate 로 재시작
 - stop   = 종료 + 상태 초기화
현재 위치는 rosbag2가 직접 제공하지 않으므로, 시작시각+배속으로 추정한다.

BagPlayer는 I/O를 하지 않는다: 상태를 바꾸고 "보낼 dict"를 만들어 리턴하면
노드(RelayBridgeNode)가 실제 전송/토픽리스트 재전송을 담당한다.
"""
import os
import signal
import subprocess
import time


def load_metadata(bag_path):
    meta_path = os.path.join(bag_path, "metadata.yaml")
    try:
        import yaml
        with open(meta_path, "r") as f:
            return yaml.safe_load(f) or {}
    except Exception:
        return {}


def read_bag_duration(bag_path):
    binfo = load_metadata(bag_path).get("rosbag2_bagfile_information", {}) or {}
    dur = (binfo.get("duration", {}) or {}).get("nanoseconds")
    if isinstance(dur, (int, float)):
        return dur / 1e9
    return 0.0


def read_bag_topics(bag_path):
    """metadata.yaml에서 bag이 담은 토픽 목록을 읽는다(재생 전에도 사이드바에 고정 표시)."""
    binfo = load_metadata(bag_path).get("rosbag2_bagfile_information", {}) or {}
    topics = []
    for entry in (binfo.get("topics_with_message_count") or []):
        tm = (entry.get("topic_metadata") or {})
        name = tm.get("name")
        ttype = tm.get("type")
        if name and ttype:
            topics.append({"name": name, "type": ttype})
    return sorted(topics, key=lambda x: x["name"])


class BagPlayer:
    def __init__(self, vehicle_id, logger=None):
        self.vehicle_id = vehicle_id
        self.logger = logger

        self.bag_proc = None
        self.bag_path = ""
        self.bag_name = ""
        self.bag_duration = 0.0        # 초
        self.bag_rate = 1.0
        self.bag_paused = False
        self.bag_base_position = 0.0   # 마지막 (재)시작/일시정지 시점의 위치(초)
        self.bag_play_started = None   # 재생(비일시정지) 구간 시작 monotonic
        self.bag_loaded = False        # bag이 로드된 세션인지(로드 중엔 토픽 목록 고정)
        self.bag_topics = None         # 로드된 bag의 토픽 목록(metadata 기준, 고정)

    # ---- 상태 조회 (노드가 참조) ----
    def is_loaded(self):
        return self.bag_loaded

    def get_topics(self):
        return self.bag_topics

    def is_active(self):
        return self.bag_loaded or self.bag_proc is not None

    # ---- 내부 유틸 ----
    def _current_position(self):
        pos = self.bag_base_position
        if self.bag_play_started is not None and not self.bag_paused:
            pos += (time.monotonic() - self.bag_play_started) * self.bag_rate
        if self.bag_duration > 0:
            pos = max(0.0, min(pos, self.bag_duration))
        else:
            pos = max(0.0, pos)
        return pos

    def _state(self):
        if not self.bag_path:
            return "idle"
        if self.bag_paused:
            return "paused"
        if self.bag_proc is not None and self.bag_proc.poll() is None:
            return "playing"
        return "idle"

    def _kill(self):
        proc = self.bag_proc
        self.bag_proc = None
        if proc is None or proc.poll() is not None:
            return
        try:
            pgid = os.getpgid(proc.pid)
            os.killpg(pgid, signal.SIGINT)
            try:
                proc.wait(timeout=1.0)
                return
            except subprocess.TimeoutExpired:
                pass
            os.killpg(pgid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        except Exception as exc:
            if self.logger:
                self.logger.warning(f"bag play kill failed: {exc}")

    def _spawn(self, offset):
        """현재 bag_path/bag_rate로 offset(초)부터 ros2 bag play 재시작."""
        self._kill()
        if not self.bag_path:
            return False, "no bag opened"

        offset = max(0.0, float(offset))
        if self.bag_duration > 0:
            offset = min(offset, self.bag_duration)

        cmd = [
            "ros2", "bag", "play", self.bag_path,
            "--rate", str(self.bag_rate),
            "--start-offset", str(offset),
        ]
        try:
            self.bag_proc = subprocess.Popen(cmd, start_new_session=True)
            if self.logger:
                self.logger.info(
                    f"▶️ ros2 bag play: {self.bag_path} rate={self.bag_rate} offset={offset:.2f}"
                )
            return True, ""
        except Exception as exc:
            self.bag_proc = None
            return False, f"failed to start ros2 bag play: {exc}"

    def reset(self):
        """재생 종료 + 상태 초기화(연결 끊김/stop 공통)."""
        self._kill()
        self.bag_paused = False
        self.bag_base_position = 0.0
        self.bag_play_started = None
        self.bag_path = ""
        self.bag_name = ""
        self.bag_duration = 0.0
        self.bag_loaded = False
        self.bag_topics = None

    def shutdown(self):
        self._kill()

    # ---- 재생 제어 ----
    # 상태를 바꾸고 결과를 리턴한다.
    #   반환: {"response": {...bag_playback_response...}, "resend_topics": bool}
    #   resend_topics=True 면 노드가 topic_list를 재전송해야 한다(open/stop).
    def handle_request(self, data):
        action = data.get("action")
        request_id = data.get("request_id")
        reply_to = data.get("reply_to")
        success = True
        error = ""
        resend_topics = False

        if action == "open":
            bag_path = data.get("bag_path") or ""
            if not bag_path:
                success, error = False, "bag_path is required for open"
            else:
                # 로드만 하고 재생은 하지 않는다(일시정지 대기 → 사용자가 play를 눌러야 재생).
                self._kill()
                self.bag_path = bag_path
                self.bag_name = os.path.basename(bag_path.rstrip("/"))
                self.bag_duration = read_bag_duration(bag_path)
                self.bag_topics = read_bag_topics(bag_path)
                self.bag_loaded = True
                self.bag_rate = float(data.get("rate") or 1.0)
                self.bag_base_position = 0.0
                self.bag_play_started = None
                self.bag_paused = True
                resend_topics = True  # 사이드바에 bag 토픽 목록 고정 표시

        elif action == "stop":
            self.reset()
            resend_topics = True  # 라이브 토픽 목록으로 복귀

        elif action == "pause":
            if self.bag_path and not self.bag_paused:
                # kill(최대 1초 대기) 이전에 위치/일시정지를 먼저 확정한다.
                # (안 그러면 kill 대기 동안 추정 위치가 계속 전진 → 재생바 원이 앞으로 갔다 돌아옴)
                self.bag_base_position = self._current_position()
                self.bag_paused = True
                self.bag_play_started = None
                self._kill()

        elif action in ("play", "resume"):
            if not self.bag_path:
                success, error = False, "no bag opened"
            else:
                if data.get("rate"):
                    self.bag_rate = float(data.get("rate"))
                self.bag_paused = False
                ok, err = self._spawn(self.bag_base_position)
                if ok:
                    self.bag_play_started = time.monotonic()
                else:
                    success, error = False, err

        elif action == "seek":
            if not self.bag_path:
                success, error = False, "no bag opened"
            else:
                pos = float(
                    data.get("position_seconds")
                    if data.get("position_seconds") is not None
                    else (data.get("position") or 0.0)
                )
                if self.bag_duration > 0:
                    pos = max(0.0, min(pos, self.bag_duration))
                self.bag_base_position = pos
                if not self.bag_paused:
                    ok, err = self._spawn(pos)
                    if ok:
                        self.bag_play_started = time.monotonic()
                    else:
                        success, error = False, err

        elif action == "rate":
            if not self.bag_path:
                success, error = False, "no bag opened"
            else:
                self.bag_base_position = self._current_position()
                self.bag_rate = float(data.get("rate") or self.bag_rate)
                if not self.bag_paused:
                    ok, err = self._spawn(self.bag_base_position)
                    if ok:
                        self.bag_play_started = time.monotonic()
                    else:
                        success, error = False, err

        else:
            success, error = False, f"unknown playback action: {action}"

        state = self._state()
        response = {
            "type": "bag_playback_response",
            "request_id": request_id,
            "reply_to": reply_to,
            "vehicle_id": self.vehicle_id,
            "success": success,
            "error": error,
            "state": state,
            "bag_path": self.bag_path,
            "bag_name": self.bag_name,
            "current_time": self._current_position(),
            "duration": self.bag_duration,
            "rate": self.bag_rate,
            "is_playing": state == "playing",
        }
        return {"response": response, "resend_topics": resend_topics}

    # 0.25초 타이머용: 보낼 상태 dict(or None)만 만들어 리턴.
    def build_status(self):
        if not self.bag_path:
            return None  # 열린 bag 없으면 상태 미전송

        # 재생 중이었는데 프로세스가 끝났으면(=bag 끝까지 재생) 완료 처리(끝 위치에서 정지)
        if (not self.bag_paused and self.bag_proc is not None
                and self.bag_proc.poll() is not None):
            self.bag_base_position = self.bag_duration
            self.bag_play_started = None
            self.bag_proc = None
            self.bag_paused = True

        pos = self._current_position()
        state = self._state()
        return {
            "type": "bag_playback_status",
            "vehicle_id": self.vehicle_id,
            "bag_path": self.bag_path,
            "bag_name": self.bag_name,
            "state": state,
            "current_time": pos,
            "duration": self.bag_duration,
            "rate": self.bag_rate,
            "is_playing": state == "playing",
        }