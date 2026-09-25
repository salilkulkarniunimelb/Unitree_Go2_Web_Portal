"""
qod_detections.py - Subscribe to the qod platform's object-detection stream.

The qod cv-worker runs YOLO on each robot's WebRTC feed and publishes the
results (normalized bounding boxes) to Redis channel "detections:<sourceId>".
The qod website draws those boxes on a canvas overlay. This module subscribes
to the same Redis channel so the dashboard can draw exactly the same overlay
server-side (see _draw_detections in lab_portal.py).

Runs its own daemon thread and quietly retries if Redis is unreachable, so it
never blocks or crashes the dashboard.
"""

import json
import logging
import os
import threading
import time

logger = logging.getLogger("qod_detections")

try:
    import redis as _redis

    _REDIS_OK = True
except Exception as exc:  # pragma: no cover - missing dep on a fresh container
    logger.warning("redis import failed: %r", exc)
    _REDIS_OK = False

DEFAULT_REDIS_URL = os.getenv("QOD_DETECTIONS_REDIS_URL", "redis://127.0.0.1:6379")


class QodDetectionsSubscriber:
    """Keep the newest detection payload for one robot source.

    Args:
        source_id: qod source id that owns the detections ("luna", "astro").
            We subscribe to channel "detections:<source_id>".
        name: display name for logging (e.g. "Luna").
        redis_url: full redis:// URL.
    """

    def __init__(self, source_id, name="QodDetections",
                 redis_url=DEFAULT_REDIS_URL):
        self._source_id = source_id
        self._name = name
        self._redis_url = redis_url
        self._latest = None          # parsed detection payload dict (or None)
        self._latest_ts = 0.0        # monotonic time the payload arrived
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = None

    @property
    def alive(self):
        return self._thread is not None and self._thread.is_alive()

    @property
    def connected(self):
        return self._latest is not None

    def start(self):
        if not _REDIS_OK:
            logger.error(
                "[%s] redis module missing - run: "
                "python3 -m pip install redis",
                self._name,
            )
            return
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(
            target=self._run, daemon=True,
            name=f"qod-detections-{self._source_id}",
        )
        self._thread.start()

    def stop(self):
        self._stop.set()

    def latest(self):
        """Return a copy of the newest payload, or None if none received."""
        with self._lock:
            if self._latest is None:
                return None
            return json.loads(json.dumps(self._latest))

    def detections(self):
        """Convenience: newest payload's "detections" list, or []."""
        payload = self.latest()
        if not payload:
            return []
        return payload.get("detections") or []

    # ---------------------- internal --------------------------------
    def _run(self):
        while not self._stop.is_set():
            try:
                client = _redis.Redis.from_url(self._redis_url,
                                               decode_responses=True)
                pubsub = client.pubsub(ignore_subscribe_messages=True)
                pubsub.psubscribe(f"detections:{self._source_id}")
                logger.info(
                    "[%s] subscribed to detections:%s (%s)",
                    self._name, self._source_id, self._redis_url,
                )
                try:
                    for message in pubsub.listen():
                        if self._stop.is_set():
                            break
                        self._handle_message(message)
                finally:
                    try:
                        pubsub.close()
                    except Exception:
                        pass
                    try:
                        client.close()
                    except Exception:
                        pass
            except Exception as exc:
                logger.warning(
                    "[%s] detections subscriber error: %r (retrying)",
                    self._name, exc,
                )
            if self._stop.is_set():
                break
            self._stop.wait(2.0)

    def _handle_message(self, message):
        if message.get("type") != "pmessage":
            return
        data = message.get("data")
        if not isinstance(data, str):
            return
        try:
            payload = json.loads(data)
        except json.JSONDecodeError:
            return
        detections = payload.get("detections")
        if not isinstance(detections, list):
            return
        with self._lock:
            self._latest = payload
            self._latest_ts = time.monotonic()
        if self._latest_ts == 0:
            pass

    @property
    def last_payload_ts(self):
        with self._lock:
            return self._latest_ts