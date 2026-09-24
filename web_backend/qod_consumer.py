"""
qod_consumer.py - WebRTC camera consumer for the QOD lab SFU.

Consumes a robot dog's camera feed from the qod streaming platform's WebRTC
signaling server (ws://<backend>/ws/signaling) using exactly the same consumer
role and message flow as the qod platform's own consumers (cv-worker and web
frontend):

    role = "consumer"  ->  send offer  ->  receive answer + ICE  ->  video track

Each incoming VideoFrame is decoded to a raw BGR ndarray and handed to an
on_frame(frame) callback. The consumer runs its own asyncio event loop in a
daemon thread so it never blocks the Gradio / rclpy threads, and it always
reconnects (with backoff) so a robot/network blip does not freeze the camera.
"""

import asyncio
import json
import logging
import os
import threading

logger = logging.getLogger("qod_consumer")

try:
    from aiortc import RTCPeerConnection, RTCSessionDescription
    from aiortc.sdp import candidate_from_sdp
    import websockets

    _DEP_OK = True
except Exception as exc:  # pragma: no cover - missing deps on a fresh container
    logger.warning("aiortc/websockets import failed: %r", exc)
    _DEP_OK = False

DEFAULT_SIGNALING_URL = os.getenv(
    "QOD_SIGNALING_URL", "ws://127.0.0.1:8000/ws/signaling"
)
RECONNECT_DELAY = float(os.getenv("QOD_RECONNECT_DELAY", "2.0"))


class QodCameraConsumer:
    """Receive a robot dog's camera from the QOD SFU and forward decoded frames.

    Args:
        source_id: QOD publisher source id under which the camera is registered
            (e.g. "luna", "astro" - see GET /api/publisher/publishers).
        on_frame: callable(frame: np.ndarray BGR) invoked for every decoded frame.
        name: label used in logs (e.g. robot display name "Luna").
        signaling_url: full ws URL of the qod backend signaling endpoint.
    """

    def __init__(self, source_id, on_frame, name="QodConsumer",
                 signaling_url=DEFAULT_SIGNALING_URL):
        self._source_id = source_id
        self._on_frame = on_frame
        self._name = name
        self._signaling_url = signaling_url
        self._stop = threading.Event()
        self._thread = None
        self._loop = None

    @property
    def alive(self):
        return self._thread is not None and self._thread.is_alive()

    def start(self):
        if not _DEP_OK:
            logger.error(
                "[%s] QOD consumer deps missing - run: "
                "python3 -m pip install aiortc websockets",
                self._name,
            )
            return
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._run_server, daemon=True,
                                        name=f"qod-consumer-{self._source_id}")
        self._thread.start()

    def stop(self):
        self._stop.set()
        if self._loop is not None:
            try:
                self._loop.call_soon_threadsafe(self._loop.stop)
            except Exception:
                pass

    # ---------------------- internal --------------------------------
    def _run_server(self):
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        self._loop = loop
        try:
            loop.run_until_complete(self._run_forever())
        except asyncio.CancelledError:
            pass
        finally:
            loop.close()
            self._loop = None

    async def _run_forever(self):
        while not self._stop.is_set():
            try:
                await self._connect_and_consume()
            except asyncio.CancelledError:
                raise
            except Exception as exc:
                logger.warning("[%s] consumer error: %r", self._source_id, exc)
            if self._stop.is_set():
                break
            logger.info("[%s] reconnecting in %.1fs...", self._source_id,
                        RECONNECT_DELAY)
            await asyncio.sleep(RECONNECT_DELAY)

    async def _connect_and_consume(self):
        """Mirror the qod cv-worker: offer a recvonly consumer, then drain the
        video track until it ends / the socket closes, then return to retry."""
        if not self._source_id:
            logger.error("no source_id configured")
            return
        consumer_id = f"dashboard-{self._source_id}-{os.getpid()}"
        logger.info("[%s] connecting to %s", self._source_id, self._signaling_url)

        async with websockets.connect(self._signaling_url) as ws:
            pc = RTCPeerConnection()
            pc.addTransceiver("video", direction="recvonly")

            async def _safe_send(text):
                try:
                    await ws.send(text)
                except Exception:
                    pass

            @pc.on("icecandidate")
            async def on_icecandidate(candidate):
                if not candidate:
                    return
                await _safe_send(json.dumps({
                    "role": "consumer",
                    "type": "ice",
                    "id": consumer_id,
                    "candidate": {
                        "candidate": candidate.candidate,
                        "sdpMid": candidate.sdpMid,
                        "sdpMLineIndex": candidate.sdpMLineIndex,
                    },
                }))

            @pc.on("track")
            async def on_track(track):
                if track.kind != "video":
                    return
                logger.info("[%s] video track received (kind=%s)",
                            self._source_id, track.kind)
                await self._drain_track(track)
                # Track ended (publisher/SFU reset) - tear down so we reconnect.
                try:
                    await ws.close()
                except Exception:
                    pass

            offer = await pc.createOffer()
            await pc.setLocalDescription(offer)
            await ws.send(json.dumps({
                "role": "consumer",
                "type": "offer",
                "id": consumer_id,
                "sourceId": self._source_id,
                "sdp": pc.localDescription.sdp,
            }))

            try:
                async for raw in ws:
                    if self._stop.is_set():
                        break
                    try:
                        msg = json.loads(raw)
                    except json.JSONDecodeError:
                        continue
                    mtype = msg.get("type")
                    if mtype == "answer" and msg.get("sdp"):
                        await pc.setRemoteDescription(
                            RTCSessionDescription(msg["sdp"], "answer"))
                    elif mtype == "ice" and msg.get("candidate"):
                        cand = msg["candidate"]
                        sdp = (cand or {}).get("candidate")
                        if not sdp:
                            continue
                        ice = candidate_from_sdp(sdp)
                        ice.sdpMid = cand.get("sdpMid")
                        ice.sdpMLineIndex = cand.get("sdpMLineIndex")
                        await pc.addIceCandidate(ice)
                    elif msg.get("error") == "no_publisher":
                        logger.info("[%s] no publisher yet; will retry",
                                    self._source_id)
                        return
            except websockets.exceptions.ConnectionClosed:
                pass
            finally:
                await pc.close()

    async def _drain_track(self, track):
        """Wait for decoded frames and forward each to the callback."""
        while not self._stop.is_set():
            try:
                frame = await track.recv()
            except Exception:
                return
            try:
                img = frame.to_ndarray(format="bgr24")
            except Exception:
                continue
            try:
                self._on_frame(img)
            except Exception:
                continue