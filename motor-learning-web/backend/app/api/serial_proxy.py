import asyncio
import threading
from typing import Optional

from fastapi import APIRouter, WebSocket, WebSocketDisconnect, HTTPException
from pydantic import BaseModel

try:
    import serial
    import serial.tools.list_ports
    HAS_SERIAL = True
except ImportError:
    HAS_SERIAL = False

router = APIRouter()


class SerialOpenRequest(BaseModel):
    port: str
    baudRate: int = 115200
    dataBits: int = 8
    stopBits: float = 1.0
    parity: str = "none"


class _PortManager:
    _instance: Optional["_PortManager"] = None
    _lock = threading.Lock()

    def __init__(self):
        self._serial: Optional[serial.Serial] = None
        self._is_open = False
        self._read_thread: Optional[threading.Thread] = None
        self._data_queue: list[bytes] = []
        self._queue_lock = threading.Lock()
        self._ws_clients: list[WebSocket] = []
        self._running = False

    @classmethod
    def get(cls) -> "_PortManager":
        if cls._instance is None:
            with cls._lock:
                if cls._instance is None:
                    cls._instance = cls()
        return cls._instance

    def list_ports(self) -> list[str]:
        if not HAS_SERIAL:
            return []
        return [p.device for p in serial.tools.list_ports.comports()]

    def open_port(self, req: SerialOpenRequest) -> bool:
        if not HAS_SERIAL:
            return False
        with self._lock:
            if self._is_open:
                return False
            parity_map = {
                "none": serial.PARITY_NONE,
                "even": serial.PARITY_EVEN,
                "odd": serial.PARITY_ODD,
                "mark": serial.PARITY_MARK,
                "space": serial.PARITY_SPACE,
            }
            try:
                self._serial = serial.Serial(
                    port=req.port,
                    baudrate=req.baudRate,
                    bytesize=req.dataBits,
                    parity=parity_map.get(req.parity.lower(), serial.PARITY_NONE),
                    stopbits=req.stopBits,
                    timeout=0.05,
                )
                self._is_open = True
                self._running = True
                self._start_read_thread()
                return True
            except serial.SerialException:
                return False

    def close_port(self) -> bool:
        with self._lock:
            if not self._is_open or self._serial is None:
                return False
            self._running = False
            try:
                self._serial.close()
            except Exception:
                pass
            self._serial = None
            self._is_open = False
            return True

    def write_data(self, data: bytes) -> int:
        with self._lock:
            if not self._is_open or self._serial is None:
                return 0
            try:
                return self._serial.write(data)
            except serial.SerialException:
                self._is_open = False
                return 0

    def pop_queue(self) -> list[bytes]:
        with self._queue_lock:
            items = self._data_queue[:]
            self._data_queue.clear()
            return items

    def add_ws_client(self, ws: WebSocket):
        self._ws_clients.append(ws)

    def remove_ws_client(self, ws: WebSocket):
        if ws in self._ws_clients:
            self._ws_clients.remove(ws)

    def _start_read_thread(self):
        t = threading.Thread(target=self._read_loop, daemon=True)
        self._read_thread = t
        t.start()

    def _read_loop(self):
        while self._running and self._serial is not None:
            try:
                data = self._serial.read(4096)
                if data:
                    with self._queue_lock:
                        self._data_queue.append(data)
            except serial.SerialException:
                self._running = False
                break
            except Exception:
                self._running = False
                break


@router.get("/ports")
async def list_serial_ports():
    if not HAS_SERIAL:
        raise HTTPException(status_code=503, detail="pyserial not installed")
    return {"ports": _PortManager.get().list_ports()}


@router.post("/open")
async def open_serial_port(req: SerialOpenRequest):
    if not HAS_SERIAL:
        raise HTTPException(status_code=503, detail="pyserial not installed")
    mgr = _PortManager.get()
    if mgr.open_port(req):
        return {"status": "ok", "port": req.port, "baudRate": req.baudRate}
    raise HTTPException(status_code=400, detail="Failed to open serial port")


@router.post("/close")
async def close_serial_port():
    mgr = _PortManager.get()
    if mgr.close_port():
        return {"status": "ok"}
    raise HTTPException(status_code=400, detail="No serial port is open")


@router.websocket("/ws")
async def serial_ws(websocket: WebSocket):
    await websocket.accept()
    mgr = _PortManager.get()
    mgr.add_ws_client(websocket)

    async def send_loop():
        try:
            while mgr._running:
                items = mgr.pop_queue()
                for data in items:
                    try:
                        await websocket.send_bytes(data)
                    except Exception:
                        return
                await asyncio.sleep(0.002)
        except Exception:
            pass

    async def recv_loop():
        try:
            while True:
                data = await websocket.receive_bytes()
                mgr.write_data(data)
        except WebSocketDisconnect:
            pass
        except Exception:
            pass

    try:
        await asyncio.gather(send_loop(), recv_loop())
    except Exception:
        pass
    finally:
        mgr.remove_ws_client(websocket)
