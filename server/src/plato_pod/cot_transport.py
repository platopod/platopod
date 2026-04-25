"""CoT transport layer — UDP unicast/multicast and TCP.

Pure Python socket-based transport for sending CoT XML events.
No external dependencies.
"""

from __future__ import annotations

import logging
import socket
import threading

logger = logging.getLogger(__name__)


class CotTransport:
    """Abstract base for CoT output."""

    def send(self, xml: str) -> None:
        """Send a CoT XML event."""
        raise NotImplementedError

    def close(self) -> None:
        """Close the transport."""
        pass


class UdpUnicastTransport(CotTransport):
    """Send CoT events via UDP unicast to a specific host:port."""

    def __init__(self, host: str, port: int) -> None:
        self._host = host
        self._port = port
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._lock = threading.Lock()

    def send(self, xml: str) -> None:
        data = xml.encode("utf-8")
        with self._lock:
            try:
                self._sock.sendto(data, (self._host, self._port))
            except OSError as e:
                logger.warning("UDP unicast send failed: %s", e)

    def close(self) -> None:
        self._sock.close()


class UdpMulticastTransport(CotTransport):
    """Send CoT events via UDP multicast."""

    def __init__(
        self, group: str = "239.2.3.1", port: int = 6969, ttl: int = 32
    ) -> None:
        self._group = group
        self._port = port
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        self._sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, ttl)
        self._lock = threading.Lock()

    def send(self, xml: str) -> None:
        data = xml.encode("utf-8")
        with self._lock:
            try:
                self._sock.sendto(data, (self._group, self._port))
            except OSError as e:
                logger.warning("UDP multicast send failed: %s", e)

    def close(self) -> None:
        self._sock.close()


class TcpTransport(CotTransport):
    """Send CoT events via TCP to a TAK Server."""

    def __init__(self, host: str, port: int) -> None:
        self._host = host
        self._port = port
        self._sock: socket.socket | None = None
        self._lock = threading.Lock()

    def _connect(self) -> bool:
        try:
            self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self._sock.settimeout(5.0)
            self._sock.connect((self._host, self._port))
            logger.info("TCP connected to %s:%d", self._host, self._port)
            return True
        except OSError as e:
            logger.warning("TCP connect failed: %s", e)
            self._sock = None
            return False

    def send(self, xml: str) -> None:
        data = xml.encode("utf-8")
        with self._lock:
            if self._sock is None:
                if not self._connect():
                    return
            try:
                self._sock.sendall(data)
            except OSError as e:
                logger.warning("TCP send failed: %s — reconnecting", e)
                self._sock = None

    def close(self) -> None:
        if self._sock:
            self._sock.close()
            self._sock = None


def create_transport(
    mode: str, host: str = "", port: int = 6969, **kwargs
) -> CotTransport:
    """Factory: create a transport from config.

    Args:
        mode: "udp_unicast", "udp_multicast", or "tcp".
        host: Target host (required for unicast and tcp).
        port: Target port.

    Returns:
        CotTransport instance.
    """
    if mode == "udp_unicast":
        return UdpUnicastTransport(host, port)
    elif mode == "udp_multicast":
        group = kwargs.get("group", "239.2.3.1")
        return UdpMulticastTransport(group, port)
    elif mode == "tcp":
        return TcpTransport(host, port)
    else:
        raise ValueError(f"Unknown transport mode: {mode}")
