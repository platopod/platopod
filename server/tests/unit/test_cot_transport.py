"""Tests for plato_pod.cot_transport — CoT network transport."""

from __future__ import annotations

from unittest.mock import MagicMock, patch

from plato_pod.cot_transport import (
    TcpTransport,
    UdpMulticastTransport,
    UdpUnicastTransport,
    create_transport,
)


class TestUdpUnicastTransport:
    @patch("plato_pod.cot_transport.socket.socket")
    def test_send_calls_sendto(self, mock_socket_cls) -> None:
        mock_sock = MagicMock()
        mock_socket_cls.return_value = mock_sock

        t = UdpUnicastTransport("192.168.1.100", 6969)
        t._sock = mock_sock
        t.send("<event/>")

        mock_sock.sendto.assert_called_once()
        args = mock_sock.sendto.call_args
        assert args[0][0] == b"<event/>"
        assert args[0][1] == ("192.168.1.100", 6969)


class TestUdpMulticastTransport:
    @patch("plato_pod.cot_transport.socket.socket")
    def test_send_calls_sendto(self, mock_socket_cls) -> None:
        mock_sock = MagicMock()
        mock_socket_cls.return_value = mock_sock

        t = UdpMulticastTransport("239.2.3.1", 6969)
        t._sock = mock_sock
        t.send("<event/>")

        mock_sock.sendto.assert_called_once()
        args = mock_sock.sendto.call_args
        assert args[0][1] == ("239.2.3.1", 6969)

    @patch("plato_pod.cot_transport.socket.socket")
    def test_sets_multicast_ttl(self, mock_socket_cls) -> None:
        mock_sock = MagicMock()
        mock_socket_cls.return_value = mock_sock
        UdpMulticastTransport("239.2.3.1", 6969, ttl=16)
        mock_sock.setsockopt.assert_called()


class TestTcpTransport:
    @patch("plato_pod.cot_transport.socket.socket")
    def test_send_connects_and_sends(self, mock_socket_cls) -> None:
        mock_sock = MagicMock()
        mock_socket_cls.return_value = mock_sock

        t = TcpTransport("tak-server", 8087)
        t.send("<event/>")

        mock_sock.connect.assert_called_once_with(("tak-server", 8087))
        mock_sock.sendall.assert_called_once_with(b"<event/>")


class TestCreateTransport:
    def test_udp_unicast(self) -> None:
        t = create_transport("udp_unicast", host="192.168.1.1", port=6969)
        assert isinstance(t, UdpUnicastTransport)
        t.close()

    def test_udp_multicast(self) -> None:
        t = create_transport("udp_multicast", port=6969)
        assert isinstance(t, UdpMulticastTransport)
        t.close()

    def test_tcp(self) -> None:
        t = create_transport("tcp", host="tak-server", port=8087)
        assert isinstance(t, TcpTransport)
        t.close()

    def test_unknown_raises(self) -> None:
        import pytest
        with pytest.raises(ValueError):
            create_transport("carrier_pigeon")
