#!/usr/bin/env python3
"""
bcm_tcp_pump.py
===============
Serveur TCP dédié pompe -- port 5000
Diffuse l'état de la pompe vers l'interface WipeWash Pump Monitor (PyQt6).

FORMAT JSON envoyé :
{
    "state":          "FORWARD",   // FORWARD / BACKWARD / OFF / FAULT / OVERCURRENT
    "current":        0.35,        // courant ADS1115 (amperes)
    "voltage":        0.0,         // pas de capteur tension → toujours 0.0
    "fault":          false,       // ST_ERROR actif
    "fault_reason":   "",          // OVERCURRENT / ""
    "pump_remaining": 3.2,         // secondes restantes (PUMP_MAX_RUNTIME - elapsed)
    "pump_duration":  5.0,         // PUMP_MAX_RUNTIME
    "source":         "BCM"        // toujours BCM
}

UTILISATION dans bcm_application.py :
    from bcm_tcp_pump import TCPPumpBroadcast
    self._tcp_pump = TCPPumpBroadcast()
    self._tcp_pump.start()
    self._tcp_pump.send(rte)       # appeler apres chaque changement d'etat pompe
"""

import json
import queue
import socket
import threading
import time

from bcm_rte import ST_ERROR, PUMP_MAX_RUNTIME, PUMP_OVERCURRENT_THRESH

TCP_PUMP_HOST = "0.0.0.0"
TCP_PUMP_PORT = 5556

# Identique à bcm_tcp_broadcast : timeout send pour ne jamais bloquer T-WSM/T-PUMP
TCP_SEND_TIMEOUT = 0.050   # 50ms max par client


class TCPPumpBroadcast:
    """
    Serveur TCP léger sur port 5000.
    Envoie l'état pompe au format attendu par WipeWash Pump Monitor.
    Lecture seule du RTE -- ne modifie rien.
    """

    def __init__(self):
        self._clients      = []
        self._clients_lock = threading.Lock()
        self._running      = False
        self._last_msg     = None   # dernier JSON envoye -- pour les nouveaux clients
        # [VARIABLE LOAD] Queue agrandie pour absorber le flux continu
        # (lecture toutes les 10ms = 100 msg/s -> 200 slots = 2s de buffer)
        # Avant : maxsize=32 -> saturation en 320ms -> messages sacrifies -> 1 seule valeur affichee
        self._send_queue   = queue.Queue(maxsize=200)

    def start(self):
        self._running = True
        threading.Thread(
            target=self._accept_loop,
            daemon=True,
            name="T-TCP-PUMP"
        ).start()
        threading.Thread(
            target=self._sender_loop,
            daemon=True,
            name="T-TCP-PUMP-SEND"
        ).start()
        print(f"[TCP-PUMP] Serveur demarre sur port {TCP_PUMP_PORT}")

    def stop(self):
        self._running = False

    def send(self, rte) -> None:
        """
        Construit le JSON pompe et le dépose dans la queue d'envoi.
        RETOUR IMMÉDIAT : ne bloque jamais T-WSM ni T-PUMP.
        """
        direction = rte.pump_direction
        if rte.pump_active:
            pump_state = "FORWARD" if direction == 1 else "BACKWARD"
        else:
            pump_state = "OFF"

        is_overcurrent = rte.motor_current_a > PUMP_OVERCURRENT_THRESH and rte.pump_active

        # ST_ERROR est une faute pompe uniquement si pump_error=True.
        # Si c'est une erreur moteur/lame (wiper_fault), la pompe continue normalement
        # → ne pas signaler FAULT côté pompe.
        pump_is_error = (rte.state == ST_ERROR) and rte.pump_error
        fault         = pump_is_error or is_overcurrent
        fault_reason  = "OVERCURRENT" if is_overcurrent else ("PUMP_ERROR" if pump_is_error else "")
        if fault and pump_state not in ("FORWARD", "BACKWARD"):
            pump_state = "FAULT"

        if rte.pump_active and rte.t_pump_start > 0:
            elapsed         = time.time() - rte.t_pump_start
            pump_remaining  = round(max(0.0, PUMP_MAX_RUNTIME - elapsed), 1)
        else:
            pump_remaining  = 0.0

        # [VARIABLE LOAD] 5 decimales pour capturer les petites variations de charge
        # Avant : round(..., 3/2/4) -> variations < 1mA invisibles -> 1 seule valeur affichee
        payload = {
            "state":          pump_state,
            "current":        round(rte.pump_current_a, 5),
            "voltage":        round(rte.pump_voltage_v, 5),
            "v_b":            round(rte.pump_v_b,       5),
            "v_a":            round(rte.pump_v_a,       5),
            "fault":          fault,
            "fault_reason":   fault_reason,
            "pump_remaining": pump_remaining,
            "pump_duration":  PUMP_MAX_RUNTIME,
            "source":         "BCM",
        }

        msg = (json.dumps(payload) + "\n").encode()
        self._last_msg = msg
        try:
            self._send_queue.put_nowait(msg)
        except queue.Full:
            pass   # client lent : message sacrifié, threads non bloqués

    # ─────────────────────────────────────────────────
    # Interne
    # ─────────────────────────────────────────────────

    def _sender_loop(self):
        """Thread T-TCP-PUMP-SEND : diffuse les messages sans bloquer T-WSM/T-PUMP."""
        while self._running:
            try:
                msg = self._send_queue.get(timeout=0.2)
                self._broadcast(msg)
            except queue.Empty:
                continue

    def _accept_loop(self):
        import time as _time
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
        except AttributeError:
            pass
        for attempt in range(10):
            try:
                srv.bind((TCP_PUMP_HOST, TCP_PUMP_PORT))
                break
            except OSError:
                print(f"[TCP-PUMP] Port {TCP_PUMP_PORT} occupe, attente 1s (tentative {attempt+1}/10)...")
                _time.sleep(1.0)
        else:
            print(f"[TCP-PUMP] ERREUR : port {TCP_PUMP_PORT} toujours occupe -- TCP-PUMP desactive")
            return
        srv.listen(5)
        srv.settimeout(1.0)
        while self._running:
            try:
                conn, addr = srv.accept()
                print(f"[TCP-PUMP] Client connecte : {addr}")
                with self._clients_lock:
                    self._clients.append(conn)
                threading.Thread(
                    target=self._watch_disconnect,
                    args=(conn, addr),
                    daemon=True
                ).start()
            except socket.timeout:
                continue
            except Exception as e:
                print(f"[TCP-PUMP] Erreur accept : {e}")
        srv.close()

    def _watch_disconnect(self, conn, addr):
        # Envoie immediatement l'etat courant au nouveau client
        if self._last_msg:
            try:
                conn.sendall(self._last_msg)
            except Exception:
                pass
        try:
            while self._running:
                data = conn.recv(64)
                if not data:
                    break
        except Exception:
            pass
        finally:
            with self._clients_lock:
                if conn in self._clients:
                    self._clients.remove(conn)
            try:
                conn.close()
            except Exception:
                pass
            print(f"[TCP-PUMP] Client deconnecte : {addr}")

    def _broadcast(self, msg: bytes):
        dead = []
        with self._clients_lock:
            for c in self._clients:
                try:
                    c.settimeout(TCP_SEND_TIMEOUT)
                    c.sendall(msg)
                except Exception:
                    dead.append(c)
            for c in dead:
                self._clients.remove(c)