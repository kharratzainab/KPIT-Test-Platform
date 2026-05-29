#!/usr/bin/env python3
"""
bcm_ws_broadcast.py
===================
Serveur WebSocket isolé — diffusion état BCM vers l'interface HTML 3D.

PORT : 6000 (WebSocket)  —  indépendant du TCP :5000 Platform.

UTILISATION dans bcm_application.py :
    from bcm_ws_broadcast import WSBroadcast
    self._ws = WSBroadcast()
    self._ws.start()
    self._ws.send(rte)   # appeler après chaque changement d'état

Le browser HTML se connecte à :
    ws://<IP_RPi>:6000

FORMAT JSON envoyé :
    {
        "state":         "SPEED1",   // état WSM (string)
        "fault":         false,      // ST_ERROR actif
        "ignition":      1,          // 0=OFF 1=ON/ACC 2=START
        "reverse":       false,      // marche arrière engagée
        "vehicle_speed": 0.0,        // km/h

        "front":         "ON",       // moteur avant ON/OFF
        "front_speed":   1,          // 0=arrêt 1=Speed1 2=Speed2
        "rear":          "OFF",      // moteur arrière ON/OFF

        "pump_active":   false,      // pompe active
        "pump_dir":      0,          // 0=OFF 1=FWD 2=BWD

        "wiper_op":      2,          // crs_wiper_op (0=OFF..7=REAR_WIPE)
        "rain":          0,          // intensité pluie 0-100

        "rest_contact":  true,       // contact repos (True=EN MOUVEMENT)
        "blade_cycles":  0,          // compteur cycles lame avant
        "current":       0.0,        // courant moteur (A)
        "fault_detail": {
            "motor":   false,        // B2001/B2002
            "wiper":   false,        // B2006/B2009
            "pump_oc": false,        // B2003
            "pump_rt": false,        // B2008
            "lin":     false,        // B2004
        }
    }
"""

import json
import queue
import socket
import threading

WS_HOST = "0.0.0.0"
WS_PORT = 6000

# Timeout envoi WebSocket (ms) — évite de bloquer T-WSM
WS_SEND_TIMEOUT = 0.050


class WSBroadcast:
    """
    Serveur WebSocket léger (sans dépendance externe — protocole WebSocket
    implémenté manuellement via HTTP Upgrade + frames RFC 6455).
    - send() est NON-BLOQUANT : dépose dans une queue, T-WS-SEND diffuse.
    - Envoie l'état courant immédiatement à chaque nouveau client.
    """

    def __init__(self, rte=None):
        self._clients      = []
        self._clients_lock = threading.Lock()
        self._running      = False
        self._last_msg     = None
        self._send_queue   = queue.Queue(maxsize=32)
        self._rte_ref      = rte   # reference RTE dès la création

    # ── API publique ──────────────────────────────────────────────────────────

    def start(self):
        self._running = True
        threading.Thread(target=self._accept_loop,  daemon=True, name="T-WS").start()
        threading.Thread(target=self._sender_loop,  daemon=True, name="T-WS-SEND").start()
        threading.Thread(target=self._periodic_loop, daemon=True, name="T-WS-TICK").start()
        print(f"[WS] Serveur WebSocket démarré sur port {WS_PORT}")

    def stop(self):
        self._running = False

    def send(self, rte) -> None:
        """Construit le JSON depuis le RTE et le dépose dans la queue."""
        try:
            from bcm_rte import ST_ERROR
            self._rte_ref = rte   # sauvegarder pour le thread periodique
            payload = self._build_payload(rte)
            msg = json.dumps(payload)
            self._last_msg = msg
            self._send_queue.put_nowait(msg)
        except queue.Full:
            pass   # client lent : message sacrifié, T-WSM non bloqué
        except Exception:
            pass

    def _periodic_loop(self):
        """
        Thread T-WS-TICK : envoie l etat RTE toutes les 50ms.
        Capture tous les changements RTE (ignition, reverse, speed, pump,
        DoIP routines, CAN 0x300, Redis...) independamment de T-WSM.
        """
        import time as _time
        while self._running:
            _time.sleep(0.05)   # 50ms — reactif aux commandes DoIP
            rte = self._rte_ref
            if rte is None:
                continue
            try:
                payload = self._build_payload(rte)
                msg = json.dumps(payload)
                # Envoyer seulement si l etat a change
                if msg != self._last_msg:
                    self._last_msg = msg
                    self._send_queue.put_nowait(msg)
            except queue.Full:
                pass
            except Exception:
                pass

    # ── Construction du payload ───────────────────────────────────────────────

    def _build_payload(self, rte) -> dict:
        from bcm_rte import ST_ERROR
        state = rte.state
        return {
            # ── WSM ───────────────────────────────────────────────────────────
            "state":         state,
            "fault":         state == ST_ERROR,

            # ── Véhicule ──────────────────────────────────────────────────────
            "ignition":      int(getattr(rte, "ignition_status",  0)),
            "reverse":       bool(getattr(rte, "reverse_gear",    False)),
            "vehicle_speed": round(float(getattr(rte, "vehicle_speed", 0.0)), 1),

            # ── Moteur avant ──────────────────────────────────────────────────
            "front":         "ON"  if rte.front_motor_on else "OFF",
            "front_speed":   int(getattr(rte, "front_motor_speed", 0)),

            # ── Moteur arrière ────────────────────────────────────────────────
            "rear":          "ON"  if rte.rear_motor_on  else "OFF",

            # ── Pompe ─────────────────────────────────────────────────────────
            "pump_active":   bool(getattr(rte, "pump_active",    False)),
            "pump_dir":      int(getattr(rte, "pump_dir_active", 0)),

            # ── Mode essuie-glace ─────────────────────────────────────────────
            # 0=OFF 1=TOUCH 2=SPEED1 3=SPEED2 4=AUTO
            # 5=WASH_FRONT 6=WASH_REAR 7=REAR_WIPE
            "wiper_op":      int(getattr(rte, "crs_wiper_op", 0)),

            # ── Pluie ─────────────────────────────────────────────────────────
            "rain":          int(getattr(rte, "rain_intensity", 0)),

            # ── Contact repos / lame ──────────────────────────────────────────
            "rest_contact":  bool(getattr(rte, "rest_contact_raw",   False)),
            "blade_cycles":  int(getattr(rte, "front_blade_cycles",  0)),
            "current":       round(float(rte.motor_current_a), 2),

            # ── Détail défauts ────────────────────────────────────────────────
            "fault_detail": {
                "motor":   bool(getattr(rte, "front_motor_error", False) or
                                getattr(rte, "rear_motor_error",  False)),
                "wiper":   bool(getattr(rte, "wiper_fault",            False)),
                "pump_oc": bool(getattr(rte, "pump_overcurrent_error", False)),
                "pump_rt": bool(getattr(rte, "pump_runtime_error",     False)),
                "lin":     bool(getattr(rte, "lin_timeout_active",     False)),
            },

            # ── Diagnostic (ST_DIAG) ──────────────────────────────────────────
            # Durée totale de la routine en cours (secondes), 0 si pas en DIAG
            "diag_duration": float(getattr(rte, "_test_duration",  0)) if getattr(rte, "_test_active", False) else 0.0,
            # Temps écoulé depuis le démarrage de la routine (secondes)
            "diag_elapsed":  float(__import__('time').time() - getattr(rte, "_t_test_start", __import__('time').time())) if getattr(rte, "_test_active", False) else 0.0,
        }

    # ── Thread envoi ──────────────────────────────────────────────────────────

    def _sender_loop(self):
        while self._running:
            try:
                msg = self._send_queue.get(timeout=0.2)
                self._broadcast(msg)
            except queue.Empty:
                continue

    def _broadcast(self, msg: str):
        frame = self._ws_frame(msg)
        dead = []
        with self._clients_lock:
            for c in list(self._clients):
                try:
                    c.settimeout(WS_SEND_TIMEOUT)
                    c.sendall(frame)
                except Exception:
                    dead.append(c)
            for c in dead:
                if c in self._clients:
                    self._clients.remove(c)

    # ── Thread acceptation ────────────────────────────────────────────────────

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
                srv.bind((WS_HOST, WS_PORT))
                break
            except OSError:
                print(f"[WS] Port {WS_PORT} occupé, attente 1s ({attempt+1}/10)...")
                _time.sleep(1.0)
        else:
            print(f"[WS] ERREUR : port {WS_PORT} toujours occupé — WebSocket désactivé")
            return
        srv.listen(10)
        srv.settimeout(1.0)
        while self._running:
            try:
                conn, addr = srv.accept()
                threading.Thread(
                    target=self._handle_client,
                    args=(conn, addr),
                    daemon=True
                ).start()
            except socket.timeout:
                continue
            except Exception as e:
                print(f"[WS] Erreur accept: {e}")
        srv.close()

    # ── Handshake WebSocket RFC 6455 ──────────────────────────────────────────

    def _handle_client(self, conn, addr):
        try:
            # Lire la requête HTTP Upgrade
            raw = b""
            while b"\r\n\r\n" not in raw:
                chunk = conn.recv(1024)
                if not chunk:
                    return
                raw += chunk
            headers = self._parse_headers(raw.decode("utf-8", errors="replace"))
            key = headers.get("sec-websocket-key", "")
            if not key:
                conn.close()
                return

            # Réponse Upgrade
            import base64, hashlib
            magic = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"
            accept = base64.b64encode(
                hashlib.sha1((key + magic).encode()).digest()
            ).decode()
            response = (
                "HTTP/1.1 101 Switching Protocols\r\n"
                "Upgrade: websocket\r\n"
                "Connection: Upgrade\r\n"
                f"Sec-WebSocket-Accept: {accept}\r\n\r\n"
            )
            conn.sendall(response.encode())
            print(f"[WS] Client connecté: {addr}")

            # Enregistrer le client
            with self._clients_lock:
                self._clients.append(conn)

            # Envoyer l'état courant immédiatement
            if self._last_msg:
                try:
                    conn.sendall(self._ws_frame(self._last_msg))
                except Exception:
                    pass

            # Boucle de lecture (keep-alive + détection déconnexion)
            conn.settimeout(30.0)
            while self._running:
                try:
                    data = conn.recv(256)
                    if not data:
                        break
                    # Répondre aux pings WebSocket (opcode 0x9)
                    if data and (data[0] & 0x0F) == 0x9:
                        pong = bytes([0x8A, 0x00])
                        conn.sendall(pong)
                except socket.timeout:
                    continue
                except Exception:
                    break

        except Exception as e:
            print(f"[WS] Erreur client {addr}: {e}")
        finally:
            with self._clients_lock:
                if conn in self._clients:
                    self._clients.remove(conn)
            try:
                conn.close()
            except Exception:
                pass
            print(f"[WS] Client déconnecté: {addr}")

    # ── Utilitaires WebSocket ─────────────────────────────────────────────────

    @staticmethod
    def _ws_frame(msg: str) -> bytes:
        """Encode un message texte en frame WebSocket RFC 6455."""
        data = msg.encode("utf-8")
        length = len(data)
        if length <= 125:
            header = bytes([0x81, length])
        elif length <= 65535:
            header = bytes([0x81, 126]) + length.to_bytes(2, "big")
        else:
            header = bytes([0x81, 127]) + length.to_bytes(8, "big")
        return header + data

    @staticmethod
    def _parse_headers(raw: str) -> dict:
        headers = {}
        for line in raw.split("\r\n")[1:]:
            if ":" in line:
                k, v = line.split(":", 1)
                headers[k.strip().lower()] = v.strip()
        return headers