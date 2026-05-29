#!/usr/bin/env python3
"""
xcp_slave.py  —  XCP Slave côté BCM (RPi)
==========================================
Transport : Redis pub/sub.

Commandes supportées :
  SHORT_UPLOAD  → lit la valeur courante d'un paramètre
  DOWNLOAD      → écrit une valeur dans bcm_rte (RAM + disque)
  GET_A2L       → retourne le descripteur complet des paramètres
  GET_STATUS    → retourne les valeurs courantes de tous les paramètres

Canaux Redis :
  xcp_cmd   ← commandes reçues du master
  xcp_resp  → réponses envoyées au master
"""

from __future__ import annotations

import json
import os
import re
import tempfile
import time
import threading
from typing import Any

from a2l_loader import load_a2l

# ── Chargement A2L ─────────────────────────────────────────────────

def _resolve_a2l_path() -> str:
    env = os.environ.get("XCP_A2L_PATH", "").strip()
    if env:
        return env
    here = os.path.dirname(os.path.abspath(__file__))
    candidate = os.path.join(here, "wiperwash_xcp.a2l")
    if os.path.isfile(candidate):
        return candidate
    return "wiperwash_xcp.a2l"

_A2L_PATH: str            = _resolve_a2l_path()
A2L:       dict[str, dict] = load_a2l(_A2L_PATH)

# Chemin de bcm_rte.py sur le disque (pour persistance)
_BCM_RTE_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "bcm_rte.py")

# Canaux Redis
_CH_CMD  = "xcp_cmd"
_CH_RESP = "xcp_resp"
_RETRY_S = 3


class XCPSlave:
    """
    Thread XCP slave — à lancer depuis bcm_main.py comme T-XCP.

    Usage :
        slave = XCPSlave("127.0.0.1", 6379)
        t = threading.Thread(target=slave.run, daemon=True, name="T-XCP")
        t.start()
    """

    VERSION = "2.0.0"

    def __init__(self, redis_host: str = "127.0.0.1", redis_port: int = 6379, rte=None):
        self._host  = redis_host
        self._port  = redis_port
        self._r_pub = None
        self._r_sub = None
        self._lock  = threading.Lock()
        self._rte   = rte  # peut être None — uniquement pour nettoyer un verrou résiduel

        import bcm_rte as _rte_mod
        self._rte_mod = _rte_mod

    # ── Connexion Redis ────────────────────────────────────────────

    def _connect(self) -> bool:
        try:
            import redis as _r
            self._r_pub = _r.Redis(
                host=self._host, port=self._port, db=0,
                socket_connect_timeout=2, socket_timeout=2,
            )
            self._r_sub = _r.Redis(
                host=self._host, port=self._port, db=0,
                socket_connect_timeout=2, socket_timeout=30,
            )
            self._r_pub.ping()
            print(f"[XCP-SLAVE] Connecté sur {self._host}:{self._port}")
            return True
        except Exception as e:
            print(f"[XCP-SLAVE] Connexion impossible: {e}")
            return False

    # ── Boucle principale ──────────────────────────────────────────

    def run(self):
        print("[T-XCP] Démarré | écoute canal 'xcp_cmd'")
        # Libérer tout verrou RTE résiduel d'une session XCP précédente.
        # L'ancien slave acquérait le verrou RTE au CONNECT — si la session
        # s'est terminée sans DISCONNECT propre, le verrou restait bloqué 30s.
        # Le nouveau slave n'utilise plus le verrou RTE : on le libère une fois
        # au démarrage pour repartir dans un état propre.
        if self._rte is not None:
            try:
                self._rte._write_lock_owner = None
                print("[T-XCP] Verrou RTE résiduel nettoyé au démarrage")
            except Exception:
                pass
        while True:
            if not self._connect():
                time.sleep(_RETRY_S)
                continue
            ps = None
            try:
                ps = self._r_sub.pubsub(ignore_subscribe_messages=True)
                ps.subscribe(_CH_CMD)
                print("[T-XCP] Abonné sur 'xcp_cmd'")
                while True:
                    msg = ps.get_message(ignore_subscribe_messages=True, timeout=1.0)
                    if msg is None:
                        continue
                    if msg.get("type") != "message":
                        continue
                    try:
                        cmd = json.loads(msg["data"])
                        self._dispatch(cmd)
                    except Exception as e:
                        print(f"[T-XCP] Erreur dispatch: {e}")
            except Exception as e:
                print(f"[T-XCP] Erreur écoute: {e} — retry {_RETRY_S}s")
            finally:
                if ps:
                    try:
                        ps.unsubscribe()
                        ps.close()
                    except Exception:
                        pass
            time.sleep(_RETRY_S)

    # ── Dispatch ───────────────────────────────────────────────────

    def _dispatch(self, cmd: dict):
        handlers = {
            "SHORT_UPLOAD": self._cmd_short_upload,
            "DOWNLOAD":     self._cmd_download,
            "GET_A2L":      self._cmd_get_a2l,
            "GET_STATUS":   self._cmd_get_status,
        }
        xcp_cmd = cmd.get("cmd", "").upper()
        client  = cmd.get("client", "unknown")
        handler = handlers.get(xcp_cmd)
        if handler is None:
            self._respond(xcp_cmd, client, cmd,
                          status="ERR", error=f"Commande inconnue: {xcp_cmd}")
            return
        handler(cmd)

    # ── Handlers ───────────────────────────────────────────────────

    def _cmd_short_upload(self, cmd: dict):
        """SHORT_UPLOAD — lit la valeur courante d'un paramètre."""
        client = cmd.get("client", "unknown")
        key    = cmd.get("key")
        if key not in A2L:
            self._respond("SHORT_UPLOAD", client, cmd, status="ERR",
                          error=f"Paramètre inconnu: {key}")
            return
        val  = getattr(self._rte_mod, key, A2L[key]["default"])
        meta = A2L[key]
        self._respond("SHORT_UPLOAD", client, cmd, status="OK", data={
            "key":     key,
            "value":   val,
            "unit":    meta["unit"],
            "default": meta["default"],
            "min":     meta["min"],
            "max":     meta["max"],
        })

    def _cmd_download(self, cmd: dict):
        """
        DOWNLOAD — écrit une valeur dans bcm_rte (RAM + disque).
        Effet immédiat au prochain cycle T-WSM (50ms).
        Survit au redémarrage grâce à la persistance disque.
        """
        client = cmd.get("client", "unknown")
        key    = cmd.get("key")
        value  = cmd.get("value")

        if key not in A2L:
            self._respond("DOWNLOAD", client, cmd, status="ERR",
                          error=f"Paramètre inconnu: {key}")
            return

        meta = A2L[key]

        # Parse et validation bornes
        try:
            typed = float(value) if meta["type"] == "float" else int(float(value))
        except (TypeError, ValueError):
            self._respond("DOWNLOAD", client, cmd, status="ERR",
                          error=f"Valeur invalide: {value}")
            return

        if not (meta["min"] <= typed <= meta["max"]):
            self._respond("DOWNLOAD", client, cmd, status="ERR",
                          error=f"{key}={typed} hors bornes [{meta['min']}..{meta['max']}]")
            return

        # Écriture RAM
        with self._lock:
            old = getattr(self._rte_mod, key, meta["default"])
            setattr(self._rte_mod, key, typed)

        # Écriture disque
        self._persist_to_file(key, typed, meta)

        print(f"[T-XCP] DOWNLOAD {key}: {old} → {typed} (client='{client}')")

        self._respond("DOWNLOAD", client, cmd, status="OK", data={
            "key":       key,
            "old_value": old,
            "new_value": typed,
            "unit":      meta["unit"],
        })

    def _cmd_get_a2l(self, cmd: dict):
        """GET_A2L — retourne le descripteur complet des paramètres."""
        client = cmd.get("client", "unknown")
        self._respond("GET_A2L", client, cmd, status="OK", data={"a2l": A2L})

    def _cmd_get_status(self, cmd: dict):
        """GET_STATUS — retourne les valeurs courantes de tous les paramètres."""
        client  = cmd.get("client", "unknown")
        current = {key: getattr(self._rte_mod, key, meta["default"])
                   for key, meta in A2L.items()}
        self._respond("GET_STATUS", client, cmd, status="OK", data={
            "slave_version":  self.VERSION,
            "current_values": current,
        })

    # ── Persistance disque ─────────────────────────────────────────

    def _persist_to_file(self, key: str, value, meta: dict):
        """
        Réécrit la ligne KEY = <valeur> dans bcm_rte.py sur le disque.
        Écriture atomique via tempfile + os.replace.
        """
        try:
            new_val_str = repr(float(value)) if meta["type"] == "float" else str(int(value))

            with open(_BCM_RTE_PATH, 'r', encoding='utf-8') as f:
                content = f.read()

            pattern = r'^(' + re.escape(key) + r'\s*=\s*)[\d.eE+\-]+(\s*(#.*)?)$'
            new_content, n = re.subn(
                pattern,
                lambda m, v=new_val_str: m.group(1) + v + (
                    ("  " + m.group(3)) if m.group(3) else ""
                ),
                content,
                flags=re.MULTILINE,
            )

            if n == 0:
                print(f"[T-XCP] WARN: pattern non trouvé pour {key} dans bcm_rte.py")
                return

            dir_ = os.path.dirname(_BCM_RTE_PATH)
            with tempfile.NamedTemporaryFile('w', dir=dir_, delete=False,
                                             encoding='utf-8', suffix='.tmp') as tmp:
                tmp.write(new_content)
                tmp_path = tmp.name
            os.replace(tmp_path, _BCM_RTE_PATH)
            print(f"[T-XCP] Persisté {key} = {new_val_str} dans bcm_rte.py")

        except Exception as e:
            print(f"[T-XCP] WARN: persistance disque échouée pour {key}: {e}")

    # ── Réponse ────────────────────────────────────────────────────

    def _respond(self, cmd: str, client: str, cmd_dict: dict,
                 status: str, data: dict = None, error: str = None):
        payload = {
            "cmd":    cmd,
            "client": client,
            "status": status,
            "ts":     time.time(),
            "req_id": cmd_dict.get("req_id", ""),
        }
        if data:
            payload["data"] = data
        if error:
            payload["error"] = error
        try:
            self._r_pub.publish(_CH_RESP, json.dumps(payload))
        except Exception as e:
            print(f"[T-XCP] Erreur réponse: {e}")