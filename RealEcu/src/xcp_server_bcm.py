#!/usr/bin/env python3
"""
xcp_server_bcm.py  —  XCP-on-UDP Server côté BCM
==================================================

Rôle
----
Sert de pont entre l'interface de test (doip-uds-simulator-HIL / XCPClient)
et le moteur DTC du BCM réel (dtc_manager.py / bcm_rte.py).

Flux :
  [HIL XCPClient]  →  UDP SHORT_DOWNLOAD  →  [ce serveur]
                                                    │
                                              memory.json  (persistence)
                                                    │
                                     écriture dans RTE (xcp_front/rear/pump_current_a)
                                                    │
                              bcm_application._check_overcurrent() [OR courant_réel + courant_XCP]
                                                    │
                                           dtc_manager.set_active / set_inactive
                                                    │
                                           dtc_database.json  (résultat visible UDS 0x19)

Protocole XCP-on-UDP (ASAM MCD-1 XCP, identique au XCPClient HIL)
------------------------------------------------------------------
  CONNECT        [0xFF, 0x00]
  DISCONNECT     [0xFE, 0x00]
  STATUS         [0xFD, 0x00]
  SHORT_DOWNLOAD [0xED, size, 0x00, 0x00, addr(4B LE), data(size B)]

  Réponses : [0xFF] = OK  |  [0xFE] = ERR

Adresses mémoire & conditions DTC (tirées de xcp_variables.json)
-----------------------------------------------------------------
  0x20000000  front_motor_current_mA  uint16  → B2001 si > 800 mA pendant 300 ms  [garde: front_motor_on]
  0x20000002  rear_motor_current_mA   uint16  → B2002 si > 800 mA pendant 300 ms  [garde: rear_motor_on]
  0x20000004  pump_current_mA         uint16  → B2003 si > 800 mA pendant 300 ms  [garde: pump_active]
  0x20000006  lin_frame_valid         uint8   → B2004 si == 0 pendant 100 ms
  0x20000007  can_wiper_status_valid  uint8   → B2005 si == 0 pendant 100 ms
  0x20000008  blade_movement_detected uint8   → B2006 si == 0 (immédiat)
  0x20000009  rain_sensor_raw         uint16  → B2007 si > 254 (latch, garde: mode AUTO)
  0x2000000A  pump_runtime_s          uint8   → B2008 si > 5 (immédiat)
  0x2000000B  rest_contact_signal     uint8   → B2009 si signal figé 500 ms

Conditions de healing (même logique que bcm_application.py)
------------------------------------------------------------
  Chaque DTC repasse INACTIVE dès que la condition disparaît, sauf B2009
  qui nécessite que le signal change au moins une fois pendant HEAL_DELAY.

Seuils matériels BCM réels
--------------------------
  - Surcourant moteur avant / arrière / pompe : 0.8 A (800 mA) pendant 300 ms
  - Capteur pluie : défaut si raw > 254 (saturation ADC)

Intégration
-----------
  Lancer depuis bcm_main.py comme thread démon AVANT les autres threads :

      from xcp_server_bcm import XCPServerBCM
      xcp_srv = XCPServerBCM(dtc_manager=dtc_mgr, rte=rte)
      t_xcp = threading.Thread(target=xcp_srv.run, daemon=True, name="T-XCP-SRV")
      t_xcp.start()

  Ou standalone pour les tests :

      python3 xcp_server_bcm.py
"""

from __future__ import annotations

import json
import os
import socket
import struct
import threading
import time
import logging
from typing import Optional

# ─────────────────────────────────────────────────────────────────────────────
# Configuration
# ─────────────────────────────────────────────────────────────────────────────
HOST         = "0.0.0.0"
PORT         = 17725          # même port que XCPClient HIL (xcp_variables.json)
MEMORY_FILE  = os.path.join(os.path.dirname(os.path.abspath(__file__)), "memory.json")

# ─────────────────────────────────────────────────────────────────────────────
# Commandes / réponses XCP  (identiques à XCPClient HIL)
# ─────────────────────────────────────────────────────────────────────────────
CMD_CONNECT        = 0xFF
CMD_DISCONNECT     = 0xFE
CMD_STATUS         = 0xFD
CMD_SHORT_DOWNLOAD = 0xED

RES_OK  = bytes([0xFF])
RES_ERR = bytes([0xFE])

# ─────────────────────────────────────────────────────────────────────────────
# Adresses mémoire → variables BCM
# (tirées de xcp_variables.json section "bcm")
# ─────────────────────────────────────────────────────────────────────────────
ADDR_FRONT_MOTOR_CURRENT  = 0x20000000   # uint16  mA
ADDR_REAR_MOTOR_CURRENT   = 0x20000002   # uint16  mA
ADDR_PUMP_CURRENT         = 0x20000004   # uint16  mA
ADDR_LIN_FRAME_VALID      = 0x20000006   # uint8   bool
ADDR_CAN_WIPER_VALID      = 0x20000007   # uint8   bool
ADDR_BLADE_MOVEMENT       = 0x20000008   # uint8   bool
ADDR_RAIN_SENSOR_RAW      = 0x20000009   # uint16  raw ADC  (NOTE: size=2 dans xcp_variables)
ADDR_PUMP_RUNTIME         = 0x2000000A   # uint8   secondes
ADDR_REST_CONTACT         = 0x2000000B   # uint8   bool

# ─────────────────────────────────────────────────────────────────────────────
# Seuils & durées  (identiques aux conditions de xcp_variables.json
#                   et à la logique de bcm_application.py / bcm_rte.py)
# ─────────────────────────────────────────────────────────────────────────────
OVERCURRENT_FRONT_THRESH   = 800    # mA   → B2001  (seuil réel BCM : 0.8 A)
OVERCURRENT_REAR_THRESH    = 800    # mA   → B2002  (seuil réel BCM : 0.8 A)
OVERCURRENT_FRONT_DELAY    = 0.300  # s    (OVERCURRENT_DELAY bcm_rte)
OVERCURRENT_REAR_DELAY     = 0.300  # s

PUMP_OVERCURRENT_THRESH    = 800    # mA   → B2003  (seuil réel BCM : 0.8 A)
PUMP_OVERCURRENT_DELAY     = 0.300  # s    → B2003 avec temporisation 300 ms

LIN_TIMEOUT_DELAY          = 0.100  # s    → B2004
CAN_TIMEOUT_DELAY          = 0.100  # s    → B2005

# B2006 : immédiat dès blade_movement_detected == 0
RAIN_VALID_MAX             = 254    # raw  → B2007 si > 254 (saturation ADC 8-bit)

PUMP_MAX_RUNTIME           = 5.0    # s    → B2008

REST_STUCK_DELAY           = 0.500  # s    → B2009 (signal figé)
HEAL_DELAY                 = 1.0    # s    healing B2009 (signal change à nouveau)

# ─────────────────────────────────────────────────────────────────────────────
# Logging
# ─────────────────────────────────────────────────────────────────────────────
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [XCP-SRV] %(message)s",
    datefmt="%H:%M:%S",
)
logger = logging.getLogger("xcp_server_bcm")


# ═════════════════════════════════════════════════════════════════════════════
# Helpers memory.json
# ═════════════════════════════════════════════════════════════════════════════
def _load_memory() -> dict:
    if not os.path.exists(MEMORY_FILE):
        return {}
    try:
        with open(MEMORY_FILE, "r") as f:
            return json.load(f)
    except Exception:
        return {}


def _save_memory(mem: dict) -> None:
    try:
        with open(MEMORY_FILE, "w") as f:
            json.dump(mem, f, indent=2)
    except Exception as e:
        logger.warning(f"memory.json write error: {e}")


def _mem_get(mem: dict, addr: int, default: int = 0) -> int:
    return mem.get(f"0x{addr:08X}", default)


def _mem_set(mem: dict, addr: int, value: int) -> None:
    mem[f"0x{addr:08X}"] = value


# ═════════════════════════════════════════════════════════════════════════════
# Moteur DTC — évalue les conditions et pilote dtc_manager
# ═════════════════════════════════════════════════════════════════════════════
class _DTCEngineXCP:
    """
    Évalue en continu les valeurs de memory.json.

    Pour B2001 / B2002 / B2003 (overcurrent moteur/pompe) :
      - N'appelle PLUS dtc_manager directement.
      - Écrit le courant injecté dans le RTE (xcp_front/rear/pump_current_a).
      - bcm_application._check_overcurrent() / _check_pump_overcurrent()
        déclenche le DTC avec la condition OR (courant_réel | courant_XCP)
        ET toutes les gardes BCM (moteur en marche, délai 300ms, ST_ERROR).

    Pour B2004 / B2005 / B2006 / B2008 / B2009 :
      - Logique inchangée : dtc_manager piloté directement depuis ce thread.

    Tournant dans son propre thread (T-XCP-DTC, période 50 ms).
    """

    POLL_PERIOD = 0.050   # 50 ms — cohérent avec PUMP_GUARD_PERIOD BCM

    def __init__(self, dtc_manager, rte, mem_lock: threading.Lock):
        self._dtc      = dtc_manager
        self._rte      = rte          # peut être None en mode standalone
        self._lock     = mem_lock
        self._running  = False

        # ── Timers de déclenchement (onset timers) ────────────────────
        self._t_front_oc  : float = 0.0   # B2001 overcurrent front
        self._t_rear_oc   : float = 0.0   # B2002 overcurrent rear
        self._t_pump_oc   : float = 0.0   # B2003 overcurrent pompe
        self._t_lin       : float = 0.0   # B2004 LIN timeout
        self._t_can       : float = 0.0   # B2005 CAN timeout

        # ── Timer B2009 — signal figé ─────────────────────────────────
        self._rest_prev   : Optional[int] = None
        self._t_rest_stuck: float = 0.0
        self._t_rest_heal : float = 0.0   # healing timer B2009

        # ── Latches — un seul déclenchement par injection XCP ─────────
        # Le latch passe True quand ce moteur XCP déclenche le DTC.
        # Il ne repasse False que lorsque la condition physique disparaît
        # (courant/signal sous le seuil, moteur arrêté, etc.).
        # Cela évite la compétition avec bcm_application._check_overcurrent
        # qui peut mettre le DTC INACTIVE via son propre cycle HEAL, puis
        # le XCP-SRV le re-déclencherait indéfiniment sur la même valeur.
        self._latch_b2001: bool = False
        self._latch_b2002: bool = False
        self._latch_b2003: bool = False
        self._latch_b2006: bool = False
        self._latch_b2007: bool = False
        self._latch_b2009: bool = False
        # B2008 : latch + derniere valeur vue pour ne declencher qu une seule
        # fois par valeur distincte de pump_runtime_s dans memory.json.
        # Reset uniquement quand pump_runtime_s revient a 0 (plateforme efface).
        self._latch_b2008: bool = False
        self._last_pump_runtime: int = 0   # derniere valeur non-nulle traitee

    # ──────────────────────────────────────────────────────────────────
    # Boucle principale
    # ──────────────────────────────────────────────────────────────────
    def run(self):
        logger.info("T-XCP-DTC démarré (période=50ms)")
        self._running = True
        while self._running:
            try:
                with self._lock:
                    mem = _load_memory()
                self._evaluate(mem)
            except Exception as e:
                logger.error(f"T-XCP-DTC erreur: {e}")
            time.sleep(self.POLL_PERIOD)

    def stop(self):
        self._running = False

    # ──────────────────────────────────────────────────────────────────
    # Évaluation de toutes les conditions DTC
    # ──────────────────────────────────────────────────────────────────
    def _evaluate(self, mem: dict):
        now = time.time()
        # ── Mise à jour des valeurs brutes XCP dans le RTE ────────────────────
        # Ces attributs sont toujours synchronisés INDÉPENDAMMENT de l'état
        # moteur/pompe, pour permettre à _check_error_healing de vérifier que
        # la plateforme a réellement retiré l'injection avant d'autoriser le
        # soft-reset depuis ST_ERROR.
        if self._rte is not None:
            self._rte.set("xcp_front_raw_ma", _mem_get(mem, ADDR_FRONT_MOTOR_CURRENT))
            self._rte.set("xcp_rear_raw_ma",  _mem_get(mem, ADDR_REAR_MOTOR_CURRENT))
            self._rte.set("xcp_pump_raw_ma",  _mem_get(mem, ADDR_PUMP_CURRENT))
            # rest_contact_signal : 1=lame EN MOUVEMENT, 0=AU REPOS
            # Miroir inconditionnel pour que _read_rest_contact() puisse
            # appliquer l injection XCP meme quand REST_CONTACT_HARDWARE_PRESENT=True.
            # Defaut 1 (safe) : si memory.json ne contient pas encore la cle.
            self._rte.set("xcp_rest_contact_raw", bool(_mem_get(mem, ADDR_REST_CONTACT, default=1)))
            # pump_runtime_s brut : permet a _check_error_healing de verifier
            # que la plateforme a remis la valeur a 0 avant le soft-reset B2008.
            self._rte.set("xcp_pump_raw_runtime", _mem_get(mem, ADDR_PUMP_RUNTIME))
            # rain_sensor_raw brut : utilise par _check_rain_sensor() pour le
            # healing B2007 (attend que la valeur repasse <= 254 pendant 1s).
            self._rte.set("xcp_rain_raw", _mem_get(mem, ADDR_RAIN_SENSOR_RAW, default=0))
        self._check_b2001(mem, now)
        self._check_b2002(mem, now)
        self._check_b2003(mem, now)
        self._check_b2004(mem, now)
        self._check_b2005(mem, now)
        self._check_b2006(mem, now)
        self._check_b2007(mem, now)
        self._check_b2008(mem, now)
        self._check_b2009(mem, now)

    # ──────────────────────────────────────────────────────────────────
    # Helpers RTE — lecture sécurisée (rte peut être None en standalone)
    # ──────────────────────────────────────────────────────────────────
    def _front_motor_on(self) -> bool:
        """True si le moteur avant est alimenté selon le RTE."""
        if self._rte is None:
            return True   # standalone : pas de garde, on évalue toujours
        try:
            return bool(self._rte.front_motor_on)
        except Exception:
            return True

    def _rear_motor_on(self) -> bool:
        """True si le moteur arrière est alimenté selon le RTE."""
        if self._rte is None:
            return True
        try:
            return bool(self._rte.rear_motor_on) or bool(self._rte.rear_motor_running)
        except Exception:
            return True

    def _pump_active(self) -> bool:
        """True si la pompe est active selon le RTE."""
        if self._rte is None:
            return True
        try:
            return bool(self._rte.pump_active)
        except Exception:
            return True

    def _wiper_mode_auto(self) -> bool:
        """True si le mode essuie-glace est AUTO selon le RTE."""
        if self._rte is None:
            return True
        try:
            return str(self._rte.state) == "AUTO"
        except Exception:
            return True

    # ── B2001 : Front Motor Blocked — Overcurrent > 800 mA pendant 300 ms ──
    # Délégué à bcm_application._check_overcurrent() via RTE.
    # Ce module écrit xcp_front_current_a dans le RTE.
    # bcm_application applique la condition OR (courant_réel | courant_XCP)
    # avec toutes les gardes BCM (front_motor_on, délai 300ms, ST_ERROR).
    def _check_b2001(self, mem: dict, now: float):
        current_ma = _mem_get(mem, ADDR_FRONT_MOTOR_CURRENT)
        motor_on   = self._front_motor_on()
        fault      = motor_on and (current_ma > OVERCURRENT_FRONT_THRESH)

        if fault:
            if not self._latch_b2001:
                if self._t_front_oc == 0.0:
                    self._t_front_oc = now
                    logger.info(f"[B2001] Surcourant XCP front ({current_ma} mA > {OVERCURRENT_FRONT_THRESH}) — timer démarré, écriture RTE")
                elif (now - self._t_front_oc) >= OVERCURRENT_FRONT_DELAY:
                    # Timer écoulé : écrire dans le RTE pour que bcm_application déclenche
                    self._latch_b2001 = True
                    self._t_front_oc  = 0.0
                    if self._rte is not None:
                        self._rte.set("xcp_front_current_a", current_ma / 1000.0)
                        logger.info(f"[B2001] xcp_front_current_a={current_ma/1000.0:.3f}A écrit dans RTE → bcm_application déclenchera B2001")
                    else:
                        # Mode standalone sans RTE : déclencher directement (tests unitaires)
                        snap = self._snapshot(mem)
                        self._dtc.set_active("B2001", snap)
                        logger.info(f"[B2001] Mode standalone : DTC ACTIVE directement")
            # latch True → valeur déjà dans le RTE, on ne retouche rien
        else:
            # Condition disparue → remettre xcp_front_current_a à 0 dans le RTE
            if self._latch_b2001 or self._t_front_oc != 0.0:
                reason = "moteur avant arrêté" if not motor_on else f"courant OK ({current_ma} mA)"
                logger.info(f"[B2001] Condition disparue ({reason}) — reset RTE xcp_front_current_a=0")
            self._latch_b2001 = False
            self._t_front_oc  = 0.0
            if self._rte is not None:
                self._rte.set("xcp_front_current_a", 0.0)
            else:
                self._dtc.set_inactive("B2001")

    # ── B2002 : Rear Motor Blocked — Overcurrent > 800 mA pendant 300 ms ───
    # Délégué à bcm_application._check_overcurrent() via RTE.
    # Ce module écrit xcp_rear_current_a dans le RTE.
    def _check_b2002(self, mem: dict, now: float):
        current_ma = _mem_get(mem, ADDR_REAR_MOTOR_CURRENT)
        motor_on   = self._rear_motor_on()
        fault      = motor_on and (current_ma > OVERCURRENT_REAR_THRESH)

        if fault:
            if not self._latch_b2002:
                if self._t_rear_oc == 0.0:
                    self._t_rear_oc = now
                    logger.info(f"[B2002] Surcourant XCP rear ({current_ma} mA > {OVERCURRENT_REAR_THRESH}) — timer démarré, écriture RTE")
                elif (now - self._t_rear_oc) >= OVERCURRENT_REAR_DELAY:
                    self._latch_b2002 = True
                    self._t_rear_oc   = 0.0
                    if self._rte is not None:
                        self._rte.set("xcp_rear_current_a", current_ma / 1000.0)
                        logger.info(f"[B2002] xcp_rear_current_a={current_ma/1000.0:.3f}A écrit dans RTE → bcm_application déclenchera B2002")
                    else:
                        snap = self._snapshot(mem)
                        self._dtc.set_active("B2002", snap)
                        logger.info(f"[B2002] Mode standalone : DTC ACTIVE directement")
        else:
            if self._latch_b2002 or self._t_rear_oc != 0.0:
                reason = "moteur arrière arrêté" if not motor_on else f"courant OK ({current_ma} mA)"
                logger.info(f"[B2002] Condition disparue ({reason}) — reset RTE xcp_rear_current_a=0")
            self._latch_b2002 = False
            self._t_rear_oc   = 0.0
            if self._rte is not None:
                self._rte.set("xcp_rear_current_a", 0.0)
            else:
                self._dtc.set_inactive("B2002")

    # ── B2003 : Pump Overcurrent — Current > 800 mA pendant 300 ms ─────────
    # Délégué à bcm_application._check_pump_overcurrent() via RTE.
    # Ce module écrit xcp_pump_current_a dans le RTE.
    def _check_b2003(self, mem: dict, now: float):
        current_ma = _mem_get(mem, ADDR_PUMP_CURRENT)
        pump_on    = self._pump_active()
        fault      = pump_on and (current_ma > PUMP_OVERCURRENT_THRESH)

        if fault:
            if not self._latch_b2003:
                if self._t_pump_oc == 0.0:
                    self._t_pump_oc = now
                    logger.info(f"[B2003] Surcourant XCP pompe ({current_ma} mA > {PUMP_OVERCURRENT_THRESH}) — timer démarré, écriture RTE")
                elif (now - self._t_pump_oc) >= PUMP_OVERCURRENT_DELAY:
                    self._latch_b2003 = True
                    self._t_pump_oc   = 0.0
                    if self._rte is not None:
                        self._rte.set("xcp_pump_current_a", current_ma / 1000.0)
                        logger.info(f"[B2003] xcp_pump_current_a={current_ma/1000.0:.3f}A écrit dans RTE → bcm_application déclenchera B2003")
                    else:
                        snap = self._snapshot(mem)
                        self._dtc.set_active("B2003", snap)
                        logger.info(f"[B2003] Mode standalone : DTC ACTIVE directement")
        else:
            if self._latch_b2003 or self._t_pump_oc != 0.0:
                reason = "pompe arrêtée" if not pump_on else f"courant OK ({current_ma} mA)"
                logger.info(f"[B2003] Condition disparue ({reason}) — reset RTE xcp_pump_current_a=0")
            self._latch_b2003 = False
            self._t_pump_oc   = 0.0
            if self._rte is not None:
                self._rte.set("xcp_pump_current_a", 0.0)
            else:
                self._dtc.set_inactive("B2003")

    # ── B2004 : LIN Timeout CRS — lin_frame_valid == 0 pendant 100 ms ───────
    def _check_b2004(self, mem: dict, now: float):
        valid = _mem_get(mem, ADDR_LIN_FRAME_VALID, default=1)
        if valid == 0:
            if self._t_lin == 0.0:
                self._t_lin = now
                logger.info("[B2004] LIN frame invalide — timer démarré")
            elif (now - self._t_lin) >= LIN_TIMEOUT_DELAY:
                snap = self._snapshot(mem)
                self._dtc.set_active("B2004", snap)
        else:
            if self._t_lin != 0.0:
                logger.info("[B2004] LIN frame valide — timer reset")
            self._t_lin = 0.0
            self._dtc.set_inactive("B2004")

    # ── B2005 : CAN Timeout WC — can_wiper_status_valid == 0 pendant 100 ms ─
    def _check_b2005(self, mem: dict, now: float):
        valid = _mem_get(mem, ADDR_CAN_WIPER_VALID, default=1)
        if valid == 0:
            if self._t_can == 0.0:
                self._t_can = now
                logger.info("[B2005] CAN Wiper_Status invalide — timer démarré")
            elif (now - self._t_can) >= CAN_TIMEOUT_DELAY:
                snap = self._snapshot(mem)
                self._dtc.set_active("B2005", snap)
        else:
            if self._t_can != 0.0:
                logger.info("[B2005] CAN Wiper_Status valide — timer reset")
            self._t_can = 0.0
            self._dtc.set_inactive("B2005")

    # ── B2006 : Blade Position Implausible — blade_movement_detected == 0 ───
    # Garde : moteur avant doit être en marche (la lame ne peut être
    #         "implausible" que si le moteur est censé la déplacer)
    # Latch : un seul déclenchement par injection
    def _check_b2006(self, mem: dict, now: float):
        detected = _mem_get(mem, ADDR_BLADE_MOVEMENT, default=1)
        motor_on  = self._front_motor_on()
        fault     = motor_on and (detected == 0)

        if fault:
            if not self._latch_b2006:
                self._latch_b2006 = True
                logger.info("[B2006] Blade immobile avec moteur avant en marche — DTC ACTIVE")
                snap = self._snapshot(mem)
                self._dtc.set_active("B2006", snap)
        else:
            if self._latch_b2006:
                reason = "moteur avant arrêté" if not motor_on else "mouvement détecté"
                logger.info(f"[B2006] Condition disparue ({reason}) — latch reset")
            self._latch_b2006 = False
            self._dtc.set_inactive("B2006")

    # ── B2007 : Rain Sensor Signal Fault — raw > 254 (saturation ADC) ───────
    # Délégué à bcm_application._check_rain_sensor() via RTE.
    # Ce module écrit rain_sensor_ok=False et rain_intensity=raw dans le RTE
    # quand raw > 254 en mode AUTO.
    # bcm_application._check_rain_sensor() déclenche le DTC avec sa propre
    # logique (rain_sensor_installed, rain_sensor_ok) → un seul déclenchement.
    def _check_b2007(self, mem: dict, now: float):
        raw     = _mem_get(mem, ADDR_RAIN_SENSOR_RAW, default=0)
        in_auto = self._wiper_mode_auto()
        fault   = in_auto and (raw > RAIN_VALID_MAX)

        if fault:
            if not self._latch_b2007:
                self._latch_b2007 = True
                logger.info(f"[B2007] Capteur pluie hors plage XCP (raw={raw} > {RAIN_VALID_MAX}) mode AUTO — écriture RTE rain_sensor_ok=False")
                if self._rte is not None:
                    # Simuler un défaut capteur dans le RTE :
                    # rain_sensor_ok=False  → _check_rain_sensor() déclenchera B2007
                    # rain_intensity=raw    → le snapshot DTC aura la bonne valeur
                    self._rte.set("rain_sensor_ok",    False)
                    self._rte.set("rain_intensity",     raw)
                else:
                    # Mode standalone sans RTE : déclencher directement
                    snap = self._snapshot(mem)
                    self._dtc.set_active("B2007", snap)
            # latch True → RTE déjà posé, on ne retouche rien
        else:
            if self._latch_b2007:
                reason = f"raw={raw} ≤ {RAIN_VALID_MAX}" if in_auto else "mode non AUTO"
                logger.info(f"[B2007] Condition disparue ({reason}) — reset RTE rain_sensor_ok=True")
                if self._rte is not None:
                    self._rte.set("rain_sensor_ok", True)
                    self._rte.set("rain_intensity",  0)
                else:
                    self._dtc.set_inactive("B2007")
            self._latch_b2007 = False

    # ── B2008 : Pump Runtime Exceeded ──────────────────────────────────────
    # Fonctionnement XCP (identique à RID 0x0203 — sens FWD fixe) :
    #   pump_runtime_s = 0        → aucune action
    #   pump_runtime_s = N (> 0)  → démarrer pompe FWD pendant N secondes
    #                                Si N > PUMP_MAX_RUNTIME (5s) → B2008 + ST_ERROR
    #
    # La pompe démarre via RTE.xcp_pump_cmd (one-shot).
    # bcm_application._check_xcp_pump_cmd() lit la commande, démarre la pompe
    # et entre en ST_DIAG avec _test_duration = durée demandée.
    # _check_pump_protection() surveille le timer : si elapsed > PUMP_MAX_RUNTIME → B2008.
    def _check_b2008(self, mem: dict, now: float):
        duration = _mem_get(mem, ADDR_PUMP_RUNTIME)   # duree demandee en secondes

        if duration == 0:
            # Plateforme a remis pump_runtime_s a 0 : liberer le latch.
            if self._latch_b2008:
                logger.info("[B2008-XCP] pump_runtime_s=0 -> latch libere")
            self._latch_b2008       = False
            self._last_pump_runtime = 0
            return

        # duration > 0 -- verifier si c est une nouvelle valeur
        if duration != self._last_pump_runtime:
            # Nouvelle valeur differente de la precedente :
            # liberer le latch pour permettre une nouvelle injection.
            # Cela couvre le cas ou la plateforme envoie 3 -> 4 -> 5 -> 6
            # sans passer par 0 entre chaque valeur.
            if self._latch_b2008:
                logger.info(
                    f"[B2008-XCP] Nouvelle valeur {duration}s "
                    f"(precedente={self._last_pump_runtime}s) -> latch libere"
                )
            self._latch_b2008 = False

        if self._latch_b2008:
            # Meme valeur deja traitee : ne rien faire.
            return

        # Nouvelle valeur non traitee -> demarrer la pompe
        if self._rte is not None:
            current_cmd = getattr(self._rte, "xcp_pump_cmd", 0)
            pump_active = getattr(self._rte, "pump_active", False)
            pump_error  = getattr(self._rte, "pump_error",  False)
            if current_cmd == 0 and not pump_active and not pump_error:
                logger.info(
                    f"[B2008-XCP] Démarrage pompe FWD durée={duration}s "
                    f"-> RTE.xcp_pump_cmd=1 / xcp_pump_duration={duration} "
                    f"(surveillance {PUMP_MAX_RUNTIME}s -> B2008)"
                )
                self._last_pump_runtime = duration
                self._latch_b2008       = True
                self._rte.set("xcp_pump_duration", duration)
                self._rte.set("xcp_pump_cmd", 1)
            else:
                # Pompe occupee ou erreur active : memoriser la valeur sans
                # poser le latch pour reessayer au prochain cycle libre.
                self._last_pump_runtime = duration
        else:
            # Mode standalone sans RTE
            if duration > PUMP_MAX_RUNTIME:
                snap = self._snapshot(mem)
                self._dtc.set_active("B2008", snap)
            self._last_pump_runtime = duration
            self._latch_b2008       = True

    # ── B2009 : Rest Contact Failure — signal figé pendant 500 ms ───────────
    # Garde : moteur avant doit être en marche (le contact repos n'a de sens
    #         que si la lame est en mouvement)
    # Latch : un seul déclenchement — se libère quand le signal change à nouveau
    #         pendant HEAL_DELAY (contact prouve qu'il fonctionne)
    def _check_b2009(self, mem: dict, now: float):
        current_val = _mem_get(mem, ADDR_REST_CONTACT, default=1)
        motor_on    = self._front_motor_on()

        if self._rest_prev is None:
            # Premier appel : initialisation sans déclenchement
            self._rest_prev    = current_val
            self._t_rest_stuck = now
            return

        if current_val != self._rest_prev:
            # Signal a changé → contact vivant
            self._rest_prev    = current_val
            self._t_rest_stuck = now

            # Healing : si latch posé et signal change à nouveau → libérer
            if self._latch_b2009:
                if self._t_rest_heal == 0.0:
                    self._t_rest_heal = now
                    logger.info("[B2009] Signal rest contact change — timer healing démarré")
                elif (now - self._t_rest_heal) >= HEAL_DELAY:
                    self._t_rest_heal  = 0.0
                    self._latch_b2009  = False
                    self._dtc.set_inactive("B2009")
                    logger.info("[B2009] INACTIVE — signal OK pendant 1s")
        else:
            # Signal figé
            self._t_rest_heal = 0.0
            # Ne déclencher que si moteur avant en marche, pas déjà latché,
            # ET signal figé à 0 (contact AU REPOS bloqué).
            # Si signal=1 (EN MOUVEMENT) figé : valeur par défaut, pas un défaut.
            if motor_on and not self._latch_b2009 and current_val == 0:
                if (now - self._t_rest_stuck) >= REST_STUCK_DELAY:
                    self._latch_b2009 = True
                    snap = self._snapshot(mem)
                    self._dtc.set_active("B2009", snap)
                    logger.info("[B2009] Contact repos figé avec moteur en marche — DTC ACTIVE")
            elif not motor_on:
                # Moteur arrêté : reset timer stuck sans déclencher
                self._t_rest_stuck = now

    # ──────────────────────────────────────────────────────────────────
    # Snapshot pour dtc_manager.set_active()
    # ──────────────────────────────────────────────────────────────────
    def _snapshot(self, mem: dict) -> dict:
        """
        Construit un snapshot au format attendu par DTCManager.set_active()
        (Section 11 du Diagnostic Spec).
        Si le RTE est disponible (mode intégré), on l'utilise pour enrichir
        le snapshot (wiper_mode, rain, vehicle_speed).
        Sinon on se base uniquement sur les valeurs memory.json.
        """
        # Courant moteur (mA → A pour snapshot)
        front_ma = _mem_get(mem, ADDR_FRONT_MOTOR_CURRENT)
        rear_ma  = _mem_get(mem, ADDR_REAR_MOTOR_CURRENT)
        motor_ma = front_ma if front_ma > 0 else rear_ma

        snap = {
            "ignition":    1,          # allumage ON (contexte test)
            "wiper_mode":  "UNKNOWN",
            "motor_curr":  motor_ma,   # en mA, converti par DTCManager
            "blade_pos":   _mem_get(mem, ADDR_BLADE_MOVEMENT),
            "rain":        0,
            "vehicle_spd": 0,
        }

        # Enrichissement depuis le RTE si disponible
        if self._rte is not None:
            try:
                snap["ignition"]   = int(self._rte.ignition_status)
                snap["wiper_mode"] = str(self._rte.state)
                snap["rain"]       = int(self._rte.rain_intensity)
                snap["vehicle_spd"]= int(self._rte.vehicle_speed)
            except Exception:
                pass

        return snap


# ═════════════════════════════════════════════════════════════════════════════
# Serveur UDP XCP
# ═════════════════════════════════════════════════════════════════════════════
class XCPServerBCM:
    """
    Serveur XCP-on-UDP pour le BCM.

    Reçoit les commandes du XCPClient HIL, met à jour memory.json,
    et lance le moteur DTC (T-XCP-DTC) qui évalue les conditions
    et pilote dtc_manager en temps réel.

    Paramètres
    ----------
    dtc_manager : DTCManager
        Instance partagée avec bcm_main.py.
    rte : RTE | None
        Runtime Environment partagé. Optionnel — utilisé pour enrichir
        les snapshots DTC. Passer None en mode standalone.
    host : str
        Adresse d'écoute UDP (défaut : 0.0.0.0).
    port : int
        Port UDP (défaut : 17725, identique au XCPClient HIL).
    """

    def __init__(self,
                 dtc_manager=None,
                 rte=None,
                 host: str = HOST,
                 port: int = PORT):
        self._dtc    = dtc_manager
        self._rte    = rte
        self._host   = host
        self._port   = port
        self._lock   = threading.Lock()   # protège memory.json
        self._engine : Optional[_DTCEngineXCP] = None

        # Démarrage en mode standalone (sans dtc_manager) : importer localement
        if self._dtc is None:
            self._dtc = self._load_standalone_dtc()

    # ──────────────────────────────────────────────────────────────────
    def _load_standalone_dtc(self):
        """Charge DTCManager depuis le répertoire courant (mode standalone)."""
        import importlib.util, sys
        here = os.path.dirname(os.path.abspath(__file__))
        if here not in sys.path:
            sys.path.insert(0, here)
        from dtc_manager import DTCManager
        return DTCManager()

    # ──────────────────────────────────────────────────────────────────
    def run(self):
        """Point d'entrée du thread T-XCP-SRV."""

        # Démarrer le moteur DTC dans un thread séparé
        self._engine = _DTCEngineXCP(self._dtc, self._rte, self._lock)
        t_dtc = threading.Thread(
            target=self._engine.run,
            daemon=True,
            name="T-XCP-DTC",
        )
        t_dtc.start()

        # Ouvrir le socket UDP
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind((self._host, self._port))
        logger.info(f"XCP Server BCM en écoute sur {self._host}:{self._port}")

        # Charger l'état persisté
        with self._lock:
            memory = _load_memory()

        try:
            while True:
                data, addr = sock.recvfrom(1024)
                if not data:
                    continue

                cmd = data[0]

                # ── CONNECT ────────────────────────────────────────────
                if cmd == CMD_CONNECT:
                    logger.info(f"[XCP] CONNECT depuis {addr}")
                    sock.sendto(RES_OK, addr)

                # ── DISCONNECT ─────────────────────────────────────────
                elif cmd == CMD_DISCONNECT:
                    logger.info(f"[XCP] DISCONNECT depuis {addr}")
                    sock.sendto(RES_OK, addr)

                # ── STATUS (keep-alive) ────────────────────────────────
                elif cmd == CMD_STATUS:
                    sock.sendto(RES_OK, addr)

                # ── SHORT_DOWNLOAD (écriture valeur) ───────────────────
                elif cmd == CMD_SHORT_DOWNLOAD:
                    resp = self._handle_short_download(data, addr, memory)
                    sock.sendto(resp, addr)

                else:
                    logger.warning(f"[XCP] Commande inconnue: 0x{cmd:02X} depuis {addr}")
                    sock.sendto(RES_ERR, addr)

        except KeyboardInterrupt:
            logger.info("XCP Server arrêté")
        finally:
            if self._engine:
                self._engine.stop()
            sock.close()

    # ──────────────────────────────────────────────────────────────────
    def _handle_short_download(self, data: bytes, addr, memory: dict) -> bytes:
        """
        Traite un paquet SHORT_DOWNLOAD.

        Format (identique à XCPClient.short_download) :
          [0xED][size][0x00][0x00][addr 4B LE][data size B]
        """
        try:
            if len(data) < 8:
                raise ValueError(f"Paquet trop court: {len(data)} octets")

            size      = data[1]
            addr_val  = struct.unpack("<I", data[4:8])[0]
            val_bytes = data[8:8 + size]

            if len(val_bytes) != size:
                raise ValueError(f"Données incomplètes: attendu {size}, reçu {len(val_bytes)}")

            # Décoder la valeur selon la taille
            if size == 1:
                value = struct.unpack("B",  val_bytes)[0]
            elif size == 2:
                value = struct.unpack("<H", val_bytes)[0]
            elif size == 4:
                value = struct.unpack("<I", val_bytes)[0]
            else:
                raise ValueError(f"Taille non supportée: {size}")

            # Écriture dans memory.json (thread-safe)
            with self._lock:
                _mem_set(memory, addr_val, value)
                _save_memory(memory)

            var_name = self._addr_name(addr_val)
            logger.info(
                f"[XCP WRITE] {var_name} @ 0x{addr_val:08X} = {value}"
                f"  (size={size})"
            )
            return RES_OK

        except Exception as e:
            logger.error(f"[XCP SHORT_DOWNLOAD] Erreur: {e}")
            return RES_ERR

    # ──────────────────────────────────────────────────────────────────
    @staticmethod
    def _addr_name(addr: int) -> str:
        """Résout l'adresse en nom de variable pour les logs."""
        names = {
            ADDR_FRONT_MOTOR_CURRENT : "front_motor_current_mA",
            ADDR_REAR_MOTOR_CURRENT  : "rear_motor_current_mA",
            ADDR_PUMP_CURRENT        : "pump_current_mA",
            ADDR_LIN_FRAME_VALID     : "lin_frame_valid",
            ADDR_CAN_WIPER_VALID     : "can_wiper_status_valid",
            ADDR_BLADE_MOVEMENT      : "blade_movement_detected",
            ADDR_RAIN_SENSOR_RAW     : "rain_sensor_raw",
            ADDR_PUMP_RUNTIME        : "pump_runtime_s",
            ADDR_REST_CONTACT        : "rest_contact_signal",
        }
        return names.get(addr, f"unknown@0x{addr:08X}")


# ═════════════════════════════════════════════════════════════════════════════
# Point d'entrée standalone (tests sans bcm_main.py)
# ═════════════════════════════════════════════════════════════════════════════
if __name__ == "__main__":
    print("=" * 60)
    print("  XCP Server BCM — mode standalone")
    print(f"  Port UDP : {PORT}")
    print(f"  memory.json : {MEMORY_FILE}")
    print("=" * 60)
    server = XCPServerBCM()
    server.run()