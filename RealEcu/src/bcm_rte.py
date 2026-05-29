#!/usr/bin/env python3
"""
bcm_rte.py
==========
RTE -- Runtime Environment (Memoire Partagee)
WipeWash System -- Architecture 3 Couches

Regle absolue : AUCUNE logique ici.
Uniquement variables partagees + constantes + acces thread-safe.

MODIFICATION v3 -- Commande par relais  :
  Moteur avant : 2 relais
    RL2 (PIN_RELAY_FRONT_ON)   : ON/OFF moteur avant  actif LOW
    RL1 (PIN_RELAY_FRONT_SPEED): Speed1/Speed2         HIGH=Speed1 / LOW=Speed2

  Moteur arriere : 1 relais
    RL3 (PIN_RELAY_REAR_ON)    : ON/OFF moteur arriere actif LOW

  Rest Contact : hardware present (PIN_REST_CONTACT pull-down)
  Courant moteur : ADS1115 canal A3 (0V=0A / 3.3V=1A)
"""

import threading
import time
import json

# =====================================================
# REDIS -- Publication RTE + Ecoute commandes
# =====================================================
_REDIS_AVAILABLE = False
try:
    import redis as _redis_mod
    _REDIS_AVAILABLE = True
except ImportError:
    print("[RTE] package 'redis' absent -- pip install redis")

# Cles publiees dans Redis (rte:<key>)
REDIS_PUBLIC_KEYS = [
    "state", "crs_wiper_op", "ignition_status", "reverse_gear",
    "vehicle_speed", "rain_intensity", "front_motor_on",
    "front_motor_speed", "rear_motor_on", "rear_motor_running",
    "pump_active", "pump_direction", "motor_current_a",
    "pump_current_a", "pump_voltage_v", "pump_v_b", "pump_v_a",
    "lin_timeout_active",
    "lin_alive_fault",
    "lin_checksum_fault",   # TC_LIN_CS : checksum 0x16 invalide détecté
    "lin_baud",
    "wc_available", "wc_timeout_active",
    "wc_crc_fault",
    "wc_alive_fault",
    "wc_b2103_active",     # TC_B2103 : publie etat DTC B2103 vers Platform
    "lin_op_locked",          # TC_FSR_010/TC_CAN_003 : verrouille crs_wiper_op contre LIN 0x16
    "front_motor_error", "rear_motor_error", "pump_error",  # erreurs individuelles
    "wiper_fault",  # B2006/B2009 : erreur lame/contact repos (moteur seul, pompe non affectee)
    "rest_contact_raw",    # etat GPIO26 temps reel (True=lame EN MOUVEMENT)
    "front_blade_cycles",  # compteur cycles lame avant (incremente par rest_contact)
    "crs_fault",           # CRS_InternalFault recu via trame LIN 0x17
    "b2011_active",        # TC_B2011_AND : publie état DTC B2011 vers Platform
]

# Cles modifiables depuis la Platform via Redis pub/sub (stimuli tests)
REDIS_WRITABLE_KEYS = frozenset([
    "crs_wiper_op", "ignition_status", "rain_intensity",
    "vehicle_speed", "reverse_gear",
    "motor_current_a",          # T38 : injection surcourant depuis Platform
    "wc_timeout_active",        # T11 post_test : reset après CAN timeout
    "lin_timeout_active",       # T10/T39 post_test : reset après LIN timeout
    "lin_checksum_fault",       # TC_LIN_CS post_test : reset après test checksum
    "rain_sensor_installed",    # T34/T35 : activer capteur pluie pour AUTO
    "rest_contact_sim",         # T20/T36 : injection rest contact depuis Platform
                                # True=bouton appuyé=lame EN MOUVEMENT
                                # False=bouton relâché=lame AU REPOS
    "rest_contact_sim_active",  # True = Platform pilote rest_contact (tests auto)
                                # False = lecture GPIO hardware normale (défaut)
    # NOTE: pump_fault_mode et pump_fault_target sont retires du BCM.
    # L'injection de defauts est maintenant geree par le RPi Simulateur.
    # La Platform envoie ces commandes directement au Simulateur via TCP.
    "pump_cmd",    # Commande directe pompe depuis Platform (Fault Injection Panel)
                   # Valeurs : "fwd" | "bwd" | "stop"
                   # Traitee par T-REDIS-CMD → bcm_application._handle_pump_cmd()
    "watchdog_test_trigger",  # TC_FSR_008 : force un timeout watchdog simulé
    "wc_crc_fault",           # TC_FSR_010 : reset après test CRC 0x201
    "wc_available",           # TC_FSR_010 : activer mode Cas B pour recevoir 0x201
    "wc_alive_fault",         # TC_CAN_003 : WC signale AliveCounter 0x200 fige
    "lin_op_locked",          # TC_FSR_010/TC_CAN_003 : verrouille crs_wiper_op contre LIN 0x16
    "bcm_error_reset",        # T38/TC_FSR_008/TC_SPD_001 : reset explicite depuis Platform apres ERROR
    "wc_b2103_active",        # TC_B2103 post_test : nettoyage cle Redis apres test B2103
    "rear_wiper_available",   # T44 : activer/désactiver essuie-glace arrière (test T44/T51)
    "alive_tx_frozen",        # FIX TC_CAN_003 : geler AliveCounter_TX dans trames 0x200 (BCM→WC)
    # T38b / T38c : injection surcourant + reset état erreur depuis Platform
    "pump_current_a",         # T38c : injection surcourant pompe (> PUMP_OVERCURRENT_THRESH)
    "pump_error",             # T38c post_test : reset flag erreur pompe
    "rear_motor_error",       # T38b post_test : reset flag erreur moteur arrière
    "front_motor_error",      # T50b post_test : reset flag erreur moteur avant
    # T_RAIN_AUTO_SENSOR_ERROR : simuler capteur pluie OK/ERROR depuis Platform
    "rain_sensor_ok",         # T_RAIN_AUTO_SENSOR_ERROR : forcer SensorStatus=ERROR (False)
    # Post_test T38/T38b/T38c/T_RAIN : remettre DTC ACTIVE → INACTIVE (pas clear)
    # Valeur = code DTC string ex: "B2001", "B2002", "B2003", "B2007"
    "dtc_inactivate",         # code DTC à passer en INACTIVE après un test
    "b2011_active",           # TC_B2011_AND post_test : reset flag B2011 après test
    # T_B2009_CAN / T51 post_test : reset flags B2009 pour permettre re-détection
    "wiper_fault",                  # reset wiper_fault après ERROR B2009/B2006
    "_rest_contact_b2009_active",   # reset garde B2009 (re-armer la détection)
    # TC_B2104 : heal et reset compteurs
    "wc_ack_heal_count",      # TC_B2104 heal : reset compteur ACK healing wc_nack
    "wc_nack_consecutive",    # TC_B2104 post_test : reset compteur NACK consécutifs
    # TC_CAN_202_ERR02/04/05 post_test : reset flags WC fault côté BCM
    "wc_ack_pos_fault",             # TC_CAN_202_ERR04 post_test : reset flag B2006 condition PosSensor
    "wc_b2006_active", 
    "wc_fault_wc_internal",         # TC_CAN_202_ERR05 post_test : reset flag FaultStatus WC_Internal
    "wc_fault_motor_driver",        # TC_CAN_202_ERR02 post_test : reset flag FaultStatus MotorDriver
    "rest_contact_b2006_active",    # TC_CAN_202_ERR04 post_test : reset garde anti-boucle B2006
])


# =====================================================
# WIPER OPERATION CODES
# =====================================================
WOP_OFF        = 0x00
WOP_TOUCH      = 0x01
WOP_SPEED1     = 0x02
WOP_SPEED2     = 0x03
WOP_AUTO       = 0x04
WOP_FRONT_WASH = 0x05
WOP_REAR_WASH  = 0x06
WOP_REAR_WIPE  = 0x07

WOP_NAMES = {
    0: "OFF",   1: "TOUCH",      2: "SPEED1",    3: "SPEED2",
    4: "AUTO",  5: "FRONT_WASH", 6: "REAR_WASH", 7: "REAR_WIPE"
}

# =====================================================
# ETATS MACHINE D'ETAT
# =====================================================
ST_OFF        = "OFF"
ST_TOUCH      = "TOUCH"
ST_SPEED1     = "SPEED1"
ST_SPEED2     = "SPEED2"
ST_AUTO       = "AUTO"
ST_WASH_FRONT = "WASH_FRONT"
ST_WASH_REAR  = "WASH_REAR"
ST_REAR_WIPE  = "REAR_WIPE"
ST_ERROR      = "ERROR"
ST_DIAG       = "DIAG"   # DoIP pilote les actionneurs -- WSM ne touche rien
ST_PARK       = "PARK"   # FSR_004 : retour lame au repos apres ignition OFF

ST_ENC = {
    ST_OFF: 0,  ST_TOUCH: 1,      ST_SPEED1: 2,  ST_SPEED2: 3,
    ST_AUTO: 4, ST_WASH_FRONT: 5, ST_WASH_REAR: 6,
    ST_ERROR: 7, ST_REAR_WIPE: 8, ST_DIAG: 9,    ST_PARK: 10
}

# =====================================================
# CONSTANTES SECURITY ACCESS
# =====================================================
SA_REQ_SEED = 0x01
SA_SEND_KEY = 0x02
SA_XOR_MASK = 0xA5A5
SA_ADD_MASK = 0x3C

# =====================================================
# LIN  —  constantes chargées dynamiquement depuis le LDF
# =====================================================
# Les valeurs ci-dessous sont les VALEURS PAR DÉFAUT utilisées
# avant que load_ldf_config() soit appelé depuis bcm_main.py.
# Après l'appel, toutes les variables de ce bloc sont écrasées
# par les données lues dans le fichier wiperwash.ldf.
# -------------------------------------------------------
import sys as _sys
import os as _os

# Chercher ldf_loader.py dans le même répertoire que ce fichier,
# puis dans le répertoire parent (cas déploiement à la racine du projet).
_here = _os.path.dirname(_os.path.abspath(__file__))
for _ldf_dir in (_here, _os.path.dirname(_here)):
    if _ldf_dir not in _sys.path:
        _sys.path.insert(0, _ldf_dir)

try:
    from ldf_loader import load_ldf as _load_ldf
    _LDF_LOADER_OK = True
except ImportError:
    _LDF_LOADER_OK = False

try:
    from dbc_loader import load_dbc as _load_dbc, pack_frame as _dbc_pack, unpack_frame as _dbc_unpack
    _DBC_LOADER_OK = True
except ImportError:
    _DBC_LOADER_OK = False

# ── Constantes fixes LIN (protocole, jamais dans le LDF) ──────────
LIN_SYNC         = 0x55
LIN_BREAK        = 0x00
LIN_PID_DIAG_REQ = 0x3C
LIN_PID_DIAG_RSP = 0x3D

LIN_TIMEOUT      = 2.000   # 2000 ms  — seuil détection silence slave
LIN_INTERFRAME   = 0.050   # 50 ms    — pause entre trames

LIN_PORT_CANDIDATES = [
    "/dev/ttyACM0", "/dev/ttyACM1",
    "/dev/ttyUSB0", "/dev/ttyUSB1",
    "/dev/serial0",
]

# ── Variables initialisées avec les valeurs par défaut ────────────
# Elles SERONT écrasées par load_ldf_config() au démarrage.
LIN_BAUD         = 19200
LIN_ID_0x16      = 0x16
LIN_ID_0x17      = 0x17
LIN_PID_0x16     = 0xD6
LIN_PID_0x17     = 0x97
LIN_CYCLE_0x16   = 0.400
LIN_CYCLE_0x17   = 0.800

# Référence vers la config LDF complète (frames, pid_map, schedule)
# Remplie par load_ldf_config() — None si LDF non encore chargé.
LIN_LDF_CONFIG: dict = None   # type: ignore


def load_ldf_config(ldf_path: str) -> dict:
    """
    Charge le fichier LDF et met à jour toutes les constantes LIN du module.

    Doit être appelé UNE SEULE FOIS depuis bcm_main.py, avant la création
    de ProtocolLayer, afin que toutes les couches voient les bonnes valeurs.

    Paramètres
    ----------
    ldf_path : str
        Chemin vers le fichier wiperwash.ldf

    Retour
    ------
    dict cfg (baud, frames, pid_map, schedule) — même objet que LIN_LDF_CONFIG
    """
    import logging
    _log = logging.getLogger("bcm_rte")

    global LIN_BAUD, LIN_ID_0x16, LIN_ID_0x17
    global LIN_PID_0x16, LIN_PID_0x17
    global LIN_CYCLE_0x16, LIN_CYCLE_0x17
    global LIN_LDF_CONFIG

    if not _LDF_LOADER_OK:
        _log.warning("[RTE-LDF] ldf_loader introuvable — constantes par défaut conservées")
        return _build_fallback_cfg()

    cfg = _load_ldf(ldf_path)
    LIN_LDF_CONFIG = cfg
    LIN_BAUD       = cfg["baud"]

    frames = cfg["frames"]

    # Frame 0x16 — LeftStickWiperRequester
    f16 = frames.get("LeftStickWiperRequester")
    if f16:
        LIN_ID_0x16    = f16["id"]
        LIN_PID_0x16   = f16["pid"]
        LIN_CYCLE_0x16 = f16["cycle_s"] if f16["cycle_s"] > 0 else 0.400

    # Frame 0x17 — WiperFaultStatus
    f17 = frames.get("WiperFaultStatus")
    if f17:
        LIN_ID_0x17    = f17["id"]
        LIN_PID_0x17   = f17["pid"]
        LIN_CYCLE_0x17 = f17["cycle_s"] if f17["cycle_s"] > 0 else 0.800

    _log.info("[RTE-LDF] Config LIN chargée depuis '%s' :", ldf_path)
    _log.info("[RTE-LDF]   baud=%d  PID_0x16=0x%02X  PID_0x17=0x%02X"
              "  cycle_16=%.0fms  cycle_17=%.0fms",
              LIN_BAUD, LIN_PID_0x16, LIN_PID_0x17,
              LIN_CYCLE_0x16 * 1000, LIN_CYCLE_0x17 * 1000)
    return cfg


def _build_fallback_cfg() -> dict:
    """Config de secours si ldf_loader absent."""
    from ldf_loader import _calculate_pid, _default_config  # noqa
    return _default_config()

# =====================================================
# CAN
# =====================================================
CAN_ID_VEHICLE     = 0x300
CAN_ID_RAIN_SENSOR = 0x301
CAN_RECV_TIMEOUT   = 0.200
CAN_IDLE_SLEEP     = 1.000

# =====================================================
# CAN -- Trames BCM <-> WC (Cas B : WcAvailable = Installed)
# MESSAGE CATALOGUE Section 2
# =====================================================
CAN_ID_WIPER_COMMAND = 0x200   # BCM → WC  Wiper_Command
CAN_ID_WIPER_STATUS  = 0x201   # WC  → BCM Wiper_Status
CAN_ID_WIPER_ACK     = 0x202   # WC  → BCM Wiper_Ack      (event)

# Periode emission Wiper_Command par BCM (Cas B)
CAN_WC_CMD_PERIOD    = 0.400   # 400ms (chargé dynamiquement depuis DBC)

# Timeout supervision WC : si BCM ne recoit plus 0x201 -> B2005
# Modifie de 100ms → 2000ms (demande utilisateur)
CAN_WC_TIMEOUT       = 2.000   # 2000ms (ancien: 100ms)

# =====================================================
# CAN  —  configuration chargée dynamiquement depuis le DBC
# =====================================================
# Référence vers la config DBC complète (messages, id_map, periods_ms)
# Remplie par load_dbc_config() — None si DBC non encore chargé.
CAN_DBC_CONFIG: dict = None   # type: ignore


def load_dbc_config(dbc_path: str) -> dict:
    """
    Charge le fichier DBC et met à jour toutes les constantes CAN du module.

    Doit être appelé depuis bcm_main.py, AVANT la création de ProtocolLayer,
    afin que toutes les couches voient les bonnes valeurs.

    Paramètres
    ----------
    dbc_path : str
        Chemin vers le fichier wiperwash.dbc

    Retour
    ------
    dict cfg (messages, id_map, periods_ms, nodes) — même objet que CAN_DBC_CONFIG
    """
    import logging
    _log = logging.getLogger("bcm_rte")

    global CAN_DBC_CONFIG
    global CAN_ID_VEHICLE, CAN_ID_RAIN_SENSOR
    global CAN_ID_WIPER_COMMAND, CAN_ID_WIPER_STATUS, CAN_ID_WIPER_ACK
    global CAN_WC_CMD_PERIOD

    if not _DBC_LOADER_OK:
        _log.warning("[RTE-DBC] dbc_loader introuvable — constantes CAN par défaut conservées")
        return None

    cfg = _load_dbc(dbc_path)
    CAN_DBC_CONFIG = cfg

    msgs = cfg["messages"]
    periods = cfg["periods_ms"]

    # Mettre à jour les IDs CAN dynamiquement depuis le DBC
    for mid, m in msgs.items():
        if m.name == "Wiper_Command":
            CAN_ID_WIPER_COMMAND = mid
        elif m.name == "Wiper_Status":
            CAN_ID_WIPER_STATUS = mid
        elif m.name == "Wiper_Ack":
            CAN_ID_WIPER_ACK = mid
        elif m.name == "Vehicle_Status":
            CAN_ID_VEHICLE = mid
        elif m.name == "RainSensorData":
            CAN_ID_RAIN_SENSOR = mid

    # Mettre à jour la période d'envoi Wiper_Command
    p_ms = periods.get(CAN_ID_WIPER_COMMAND, 400)
    if p_ms > 0:
        CAN_WC_CMD_PERIOD = p_ms / 1000.0

    _log.info("[RTE-DBC] Config CAN chargée depuis '%s' :", dbc_path)
    _log.info("[RTE-DBC]   CMD=0x%03X  STS=0x%03X  ACK=0x%03X"
              "  VEH=0x%03X  RAIN=0x%03X  period_cmd=%.0fms",
              CAN_ID_WIPER_COMMAND, CAN_ID_WIPER_STATUS, CAN_ID_WIPER_ACK,
              CAN_ID_VEHICLE, CAN_ID_RAIN_SENSOR, CAN_WC_CMD_PERIOD * 1000)
    return cfg

# =====================================================
# TIMINGS THREADS
# =====================================================
CONTROL_LOOP_PERIOD   = 0.050   # T-WSM  : 200 ms
PUMP_GUARD_PERIOD     = 0.010   # T-PUMP : 200 ms
DIAG_LOOP_PERIOD      = 0.010   # T-DIAG : 10 ms
ACTUATOR_TEST_PERIOD  = 0.100   # verification duree test actionneur : 500 ms
WATCHDOG_CHECK_PERIOD = 0.800

# =====================================================
# PARAMETRES CALIBRATION
# =====================================================
TOUCH_DURATION          = 1.700
PARK_TIMEOUT            = 5.0  # FSR_004 : timeout max retour repos apres ignition OFF (s)
PUMP_MAX_RUNTIME        = 5.0
WASH_FRONT_CYCLES       = 3
WASH_REAR_CYCLES        = 2
RAIN_SPEED2_THRESH      = 20
OVERCURRENT_DELAY       = 0.3
PUMP_OVERCURRENT_DELAY  = 0.3
REST_STUCK_DELAY        = 3.0
HEAL_DELAY              = 1.0    # ISO 14229 healing : condition OK pendant 1s -> DTC INACTIVE
REVERSE_REAR_PERIOD     = 1.7
WATCHDOG_MAX_MS         = 500  # seuil watchdog : 250ms (x10 = 2500ms avant reset)
WIPE_CYCLE_DURATION     = TOUCH_DURATION

# =====================================================
# SEUILS COURANT -- ADS1115 (amperes)
# =====================================================
# ADS1115 gain=1 : plage 0 - 4.096V
# Potentiometre calibre : 0V = 0A / 3.3V = 1A
# Seuils exprimes en amperes (0.0 - 1.0)
OVERCURRENT_THRESH      = 0.8  # 0.8A seuil moteur avant/arriere
RAIN_VALID_MAX          = 254   # raw ADC max valide -> B2007 si > 254 (saturation)
PUMP_OVERCURRENT_THRESH = 0.811  # 0.8A seuil pompe

# =====================================================
# SEUILS FAULT INJECTION POMPE -- B2003
# =====================================================
# Open Load  : chute brusque du courant (0.7A -> 0.3A ou moins en un cycle)
PUMP_OPEN_LOAD_DROP     = 0.30   # delta de chute minimum pour détecter open load (A)
PUMP_OPEN_LOAD_MIN      = 0.20  # courant min après chute pour confirmer open load (A) -- borne basse intervalle [0.2, 0.4]
PUMP_OPEN_LOAD_MAX      = 0.40  # courant max après chute pour confirmer open load (A) -- borne haute intervalle [0.2, 0.4]

# Short to GND : courant quasi nul alors que pompe active
PUMP_SHORT_GND_THRESH   = 0.20  # courant < 0.20A = court-circuit vers masse (A)
PUMP_SHORT_GND_DELAY    = 0.300 # durée confirmation short to GND (s)

# Variable Load : courant instable (oscillation perturbante)
PUMP_VARIABLE_RANGE     = 0.4   # amplitude (max-min) sur fenêtre = variable load (A)
PUMP_VARIABLE_WINDOW    = 0.5   # fenêtre glissante d'analyse variable load (s)
PUMP_VARIABLE_SAMPLES   = 5     # nombre minimum d'échantillons pour analyse

# Courant nominal pompe en fonctionnement normal
PUMP_NORMAL_MIN         = 0.3   # courant minimum attendu pompe active (A)

# Retry et désactivation permanente
PUMP_FAULT_MAX_RETRY    = 2     # nb de tentatives de réactivation avant désactivation permanente

ADS_VOLTAGE_MIN = 0.0           # tension minimale calibree (V)
ADS_VOLTAGE_MAX = 3.3           # tension maximale calibree (V)
ADS_CURRENT_MIN = 0.0           # courant minimal (A)
ADS_CURRENT_MAX = 1.0           # courant maximal (A)
ADS_GAIN        = 1             # gain ADS1115 
ADS_CHANNEL     = 3             # canal A3 potentiometre courant moteur wiper
ADS_READ_PERIOD = 0.100         # lecture courant toutes les 100ms

# ADS1115 canal A0 -- ACS712 courant pompe
ADS_PUMP_CHANNEL    = 0         # canal A0 noeud B diviseur pompe
ADS_PUMP_R_CHARGE   = 10.0      # resistance de charge serie (ohm) -- loi d'Ohm courant
ADS_PUMP_R_HAUTE    = 10000.0   # diviseur haute (ohm) noeud A -> noeud B
ADS_PUMP_R_BASSE    =  2200.0   # diviseur basse (ohm) noeud B -> GND
ADS_PUMP_RATIO_DIV  = ADS_PUMP_R_BASSE / (ADS_PUMP_R_HAUTE + ADS_PUMP_R_BASSE)  # ~0.160
ADS_PUMP_NOISE      = 0.005     # seuil bruit V_b (V) en dessous -> 0
ADS_PUMP_NB_SAMPLES = 5         # nb echantillons mediane pour lecture pompe

# NOTE: Les constantes PUMP_FAULT_* et les pins ISO/MUX/SPDT/Y0/Y2 ont ete
# deplacees vers rpisimulator12/bcmcan.py. Le BCM ne gere plus l'injection
# de defauts : c'est le RPi Simulateur qui pilote ces GPIOs.
# Le BCM conserve uniquement PUMP_FAULT_NORMAL et PUMP_FAULT_ISO_OUVERT
# pour la logique de lecture ADS1115 (compensation offset).
PUMP_FAULT_NORMAL     = "NORMAL"
PUMP_FAULT_OPEN_LOAD  = "OPEN LOAD"
PUMP_FAULT_ISO_OUVERT = {"OPEN LOAD", "SIGNAL VARIABLE", "SHORT TO VCC"}

# =====================================================
# GPIO PINS -- RELAIS (actif LOW)
# =====================================================
# Moteur avant
PIN_RELAY_FRONT_ON    = 20   # RL2 : ON/OFF moteur avant   (LOW=ON  / HIGH=OFF)
PIN_RELAY_FRONT_SPEED = 23   # RL1 : vitesse moteur avant  (HIGH=Speed1 / LOW=Speed2)

# Moteur arriere
PIN_RELAY_REAR_ON     = 21   # RL3 : ON/OFF moteur arriere (LOW=ON  / HIGH=OFF)

# Pompe
PIN_PUMP_FWD          = 24   # pompe avant (actif HIGH)
PIN_PUMP_BWD          = 18   # pompe arriere (actif HIGH)

# NOTE: Les pins d'injection de defauts (ISO, SPDT, MUX_A/B, Y0_GATE, Y2_BASE)
# ont ete deplacees sur le RPi Simulateur. Voir bcmcan.py dans rpisimulator12.

# Rest Contact (pull-down, HIGH = lame en position repos)
PIN_REST_CONTACT      = 26   # bouton pull-down 10kohm

# Logique relais
RELAY_ON    = 0   # LOW  = relais active  (actif LOW)
RELAY_OFF   = 1   # HIGH = relais inactif
RELAY_SPEED1 = 1  # HIGH = Speed1 (vitesse lente)
RELAY_SPEED2 = 0  # LOW  = Speed2 (vitesse rapide)

# Rest contact hardware toujours present sur ce prototype
REST_CONTACT_HARDWARE_PRESENT = True

# =====================================================
# RTE -- RUNTIME ENVIRONMENT
# =====================================================
_WRITE_LOCK_TTL = 30.0   # secondes — TTL verrou Single Writer

class RTE:
    """
    Memoire partagee entre toutes les couches.
    AUCUNE logique ici -- uniquement stockage thread-safe.
    """

    def __init__(self):
        self._lock = threading.RLock()

        # ── [CAN] ────────────────────────────────────────
        self.ignition_status =1
        self.reverse_gear    = False
        self.vehicle_speed   = 0
        self.rain_intensity  = 0
        self.rain_sensor_ok  = True

        # ── [LIN] ────────────────────────────────────────
        self.crs_wiper_op         = WOP_OFF
        self.crs_stick_valid      = True
        self.crs_alive_prev       = 0xFF
        self.crs_fault            = 0x00
        self.t_last_lin0x16       = 0.0
        self.t_last_lin0x17       = 0.0
        self._t_ignition_redis    = 0.0   # timestamp dernier SET Redis ignition_status
        self.lin_timeout_active   = False
        self.lin_alive_fault      = False   # alive counter figé détecté (TC_LIN_002)
        self.lin_checksum_fault   = False   # checksum 0x16 invalide détecté (TC_LIN_CS)
        self.lin_baud             = LIN_BAUD  # baudrate LIN actuel (TC_COM_001)
        self._auto_ignored_logged = False
        self._rear_ignored_logged = False

        # ── [WSM] ────────────────────────────────────────
        self.state            = ST_OFF
        self.prev_state       = ST_OFF
        self.t_motor_stop     = 0.0
        self.t_rear_last      = 0.0
        self._reverse_active  = False

        self.t_touch_start      = 0.0
        self.wash_cycles_done   = 0
        self.t_wash_cycle_start = 0.0
        self._one_shot_armed    = True
        self._freeze_pending    = False
        self._freeze_last_op    = WOP_OFF
        self._auto_speed_prev   = -1
        self._t_park_start      = 0.0   # FSR_004 : timestamp entree ST_PARK

        # ── [PUMP] ───────────────────────────────────────
        self.pump_active             = False
        self.pump_direction          = 0
        self.t_pump_start            = 0.0
        self._pump_overcurrent_start = 0.0
        self.pump_cmd                = ""   # commande directe Platform : "fwd"/"bwd"/"stop"
        self.watchdog_test_trigger   = False  # TC_FSR_008 : force timeout watchdog simulé

        # ── [ACTIONNEURS -- etat temps reel] ──────────────
        self.front_motor_on     = False   # relais RL2 etat (True=ON)
        self.front_motor_speed  = 0       # 0=off / 1=speed1 / 2=speed2
        self.front_blade_moving = False
        self.rear_motor_on      = False   # relais RL3 etat (True=ON)
        self.rear_motor_running = False
        self.pump_dir_active    = 0

        # ── [ERREURS INDIVIDUELLES -- composant isole] ────
        # Chaque composant a son propre flag d'erreur independant.
        # Une erreur moteur n'affecte pas la pompe, et vice versa.
        self.front_motor_error  = False   # B2001 : surcourant moteur avant
        self.rear_motor_error   = False   # B2002 : surcourant moteur arriere
        self.pump_error              = False   # B2003 ou B2008 (flag général conservé)
        self.pump_overcurrent_error  = False   # B2003 uniquement : surcourant pompe
        self.pump_runtime_error      = False   # B2008 uniquement : dépassement runtime 5s
        self.wiper_fault        = False   # B2006/B2009 : contact repos / lame bloquee (pompe non affectee)

        # ── [FAULT INJECTION POMPE -- type et gestion retry] ──────────
        # pump_fault_type : qualifie la nature du défaut détecté sur la pompe
        # Valeurs : "NONE" | "OVERCURRENT" | "OPEN_LOAD" | "SHORT_GND" | "VARIABLE_LOAD"
        self.pump_fault_type         = "NONE"  # type de défaut actif
        self.pump_fault_retry_count  = 0       # nb de tentatives de réactivation après B2003
        self.pump_disabled_permanent = False   # True = pompe désactivée jusqu'à healing complet
        # Variables internes détection
        self._pump_prev_current      = 0.0     # courant cycle précédent (delta open load)
        self._pump_current_history   = []      # historique (timestamp, valeur) pour variable load
        self._pump_short_gnd_start   = 0.0     # timer confirmation short to GND

        # ── [COURANT MOTEUR -- ADS1115] ───────────────────
        # Valeur lue toutes les ADS_READ_PERIOD ms par T-PUMP
        # Unité : amperes (0.0 - 1.0A)
        self.motor_current_a         = 0.0
        self.pump_current_a          = 0.0   # courant pompe ACS712 (amperes)
        self.pump_voltage_v          = 0.0   # tension charge pompe (volts)
        self.pump_v_b                = 0.0   # tension brute ADS1115 (volts)
        self.pump_v_a                = 0.0   # tension calculee noeud A (volts)
        self.t_overcurrent_start     = {}
        # ── [COURANT INJECTE PAR XCP -- xcp_server_bcm] ──────────────
        # Valeurs écrites par xcp_server_bcm via memory.json → RTE.
        # Unité : amperes (converti depuis mA). 0.0 = pas d'injection active.
        # bcm_application._check_overcurrent() fait un OR :
        #   condition_fault = (courant_réel > seuil) OR (courant_XCP > seuil)
        # Cela permet de déclencher le DTC par XCP AVEC les mêmes gardes
        # (front_motor_on, rear_motor_on, pump_active) que le hardware réel.
        self.xcp_front_current_a     = 0.0   # B2001 : courant moteur avant injecté XCP (remis à 0 si moteur OFF)
        self.xcp_rear_current_a      = 0.0   # B2002 : courant moteur arrière injecté XCP (remis à 0 si moteur OFF)
        self.xcp_pump_current_a      = 0.0   # B2003 : courant pompe injecté XCP (remis à 0 si pompe OFF)

        # Valeurs brutes XCP (mA) : toujours synchronisées depuis memory.json
        # par xcp_server_bcm, INDÉPENDAMMENT de l'état moteur/pompe.
        # Utilisées par _check_error_healing pour vérifier que la plateforme
        # a réellement retiré l'injection avant d'autoriser le soft-reset.
        self.xcp_front_raw_ma        = 0     # mirror de ADDR_FRONT_MOTOR_CURRENT (memory.json)
        self.xcp_rear_raw_ma         = 0     # mirror de ADDR_REAR_MOTOR_CURRENT  (memory.json)
        self.xcp_pump_raw_ma         = 0     # mirror de ADDR_PUMP_CURRENT        (memory.json)
        self.xcp_pump_raw_runtime    = 0     # mirror de ADDR_PUMP_RUNTIME        (memory.json)
        # Mis a jour inconditionnellement par xcp_server_bcm._evaluate().
        # _check_error_healing cas "pump" verifie que cette valeur est revenue
        # a 0 avant d autoriser le soft-reset depuis ST_ERROR (B2008).
        # Evite la boucle ERROR->OFF->DIAG->ERROR quand pump_runtime_s > 5
        # est encore dans memory.json au moment du healing.
        # ── [COMMANDE POMPE XCP -- xcp_server_bcm] ───────────────────
        # xcp_pump_cmd : direction demandée par XCP (0=stop, 1=FWD, 2=BWD).
        # Mise à 0 après démarrage effectif (lecture one-shot par bcm_application).
        # Permet à _check_pump_protection() de surveiller le timer 5s → B2008.
        self.xcp_pump_cmd            = 0     # 0=inactif, 1=FWD — one-shot lu par bcm_application
        self.xcp_pump_duration       = 0     # durée demandée en secondes (injectée par XCP)
        self._pump_overcurrent_start = 0.0
        # ── [HEALING TIMERS -- ISO 14229] ─────────────────
        self._t_heal_front  = 0.0   # timer healing B2001 : courant moteur avant OK
        self._t_heal_rear   = 0.0   # timer healing B2002 : courant moteur arriere OK
        self._t_heal_pump   = 0.0   # timer healing B2003 : courant pompe OK
        self._t_heal_b2007  = 0.0   # timer healing B2007 : rain_sensor_raw <= 254 pendant 1s
        self._t_heal_b2008  = 0.0   # timer healing B2008 : cycle pompe normal
        self._t_heal_b2009  = 0.0   # timer healing B2009 : contact repos OK

        # Timers auto-healing ST_ERROR (soft-reset interne apres 1s courant < seuil)
        self._t_heal_error_front = 0.0  # healing B2001 en ST_ERROR -> retour prev_state
        self._t_heal_error_rear  = 0.0  # healing B2002 en ST_ERROR -> retour prev_state
        self._t_heal_error_pump  = 0.0  # healing B2003 en ST_ERROR -> retour prev_state

        # ── [MODES DEFAUT H-BRIDGE] ───────────────────────
        # NOTE: pump_fault_mode et pump_fault_target supprimes du BCM.
        # L'injection de defauts est desormais geree par le RPi Simulateur.
        self._pump_vb_last_fwd       = 0.0        # [C6] derniere V_b FORWARD memorisee

        # ── [SECURITE] ───────────────────────────────────
        self._rest_contact_stuck_start  = 0.0
        self._rest_contact_last_state   = -1
        self._rest_contact_b2009_active = False
        self._rest_contact_b2006_active = False  # garde anti-boucle B2006 (reset par Clear/Reset UDS)
        self._rest_contact_prev         = None   # etat precedent (edge detection cycles)
        self._t_last_blade_cycle        = 0.0   # timestamp dernier cycle detecte (NE555 sync)
        self.wc_blade_position          = -1     # -1=pas recu, 0=repos, >0=en mouvement (CAS B 0x201 Byte2)
        self._b2006_can_start           = 0.0    # timer detection B2006 CAS B
        self._front_blade_cycles        = 0      # compteur cycles lame avant
        # Variables publiques Redis (lues par Platform)
        self.rest_contact_raw           = False  # etat GPIO26 brut (True=lame EN MOUVEMENT)
        self.xcp_rest_contact_raw       = True   # mirror ADDR_REST_CONTACT (memory.json)
        self.xcp_rain_raw               = 0     # mirror de ADDR_RAIN_SENSOR_RAW (memory.json)
        # Mis a jour inconditionnellement par xcp_server_bcm._evaluate().
        # Utilise par _check_rain_sensor() pour le healing B2007 :
        # attend que la valeur repasse <= RAIN_VALID_MAX (254) pendant
        # HEAL_DELAY (1s) avant de mettre B2007 INACTIVE.
        # True  = signal actif (lame EN MOUVEMENT) — valeur par defaut safe
        # False = signal bloque au repos (injecte par XCP pour simuler B2009)
        # Mis a jour inconditionnellement par xcp_server_bcm._evaluate()
        # Utilise par _read_rest_contact() quand REST_CONTACT_HARDWARE_PRESENT=True
        # pour appliquer l injection XCP sur le chemin de detection B2009 de bcm_application.
        self.front_blade_cycles         = 0      # alias public de _front_blade_cycles
        self.crs_fault                  = 0x00   # CRS_InternalFault recu LIN 0x17
        # Injection test Platform (T20/T36)
        self.rest_contact_sim_active    = False  # False = GPIO hardware (défaut)
        self.rest_contact_sim           = False  # valeur injectée par Platform

        # ── [DIAG UDS] ───────────────────────────────────
        self._session            = 1
        self._sec_level          = 0
        self._pending_seed       = {}
        self._comm_tx_enabled    = True
        self._comm_rx_enabled    = True
        self._watchdog_kick_time = time.time()

        self._test_active        = False

        # ── [REDIS] ──────────────────────────────────────
        self._redis    = None

        # ── [SINGLE WRITER LOCK] ─────────────────────────────
        # Un seul client externe peut écrire à la fois,
        # quel que soit le serveur utilisé (Redis, DoIP, TCP…).
        # TTL automatique : verrou libéré si inactif 30s.
        self._write_lock_owner = None   # str | None
        self._write_lock_time  = 0.0    # float

        self._redis_ok = False
        self._test_routine       = 0
        self._test_duration      = 0
        self._t_test_start       = 0.0

        # ── [DoIP -- requete entrante decodee] ────────────────
        self._uds_mutex          = threading.Lock()
        self._uds_event          = threading.Event()

        self.uds_sid             = 0
        self.uds_payload         = b""
        self.uds_request_pending = False
        self.uds_response        = b""
        self.uds_response_ready  = False
        self.uds_src_addr        = 0
        self.uds_dst_addr        = 0

        # ── [CODING] ─────────────────────────────────────
        self.rain_sensor_installed = False
        self.rear_wiper_available  = True
        self.channel_front_wash    = 0
        self.channel_rear_camera   = 1

        # ── [CAS B -- WC disponible] ──────────────────────
        # Quand wc_available=True, le BCM envoie CAN 0x200 vers WC
        # et WC commande le moteur avant (Cas B).
        # Quand wc_available=False, le BCM commande directement
        # les relais moteur avant (Cas A).
        self.wc_available           = False  # code via WDID 0xF201
        self.t_last_wiper_status    = 0.0    # timestamp derniere trame 0x201 recue
        self.wc_timeout_active      = False  # B2005 CAN Timeout WC
        self.wc_crc_fault           = False  # CRC invalide sur 0x201 (TC_FSR_010)
        self.wc_alive_fault         = False  # AliveCounter 0x200 fige (TC_CAN_003)
        self.wc_b2103_active        = False  # TC_B2103 : DTC B2103 actif cote WC (nettoyage post-test)
        self.lin_op_locked          = False  # TC_FSR_010/TC_CAN_003 : LIN ne peut pas écraser crs_wiper_op
        self.bcm_error_reset        = False  # Reset explicite etat ERROR depuis Platform
        self.dtc_inactivate         = ""     # Post_test : code DTC à remettre INACTIVE ex "B2002"
        self.wc_alive_rx            = 0      # AliveCounter recu de WC
        self.wc_can_alive_tx        = 0      # AliveCounter emis par BCM
        # FIX TC_CAN_003 : geler l'AliveCounter dans les trames 0x200 (BCM→WC)
        # Quand True, _build_wiper_command() n'incrémente plus wc_can_alive_tx
        # → le WC simulé reçoit des 0x200 avec un counter constant → détecte la faute
        self.alive_tx_frozen        = False  # TC_CAN_003 : freeze AliveCounter TX

        # ── [CAN 0x201 — FaultStatus bits WC] ────────────────────────────
        self.wc_fault_wc_internal   = False  # byte4 bit0 : WC internal fault → B2101
        self.wc_fault_motor_driver  = False  # byte4 bit1 : motor driver fault → B2102
        self.wc_fault_pos_sensor    = False  # byte4 bit2 : position sensor fault → B2103/B2006
        self.wc_fault_supply        = False  # byte4 bit3 : supply voltage fault
        self.wc_fault_can_timeout   = False  # byte4 bit4 : CAN timeout fault
        self.wc_fault_motor_blocked = False  # byte4 bit5 : motor blocked fault

        # ── [CAN 0x202 — Wiper_Ack reception BCM] ────────────────────────
        self.wc_last_ack_status     = 0      # AckStatus bit0 reçu (0=ACK, 1=NACK)
        self.wc_last_error_code     = 0      # ErrorCode byte1 reçu
        self.wc_nack_consecutive    = 0      # compteur NACK consécutifs ErrorCode=0x01
        self.wc_current_mode = -1
        self.wc_ack_pending         = False  # True si nouvelle trame 0x202 reçue non traitée
        self.wc_ack_heal_count      = 0      # compteur ACK consécutifs pour healing wc_nack ST_ERROR
        self._t_b2101_heal          = 0.0   # timer healing B2101 (wc_fault_wc_internal=0)
        self._t_b2102_heal          = 0.0   # timer healing B2102 (wc_fault_motor_driver=0)
        self._t_b2103_heal          = 0.0   # timer healing B2103/B2006 pos sensor

        # ── [LIN 0x17 — CRS_Version / filtrage] ──────────────────────────
        self.crs_version            = 0x00   # version FW CRS reçue (byte1 de 0x17)
        self.crs_fault_stick        = False  # byte0 bit0 : CRS_InternalFault_Stick
        self.crs_fault_supply       = False  # byte0 bit1 : CRS_InternalFault_Supply
        self.crs_fault_comms        = False  # byte0 bit2 : CRS_InternalFault_Comms

        # ── [LIN 0x16 — StickStatus bits] ────────────────────────────────
        self.crs_stick_valid        = True   # bit0 (bit4 de byte0)
        self.crs_stick_debounce     = False  # bit1
        self.crs_stuck              = False  # bit2 (bit6 de byte0) — levier coincé
        self._t_stuck_start         = 0.0   # timestamp premier bit6=1 consécutif
        self._t_b2004_invalid_start = 0.0   # timestamp premier bit4=0 consécutif

        # ── [B2011 — condition conjointe 0x16 bit6 ET 0x17 bit0] ─────────
        self.b2011_active           = False  # B2011 actuellement ACTIVE

        # ── [B2006 — PosSensor CAN : condition conjointe 0x201 bit2 ET 0x202 0x04] ─
        self._b2006_pos_sensor_start = 0.0  # timestamp début double condition B2006


    # ─────────────────────────────────────────────────
    # Acces generique thread-safe
    # ─────────────────────────────────────────────────
    def get(self, key: str):
        with self._lock:
            return getattr(self, key)

    def set(self, key: str, value):
        with self._lock:
            setattr(self, key, value)

    def set_multi(self, **kwargs):
        with self._lock:
            for k, v in kwargs.items():
                setattr(self, k, v)

    def make_snapshot(self, motor_curr_ma: int = None) -> dict:
        """
        Capture l'état courant pour le snapshot DTC.
        motor_curr_ma : valeur effective en mA à utiliser pour motor_curr
                        (override la valeur ADS). Utile quand le DTC est
                        déclenché par une injection XCP dont la valeur est
                        plus élevée que la mesure hardware ADS.
                        Si None, utilise self.motor_current_a (valeur ADS).
        """
        with self._lock:
            curr_ma = motor_curr_ma if motor_curr_ma is not None                       else int(self.motor_current_a * 1000)
            return {
                "ignition":    self.ignition_status,
                "wiper_mode":  self.state,
                "motor_curr":  curr_ma,
                "blade_pos":   1 if self.front_blade_moving else 0,
                "rain":        self.rain_intensity,
                "vehicle_spd": self.vehicle_speed,
            }


    # ─────────────────────────────────────────────────
    # REDIS -- Publication + Ecoute commandes
    # ─────────────────────────────────────────────────
    def redis_connect(self, host: str = "127.0.0.1", port: int = 6379) -> bool:
        """
        Connecte le RTE a Redis.
        Appele une seule fois depuis bcm_main.py apres creation du RTE.
        Memorise host/port pour la reconnexion automatique.
        """
        if not _REDIS_AVAILABLE:
            print("[RTE-REDIS] redis non disponible -- tests BCM via backup CAN")
            return False
        # Memoriser pour reconnexion automatique
        self._redis_host = host
        self._redis_port = port
        return self._redis_reconnect()

    def _redis_reconnect(self) -> bool:
        """
        (Re)connecte Redis. Appele depuis redis_connect() et en cas d'erreur.
        Thread-safe via _lock deja acquis ou non (utilise son propre mutex).
        """
        if not _REDIS_AVAILABLE:
            return False
        host = getattr(self, "_redis_host", "127.0.0.1")
        port = getattr(self, "_redis_port", 6379)
        try:
            # Fermer l'ancienne connexion proprement
            if self._redis is not None:
                try:
                    self._redis.close()
                except Exception:
                    pass
            self._redis = _redis_mod.Redis(
                host=host, port=port, db=0,
                socket_connect_timeout=2,
                socket_timeout=1,
                # Desactive le pool de connexions interne pour eviter les
                # chevauchements entre T-REDIS (publish) et T-REDIS-CMD (pubsub).
                # Chaque thread utilise sa propre instance Redis.
                connection_pool=_redis_mod.ConnectionPool(
                    host=host, port=port, db=0,
                    socket_connect_timeout=2,
                    socket_timeout=1,
                    max_connections=4,
                ),
            )
            self._redis.ping()
            self._redis_ok = True
            print(f"[RTE-REDIS] Connecte sur {host}:{port}")
            return True
        except Exception as e:
            self._redis_ok = False
            self._redis = None
            print(f"[RTE-REDIS] Connexion impossible ({e}) -- mode degrade")
            return False

    def redis_publish(self) -> None:
        """
        Publie toutes les cles REDIS_PUBLIC_KEYS dans Redis (rte:<key>).
        Appele par T-REDIS toutes les 100ms.
        Publie aussi sur le canal rte_changed la liste des cles mises a jour.

        Gestion d'erreur robuste :
        - En cas d'erreur Redis transitoire, on tente UNE reconnexion.
        - Si la reconnexion echoue, on attend le prochain cycle (100ms).
        - N'influence JAMAIS les communications TCP/LIN/CAN reelles.
        """
        if not self._redis_ok or self._redis is None:
            # Tentative de reconnexion periodique (toutes les ~5s via 50 cycles)
            if not hasattr(self, "_redis_retry_ctr"):
                self._redis_retry_ctr = 0
            self._redis_retry_ctr += 1
            if self._redis_retry_ctr >= 50:
                self._redis_retry_ctr = 0
                self._redis_reconnect()
            return

        try:
            # Snapshot atomique sous lock (ne jamais tenir le lock pendant IO Redis)
            with self._lock:
                snapshot = {}
                for k in REDIS_PUBLIC_KEYS:
                    val = getattr(self, k, None)
                    snapshot[k] = str(val).lower() if isinstance(val, bool) else str(val)

            # Pipeline non-transactionnel pour performance maximale
            pipe = self._redis.pipeline(transaction=False)
            for key, encoded in snapshot.items():
                pipe.set(f"rte:{key}", encoded, ex=10)   # TTL 10s securite
            pipe.publish("rte_changed", json.dumps(list(snapshot.keys())))
            pipe.execute()

        except _redis_mod.ConnectionError as e:
            print(f"[RTE-REDIS] Connexion perdue ({e}) -- reconnexion...")
            self._redis_ok = False
            # Reconnexion immediate sans bloquer T-REDIS longtemps
            self._redis_reconnect()
        except _redis_mod.TimeoutError as e:
            # Timeout court (1s) : on logue et on continue sans bloquer
            print(f"[RTE-REDIS] Timeout publish ({e}) -- cycle suivant")
        except Exception as e:
            # Toute autre erreur Redis : logue, desactive, reconnexion au prochain cycle
            print(f"[RTE-REDIS] Erreur publish inattendue: {e}")
            self._redis_ok = False

    def redis_apply_cmd(self, key: str, value) -> bool:
        """
        Applique une commande SET venue de la Platform via Redis pub/sub.
        Seules les cles REDIS_WRITABLE_KEYS sont acceptees (securite).
        Retourne True si appliquee.
        """
        if key not in REDIS_WRITABLE_KEYS:
            print(f"[RTE-REDIS] Cle refusee (non inscriptible): {key}")
            return False
        try:
            with self._lock:
                attr_type = type(getattr(self, key, 0))
            # Convertir selon le type reel de l'attribut
            if attr_type == bool:
                typed = str(value).lower() in ("true", "1", "yes")
            elif attr_type == float:
                typed = float(value)
            elif attr_type == str:
                typed = str(value)    # pump_cmd et autres clés string
            else:
                typed = int(value)

            # FIX TC_FSR_010 — Race condition B2005 :
            # L'ancien code faisait deux opérations séparées :
            #   self.set("wc_available", True)       ← wc_available=True visible
            #   self.t_last_wiper_status = time()    ← reset APRÈS (hors lock)
            # Le thread cyclic _check_can_wc_timeout() pouvait se glisser entre
            # les deux et voir wc_available=True + t_last_wiper_status=77.6s
            # → B2005 déclenché immédiatement → wc_timeout_active=True
            # → _check_rte() lit wc_timeout_active=True mais delta déjà > LIMIT_MS=3000
            # → FAIL au lieu de PASS.
            # CORRECTION : setter wc_available ET t_last_wiper_status dans le même
            # set_multi() sous lock unique → le thread cyclic ne peut plus se glisser.
            if key == "wc_available" and typed is True:
                import time as _time
                self.set_multi(wc_available=True,
                               t_last_wiper_status=_time.time())
                print("[RTE-REDIS] SET wc_available=True + t_last_wiper_status "
                      "reinitialise (atomique, anti-race B2005)")
                return True

            # Priorité Redis sur CAN 0x300 pour ignition_status :
            # mémoriser le timestamp du dernier SET Redis pour bloquer
            # _can_process_0x300 pendant 1s (5 trames CAN @ 200ms).
            if key == "ignition_status":
                import time as _time
                self._t_ignition_redis = _time.time()

            self.set(key, typed)
            print(f"[RTE-REDIS] SET {key}={typed} (type={attr_type.__name__})")
            return True
        except Exception as e:
            print(f"[RTE-REDIS] Erreur apply_cmd({key}={value}): {e}")
            return False


    # ════════════════════════════════════════════════════════
    #  SINGLE WRITER LOCK — un seul client externe à la fois
    # ════════════════════════════════════════════════════════

    def acquire_write_lock(self, owner: str) -> bool:
        with self._lock:
            now = time.time()
            if (self._write_lock_owner is not None and
                    now - self._write_lock_time > _WRITE_LOCK_TTL):
                self._write_lock_owner = None
            if self._write_lock_owner is None:
                self._write_lock_owner = owner
                self._write_lock_time  = now
                return True
            if self._write_lock_owner == owner:
                self._write_lock_time = now
                return True
            held = int(now - self._write_lock_time)
            left = max(0, int(_WRITE_LOCK_TTL - held))
            return False

    def release_write_lock(self, owner: str) -> bool:
        with self._lock:
            if self._write_lock_owner == owner:
                self._write_lock_owner = None
                return True
            return False

    def renew_write_lock(self, owner: str) -> bool:
        with self._lock:
            if self._write_lock_owner == owner:
                self._write_lock_time = time.time()
                return True
            return False

    def get_write_lock_info(self) -> dict:
        with self._lock:
            now   = time.time()
            owner = self._write_lock_owner
            if owner and (now - self._write_lock_time) > _WRITE_LOCK_TTL:
                self._write_lock_owner = None
                owner = None
            held = int(now - self._write_lock_time) if owner else 0
            left = max(0, int(_WRITE_LOCK_TTL - held)) if owner else 0
            return {
                "owner":      owner,
                "free":       owner is None,
                "held_for_s": held,
                "ttl_left_s": left,
            }

    def set_locked(self, key: str, value, owner: str) -> bool:
        with self._lock:
            now = time.time()
            if (self._write_lock_owner is not None and
                    now - self._write_lock_time > _WRITE_LOCK_TTL):
                self._write_lock_owner = None
            if self._write_lock_owner != owner:
                current = self._write_lock_owner or "personne"
                return False
            self._write_lock_time = now
        self.set(key, value)
        return True

    def set_multi_locked(self, owner: str, **kwargs) -> bool:
        with self._lock:
            now = time.time()
            if (self._write_lock_owner is not None and
                    now - self._write_lock_time > _WRITE_LOCK_TTL):
                self._write_lock_owner = None
            if self._write_lock_owner != owner:
                current = self._write_lock_owner or "personne"
                return False
            self._write_lock_time = now
        self.set_multi(**kwargs)
        return True

    def __repr__(self):
        with self._lock:
            return (f"RTE(state={self.state}, "
                    f"req={WOP_NAMES.get(self.crs_wiper_op,'?')}, "
                    f"ign={self.ignition_status}, "
                    f"current={self.motor_current_a:.2f}A, "
                    f"pump={self.pump_active})")