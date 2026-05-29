#!/usr/bin/env python3
"""
bcm_application.py
==================
Couche Application -- Machine d'etat + Fonctions Wiper + Diagnostic UDS
WipeWash System -- Architecture 3 Couches

ADS1115 chip (I2C)
  ├── Canal A3 ── Potentiomètre ── _read_ads_current() ── rte.motor_current_a
  └── Canal A0 ── H-Bridge diviseur ─ _read_ads_pump()  ── rte.pump_current_a
                  noeud B (R_HAUTE/R_BASSE)                 rte.pump_voltage_v
                                                            rte.pump_v_b / pump_v_a
                                                            rte.pump_voltage_v
"""

import os
import bcm_rte  # lecture directe des constantes calibrables XCP (bcm_rte.TOUCH_DURATION, etc.)
import struct
import threading
import time

from bcm_rte import (
    RTE,
    ST_OFF, ST_TOUCH, ST_SPEED1, ST_SPEED2, ST_AUTO,
    ST_WASH_FRONT, ST_WASH_REAR, ST_REAR_WIPE, ST_ERROR, ST_DIAG, ST_ENC,
    ST_PARK,
    WOP_OFF, WOP_TOUCH, WOP_SPEED1, WOP_SPEED2, WOP_AUTO,
    WOP_FRONT_WASH, WOP_REAR_WASH, WOP_REAR_WIPE, WOP_NAMES,
    # Constantes calibrables lues via bcm_rte.<NOM> (XCP live) — PAS importées ici
    # TOUCH_DURATION, PARK_TIMEOUT, PUMP_MAX_RUNTIME,
    # WASH_FRONT_CYCLES, WASH_REAR_CYCLES,
    # RAIN_SPEED2_THRESH,
    # OVERCURRENT_THRESH, OVERCURRENT_DELAY,
    # PUMP_OVERCURRENT_THRESH, PUMP_OVERCURRENT_DELAY,
    # REST_STUCK_DELAY, HEAL_DELAY, REVERSE_REAR_PERIOD,
    # WATCHDOG_MAX_MS, WIPE_CYCLE_DURATION,
    PIN_RELAY_FRONT_ON, PIN_RELAY_FRONT_SPEED,
    PIN_RELAY_REAR_ON,
    PIN_PUMP_FWD, PIN_PUMP_BWD,
    PIN_REST_CONTACT,
    RELAY_ON, RELAY_OFF, RELAY_SPEED1, RELAY_SPEED2,
    REST_CONTACT_HARDWARE_PRESENT,
    CONTROL_LOOP_PERIOD, PUMP_GUARD_PERIOD,
    ACTUATOR_TEST_PERIOD,
    SA_REQ_SEED, SA_SEND_KEY, SA_XOR_MASK, SA_ADD_MASK,
    ADS_VOLTAGE_MIN, ADS_VOLTAGE_MAX,
    ADS_CURRENT_MIN, ADS_CURRENT_MAX,
    ADS_GAIN, ADS_CHANNEL, ADS_READ_PERIOD,
    ADS_PUMP_CHANNEL,
    ADS_PUMP_NOISE, ADS_PUMP_R_CHARGE, ADS_PUMP_R_HAUTE, ADS_PUMP_R_BASSE,
    ADS_PUMP_RATIO_DIV, ADS_PUMP_NB_SAMPLES,
)

from bcm_tcp_broadcast import TCPBroadcast
from bcm_ws_broadcast  import WSBroadcast
from bcm_tcp_pump     import TCPPumpBroadcast
from bcm_protocol import (
    SID_DSC, SID_RESET, SID_CLEAR, SID_RDTC,
    SID_RDID, SID_WDID, SID_SA, SID_CC, SID_RC, SID_TP,
    DSC_DEFAULT, DSC_EXTENDED,
)

# =====================================================
# GPIO via RPi.GPIO
# =====================================================
GPIO_AVAILABLE = False

try:
    import RPi.GPIO as GPIO
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    GPIO_AVAILABLE = True
    print("[GPIO] RPi.GPIO disponible")
except ImportError:
    print("[GPIO] RPi.GPIO non disponible -- mode simulation")


# =====================================================
# ADS1115 -- Lecture courant moteur + pompe
# =====================================================
ADS_AVAILABLE     = False
_ads_channel      = None
_ads_pump_channel = None

# Offset calibre mode OPEN LOAD (moyenne 5 lectures, delai 300ms)
_pump_offset_open_load = 0.0

try:
    import board
    import busio
    from adafruit_ads1x15 import ads1115 as ADS
    from adafruit_ads1x15.analog_in import AnalogIn

    _i2c         = busio.I2C(board.SCL, board.SDA)
    _ads         = ADS.ADS1115(_i2c)
    _ads.gain    = ADS_GAIN
    _ads_channel      = AnalogIn(_ads, ADS_CHANNEL)
    _ads_pump_channel = AnalogIn(_ads, ADS_PUMP_CHANNEL)
    ADS_AVAILABLE = True
    print(f"[ADS1115] Initialise | gain={ADS_GAIN} | canal=A{ADS_CHANNEL}")
    print(f"[ADS1115] Pompe H-bridge diviseur | canal=A{ADS_PUMP_CHANNEL}")
    print(f"[ADS1115] RATIO_DIV={ADS_PUMP_RATIO_DIV:.4f}  R_CHARGE={ADS_PUMP_R_CHARGE}ohm")
except Exception as e:
    print(f"[ADS1115] ERREUR INIT : {type(e).__name__}: {e}")
    print("[ADS1115] Courant/tension simules a 0")
    _ads_pump_channel = None





def _calibrer_offset_open_load(canal) -> float:
    """
    [C4] Moyenne 5 lectures + delai 300ms de stabilisation.
    Retourne l'offset mesure.
    """
    global _pump_offset_open_load
    time.sleep(0.3)
    lectures = []
    for _ in range(5):
        try:
            lectures.append(abs(canal.voltage))
            time.sleep(0.05)
        except OSError as e:
            print(f"[CALIB] Erreur lecture : {e}")
    if lectures:
        offset = sum(lectures) / len(lectures)
        _pump_offset_open_load = offset
        print(f"[CALIB] Offset OPEN LOAD ({len(lectures)} lectures) : {offset:.4f} V")
    else:
        _pump_offset_open_load = 0.0
        print("[CALIB] Erreur lecture offset -- compensation desactivee")
    return _pump_offset_open_load

# =====================================================
# LECTURE COURANT MOTEUR WIPER (ADS1115 A3)
# =====================================================
def _read_ads_current() -> float:
    """
    Lit le courant moteur wiper via ADS1115 canal A3.
    Remapping lineaire : ADS_VOLTAGE_MIN->0A / ADS_VOLTAGE_MAX->1A
    Retourne 0.0 si ADS non disponible.

    NOTE : on ne clamp PAS la valeur en sortie à ADS_CURRENT_MAX (1.0A).
    Le clamp haute à 1.0A empêchait le potentiomètre de descendre : quand
    le pot était physiquement à fond (≥3.3V), la moindre variation ne
    produisait aucun changement de valeur car min(1.0, x>=1.0) = 1.0
    en permanence. La valeur brute (éventuellement > 1.0A) est retournée
    telle quelle ; c'est _check_overcurrent qui réagit au seuil 0.8A,
    et l'affichage clamp côté Platform pour la jauge.
    """
    if not ADS_AVAILABLE or _ads_channel is None:
        return 0.0
    try:
        voltage = _ads_channel.voltage
        ratio   = (voltage - ADS_VOLTAGE_MIN) / (ADS_VOLTAGE_MAX - ADS_VOLTAGE_MIN)
        current = ratio * ADS_CURRENT_MAX
        # Clamp uniquement côté bas (pas de courant négatif)
        return round(max(ADS_CURRENT_MIN, current), 3)
    except Exception as e:
        print(f"[ADS1115] Erreur lecture moteur : {e}")
        return 0.0


# =====================================================
# LECTURE COURANT + TENSION POMPE (ADS1115 A0 / H-Bridge)
# =====================================================

def _read_ads_pump(rte) -> tuple:
    """
    Lit courant et tension pompe via diviseur H-bridge (canal A0).

    Schema :
      OUT1 L298N --- R_CHARGE(10ohm) --- noeud A --- R_CHARGE(10ohm) --- OUT2
      noeud A --- R_HAUTE(10k) --- noeud B --- R_BASSE(1.9k) --- GND
      ADS1115 A0 mesure noeud B (via relais ISO sur RPi Simulateur)

    Formules :
      V_b = lecture ADS1115 A0
      V_a = V_b / RATIO_DIV      (RATIO = R_BASSE / (R_HAUTE + R_BASSE))
      I   = V_a / R_CHARGE

    [C6] En BACKWARD : estimation depuis derniere V_b FORWARD memorisee.
    Retourne (current_A, voltage_v, v_b, v_a).

    NOTE : La gestion des modes défaut (ISO, MUX, etc.) est désormais
    entièrement prise en charge par le RPi Simulateur. Le BCM lit
    simplement ce qu'il mesure sur ses entrées ADS1115.
    """
    pump_active = rte.pump_active
    direction   = rte.pump_direction   # 1=FWD / 2=BWD

    # [C6] BACKWARD : estimation depuis derniere mesure FORWARD
    if pump_active and direction == 2:
        v_b     = rte._pump_vb_last_fwd
        v_a     = v_b / ADS_PUMP_RATIO_DIV if ADS_PUMP_RATIO_DIV > 0 else 0.0
        current = v_a / ADS_PUMP_R_CHARGE
        voltage = round(v_a, 3)
        return round(current, 3), round(voltage, 3), round(v_b, 4), round(v_a, 4)

    if not ADS_AVAILABLE or _ads_pump_channel is None:
        return 0.0, 0.0, 0.0, 0.0

    # Collecte echantillons
    samples = []
    for _ in range(ADS_PUMP_NB_SAMPLES):
        try:
            samples.append(abs(_ads_pump_channel.voltage))
            time.sleep(0.005)
        except OSError as e:
            print(f"[ADS-PUMP] Erreur I2C : {e}")
            time.sleep(0.02)

    if not samples:
        return 0.0, 0.0, 0.0, 0.0

    # Mediane robuste
    s   = sorted(samples)
    n   = len(s)
    v_b = (s[n//2 - 1] + s[n//2]) / 2 if n % 2 == 0 else s[n//2]

    # Seuil bruit
    if v_b < ADS_PUMP_NOISE:
        v_b = 0.0

    v_a     = v_b / ADS_PUMP_RATIO_DIV if ADS_PUMP_RATIO_DIV > 0 else 0.0
    current = v_a / ADS_PUMP_R_CHARGE
    voltage = v_a   # tension noeud A = tension aux bornes de la charge

    # [C6] Memorise V_b si FORWARD
    if pump_active and direction == 1:
        rte._pump_vb_last_fwd = v_b

    return round(current, 3), round(voltage, 3), round(v_b, 4), round(v_a, 4)


# =====================================================
# PRIMITIVES GPIO -- RELAIS
# =====================================================

def _gpio_setup():
    """Initialise toutes les broches GPIO."""
    if not GPIO_AVAILABLE:
        return
    GPIO.setup(PIN_RELAY_FRONT_ON,    GPIO.OUT, initial=RELAY_OFF)
    GPIO.setup(PIN_RELAY_FRONT_SPEED, GPIO.OUT, initial=RELAY_SPEED1)
    GPIO.setup(PIN_RELAY_REAR_ON,     GPIO.OUT, initial=RELAY_OFF)
    GPIO.setup(PIN_PUMP_FWD,          GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(PIN_PUMP_BWD,          GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(PIN_REST_CONTACT, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    print("[GPIO] Broches initialisees")
    print(f"  PIN_RELAY_FRONT_ON    = GPIO{PIN_RELAY_FRONT_ON}  (RL2 ON/OFF avant)")
    print(f"  PIN_RELAY_FRONT_SPEED = GPIO{PIN_RELAY_FRONT_SPEED}  (RL1 Speed1/Speed2)")
    print(f"  PIN_RELAY_REAR_ON     = GPIO{PIN_RELAY_REAR_ON}  (RL3 ON/OFF arriere)")
    print(f"  PIN_REST_CONTACT      = GPIO{PIN_REST_CONTACT}  (contact repos pull-down)")


def _gpio_cleanup():
    if not GPIO_AVAILABLE:
        return
    try:
        GPIO.cleanup()
        print("[GPIO] Libere")
    except Exception:
        pass


# =====================================================
# COUCHE APPLICATION
# =====================================================
class ApplicationLayer:

    def __init__(self, rte: RTE, dtc_manager):
        self._rte     = rte
        self._dtc     = dtc_manager
        self._running = False
        self._tcp      = TCPBroadcast()
        self._ws       = WSBroadcast(rte=self._rte)   # rte ref pour T-WS-TICK
        self._tcp_pump = TCPPumpBroadcast()
        _gpio_setup()
        # Origine d erreur capturee au moment de _enter_error(), independante
        # des flags RTE (front_motor_error / rear_motor_error / pump_error) qui
        # peuvent etre remis a False par la Platform via Redis pendant ST_ERROR.
        # Valeurs possibles : "front" | "rear" | "pump" | "general" | None
        self._error_origin = None



    # ==================================================
    # SECTION B -- PRIMITIVES MOTEUR / POMPE (RELAIS)
    # ==================================================

    def _front_motor_run(self, speed_level: int):
        # Cas B : si WcAvailable, le BCM ne touche pas les relais moteur avant.
        # La commande est transmise via CAN 0x200 par ProtocolLayer (thread T-CAN-WC).
        if getattr(self._rte, 'wc_available', False):
            print(f"[CAS B] Moteur avant -> commande CAN 0x200 speed={speed_level} "
                  f"(WC commande le relais)")
            return
        if not GPIO_AVAILABLE:
            print(f"[SIM] Moteur avant ON speed={speed_level}")
            return
        speed_val = RELAY_SPEED1 if speed_level == 1 else RELAY_SPEED2
        GPIO.output(PIN_RELAY_FRONT_SPEED, speed_val)
        GPIO.output(PIN_RELAY_FRONT_ON,    RELAY_ON)
        speed_name = "Speed1 (lente)" if speed_level == 1 else "Speed2 (rapide)"
        print(f"[MOTEUR] Avant ON | {speed_name} | "
              f"RL2=LOW(ON) RL1={'HIGH' if speed_level==1 else 'LOW'}")

    def _front_motor_stop(self):
        # Cas B : si WcAvailable, arret transmis via CAN 0x200 (WOP_OFF).
        if getattr(self._rte, 'wc_available', False):
            print("[CAS B] Moteur avant -> arret CAN 0x200 WOP_OFF")
            self._rte.set("t_motor_stop", time.time())
            return
        if not GPIO_AVAILABLE:
            print("[SIM] Moteur avant OFF")
            return
        GPIO.output(PIN_RELAY_FRONT_ON,    RELAY_OFF)
        GPIO.output(PIN_RELAY_FRONT_SPEED, RELAY_SPEED1)
        self._rte.set("t_motor_stop", time.time())
        print("[MOTEUR] Avant OFF | RL2=HIGH(OFF)")

    def _rear_motor_run(self):
        if not GPIO_AVAILABLE:
            print("[SIM] Moteur arriere ON")
            return
        GPIO.output(PIN_RELAY_REAR_ON, RELAY_ON)
        print("[MOTEUR] Arriere ON | RL3=LOW(ON)")

    def _rear_motor_stop(self):
        if not GPIO_AVAILABLE:
            print("[SIM] Moteur arriere OFF")
            return
        GPIO.output(PIN_RELAY_REAR_ON, RELAY_OFF)
        print("[MOTEUR] Arriere OFF | RL3=HIGH(OFF)")

    def _read_rest_contact(self) -> bool:
        """
        Lit l etat brut du GPIO contact repos (pull-down).
          GPIO = 0 (bouton relache) -> False  -> lame AU REPOS
          GPIO = 1 (bouton appuye)  -> True   -> lame EN MOUVEMENT

        PRIORITE 1 : injection Platform via Redis (rest_contact_sim_active=True)
          Retourne rte.rest_contact_sim. Permet T20/T36.

        PRIORITE 2 : injection XCP via memory.json (xcp_rest_contact_raw)
          Si xcp_rest_contact_raw=False (0x2000000B=0 injecte par HIL),
          le signal est force a False (lame AU REPOS) meme si GPIO=1.
          Ceci permet a _check_rest_contact_stuck de detecter B2009
          quand la plateforme injecte un contact repos bloque via XCP,
          que REST_CONTACT_HARDWARE_PRESENT soit True ou False.

        PRIORITE 3 : lecture GPIO hardware.
        """
        rte = self._rte
        # P1 : injection Platform Redis (T20/T36)
        if rte.rest_contact_sim_active:
            result = bool(rte.rest_contact_sim)
        elif not GPIO_AVAILABLE:
            # Pas de GPIO : utiliser uniquement la valeur XCP
            result = bool(getattr(rte, "xcp_rest_contact_raw", True))
        else:
            # P2 : lecture GPIO hardware
            hw_val = bool(GPIO.input(PIN_REST_CONTACT))
            # Injection XCP : AND logique — si XCP force 0, le signal devient 0
            # quelle que soit la valeur GPIO (simule un contact bloque au repos)
            xcp_val = bool(getattr(rte, "xcp_rest_contact_raw", True))
            result = hw_val and xcp_val
        # Publier l etat effectif dans le RTE -> Redis -> Platform
        rte.set("rest_contact_raw", result)
        return result

    def _track_blade_cycle(self, count_on_rest=True):
        """
        Détecte les cycles de la lame avant via le contact repos.
        
        Args:
            count_on_rest: True = compte un cycle quand la lame revient au repos (front descendant)
                           False = compte un cycle quand la lame commence un mouvement (front montant)
        """
        if not REST_CONTACT_HARDWARE_PRESENT and not self._rte.rest_contact_sim_active:
            return
        rte = self._rte
        # GPIO=1 (bouton appuye)  -> True  -> lame EN MOUVEMENT
        # GPIO=0 (bouton relache) -> False -> lame AU REPOS
        blade_moving = self._read_rest_contact()

        if rte._rest_contact_prev is None:
            rte._rest_contact_prev = blade_moving
            return

        # Détection selon le mode demandé
        if count_on_rest:
            # Compter quand la lame revient au repos (front descendant)
            if rte._rest_contact_prev is True and not blade_moving:
                rte._front_blade_cycles += 1
                rte.front_blade_cycles   = rte._front_blade_cycles  # sync public Redis
                print(f"[REST CONTACT] Cycle lame #{rte._front_blade_cycles} "
                      f"(contact repos atteint, etat={rte.state})")
                self._tcp.send(rte); self._ws.send(rte)
                # Cycle détecté : mettre à jour le timestamp du dernier cycle.
                # _check_rest_contact_stuck (T-PUMP) utilise ce timestamp pour
                # savoir si le contact est vivant, sans dépendre des fronts GPIO.
                rte._t_last_blade_cycle = time.time()
        else:
            # Compter quand la lame commence un mouvement (front montant)
            if not rte._rest_contact_prev and blade_moving:
                rte._front_blade_cycles += 1
                rte.front_blade_cycles   = rte._front_blade_cycles  # sync public Redis
                print(f"[REST CONTACT] Cycle lame #{rte._front_blade_cycles} "
                      f"(debut mouvement, etat={rte.state})")
                self._tcp.send(rte); self._ws.send(rte)
                rte._t_last_blade_cycle = time.time()

        rte._rest_contact_prev = blade_moving

    # Timestamp de la dernière injection test (Redis set externe > 0.8 A)
    _motor_inject_ts: float = 0.0
    # Durée pendant laquelle on conserve la valeur injectée (secondes)
    _MOTOR_INJECT_HOLD: float = 3.0

    # Même mécanisme pour la pompe (T38c : injection pump_current_a via Redis)
    _pump_inject_ts: float = 0.0
    _PUMP_INJECT_HOLD: float = 3.0

    def _read_motor_current(self) -> float:
        real = _read_ads_current()   # lecture ADS toujours faite (arriere-plan)
        rte  = self._rte
        now  = time.time()

        # Si le moteur est a l'arret -> courant affiche = 0.0 (meme si ADS mesure autre chose)
        motor_running = rte.front_motor_on or rte.rear_motor_running
        if not motor_running:
            self._motor_inject_ts = 0.0
            if rte.motor_current_a != 0.0:
                rte.set("motor_current_a", 0.0)
            return 0.0

        # Detection d'une injection test active : valeur Redis > seuil ET
        # superieure a la mesure ADC courante.
        if rte.motor_current_a > 0.8 and real < rte.motor_current_a:
            # Memoriser le debut du plateau si ce n'est pas encore fait.
            if self._motor_inject_ts == 0.0:
                self._motor_inject_ts = now
            # Conserver la valeur injectee uniquement pendant la fenetre de hold.
            if (now - self._motor_inject_ts) < self._MOTOR_INJECT_HOLD:
                return rte.motor_current_a
        else:
            # Mesure ADC redescendue ou injection terminee -> reset du timer.
            self._motor_inject_ts = 0.0

        # Ecrire la valeur ADC reelle (apres hold ou hors injection).
        rte.set("motor_current_a", real)
        return real

    def _read_pump_current(self) -> tuple:
        """
        Lit le courant pompe ADS1115 avec protection injection test (T38c).

        Meme logique que _read_motor_current : si pump_current_a a ete injecte
        via Redis (> bcm_rte.PUMP_OVERCURRENT_THRESH=0.8A) et que la mesure ADC reelle
        est inferieure, on conserve la valeur injectee pendant _PUMP_INJECT_HOLD
        secondes pour laisser le temps a _check_pump_overcurrent de declencher B2003.
        Sans ce hold, la lecture ADS (approx 0A hardware) ecraserait immediatement la
        valeur injectee au cycle suivant (100ms), rendant T38c non reproductible.

        NOTE : la lecture ADS est toujours effectuee (arriere-plan),
        mais si la pompe est inactive, retourne (0.0, 0.0) sans ecrire dans RTE.
        """
        current, voltage, v_b, v_a = _read_ads_pump(self._rte)   # lecture ADS arriere-plan
        rte = self._rte
        now = time.time()

        # Si la pompe est a l'arret -> courant/tension affiches = 0.0
        if not rte.pump_active:
            self._pump_inject_ts = 0.0
            if rte.pump_current_a != 0.0 or rte.pump_voltage_v != 0.0:
                rte.set_multi(
                    pump_current_a = 0.0,
                    pump_voltage_v = 0.0,
                    pump_v_b       = 0.0,
                    pump_v_a       = 0.0,
                )
            return 0.0, 0.0

        if rte.pump_current_a > bcm_rte.PUMP_OVERCURRENT_THRESH and current < rte.pump_current_a:
            # Injection test active : maintenir la valeur Redis injectee
            if self._pump_inject_ts == 0.0:
                self._pump_inject_ts = now
            if (now - self._pump_inject_ts) < self._PUMP_INJECT_HOLD:
                # Ne pas ecraser pump_current_a - laisser la valeur injectee
                rte.set_multi(
                    pump_voltage_v = voltage,
                    pump_v_b       = v_b,
                    pump_v_a       = v_a,
                )
                return rte.pump_current_a, voltage
        else:
            self._pump_inject_ts = 0.0

        # Mesure ADC reelle (hors injection ou apres hold)
        rte.set_multi(
            pump_current_a = current,
            pump_voltage_v = voltage,
            pump_v_b       = v_b,
            pump_v_a       = v_a,
        )
        return current, voltage

    def _pump_start(self, direction: int):
        rte = self._rte
        if rte.pump_active:
            return
        rte.set_multi(
            pump_active            = True,
            pump_direction         = direction,
            t_pump_start           = time.time(),
            _pump_overcurrent_start= 0.0,
        )
        if GPIO_AVAILABLE:
            if direction == 1:
                GPIO.output(PIN_PUMP_FWD, GPIO.HIGH)
                GPIO.output(PIN_PUMP_BWD, GPIO.LOW)
            else:
                GPIO.output(PIN_PUMP_FWD, GPIO.LOW)
                GPIO.output(PIN_PUMP_BWD, GPIO.HIGH)
        dirs = {1: "FWD (FrontWash)", 2: "BWD (RearWash)"}
        print(f"[POMPE] Demarrage {dirs.get(direction,'?')}")
        self._log_sensors("POMPE START")
        self._tcp_pump.send(rte)

    def _pump_stop(self, reason: str = "normal"):
        rte = self._rte
        if not rte.pump_active:
            return
        elapsed = time.time() - rte.t_pump_start
        rte.set_multi(
            pump_active            = False,
            pump_direction         = 0,
            _pump_overcurrent_start= 0.0,
        )
        if GPIO_AVAILABLE:
            GPIO.output(PIN_PUMP_FWD, GPIO.LOW)
            GPIO.output(PIN_PUMP_BWD, GPIO.LOW)
        print(f"[POMPE] Arret ({reason}, runtime={elapsed:.1f}s)")
        self._log_sensors("POMPE STOP")
        self._tcp_pump.send(rte)
        # Healing B2008 : si la pompe s est arretee normalement (< 5s) -> DTC INACTIVE
        if reason not in ("diag_max_runtime_b2008", "overcurrent_b2003", "max_runtime_5s"):
            if elapsed <= 5.0 and self._dtc.get_status("B2008") == "ACTIVE":
                self._dtc.set_inactive("B2008")
                print("[HEAL] B2008 INACTIVE : cycle pompe normal (arret < 5s)")

    def _stop_all(self):
        self._front_motor_stop()
        self._rear_motor_stop()
        self._pump_stop("stop_all")

    def _log_sensors(self, tag: str = ""):
        """
        Affiche courant moteur + courant/tension pompe en temps reel.
        Lit uniquement les valeurs deja presentes dans le RTE
        (mises a jour par T-PUMP) pour ne pas bloquer T-WSM avec
        des lectures I2C/ADS1115 synchrones.
        """
        rte    = self._rte
        prefix = f"[{tag}] " if tag else ""

        m_curr = self._read_motor_current()
        if rte.pump_active:
               p_curr, p_volt = self._read_pump_current()
        else:
              p_curr = 0.0
              p_volt = 0.0
              self._rte.set_multi(pump_current_a=0.0, pump_voltage_v=0.0)

        print(f"{prefix}"
              f"[MOTEUR] I={m_curr:.3f}A  "
              f"[POMPE]  I={p_curr:.3f}A  U={p_volt:.2f}V")

    # ==================================================
    # SECTION C -- MACHINE D'ETAT (WSM)
    # ==================================================

    # --------------------------------------------------
    # Nettoyage actionneurs avant changement d'etat
    # --------------------------------------------------
    # Groupes d'actionneurs mutuellement exclusifs :
    #   GROUPE_FRONT : etats utilisant le moteur avant
    #   GROUPE_REAR  : etats utilisant le moteur arriere
    #
    # Regle : si on quitte un etat FRONT pour aller vers
    # un etat REAR (ou inversement), on arrete les
    # actionneurs du groupe qu'on quitte AVANT d'entrer
    # dans le nouvel etat. Cela garantit qu'un moteur ne
    # reste jamais actif en arriere-plan.

    _STATES_USING_FRONT = {ST_TOUCH, ST_SPEED1, ST_SPEED2, ST_AUTO, ST_WASH_FRONT, ST_PARK}
    _STATES_USING_REAR  = {ST_WASH_REAR, ST_REAR_WIPE}

    def _exit_current_state(self, new_state: str):
        """
        Arrete les actionneurs du groupe abandonne si le nouvel etat
        appartient a un groupe incompatible.

        Cas traites :
          FRONT  -> REAR  : arret moteur avant
          REAR   -> FRONT : arret moteur arriere (+ pompe BWD si active)
          ANY    -> OFF   : rien (les _enter_off/_enter_error appellent _stop_all)
          ANY    -> ERROR : rien (idem)
        """
        rte       = self._rte
        old_state = rte.state

        # Pas de nettoyage necessaire si le nouvel etat gere lui-meme tout
        if new_state in (ST_OFF, ST_ERROR, ST_DIAG, ST_PARK):
            return

        # Transition d'un etat FRONT vers un etat REAR
        if old_state in self._STATES_USING_FRONT and new_state in self._STATES_USING_REAR:
            print(f"[WSM] EXIT-CLEAN: arret moteur AVANT ({old_state} -> {new_state})")
            self._front_motor_stop()
            if rte.pump_active and rte.pump_direction == 1:   # pompe FWD (avant)
                self._pump_stop("transition_front_to_rear")
            rte.set_multi(
                front_motor_on     = False,
                front_motor_speed  = 0,
                front_blade_moving = False,
            )

        # Transition d'un etat REAR vers un etat FRONT
        elif old_state in self._STATES_USING_REAR and new_state in self._STATES_USING_FRONT:
            print(f"[WSM] EXIT-CLEAN: arret moteur ARRIERE ({old_state} -> {new_state})")
            self._rear_motor_stop()
            if rte.pump_active and rte.pump_direction == 2:   # pompe BWD (arriere)
                self._pump_stop("transition_rear_to_front")
            rte.set_multi(
                rear_motor_on      = False,
                rear_motor_running = False,
            )

    def _enter_state(self, new_state: str):
        rte = self._rte
        if new_state == rte.state:
            return
        print(f"\n{'='*50}")
        print(f"[WSM] TRANSITION: {rte.state} --> {new_state}")
        print(f"{'='*50}")

        # --- Nettoyage des actionneurs du groupe quitte ---
        self._exit_current_state(new_state)

        rte.set_multi(prev_state=rte.state, state=new_state)
        self._log_sensors("TRANSITION")
        ENTRY = {
            ST_OFF:        self._enter_off,
            ST_TOUCH:      self._enter_touch,
            ST_SPEED1:     self._enter_speed1,
            ST_SPEED2:     self._enter_speed2,
            ST_AUTO:       self._enter_auto,
            ST_WASH_FRONT: self._enter_front_wash,
            ST_WASH_REAR:  self._enter_rear_wash,
            ST_REAR_WIPE:  self._enter_rear_wipe,
            ST_ERROR:      self._enter_error,
            ST_DIAG:       self._enter_diag,
            ST_PARK:       self._enter_park,
        }
        fn = ENTRY.get(new_state)
        if fn:
            fn()
        # send() est desormais non bloquant (queue async) -- pas besoin de kick apres
        self._tcp.send(rte); self._ws.send(rte)

    def _handle_pump_cmd(self):
        """
        Traite la commande pompe directe pump_cmd émise par la Platform
        (Fault Injection Panel) via Redis set_cmd("pump_cmd", "fwd"/"bwd"/"stop").
        Appelé au début de chaque cycle T-WSM (200ms).
        La commande est consommée (remise à "") après traitement pour éviter
        les déclenchements répétés.
        """
        rte = self._rte
        cmd = getattr(rte, "pump_cmd", "")
        if not cmd:
            return
        # Consommer immédiatement pour ne pas re-déclencher
        rte.set("pump_cmd", "")
        cmd = cmd.strip().lower()
        print(f"[BCM] pump_cmd reçu : '{cmd}' (depuis Platform Fault Injection Panel)")
        if cmd == "fwd":
            if rte.pump_active and rte.pump_direction == 1:
                print("[BCM] pump_cmd fwd : pompe déjà en FWD")
                return
            if rte.pump_active:
                self._pump_stop("pump_cmd_direction_change")
            self._pump_start(1)
        elif cmd == "bwd":
            if rte.pump_active and rte.pump_direction == 2:
                print("[BCM] pump_cmd bwd : pompe déjà en BWD")
                return
            if rte.pump_active:
                self._pump_stop("pump_cmd_direction_change")
            self._pump_start(2)
        elif cmd == "stop":
            if rte.pump_active:
                self._pump_stop("pump_cmd_stop")
            else:
                print("[BCM] pump_cmd stop : pompe déjà arrêtée")
        else:
            print(f"[BCM] pump_cmd inconnu : '{cmd}' ignoré")

    def _update_state_machine(self):
        self._watchdog_kick()
        rte = self._rte

        # ── Commande pompe directe depuis Platform (Fault Injection Panel) ──
        self._handle_pump_cmd()

        # SRD_WW_001 : inhibition SEULEMENT si Ignition = OFF(0)
        # 1=ON/ACC, 2=START → essuie-glaces autorisés → FSR_004 retour lame
        if rte.ignition_status == 0:
            # Même en ST_OFF : forcer crs_wiper_op=WOP_OFF pour que _process_off_state
            # n'accepte pas une commande résiduelle (ex: crs_wiper_op=2 du pré-test)
            # et ne redémarre pas le moteur avec ignition=0.
            if rte.crs_wiper_op != WOP_OFF:
                rte.crs_wiper_op = WOP_OFF
            if rte.state not in (ST_OFF, ST_PARK):
                # FSR_004 : ignition=0 -> retour lame au repos avant arrêt définitif
                # Si la lame est déjà au repos (ou pas de moteur actif) -> ST_OFF direct
                # Sinon -> ST_PARK : moteur tourne jusqu'au contact repos (ou timeout)
                if rte.state == ST_DIAG:
                    rte.set("_test_active", False)
                blade_moving = self._read_rest_contact()
                front_active = rte.state in self._STATES_USING_FRONT
                ign_name = {0: "OFF", 1: "ON/ACC", 2: "START"}.get(
                    rte.ignition_status, f"IGN={rte.ignition_status}")
                if front_active and blade_moving:
                    print(f"[WSM] Ignition {ign_name} -> ST_PARK (retour lame au repos FSR_004)")
                    self._enter_state(ST_PARK)
                else:
                    print(f"[WSM] Ignition {ign_name} -> ST_OFF direct (lame déjà au repos)")
                    self._enter_state(ST_OFF)
                return

        if rte.lin_timeout_active and rte.state not in (ST_OFF, ST_ERROR, ST_DIAG):
            self._enter_state(ST_OFF)
            return

        # Détection erreur pompe posée par T-PUMP (_check_pump_overcurrent) :
        # pump_error=True signifie que B2003 a été déclenché dans T-PUMP mais
        # _enter_error n'a pas encore été appelé (T-WSM traite 200ms après T-PUMP).
        # On entre en ST_ERROR ici pour que le BCM ne reste pas bloqué en WASH_FRONT.
        if rte.pump_error and rte.state not in (ST_ERROR, ST_OFF, ST_DIAG):
            self._enter_state(ST_ERROR)
            return

        # FIX T40 : ne pas capturer state en variable locale avant le dispatch.
        # _handle_pump_cmd() peut appeler _enter_state() et modifier rte.state.
        # Lire rte.state directement garantit que le dispatch utilise l'état courant.
        if rte.state == ST_OFF:
            self._process_off_state(rte.crs_wiper_op)
        elif rte.state == ST_PARK:
            self._process_park()
        elif rte.state == ST_TOUCH:
            self._process_touch()
        elif rte.state == ST_SPEED1:
            self._process_speed1(rte.crs_wiper_op)
        elif rte.state == ST_SPEED2:
            self._process_speed2(rte.crs_wiper_op)
        elif rte.state == ST_AUTO:
            self._process_auto(rte.crs_wiper_op)
        elif rte.state == ST_WASH_FRONT:
            self._process_front_wash()
        elif rte.state == ST_WASH_REAR:
            self._process_rear_wash()
        elif rte.state == ST_REAR_WIPE:
            self._process_rear_wipe()
        elif rte.state == ST_DIAG:
            # Rain sim (0x0205) : traiter le tick comme AUTO (moteur reagit a rain_intensity)
            if rte._test_active and rte._test_routine == 0x0205:
                self._process_auto(rte.crs_wiper_op)
            # else : DoIP pilote les actionneurs directement -- WSM ne touche rien
        elif rte.state == ST_ERROR:
            # ST_ERROR : deux chemins de sortie :
            #   1) Reset explicite Platform (bcm_error_reset=True) → ST_OFF
            #   2) Auto-healing : courant < seuil pendant HEAL_DELAY (1s)
            #      → soft-reset interne, retour à prev_state (ou ST_OFF si inconnu)
            if rte.bcm_error_reset:
                rte.set("bcm_error_reset", False)
                # Lever l'inhibition B2009 posée en pump_only pour que la
                # prochaine session détecte un vrai blocage de contact repos.
                rte.set("_rest_contact_b2009_active", False)
                rte.set_multi(_t_heal_error_front=0.0,
                              _t_heal_error_rear=0.0,
                              _t_heal_error_pump=0.0)
                self._error_origin = None
                print("[MODE ERROR] Reset Platform -> OFF")
                self._enter_state(ST_OFF)
            else:
                self._check_error_healing()

        if rte.state not in (ST_ERROR, ST_OFF):
            self._handle_reverse_intermittent()

        # ── DTC inactivate depuis Platform (post_test T38/T38b/T38c/T_RAIN) ─
        # La Platform envoie dtc_inactivate="B2002" (ou B2001/B2003/B2007)
        # après un test pour remettre le DTC ACTIVE → INACTIVE.
        # Cycle correct : INACTIVE → ACTIVE (faute) → INACTIVE (cleanup)
        # → ACTIVE (prochain run, affichage complet avec snapshot).
        dtc_code = rte.dtc_inactivate
        if dtc_code:
            rte.set("dtc_inactivate", "")   # consommer le flag
            self._dtc.set_inactive(dtc_code)
            print(f"[DTC] {dtc_code} remis INACTIVE (demande Platform post-test)")

    # ── Etat OFF ──────────────────────────────────────

    # ── Etat PARK (FSR_004) ──────────────────────────────────────────────────
    # Transitoire : déclenché par ignition OFF quand la lame est en mouvement.
    # Le moteur avant continue à Speed1 jusqu'au contact repos ou timeout.
    # Pompe et moteur arrière sont stoppés immédiatement.

    def _enter_park(self):
        """FSR_004 : Initie le retour lame au repos (ignition OFF)."""
        rte = self._rte
        print(f"[MODE PARK] FSR_004 : retour lame au repos "
              f"(timeout max {bcm_rte.PARK_TIMEOUT*1000:.0f}ms)")
        # Stopper immédiatement pompe + moteur arrière
        self._pump_stop("park_entry")
        self._rear_motor_stop()
        # Maintenir le moteur avant à Speed1 pour finir le cycle
        self._front_motor_run(1)
        rte.set_multi(
            front_motor_on     = True,
            front_motor_speed  = 1,
            front_blade_moving = True,
            rear_motor_on      = False,
            rear_motor_running = False,
            pump_dir_active    = 0,
            _t_park_start      = time.time(),
            _rest_contact_prev = self._read_rest_contact(),
        )

    def _process_park(self):
        """FSR_004 : Surveille le contact repos ou le timeout, puis passe ST_OFF."""
        rte     = self._rte
        elapsed = time.time() - rte._t_park_start

        # Contact repos atteint → lame en position, arrêt propre
        if not self._read_rest_contact():
            print(f"[PARK] Contact repos détecté ({elapsed*1000:.0f}ms) -> ST_OFF")
            self._front_motor_stop()
            rte.set_multi(
                front_motor_on     = False,
                front_motor_speed  = 0,
                front_blade_moving = False,
            )
            self._enter_state(ST_OFF)
            return

        # Timeout dépassé → arrêt forcé (B2006 desactive pour le moment)
        if elapsed >= bcm_rte.PARK_TIMEOUT:
            print(f"[PARK] Timeout {bcm_rte.PARK_TIMEOUT*1000:.0f}ms : "
                  f"contact repos jamais détecté -> ST_OFF (B2006 desactive)")
            self._front_motor_stop()
            rte.set_multi(
                front_motor_on     = False,
                front_motor_speed  = 0,
                front_blade_moving = False,
            )
            self._enter_state(ST_OFF)

    def _enter_off(self):
        print("[MODE OFF] Arret de tous les actionneurs")
        self._stop_all()
        self._rte.set_multi(
            front_motor_on            = False,
            front_motor_speed         = 0,
            front_blade_moving        = False,
            rear_motor_on             = False,
            rear_motor_running        = False,
            pump_dir_active           = 0,
            _rest_contact_stuck_start = 0.0,   # reset timer B2009
            _rest_contact_last_state  = -1,    # reset etat precedent B2009
            _rest_contact_prev        = None,  # Reset pour la prochaine detection de front
            _front_blade_cycles       = 0,     # reset compteur interne
            front_blade_cycles        = 0,     # reset compteur public Redis
            # ── Reset erreurs individuelles au retour OFF ──
            front_motor_error         = False,
            rear_motor_error          = False,
            pump_error                = False,
            pump_overcurrent_error    = False,
            pump_runtime_error        = False,
            t_motor_stop              = 0.0,
            # ── Reset fault injection pump state au retour OFF ──
            # Le signal wash est revenu à 0 : le prochain cycle est autorisé
            # à tenter une réactivation (sauf si désactivation permanente active).
            # pump_fault_retry_count et pump_disabled_permanent sont conservés
            # entre requêtes — seul le healing B2003 les remet à 0.
            pump_fault_type           = "NONE" if not getattr(self._rte, "pump_disabled_permanent", False) else getattr(self._rte, "pump_fault_type", "NONE"),
        )

    def _process_off_state(self, op: int):
        rte = self._rte
        if op == WOP_OFF:
            if not rte._freeze_pending:
                rte._one_shot_armed = True
            return

        if rte._freeze_pending and op != rte._freeze_last_op:
            rte._freeze_pending = False
            rte._one_shot_armed = True

        if rte._freeze_pending:
            return

        if op == WOP_TOUCH:
            if rte._one_shot_armed:
                rte._one_shot_armed = False
                self._enter_state(ST_TOUCH)
        elif op == WOP_SPEED1:
            self._enter_state(ST_SPEED1)
        elif op == WOP_SPEED2:
            self._enter_state(ST_SPEED2)
        elif op == WOP_AUTO:
            if rte.rain_sensor_installed:
                rte._auto_ignored_logged = False
                self._enter_state(ST_AUTO)
            else:
                if not rte._auto_ignored_logged:
                    print("[WSM] AUTO ignore: RainSensorInstalled=False")
                    rte._auto_ignored_logged = True
        elif op == WOP_FRONT_WASH:
            if rte._one_shot_armed:
                rte._one_shot_armed = False
                self._enter_state(ST_WASH_FRONT)
        elif op == WOP_REAR_WASH:
            if rte._one_shot_armed and rte.rear_wiper_available:
                rte._one_shot_armed = False
                self._enter_state(ST_WASH_REAR)
            elif rte._one_shot_armed and not rte.rear_wiper_available:
                if not rte._rear_ignored_logged:
                    print("[WSM] REAR_WASH ignore: RearWiperAvailable=False")
                    rte._rear_ignored_logged = True
        elif op == WOP_REAR_WIPE:
            # SRD_WW_092 : REAR_WIPE est une commande directe (pas one-shot)
            # Le declenchement est autorise a chaque fois que le levier est en position REAR_WIPE
            if rte.rear_wiper_available:
                self._enter_state(ST_REAR_WIPE)
            elif not rte._rear_ignored_logged:
                print("[WSM] REAR_WIPE ignore: RearWiperAvailable=False")
                rte._rear_ignored_logged = True

    # ── Etat DIAG ─────────────────────────────────────

    def _enter_diag(self):
        print("[MODE DIAG] DoIP prend le controle -- WSM suspendu")

    # ── Etat TOUCH ────────────────────────────────────

    def _enter_touch(self):
        print(f"[MODE TOUCH] 1 cycle <= {bcm_rte.TOUCH_DURATION*1000:.0f}ms (SRD_WW_020)")
        rte = self._rte
        rte._front_blade_cycles = 0
        rte._rest_contact_prev  = self._read_rest_contact()
        # Flag : on attend un cycle T-WSM supplementaire apres detection
        # du relachement du contact repos avant de passer a OFF
        rte._touch_rest_pending = False
        # Flag : la lame a quitte la position repos (bouton appuye au moins
        # une fois). Tant que ce flag est False, on ignore l'etat "repos"
        # initial -- sinon on sortirait immediatement de TOUCH.
        rte._touch_left_rest    = False
        self._rte.set_multi(
            t_touch_start             = time.time(),
            front_motor_on            = True,
            front_motor_speed         = 1,
            front_blade_moving        = True,
            t_motor_stop              = 0.0,
            # FIX BUG 3 : reset timers B2009 au demarrage de chaque mode moteur
            _rest_contact_stuck_start = 0.0,
            _rest_contact_last_state  = -1,
            _t_last_blade_cycle       = 0.0,
        )
        self._front_motor_run(1)

    def _process_touch(self):
        rte = self._rte
        elapsed = time.time() - rte.t_touch_start
        # GPIO=0 (bouton relache) -> False -> lame AU REPOS
        # GPIO=1 (bouton appuye)  -> True  -> lame EN MOUVEMENT (hors repos)
        blade_moving = self._read_rest_contact()

        # Etape 1 : tant que la lame n'a pas quitte le repos (bouton jamais
        # appuye), on ne peut pas considerer le cycle comme termine. On attend.
        if not rte._touch_left_rest:
            if blade_moving:
                rte._touch_left_rest = True
                print("[MODE TOUCH] Lame a quitte le repos (bouton appuye)")
            # Sinon : rien, on patiente (mais on verifie tout de meme le timeout plus bas)

        # Etape 2 : cycle d'attente deja arme -> on passe a OFF maintenant
        if getattr(rte, "_touch_rest_pending", False):
            print("[MODE TOUCH] Cycle d'attente apres relachement -> OFF")
            rte._touch_rest_pending = False
            rte.crs_wiper_op    = WOP_OFF
            rte._one_shot_armed = False
            rte._freeze_pending = True
            rte._freeze_last_op = WOP_TOUCH
            self._rte.set_multi(
                front_motor_on     = False,
                front_motor_speed  = 0,
                front_blade_moving = False,
            )
            self._front_motor_stop()
            self._enter_state(ST_OFF)
            return

        # Etape 3 : timeout -- la lame a quitte le repos mais n'y est pas
        # revenue dans TOUCH_DURATION. Anciennement armait B2006 ; B2006 est
        # desactive pour le moment -> on se contente d'arreter le moteur et
        # de passer en OFF (pas d'ERROR, pas de DTC).
        if elapsed >= bcm_rte.TOUCH_DURATION and (rte._touch_left_rest and blade_moving):
            print(f"[MODE TOUCH] Timeout ({elapsed*1000:.0f}ms) sans retour au "
                  f"repos -> OFF (B2006 desactive)")
            rte.crs_wiper_op    = WOP_OFF
            rte._one_shot_armed = False
            rte._freeze_pending = True
            rte._freeze_last_op = WOP_TOUCH
            self._rte.set_multi(
                front_motor_on     = False,
                front_motor_speed  = 0,
                front_blade_moving = False,
            )
            self._front_motor_stop()
            self._enter_state(ST_OFF)
            return

        # Etape 4 : relachement detecte (la lame avait quitte le repos
        # et vient d'y revenir) -> on arme l'attente d'un cycle avant OFF
        if rte._touch_left_rest and not blade_moving:
            print("[MODE TOUCH] Relachement detecte -> attente 1 cycle avant OFF")
            rte._touch_rest_pending = True

    # ── Etat SPEED1 ───────────────────────────────────

    def _enter_speed1(self):
        print("[MODE SPEED1] Relais Speed1 (SRD_WW_030)")
        rte = self._rte
        rte._front_blade_cycles = 0
        rte._rest_contact_prev  = None  # NE555 astable : ignorer phase courante, attendre prochain front
        self._rte.set_multi(
            front_motor_on            = True,
            front_motor_speed         = 1,
            front_blade_moving        = True,
            t_motor_stop              = 0.0,
            # FIX BUG 3 : reset timers B2009 au demarrage de chaque mode moteur
            _rest_contact_stuck_start = 0.0,
            _rest_contact_last_state  = -1,
            _t_last_blade_cycle       = 0.0,
        )
        self._front_motor_run(1)

    def _process_speed1(self, op: int):
        self._track_blade_cycle(count_on_rest=True)   # suivi cycles lame via contact repos
        rte = self._rte
        if op == WOP_OFF:
            self._enter_state(ST_OFF)
        elif op == WOP_TOUCH:
            self._enter_state(ST_TOUCH)
        elif op == WOP_SPEED2:
            self._enter_state(ST_SPEED2)
        elif op == WOP_AUTO and rte.rain_sensor_installed:
            self._enter_state(ST_AUTO)
        elif op == WOP_FRONT_WASH:
            self._enter_state(ST_WASH_FRONT)
        elif op == WOP_REAR_WASH and rte.rear_wiper_available:
            self._enter_state(ST_WASH_REAR)
        elif op == WOP_REAR_WIPE and rte.rear_wiper_available:
            self._enter_state(ST_REAR_WIPE)

    # ── Etat SPEED2 ───────────────────────────────────

    def _enter_speed2(self):
        print("[MODE SPEED2] Relais Speed2 (SRD_WW_040)")
        rte = self._rte
        rte._front_blade_cycles = 0
        rte._rest_contact_prev  = None  # NE555 astable : ignorer phase courante, attendre prochain front
        self._rte.set_multi(
            front_motor_on            = True,
            front_motor_speed         = 2,
            front_blade_moving        = True,
            t_motor_stop              = 0.0,
            # FIX BUG 3 : reset timers B2009 au demarrage de chaque mode moteur
            _rest_contact_stuck_start = 0.0,
            _rest_contact_last_state  = -1,
            _t_last_blade_cycle       = 0.0,
        )
        self._front_motor_run(2)

    def _process_speed2(self, op: int):
        self._track_blade_cycle(count_on_rest=True)   # suivi cycles lame via contact repos
        rte = self._rte
        if op == WOP_OFF:
            self._enter_state(ST_OFF)
        elif op == WOP_TOUCH:
            self._enter_state(ST_TOUCH)
        elif op == WOP_SPEED1:
            self._enter_state(ST_SPEED1)
        elif op == WOP_AUTO and rte.rain_sensor_installed:
            self._enter_state(ST_AUTO)
        elif op == WOP_FRONT_WASH:
            self._enter_state(ST_WASH_FRONT)
        elif op == WOP_REAR_WASH and rte.rear_wiper_available:
            self._enter_state(ST_WASH_REAR)
        elif op == WOP_REAR_WIPE and rte.rear_wiper_available:
            self._enter_state(ST_REAR_WIPE)

    # ── Etat AUTO ─────────────────────────────────────

    def _enter_auto(self):
        print("[MODE AUTO] Pluie automatique (SRD_WW_050)")
        rte = self._rte
        rte._front_blade_cycles = 0
        rte._rest_contact_prev  = None  # NE555 astable : ignorer phase courante, attendre prochain front
        self._rte.set_multi(
            _auto_speed_prev   = -1,
            front_motor_on     = False,
            front_motor_speed  = 0,
            front_blade_moving = False,
            _rest_contact_stuck_start = 0.0,
            _rest_contact_last_state  = -1,
            _t_last_blade_cycle       = 0.0,
        )

    def _process_auto(self, op: int):
        rte = self._rte
        if not rte.rain_sensor_installed:
            self._enter_state(ST_OFF)
            return
        if op == WOP_OFF:
            self._enter_state(ST_OFF);        return
        elif op == WOP_TOUCH:
            self._enter_state(ST_TOUCH);      return
        elif op == WOP_SPEED1:
            self._enter_state(ST_SPEED1);     return
        elif op == WOP_SPEED2:
            self._enter_state(ST_SPEED2);     return
        elif op == WOP_FRONT_WASH:
            self._enter_state(ST_WASH_FRONT); return
        elif op == WOP_REAR_WASH and rte.rear_wiper_available:
            self._enter_state(ST_WASH_REAR);  return

        rain    = rte.rain_intensity
        new_spd = 2 if rain >= bcm_rte.RAIN_SPEED2_THRESH else (1 if rain > 0 else 0)
        if new_spd != rte._auto_speed_prev:
            rte.set("_auto_speed_prev", new_spd)
            if new_spd == 2:
                self._rte.set_multi(front_motor_on=True, front_motor_speed=2, front_blade_moving=True)
                self._front_motor_run(2)
                print(f"[MODE AUTO] Speed2 (pluie={rain}>={bcm_rte.RAIN_SPEED2_THRESH})")
            elif new_spd == 1:
                self._rte.set_multi(front_motor_on=True, front_motor_speed=1, front_blade_moving=True)
                self._front_motor_run(1)
                print(f"[MODE AUTO] Speed1 (pluie={rain}>0)")
            else:
                self._rte.set_multi(front_motor_on=False, front_motor_speed=0, front_blade_moving=False)
                self._front_motor_stop()
                print("[MODE AUTO] Moteur STOP (pluie=0)")
            self._tcp.send(rte); self._ws.send(rte)   # notifier la Platform du changement vitesse/etat moteur

        # Suivi cycles lame via contact repos (uniquement si moteur avant actif)
        if rte.front_motor_on:
            self._track_blade_cycle(count_on_rest=True)

    # ── Etat WASH_FRONT ───────────────────────────────

    def _enter_front_wash(self):
        rte = self._rte

        # ── Vérification désactivation permanente pompe (B2003 fault persiste) ──
        if getattr(rte, "pump_disabled_permanent", False):
            print(f"[MODE FRONT WASH] Pompe DESACTIVEE PERMANENTEMENTE (fault_type={getattr(rte, 'pump_fault_type', '?')}) "
                  f"— wash refusé jusqu'à healing B2003")
            rte.crs_wiper_op    = WOP_OFF
            rte._one_shot_armed = False
            rte._freeze_pending = True
            rte._freeze_last_op = WOP_FRONT_WASH
            self._enter_state(ST_OFF)
            return

        # ── Vérification fault active : tentative de réactivation ──
        if getattr(rte, "pump_error", False):
            retry = getattr(rte, "pump_fault_retry_count", 0)
            print(f"[MODE FRONT WASH] Pompe en erreur (fault_type={getattr(rte, 'pump_fault_type', '?')}, "
                  f"retry={retry}/{bcm_rte.PUMP_FAULT_MAX_RETRY}) — tentative de réactivation")
            # Reset pump_error pour permettre la tentative (le détecteur re-posera si faute persiste)
            rte.set_multi(
                pump_error             = False,
                pump_overcurrent_error = False,
            )

        print(f"[MODE FRONT WASH] Pompe FWD + {bcm_rte.WASH_FRONT_CYCLES} cycles")
        rte._front_blade_cycles = 0
        rte._rest_contact_prev  = None  # NE555 astable : ignorer phase courante, attendre prochain front
        self._pump_start(1)
        rte.set_multi(
            wash_cycles_done          = 0,
            t_wash_cycle_start        = time.time(),
            front_motor_on            = True,
            front_motor_speed         = 1,
            front_blade_moving        = True,
            pump_dir_active           = 1,
            t_motor_stop              = 0.0,
            _rest_contact_stuck_start = 0.0,
            _rest_contact_last_state  = -1,
            _t_last_blade_cycle       = 0.0,
        )
        self._front_motor_run(1)

    def _process_front_wash(self):
        rte     = self._rte
        elapsed = time.time() - rte.t_wash_cycle_start

        if rte.pump_active and elapsed >= bcm_rte.PUMP_MAX_RUNTIME:
            self._pump_stop("front_wash_fsr005")
            rte.set("pump_dir_active", 0)
            # FSR_005 : dépassement runtime → B2008 + pump_error
            # Sans pump_error=True, T22 ne détecte jamais le DTC.
            snap = rte.make_snapshot()
            self._dtc.set_active("B2008", snap)
            rte.set_multi(
                pump_error           = True,
                pump_runtime_error   = True,   # B2008 : dépassement runtime
                pump_dir_active      = 0,
            )
            # Stopper aussi le moteur avant pour éviter B2009 STUCK CLOSED
            self._front_motor_stop()
            rte.set_multi(
                front_motor_on    = False,
                front_motor_speed = 0,
                front_blade_moving= False,
            )
            rte.crs_wiper_op    = WOP_OFF
            rte._one_shot_armed = False
            # freeze_pending=True : empêche le re-déclenchement FRONT_WASH
            # si le LIN renvoie encore WOP_FRONT_WASH après le retour en ST_OFF
            rte._freeze_pending = True
            rte._freeze_last_op = WOP_FRONT_WASH
            self._enter_state(ST_OFF)
            return

        # Comptage de cycles : contact repos si hardware present OU simulation active,
        # sinon fallback par temps (banc sans GPIO ni simulation)
        if REST_CONTACT_HARDWARE_PRESENT or rte.rest_contact_sim_active:
            self._track_blade_cycle(count_on_rest=True)
            cycles = rte._front_blade_cycles
        else:
            cycles = int(elapsed / bcm_rte.WIPE_CYCLE_DURATION)

        if cycles > rte.wash_cycles_done:
            rte.set("wash_cycles_done", cycles)
            print(f"[MODE FRONT WASH] Cycle {cycles}/{bcm_rte.WASH_FRONT_CYCLES}")
            self._log_sensors(f"Cycle {cycles}/{bcm_rte.WASH_FRONT_CYCLES}")

        if rte.wash_cycles_done >= bcm_rte.WASH_FRONT_CYCLES:
            print(f"[MODE FRONT WASH] {bcm_rte.WASH_FRONT_CYCLES} cycles -> OFF")
            if rte.pump_active:
                self._pump_stop("front_wash_complet")
            self._rte.set_multi(
                front_motor_on     = False,
                front_motor_speed  = 0,
                front_blade_moving = False,
                pump_dir_active    = 0,
            )
            self._front_motor_stop()
            rte.crs_wiper_op    = WOP_OFF
            rte._one_shot_armed = False
            rte._freeze_pending = True
            rte._freeze_last_op = WOP_FRONT_WASH
            self._enter_state(ST_OFF)

    # ── Etat WASH_REAR ────────────────────────────────

    def _enter_rear_wash(self):
        rte = self._rte

        # ── Vérification désactivation permanente pompe (B2003 fault persiste) ──
        if getattr(rte, "pump_disabled_permanent", False):
            print(f"[MODE REAR WASH] Pompe DESACTIVEE PERMANENTEMENTE (fault_type={getattr(rte, 'pump_fault_type', '?')}) "
                  f"— XSM -> ST_ERROR (pump_disabled), commande REAR_WASH mise en attente")
            rte.crs_wiper_op    = WOP_OFF
            rte._one_shot_armed = False
            rte._freeze_pending = True
            rte._freeze_last_op = WOP_REAR_WASH
            self._error_origin  = "pump_disabled"
            self._enter_state(ST_ERROR)
            return

        # ── Vérification fault active : tentative de réactivation ──
        if getattr(rte, "pump_error", False):
            retry = getattr(rte, "pump_fault_retry_count", 0)
            print(f"[MODE REAR WASH] Pompe en erreur (fault_type={getattr(rte, 'pump_fault_type', '?')}, "
                  f"retry={retry}/{bcm_rte.PUMP_FAULT_MAX_RETRY}) — tentative de réactivation")
            rte.set_multi(
                pump_error             = False,
                pump_overcurrent_error = False,
            )

        print(f"[MODE REAR WASH] Pompe BWD + {bcm_rte.WASH_REAR_CYCLES} cycles")
        self._pump_start(2)
        rte.set_multi(
            wash_cycles_done   = 0,
            t_wash_cycle_start = time.time(),
            pump_dir_active    = 2,
        )
        if rte.rear_wiper_available:
            rte.set("rear_motor_running", True)
            rte.set("rear_motor_on",      True)
            self._rear_motor_run()

    def _process_rear_wash(self):
        rte = self._rte
        if not rte.rear_wiper_available:
            self._pump_stop("coding_f202")
            self._rear_motor_stop()
            rte.set_multi(rear_motor_on=False, rear_motor_running=False)
            rte.set("_one_shot_armed", True)
            self._enter_state(ST_OFF)
            return
        elapsed = time.time() - rte.t_wash_cycle_start
        cycles  = int(elapsed / bcm_rte.WIPE_CYCLE_DURATION)

        if rte.pump_active and elapsed >= bcm_rte.PUMP_MAX_RUNTIME:
            self._pump_stop("rear_wash_fsr005")
            rte.set("pump_dir_active", 0)

        if cycles > rte.wash_cycles_done:
            rte.set("wash_cycles_done", cycles)
            print(f"[MODE REAR WASH] Cycle {cycles}/{bcm_rte.WASH_REAR_CYCLES}")
            self._log_sensors(f"Cycle {cycles}/{bcm_rte.WASH_REAR_CYCLES}")

        if rte.wash_cycles_done >= bcm_rte.WASH_REAR_CYCLES:
            print(f"[MODE REAR WASH] {bcm_rte.WASH_REAR_CYCLES} cycles -> OFF")
            if rte.pump_active:
                self._pump_stop("rear_wash_complet")
            self._rear_motor_stop()
            rte.set_multi(
                rear_motor_on      = False,
                rear_motor_running = False,
                pump_dir_active    = 0,
            )
            rte.crs_wiper_op    = WOP_OFF
            rte._one_shot_armed = False
            rte._freeze_pending = True
            rte._freeze_last_op = WOP_REAR_WASH
            self._enter_state(ST_OFF)

    # ── Etat REAR_WIPE ────────────────────────────────

    def _enter_rear_wipe(self):
        print(f"[MODE REAR WIPE] Moteur arriere ON -- cycles de {bcm_rte.TOUCH_DURATION*1000:.0f}ms (SRD_WW_092)")
        rte = self._rte
        rte.set("t_touch_start", time.time())
        if rte.rear_wiper_available:
            rte.set_multi(rear_motor_running=True, rear_motor_on=True)
            self._rear_motor_run()   # demarrage unique -- moteur reste ON

    def _process_rear_wipe(self):
        rte = self._rte
        if not rte.rear_wiper_available:
            self._rear_motor_stop()
            rte.set_multi(rear_motor_on=False, rear_motor_running=False)
            # SRD_WW_091 : demande ignoree si RearWiperAvailable=False
            self._enter_state(ST_OFF)
            return

        # Surveiller le levier : si relache → arreter et passer OFF
        if rte.crs_wiper_op != WOP_REAR_WIPE:
            self._rear_motor_stop()
            rte.set_multi(rear_motor_on=False, rear_motor_running=False)
            self._enter_state(ST_OFF)
            return

        # Compter les cycles (pour log) sans arreter le moteur
        elapsed = time.time() - rte.t_touch_start
        if elapsed >= bcm_rte.TOUCH_DURATION:
            # Nouveau cycle : juste reset du timer, moteur reste ON
            rte.set("t_touch_start", time.time())
            print(f"[MODE REAR WIPE] Cycle suivant (levier maintenu)")

    # ── Etat ERROR ────────────────────────────────────

    def _check_error_healing(self):
        """
        Auto-healing depuis ST_ERROR : soft-reset interne.

        Utilise self._error_origin (capture a l entree en ST_ERROR) au lieu
        des flags RTE front_motor_error / rear_motor_error / pump_error.
        Ces flags peuvent etre remis a False par la Platform via Redis (T38b
        post_test) pendant que le WSM est encore en ST_ERROR, ce qui rendait
        le healing inoperant (cas 4 : aucune erreur active detectee).

        _error_origin est une variable PRIVEE de bcm_application, immuable
        tant que le soft-reset n a pas eu lieu.

        Condition stricte : les DEUX sources doivent etre sous le seuil :
          1) courant hardware ADS
          2) xcp_*_raw_ma (valeur brute memory.json, toujours a jour)
        """
        rte          = self._rte
        now          = time.time()
        thresh_ma    = int(bcm_rte.OVERCURRENT_THRESH * 1000)
        pump_thresh_ma = int(bcm_rte.PUMP_OVERCURRENT_THRESH * 1000)

        origin = self._error_origin  # capture locale : ne peut pas changer sous nos pieds

        # ── Cas 1 : surcourant moteur avant (B2001) ──────────────────────────
        if origin == "front":
            hw_ok  = rte.motor_current_a < bcm_rte.OVERCURRENT_THRESH
            xcp_ok = getattr(rte, "xcp_front_raw_ma", 0) < thresh_ma
            if hw_ok and xcp_ok:
                if rte._t_heal_error_front == 0.0:
                    rte.set("_t_heal_error_front", now)
                    print("[HEAL-ERROR] B2001 : courant HW + XCP OK, "
                          "timer healing demarre (1s)")
                elif now - rte._t_heal_error_front >= bcm_rte.HEAL_DELAY:
                    rte.set("_t_heal_error_front", 0.0)
                    self._dtc.set_inactive("B2001")
                    rte.set_multi(front_motor_error=False,
                                  _rest_contact_b2009_active=False)
                    self._error_origin = None
                    target = rte.prev_state if rte.prev_state not in (
                        ST_ERROR, ST_OFF, ST_DIAG) else ST_OFF
                    print(f"[HEAL-ERROR] B2001 RESOLVED : HW + XCP < seuil "
                          f"pendant {bcm_rte.HEAL_DELAY}s -> soft-reset -> {target}")
                    self._enter_state(target)
            else:
                if rte._t_heal_error_front != 0.0:
                    reason = []
                    if not hw_ok:
                        reason.append(f"HW={rte.motor_current_a:.2f}A")
                    if not xcp_ok:
                        reason.append(f"XCP={getattr(rte, 'xcp_front_raw_ma', 0)}mA")
                    print(f"[HEAL-ERROR] B2001 : timer reset ({', '.join(reason)} >= seuil)")
                rte.set("_t_heal_error_front", 0.0)

        # ── Cas 2 : surcourant moteur arriere (B2002) ────────────────────────
        elif origin == "rear":
            hw_ok  = rte.motor_current_a < bcm_rte.OVERCURRENT_THRESH
            xcp_ok = getattr(rte, "xcp_rear_raw_ma", 0) < thresh_ma
            if hw_ok and xcp_ok:
                if rte._t_heal_error_rear == 0.0:
                    rte.set("_t_heal_error_rear", now)
                    print("[HEAL-ERROR] B2002 : courant HW + XCP OK, "
                          "timer healing demarre (1s)")
                elif now - rte._t_heal_error_rear >= bcm_rte.HEAL_DELAY:
                    rte.set("_t_heal_error_rear", 0.0)
                    self._dtc.set_inactive("B2002")
                    rte.set_multi(rear_motor_error=False,
                                  _rest_contact_b2009_active=False)
                    self._error_origin = None
                    target = rte.prev_state if rte.prev_state not in (
                        ST_ERROR, ST_OFF, ST_DIAG) else ST_OFF
                    print(f"[HEAL-ERROR] B2002 RESOLVED : HW + XCP < seuil "
                          f"pendant {bcm_rte.HEAL_DELAY}s -> soft-reset -> {target}")
                    self._enter_state(target)
            else:
                if rte._t_heal_error_rear != 0.0:
                    reason = []
                    if not hw_ok:
                        reason.append(f"HW={rte.motor_current_a:.2f}A")
                    if not xcp_ok:
                        reason.append(f"XCP={getattr(rte, 'xcp_rear_raw_ma', 0)}mA")
                    print(f"[HEAL-ERROR] B2002 : timer reset ({', '.join(reason)} >= seuil)")
                rte.set("_t_heal_error_rear", 0.0)

        # ── Cas 3b : désactivation permanente pompe (fault persistante au 2ème essai) ──
        elif origin == "pump_disabled":
            # Healing : pump_disabled_permanent redevient False
            # (posé par _check_pump_overcurrent quand courant OK pendant HEAL_DELAY)
            healed = not getattr(rte, "pump_disabled_permanent", True)
            if healed:
                if rte._t_heal_error_pump == 0.0:
                    rte.set("_t_heal_error_pump", now)
                    print("[HEAL-ERROR] pump_disabled : pump_disabled_permanent=False détecté, "
                          "timer confirmation demarre (1s)")
                elif now - rte._t_heal_error_pump >= bcm_rte.HEAL_DELAY:
                    rte.set("_t_heal_error_pump", 0.0)
                    rte.set_multi(pump_error=False,
                                  pump_overcurrent_error=False,
                                  _rest_contact_b2009_active=False)
                    self._error_origin = None
                    target = rte.prev_state if rte.prev_state not in (
                        ST_ERROR, ST_OFF, ST_DIAG) else ST_OFF
                    print(f"[HEAL-ERROR] pump_disabled RESOLVED : fault guérie, "
                          f"retry counter reset -> soft-reset -> {target}")
                    self._enter_state(target)
            else:
                if rte._t_heal_error_pump != 0.0:
                    print("[HEAL-ERROR] pump_disabled : timer reset "
                          "(pump_disabled_permanent toujours True)")
                rte.set("_t_heal_error_pump", 0.0)

        # ── Cas 3 : surcourant pompe (B2003) ou depassement runtime (B2008) ──
        elif origin == "pump":
            hw_ok      = getattr(rte, "pump_current_a", 0.0) < bcm_rte.PUMP_OVERCURRENT_THRESH
            xcp_ok     = getattr(rte, "xcp_pump_raw_ma", 0) < pump_thresh_ma
            # B2008 : bloquer le healing tant que pump_runtime_s > 0 dans memory.json.
            # Sans ce blocage, la pompe repart immediatement apres le soft-reset
            # car xcp_server_bcm._check_b2008() relit la valeur et reinjecte
            # xcp_pump_cmd=1 des que pump_active=False.
            runtime_ok = getattr(rte, "xcp_pump_raw_runtime", 0) == 0
            if hw_ok and xcp_ok and runtime_ok:
                if rte._t_heal_error_pump == 0.0:
                    rte.set("_t_heal_error_pump", now)
                    print("[HEAL-ERROR] B2003 : courant pompe HW + XCP OK, "
                          "timer healing demarre (1s)")
                elif now - rte._t_heal_error_pump >= bcm_rte.HEAL_DELAY:
                    rte.set("_t_heal_error_pump", 0.0)
                    self._dtc.set_inactive("B2003")
                    self._dtc.set_inactive("B2008")
                    rte.set_multi(pump_error=False,
                                  pump_overcurrent_error=False,
                                  pump_runtime_error=False,
                                  _rest_contact_b2009_active=False)
                    self._error_origin = None
                    target = rte.prev_state if rte.prev_state not in (
                        ST_ERROR, ST_OFF, ST_DIAG) else ST_OFF
                    print(f"[HEAL-ERROR] B2003 RESOLVED : pompe HW + XCP < seuil "
                          f"pendant {bcm_rte.HEAL_DELAY}s -> soft-reset -> {target}")
                    self._enter_state(target)
            else:
                if rte._t_heal_error_pump != 0.0:
                    reason = []
                    if not hw_ok:
                        reason.append(f"HW={getattr(rte, 'pump_current_a', 0):.2f}A")
                    if not xcp_ok:
                        reason.append(f"XCP={getattr(rte, 'xcp_pump_raw_ma', 0)}mA")
                    if not runtime_ok:
                        reason.append(f"runtime={getattr(rte, 'xcp_pump_raw_runtime', 0)}s>0")
                    print(f"[HEAL-ERROR] B2003/B2008 : timer reset ({', '.join(reason)})")
                rte.set("_t_heal_error_pump", 0.0)

        # ── Cas 4 : B2009 / B2006 — erreur lame seule (wiper_fault) ────────────
        elif origin == "wiper":
            # Healing : xcp_rest_contact_raw revenu a True (signal actif)
            # ET confirme pendant HEAL_DELAY (1s) → soft-reset vers prev_state.
            # Si le defaut vient du GPIO hardware (pas d injection XCP),
            # le healing n a pas lieu ici — il faut un reset Platform explicite.
            xcp_contact_ok = bool(getattr(rte, "xcp_rest_contact_raw", True))
            if xcp_contact_ok:
                if rte._t_heal_error_front == 0.0:
                    rte.set("_t_heal_error_front", now)
                    print("[HEAL-ERROR] B2009/B2006 : rest_contact_signal=1 (XCP OK), "
                          "timer healing demarre (1s)")
                elif now - rte._t_heal_error_front >= bcm_rte.HEAL_DELAY:
                    rte.set("_t_heal_error_front", 0.0)
                    self._dtc.set_inactive("B2009")
                    self._dtc.set_inactive("B2006")
                    rte.set_multi(wiper_fault=False,
                                  _rest_contact_b2009_active=False)
                    self._error_origin = None
                    target = rte.prev_state if rte.prev_state not in (
                        ST_ERROR, ST_OFF, ST_DIAG) else ST_OFF
                    print(f"[HEAL-ERROR] B2009/B2006 RESOLVED : contact OK pendant "
                          f"{bcm_rte.HEAL_DELAY}s -> soft-reset -> {target}")
                    self._enter_state(target)
            else:
                if rte._t_heal_error_front != 0.0:
                    print("[HEAL-ERROR] B2009/B2006 : timer reset "
                          "(rest_contact_signal toujours 0)")
                rte.set("_t_heal_error_front", 0.0)

        # ── Cas 5 : erreur generale ou origin inconnue → pas de healing auto ─
        elif origin == "general" or origin is None:
            # general / None : LIN timeout, watchdog, erreurs combinees
            # -> seul un reset Platform explicite (bcm_error_reset) peut sortir.
            rte.set_multi(_t_heal_error_front=0.0,
                          _t_heal_error_rear=0.0,
                          _t_heal_error_pump=0.0)

        # ── Cas 6 : 3 NACKs 0x01 WC → healing sur exactement 3 ACK consécutifs ─
        elif origin == "wc_nack":
            # wc_ack_heal_count est incrémenté dans _check_wc_ack à chaque ACK reçu
            # en ST_ERROR avec _error_origin=="wc_nack".
            # La boucle T-CAN-WC doit rester active (wc_available=True) pour recevoir
            # les 0x202 ACK du simulateur.
            ack_count = rte.wc_ack_heal_count
            if ack_count >= 3:
                rte.wc_ack_heal_count = 0
                rte.wc_nack_consecutive = 0
                self._error_origin = None
                # Couper wc_available APRÈS le heal : stoppe la boucle T-CAN-WC proprement
                rte.set("wc_available", False)
                target = rte.prev_state if rte.prev_state not in (
                    ST_ERROR, ST_OFF, ST_DIAG) else ST_OFF
                print(f"[HEAL-WC_NACK] 3 ACK consécutifs reçus → wc_available=False → soft-reset → {target}")
                self._enter_state(target)


    def _enter_error(self):
        rte = self._rte

        # Arrêt sélectif selon l'origine de l'erreur :
        #
        #   Erreurs côté MOTEUR/LAME (pompe non affectée → continue) :
        #     - front_motor_error  : B2001 surcourant moteur avant
        #     - rear_motor_error   : B2002 surcourant moteur arrière
        #     - wiper_fault        : B2006 contact repos absent / B2009 contact bloqué
        #
        #   Erreurs côté POMPE seule (moteur non affecté → continue) :
        #     - pump_error         : B2003 surcourant pompe / B2008 dépassement runtime
        #
        #   Erreur générale (LIN timeout, watchdog…) → arrêt total
        #
        motor_side = rte.front_motor_error or rte.rear_motor_error or rte.wiper_fault
        pump_only  = (rte.pump_error or getattr(rte, "pump_disabled_permanent", False)) and not motor_side

        # Capturer l origine au moment de l entree en ST_ERROR.
        # _error_origin est une variable INTERNE a bcm_application — elle ne
        # peut pas etre effacee par Redis. _check_error_healing l utilise
        # pour savoir quel courant surveiller, meme si la Platform a remis
        # rear_motor_error / front_motor_error / pump_error a False via Redis.
        if self._error_origin == "pump_disabled":
            # Origine déjà capturée par _enter_front/rear_wash (désactivation permanente) — ne pas écraser
            pass
        elif rte.front_motor_error and not rte.rear_motor_error and not rte.pump_error:
            self._error_origin = "front"
        elif rte.rear_motor_error and not rte.front_motor_error and not rte.pump_error:
            self._error_origin = "rear"
        elif pump_only:
            self._error_origin = "pump"
        elif rte.wiper_fault and not rte.pump_error and not rte.front_motor_error and not rte.rear_motor_error:
            # B2009 (contact repos bloque) ou B2006 (contact absent) : erreur lame seule
            self._error_origin = "wiper"
        elif self._error_origin == "wc_nack":
            # Origine déjà capturée par _check_wc_ack (3 NACKs 0x01) — ne pas écraser
            pass
        else:
            self._error_origin = "general"
        print(f"[MODE ERROR] Origine capturee : _error_origin={self._error_origin}")

        if pump_only:
            # ── Erreur isolée pompe : arrêt pompe, moteur maintenu ────────
            print("[MODE ERROR] Erreur pompe -> arret pompe seule (moteur maintenu)")
            self._pump_stop("error_pump_only")
            rte.set_multi(
                pump_dir_active           = 0,
                _rest_contact_stuck_start = 0.0,   # reset timer B2009 accumulé avant ERROR
                _rest_contact_last_state  = -1,
                _front_blade_cycles       = 0,
                front_blade_cycles        = 0,
                # Inhiber B2009 pendant ST_ERROR pump_only :
                # le moteur avant peut rester actif (maintenu intentionnellement)
                # mais le contact repos n'est plus fiable → ne pas détecter STUCK.
                # Sera remis à False lors du reset ERROR→OFF (bcm_error_reset).
                _rest_contact_b2009_active = True,
            )
        elif motor_side and not rte.pump_error:
            # ── Erreur isolée moteur/lame : arrêt moteur, pompe maintenue ─
            print("[MODE ERROR] Erreur moteur/lame -> arret moteur seul (pompe maintenue)")
            self._front_motor_stop()
            self._rear_motor_stop()
            rte.set_multi(
                front_motor_on            = False,
                front_motor_speed         = 0,
                front_blade_moving        = False,
                rear_motor_on             = False,
                rear_motor_running        = False,
                _rest_contact_stuck_start = 0.0,
                _rest_contact_last_state  = -1,
                _front_blade_cycles       = 0,
                front_blade_cycles        = 0,
            )
        else:
            # ── Erreur générale (LIN timeout, watchdog, erreur inconnue) ──
            print("[MODE ERROR] Erreur -> arret tous actionneurs")
            self._stop_all()
            rte.set_multi(
                front_motor_on            = False,
                front_motor_speed         = 0,
                front_blade_moving        = False,
                rear_motor_on             = False,
                rear_motor_running        = False,
                pump_dir_active           = 0,
                _rest_contact_stuck_start = 0.0,   # reset timer B2009
                _rest_contact_last_state  = -1,    # reset etat precedent B2009
                _front_blade_cycles       = 0,     # reset compteur interne
                front_blade_cycles        = 0,     # reset compteur public Redis
            )

        # Annuler tout test DIAG en cours : le test est interrompu par l'erreur.
        # Sans ce reset, T-DIAG detecte l'expiration du timer et appelle
        # _stop_test() -> _enter_state(ST_OFF), court-circuitant le healing.
        if rte._test_active:
            print(f"[MODE ERROR] Test DIAG 0x{rte._test_routine:04X} interrompu par erreur -> _test_active=False")
            # Rain Sim (0x0205) : remettre crs_wiper_op=WOP_OFF et lin_op_locked=False.
            # Sans ce reset, le healing ERROR->OFF voit encore crs_wiper_op=WOP_AUTO
            # et _process_off_state() declenche immediatement ST_AUTO.
            if rte._test_routine == 0x0205:
                rte.set_multi(
                    crs_wiper_op   = WOP_OFF,
                    lin_op_locked  = False,
                    rain_intensity = 0,
                )
                print("[MODE ERROR] Rain Sim 0x0205 : crs_wiper_op=WOP_OFF, lin_op_locked=False, rain=0")
            rte.set("_test_active", False)

    # ── Marche arriere ────────────────────────────────

    # Etats consideres comme "Front wiping active" (SRD_WW_060)
    _STATES_FRONT_ACTIVE = {ST_SPEED1, ST_SPEED2, ST_AUTO, ST_TOUCH, ST_DIAG}

    def _handle_reverse_intermittent(self):
        """
        SRD_WW_060 : If ReverseGear=TRUE and Front wiping active
                     → Rear wiper shall perform one cycle every 1700ms.

        Implementation :
          - "Front wiping active" = state in {ST_SPEED1, ST_SPEED2, ST_AUTO, ST_TOUCH}
          - Moteur arriere demarre immediatement (ON continu, comme ST_REAR_WIPE)
          - Timer 1700ms : reset a chaque cycle, moteur ne s'arrete PAS entre cycles
          - reverse_gear → False : arret moteur immediat
          - Front wiper quitte un etat actif : arret moteur immediat
        """
        rte = self._rte

        # ── Cas 1 : marche arriere desactivee ──────────────────────────
        if not rte.reverse_gear:
            if rte._reverse_active:
                self._rear_motor_stop()
                rte.set_multi(
                    rear_motor_on       = False,
                    rear_motor_running  = False,
                    _reverse_active     = False,
                    _reverse_cycle_num  = 0,
                    t_rear_last         = 0.0,
                )
                print("[SRD_WW_060] Moteur arriere OFF (ReverseGear=False)")
                self._tcp.send(rte); self._ws.send(rte)
            return

        # ── Cas 2 : front wiper n'est plus actif (levier relache) ──────
        if rte.state not in self._STATES_FRONT_ACTIVE:
            if rte._reverse_active:
                self._rear_motor_stop()
                rte.set_multi(
                    rear_motor_on       = False,
                    rear_motor_running  = False,
                    _reverse_active     = False,
                    _reverse_cycle_num  = 0,
                    t_rear_last         = 0.0,
                )
                print("[SRD_WW_060] Moteur arriere OFF (front wiper inactif)")
                self._tcp.send(rte); self._ws.send(rte)
            return

        # ── Cas 3 : RearWiperAvailable requis ──────────────────────────
        if not rte.rear_wiper_available:
            return

        # ── Cas 4 : premiere detection → demarrage moteur arriere ──────
        if not rte._reverse_active:
            rte._reverse_active    = True
            rte._reverse_cycle_num = 1
            rte.set("t_rear_last", time.time())
            self._rear_motor_run()
            rte.set_multi(
                rear_motor_on      = True,
                rear_motor_running = True,
            )
            print(f"[SRD_WW_060] Moteur arriere ON (front={rte.state} | reverse=True)")
            self._tcp.send(rte); self._ws.send(rte)
            return

        # ── Cas 5 : moteur deja ON → cycle 1700ms, impulsion OFF 50ms ──
        # CORRECTION T43 : une brève impulsion OFF (50ms) est générée à
        # chaque nouvelle période pour que la détection de front True→False
        # dans test_cases.py puisse mesurer l'intervalle inter-cycles.
        # Sans cette impulsion, rear_motor_on reste True en permanence et
        # le test ne détecte jamais de cycle → TIMEOUT.
        now = time.time()
        if now - rte.t_rear_last >= bcm_rte.REVERSE_REAR_PERIOD:
            rte.set("t_rear_last", now)
            rte._reverse_cycle_num += 1
            # --- impulsion OFF (250ms) ---
            # > période poll Redis (200ms) : garantit qu'au moins un poll
            # tombe dans la fenêtre False. Validé PASS en test réel.
            self._rear_motor_stop()
            rte.set_multi(rear_motor_on=False, rear_motor_running=False)
            self._tcp.send(rte); self._ws.send(rte)
            time.sleep(0.250)          # 250ms OFF > 200ms Redis poll period
            # --- retour ON ---
            self._rear_motor_run()
            rte.set_multi(rear_motor_on=True, rear_motor_running=True)
            self._tcp.send(rte); self._ws.send(rte)


    # ==================================================
    # SECTION E -- SURVEILLANCE (PUMP GUARD + ADS1115)
    # ==================================================

    # NOTE : _check_blade_position (B2006) supprime -- role a clarifier avec
    # l'encadrant avant reactivation. L'entree B2006 reste dans dtc_database.json
    # mais aucun code ne l'arme pour le moment.

    def _check_rest_contact_stuck(self):
        # ---------------------------------------------------------------
        # B2009 : Rest Contact Failure -- signal contact repos bloqué (stuck)
        #
        # Le contact repos est TOUJOURS câblé au BCM (GPIO26), CAS A et CAS B.
        # Seule la référence moteur change selon le cas.
        #
        # CAS A (wc_available=False) -- référence moteur = GPIO/relais BCM :
        #   STUCK CLOSED : moteur avant EN MARCHE (front_motor_on=True via relais)
        #                  ET aucun front montant (False->True) sur contact repos
        #                  depuis >REST_STUCK_DELAY (3s) -> contact bloqué ouvert
        #   STUCK OPEN   : moteur A L'ARRET ET contact repos = EN MOUVEMENT (GPIO=1)
        #                  depuis >REST_STUCK_DELAY -> contact bloqué fermé
        #
        # CAS B (wc_available=True) -- référence moteur = (CurrentSpeed>0 ET
        # BladePosition>0) dans la trame 0x201 :
        #   STUCK CLOSED : CurrentSpeed>0 ET BladePosition>0 (WC confirme moteur
        #                  en marche ET lame en mouvement) ET aucun front montant
        #                  sur contact repos GPIO depuis >3s -> contact bloqué ouvert
        #   STUCK OPEN   : BCM ne commande aucun moteur avant/arriere ET contact
        #                  repos GPIO = EN MOUVEMENT depuis >3s -> contact bloqué
        #                  fermé. BladePosition n'est PAS consultee ici (le
        #                  capteur GPIO suffit pour detecter l'incoherence).
        #
        # Dans les deux cas, les états REAR (WASH_REAR, REAR_WIPE) sont exclus :
        # le contact repos est un capteur du moteur AVANT uniquement.
        # ---------------------------------------------------------------
        rte = self._rte
        if not REST_CONTACT_HARDWARE_PRESENT:
            return
        if rte._rest_contact_b2009_active:
            # ── Healing B2009 même si garde active ──────────────────────────
            # Si B2009 est ACTIVE et que le moteur tourne + contact fonctionne
            # normalement pendant HEAL_DELAY (1s) -> DTC INACTIVE
            if rte.state not in (ST_ERROR, ST_DIAG, ST_OFF):
                now  = time.time()
                blade_moving = self._read_rest_contact()
                _active_states = {ST_SPEED1, ST_SPEED2, ST_AUTO, ST_TOUCH, ST_WASH_FRONT}
                if not rte.wc_available:
                    front_motor_running = (rte.state in _active_states) and rte.front_motor_on
                else:
                    wc_speed  = getattr(rte, "front_motor_speed", 0)
                    blade_pos = getattr(rte, "wc_blade_position", -1)
                    front_motor_running = (rte.state in _active_states) and (wc_speed > 0) and (blade_pos > 0)
                # Contact OK = moteur tourne ET lame se déplace (front montant détecté)
                contact_ok = front_motor_running and (blade_moving or rte._rest_contact_prev is False)
                if contact_ok and self._dtc.get_status("B2009") == "ACTIVE":
                    if rte._t_heal_b2009 == 0.0:
                        rte._t_heal_b2009 = now
                        print("[HEAL] B2009 : contact OK, timer healing demarre (1s)")
                    elif now - rte._t_heal_b2009 >= bcm_rte.HEAL_DELAY:
                        rte._t_heal_b2009 = 0.0
                        rte.set_multi(_rest_contact_b2009_active=False, wiper_fault=False)
                        self._dtc.set_inactive("B2009")
                        print(f"[HEAL] B2009 INACTIVE : contact repos OK pendant {bcm_rte.HEAL_DELAY}s")
                else:
                    rte._t_heal_b2009 = 0.0
            return
        if rte.state == ST_ERROR:
            return
        # ST_DIAG normal : B2009 inhibe (DoIP pilote les actionneurs)
        # ST_DIAG : B2009 inhibe pour toutes les routines actionneurs (0x0201..0x0204)
        # Le signal contact repos peut etre un signal horloge fixe (NE555 simule)
        # qui ne genere pas de front montant -> faux STUCK CLOSED inevitable.
        # Exception : routine 0x0205 (rain sim) avec injection Platform Redis uniquement.
        # Routine 0x0202 (rear motor) : pas de capteur contact repos -> inhibe aussi.
        _rain_sim_active = (rte._test_active and rte._test_routine == 0x0205)
        if rte.state == ST_DIAG and not _rain_sim_active:
            return
        if rte.state == ST_DIAG and _rain_sim_active and not rte.rest_contact_sim_active:
            return
        # En mode simulation (rest_contact_sim_active=True) :
        #   - rest_contact_sim=True  : lame EN MOUVEMENT simulée → cycles normaux
        #                              → pas un défaut (T22 : cycles bloqués) → return
        #   - rest_contact_sim=False : lame AU REPOS figée simulée → AUCUN cycle
        #                              → défaut intentionnel (T_B2009_CAN/CASA) → continuer
        # Sans cette distinction, T22 déclenche B2009 avant FSR_005 à 5s.
        if rte.rest_contact_sim_active and rte.rest_contact_sim:
            return

        blade_moving = self._read_rest_contact()   # True=lame EN MOUVEMENT / False=AU REPOS
        now          = time.time()

        # Contact repos : moteur AVANT uniquement
        motor_running_any = rte.state in (ST_WASH_REAR, ST_REAR_WIPE)

        # ── Détermination état moteur avant selon CAS A ou CAS B ─────────────
        # Rain sim (0x0205) en ST_DIAG : traiter comme ST_AUTO pour la detection moteur
        _rain_sim_diag = (rte.state == ST_DIAG and rte._test_active and rte._test_routine == 0x0205)
        _active_states = {ST_SPEED1, ST_SPEED2, ST_AUTO, ST_TOUCH, ST_WASH_FRONT}

        if not rte.wc_available:
            # CAS A : état moteur = relais GPIO BCM
            front_motor_running = (rte.state in _active_states or _rain_sim_diag) \
                                  and rte.front_motor_on
            motor_running_any = front_motor_running or motor_running_any
        else:
            # CAS B : état moteur = CurrentSpeed ET BladePosition reçus dans 0x201
            # Les deux informations doivent confirmer que le moteur tourne ET
            # que la lame bouge reellement (ET logique).
            wc_speed   = getattr(rte, "front_motor_speed", 0)     # Byte CurrentSpeed
            blade_pos  = getattr(rte, "wc_blade_position", -1)    # Byte BladePosition
            # blade_pos == -1 : aucune trame 0x201 recue -> on ne sait rien,
            # on considere le moteur comme a l'arret pour B2009 (pas de detection).
            blade_pos_ok = (blade_pos > 0)
            front_motor_running = (rte.state in _active_states or _rain_sim_diag) \
                                  and (wc_speed > 0) and blade_pos_ok
            motor_running_any = front_motor_running or motor_running_any

        # ── STUCK CLOSED : moteur AVANT EN MARCHE + aucun mouvement détecté ──
        if front_motor_running:
            # NE555 astable : on ne se base PAS sur les fronts GPIO (désynchronisation
            # inter-threads T-WSM/T-PUMP inévitable). On utilise uniquement le timestamp
            # du dernier cycle détecté par _track_blade_cycle (T-WSM).
            #
            # Logique :
            #   _t_last_blade_cycle == 0.0  → moteur vient de démarrer, aucun cycle encore
            #                                 → démarrer le timer d'attente du premier cycle
            #   _t_last_blade_cycle > 0.0   → au moins un cycle détecté
            #                                 → timer = temps écoulé depuis ce dernier cycle
            #   Si timer >= REST_STUCK_DELAY → contact bloqué → B2009
            #
            # REST_STUCK_DELAY (3s) >> période NE555 (1.76s) : un contact vivant
            # fournira toujours un cycle avant l'expiration.

            if rte._t_last_blade_cycle == 0.0:
                # Aucun cycle encore : démarrer le timer d'attente
                if rte._rest_contact_stuck_start == 0.0:
                    rte._rest_contact_stuck_start = now
                elif (now - rte._rest_contact_stuck_start) >= bcm_rte.REST_STUCK_DELAY:
                    cas = "CAS B" if rte.wc_available else "CAS A"
                    print(f"[B2009][{cas}] Contact repos STUCK CLOSED "
                          f"(moteur avant marche + aucun mouvement depuis >{bcm_rte.REST_STUCK_DELAY}s) -> ERROR")
                    self._dtc.set_active("B2009", rte.make_snapshot())
                    rte.set_multi(
                        _rest_contact_b2009_active = True,
                        _rest_contact_stuck_start  = 0.0,
                        wiper_fault                = True,
                    )
                    self._enter_state(ST_ERROR)
                    return
            else:
                # Au moins un cycle détecté : vérifier le délai depuis le dernier cycle
                rte._rest_contact_stuck_start = 0.0  # timer premier cycle plus utile
                if (now - rte._t_last_blade_cycle) >= bcm_rte.REST_STUCK_DELAY:
                    cas = "CAS B" if rte.wc_available else "CAS A"
                    print(f"[B2009][{cas}] Contact repos STUCK CLOSED "
                          f"(moteur avant marche + aucun mouvement depuis >{bcm_rte.REST_STUCK_DELAY}s) -> ERROR")
                    self._dtc.set_active("B2009", rte.make_snapshot())
                    rte.set_multi(
                        _rest_contact_b2009_active = True,
                        _rest_contact_stuck_start  = 0.0,
                        wiper_fault                = True,
                    )
                    self._enter_state(ST_ERROR)
                    return

        # ── STUCK OPEN : moteur A L'ARRET + contact toujours EN MOUVEMENT ────
        # Le BCM ne commande aucun moteur avant/arriere mais GPIO26 indique
        # encore "lame en mouvement" -> physiquement impossible, contact
        # bloque en position ouverte. Identique en CAS A et CAS B.
        # On ne consulte PAS BladePosition ici : elle sert uniquement au
        # STUCK CLOSED pour confirmer que le moteur tourne reellement.
        # ST_OFF exclu : en mode OFF le moteur est arrete volontairement, un signal
        # contact repos fixe a 1 (ex: horloge simulee) ne constitue pas un defaut.
        elif (not motor_running_any) and blade_moving and rte.state != ST_OFF:
            if rte._rest_contact_stuck_start == 0.0:
                rte._rest_contact_stuck_start = now
                rte._rest_contact_last_state  = 1
            elif rte._rest_contact_last_state == 1:
                if (now - rte._rest_contact_stuck_start) >= bcm_rte.REST_STUCK_DELAY:
                    cas = "CAS B" if rte.wc_available else "CAS A"
                    print(f"[B2009][{cas}] Contact repos STUCK OPEN "
                          f"(moteur arrêté + contact EN MOUVEMENT depuis >{bcm_rte.REST_STUCK_DELAY}s) -> ERROR")
                    self._dtc.set_active("B2009", rte.make_snapshot())
                    rte.set_multi(
                        _rest_contact_b2009_active = True,
                        _rest_contact_stuck_start  = 0.0,
                        wiper_fault                = True,   # erreur lame seule -> pompe non affectee
                    )
                    self._enter_state(ST_ERROR)
                    return

        # ── État cohérent ou moteur arrière -> reset timers ──────────────────
        else:
            if not front_motor_running:
                rte._rest_contact_stuck_start = 0.0
                rte._rest_contact_last_state  = -1
            # ── Healing B2009 : contact fonctionne normalement avec moteur actif ──
            # Si B2009 est ACTIVE et que le contact génère des transitions normales
            # (ni STUCK CLOSED ni STUCK OPEN) pendant HEAL_DELAY (1s) -> INACTIVE
            if front_motor_running and self._dtc.get_status("B2009") == "ACTIVE":
                if rte._t_heal_b2009 == 0.0:
                    rte._t_heal_b2009 = now
                    print("[HEAL] B2009 : contact OK, timer healing demarre (1s)")
                elif now - rte._t_heal_b2009 >= bcm_rte.HEAL_DELAY:
                    rte._t_heal_b2009 = 0.0
                    rte.set("_rest_contact_b2009_active", False)
                    self._dtc.set_inactive("B2009")
                    print(f"[HEAL] B2009 INACTIVE : contact repos OK pendant {bcm_rte.HEAL_DELAY}s")
            else:
                rte._t_heal_b2009 = 0.0

    def _check_pump_overcurrent(self):
        """
        Détection des défauts électriques pompe → DTC B2003 (Pump Overcurrent/Electrical Fault).

        Quatre types de fautes détectées :
          1. OVERCURRENT   : courant > PUMP_OVERCURRENT_THRESH (0.8A) pendant 300ms
          2. OPEN_LOAD     : chute brusque du courant (delta > 0.3A en un cycle, courant final < 0.35A)
          3. SHORT_GND     : courant quasi nul (< 0.15A) alors que pompe active, pendant 300ms
          4. VARIABLE_LOAD : amplitude (max-min) > 0.4A sur fenêtre glissante 500ms

        Scénario de réaction :
          - Faute détectée → arrêt immédiat pompe + B2003 CONFIRMED
          - Pompe reste OFF pour la requête courante
          - Quand signal wash revient à 0, compteur retry autorise réactivation
          - Si faute persiste au 2ème essai → pump_disabled_permanent = True
          - Si faute guérie → fonctionnement normal repris
        """
        rte = self._rte
        now = time.time()

        if not rte.pump_active:
            # Reset tous les timers et historiques quand pompe inactive
            rte.set("_pump_overcurrent_start", 0.0)
            rte._pump_short_gnd_start  = 0.0
            rte._pump_prev_current     = 0.0
            rte._pump_current_history  = []
            return

        current     = rte.pump_current_a          # courant ACS712 réel
        xcp_current = getattr(rte, "xcp_pump_current_a", 0.0)
        # OR : courant réel OU courant injecté XCP
        current = max(current, xcp_current)

        # ══════════════════════════════════════════════════════════════
        # 1. DÉTECTION OVERCURRENT (comportement nominal conservé)
        # ══════════════════════════════════════════════════════════════
        if current > bcm_rte.PUMP_OVERCURRENT_THRESH:
            if rte._pump_overcurrent_start == 0.0:
                rte.set("_pump_overcurrent_start", now)
                hw_c = rte.pump_current_a
                if xcp_current > bcm_rte.PUMP_OVERCURRENT_THRESH and hw_c <= bcm_rte.PUMP_OVERCURRENT_THRESH:
                    print(f"[XCP-INJECT] B2003 : surcourant XCP pompe {xcp_current:.2f}A > {bcm_rte.PUMP_OVERCURRENT_THRESH}A (hardware={hw_c:.2f}A) — timer 300ms démarré")
            elif (now - rte._pump_overcurrent_start) > bcm_rte.PUMP_OVERCURRENT_DELAY:
                rte.set("_pump_overcurrent_start", 0.0)
                print(f"[B2003] Pump Electrical Fault : courant={current:.2f}A -> arret pompe")
                self._trigger_pump_b2003("OVERCURRENT", current)
            self._update_pump_current_history(rte, now, current)
            rte._pump_prev_current = current
            return

        # Courant dans plage normale → reset timer overcurrent
        rte.set("_pump_overcurrent_start", 0.0)

        # ══════════════════════════════════════════════════════════════
        # 2. DÉTECTION OPEN LOAD
        #    Chute brusque : courant précédent >= 0.5A ET courant actuel < 0.35A
        #    ET delta de chute > PUMP_OPEN_LOAD_DROP (0.3A)
        # ══════════════════════════════════════════════════════════════
        prev_current = getattr(rte, "_pump_prev_current", 0.0)
        if (prev_current >= 0.5
                and current < bcm_rte.PUMP_OPEN_LOAD_MAX
                and (prev_current - current) >= bcm_rte.PUMP_OPEN_LOAD_DROP):
            print(f"[B2003] Pump Electrical Fault : courant={current:.2f}A -> arret pompe")
            self._trigger_pump_b2003("OPEN_LOAD", current)
            rte._pump_prev_current = current
            self._update_pump_current_history(rte, now, current)
            return

        # ══════════════════════════════════════════════════════════════
        # 3. DÉTECTION SHORT TO GND
        #    Courant quasi nul (< PUMP_SHORT_GND_THRESH = 0.15A) pendant 300ms
        # ══════════════════════════════════════════════════════════════
        if current < bcm_rte.PUMP_SHORT_GND_THRESH:
            short_start = getattr(rte, "_pump_short_gnd_start", 0.0)
            if short_start == 0.0:
                rte._pump_short_gnd_start = now
                print(f"[B2003] Pump Electrical Fault : courant={current:.2f}A — timer 300ms démarré")
            elif (now - short_start) >= bcm_rte.PUMP_SHORT_GND_DELAY:
                rte._pump_short_gnd_start = 0.0
                print(f"[B2003] Pump Electrical Fault : courant={current:.2f}A -> arret pompe")
                self._trigger_pump_b2003("SHORT_GND", current)
                rte._pump_prev_current = current
                self._update_pump_current_history(rte, now, current)
                return
        else:
            rte._pump_short_gnd_start = 0.0

        # ══════════════════════════════════════════════════════════════
        # 4. DÉTECTION VARIABLE LOAD
        #    Amplitude (max-min) > PUMP_VARIABLE_RANGE (0.4A) sur fenêtre 500ms
        # ══════════════════════════════════════════════════════════════
        self._update_pump_current_history(rte, now, current)
        history = getattr(rte, "_pump_current_history", [])
        if len(history) >= bcm_rte.PUMP_VARIABLE_SAMPLES:
            values = [v for (_, v) in history]
            amp = max(values) - min(values)
            if amp >= bcm_rte.PUMP_VARIABLE_RANGE:
                print(f"[B2003] Pump Electrical Fault : courant={current:.2f}A -> arret pompe")
                self._trigger_pump_b2003("VARIABLE_LOAD", current)
                rte._pump_prev_current = current
                return

        # ══════════════════════════════════════════════════════════════
        # Courant pompe OK : healing B2003
        # ══════════════════════════════════════════════════════════════
        if self._dtc.get_status("B2003") == "ACTIVE" and rte.pump_active:
            if rte._t_heal_pump == 0.0:
                rte._t_heal_pump = now
                print("[HEAL] B2003 : courant pompe OK, timer healing demarre (1s)")
            elif now - rte._t_heal_pump >= bcm_rte.HEAL_DELAY:
                rte._t_heal_pump = 0.0
                self._dtc.set_inactive("B2003")
                rte.set("pump_fault_type", "NONE")
                # Healing complet : autoriser réactivation même après désactivation permanente
                rte.set_multi(
                    pump_disabled_permanent = False,
                    pump_fault_retry_count  = 0,
                )
                print(f"[HEAL] B2003 INACTIVE : courant pompe < seuil pendant {bcm_rte.HEAL_DELAY}s — retry counter reset")
        else:
            rte._t_heal_pump = 0.0

        rte._pump_prev_current = current

    def _update_pump_current_history(self, rte, now: float, current: float):
        """Maintient l'historique glissant du courant pompe sur PUMP_VARIABLE_WINDOW secondes."""
        if not hasattr(rte, "_pump_current_history") or rte._pump_current_history is None:
            rte._pump_current_history = []
        rte._pump_current_history.append((now, current))
        cutoff = now - bcm_rte.PUMP_VARIABLE_WINDOW
        rte._pump_current_history = [(t, v) for (t, v) in rte._pump_current_history if t >= cutoff]

    def _trigger_pump_b2003(self, fault_type: str, current: float):
        """
        Déclenche B2003 (Pump Overcurrent/Electrical Fault) pour un type de faute donné.
        Arrête la pompe immédiatement, pose pump_error=True, gère le retry/désactivation permanente.

        fault_type : "OVERCURRENT" | "OPEN_LOAD" | "SHORT_GND" | "VARIABLE_LOAD"
        """
        rte  = self._rte
        snap = rte.make_snapshot()
        self._dtc.set_active("B2003", snap)
        self._pump_stop("overcurrent_b2003")

        rte.set("pump_fault_type", "Electrical Fault")

        retry = getattr(rte, "pump_fault_retry_count", 0) + 1
        rte.set("pump_fault_retry_count", retry)

        permanent = retry >= bcm_rte.PUMP_FAULT_MAX_RETRY
        rte.set("pump_disabled_permanent", permanent)

        if permanent:
            print(f"[B2003] DESACTIVATION PERMANENTE pompe (retry={retry}/{bcm_rte.PUMP_FAULT_MAX_RETRY}) "
                  f"— fault_type={fault_type}, courant={current:.2f}A — WSM -> ST_ERROR")
            # Fault persiste à la 2ème tentative → WSM entre en ST_ERROR
            self._error_origin = "pump_disabled"
            self._enter_state(ST_ERROR)
        else:
            print(f"[B2003] Pompe OFF pour cette requête (retry={retry}/{bcm_rte.PUMP_FAULT_MAX_RETRY}) "
                  f"— fault_type={fault_type}, courant={current:.2f}A")

        rte.set_multi(
            pump_error             = True,
            pump_overcurrent_error = True,
        )
        # Inhiber B2009 immédiatement (T-PUMP 10ms < T-WSM 200ms)
        rte.set_multi(
            _rest_contact_b2009_active = True,
            _rest_contact_stuck_start  = 0.0,
            _rest_contact_last_state   = -1,
        )
        # Reset historiques pour éviter fausse détection au prochain cycle
        rte._pump_current_history = []
        rte._pump_short_gnd_start = 0.0
        self._tcp_pump.send(rte)

    def _check_xcp_pump_cmd(self):
        """
        Lit RTE.xcp_pump_cmd posé par xcp_server_bcm._check_b2008().
        Si une commande est en attente (cmd=1, FWD fixe) et que la pompe
        est arrêtée, démarre la pompe FWD et entre en ST_DIAG exactement
        comme le RID 0x0203.
        La durée est lue depuis RTE.xcp_pump_duration (injectée par XCP).
        _check_pump_protection() surveille : si elapsed > PUMP_MAX_RUNTIME → B2008 + ST_ERROR.
        """
        rte = self._rte
        cmd = getattr(rte, "xcp_pump_cmd", 0)
        if cmd != 1:
            return
        # Consommer la commande immédiatement (one-shot)
        rte.set("xcp_pump_cmd", 0)
        duration = getattr(rte, "xcp_pump_duration", bcm_rte.PUMP_MAX_RUNTIME)
        if rte.pump_active or rte.state == ST_DIAG:
            print(f"[XCP-PUMP] Commande ignorée : pompe déjà active ou état DIAG")
            return
        if rte.pump_error:
            print(f"[XCP-PUMP] Commande ignorée : pump_error actif")
            return
        print(f"[XCP-PUMP] Démarrage pompe FWD durée={duration}s via XCP "
              f"(identique à RID 0x0203) — surveillance {bcm_rte.PUMP_MAX_RUNTIME}s → B2008 si dépassé")
        rte.set_multi(
            _test_active       = True,
            _test_routine      = 0x0203,
            _test_duration     = duration,
            _t_test_start      = time.time(),
            pump_dir_active    = 1,
            front_motor_on     = False,
            front_motor_speed  = 0,
            front_blade_moving = False,
            rear_motor_on      = False,
            rear_motor_running = False,
        )
        self._pump_start(1)
        self._enter_state(ST_DIAG)

    def _check_pump_protection(self):
        rte = self._rte
        if not rte.pump_active:
            return
        elapsed = time.time() - rte.t_pump_start
        if rte.state in (ST_WASH_FRONT, ST_WASH_REAR):
            # Fonctionnement normal wash : arrêt propre à ~4.9s (avant le guard 5s)
            # EXCEPTION : si les cycles ne sont pas encore terminés (hardware NE555 ou
            # simulation), laisser _process_front_wash() gérer l'arrêt.
            # - T21 (sim) : cycles à ~900ms → 3 cycles à ~2.7s → arrêt via cycles.
            # - T22 (sim) : cycles bloqués → FSR_005 à 5s.
            # - NE555 hardware : période 1.76s → 3 cycles à ~5.3s > 4.9s → ne pas couper
            #   avant que _process_front_wash() ait compté les 3 cycles (PUMP_MAX_RUNTIME=5s
            #   est le guard absolu).
            wash_in_progress = (rte.wash_cycles_done < bcm_rte.WASH_FRONT_CYCLES)
            if elapsed >= bcm_rte.PUMP_MAX_RUNTIME - 0.1 and not wash_in_progress:
                self._pump_stop("wash_normal_4.9s")
                # Stopper aussi le moteur avant pour éviter B2009 STUCK CLOSED :
                # sans cela, front_motor_on=True + contact repos figé → détection
                # "aucun mouvement depuis >3s" → B2009 → ST_ERROR.
                self._front_motor_stop()
                rte.set_multi(
                    front_motor_on     = False,
                    front_motor_speed  = 0,
                    front_blade_moving = False,
                    pump_dir_active    = 0,
                )
                rte.crs_wiper_op    = WOP_OFF
                rte._one_shot_armed = False
                self._enter_state(ST_OFF)
        elif rte.state == ST_ERROR and rte.wiper_fault and not rte.pump_error:
            # Pompe maintenue intentionnellement après erreur moteur/lame (B2006/B2009).
            # On lui laisse finir son runtime normal (PUMP_MAX_RUNTIME) puis arrêt propre,
            # SANS déclencher B2008 (ce n'est pas une anomalie pompe).
            if elapsed >= bcm_rte.PUMP_MAX_RUNTIME:
                print(f"[POMPE] Runtime {bcm_rte.PUMP_MAX_RUNTIME}s atteint en ST_ERROR (wiper_fault) -> arret propre")
                self._pump_stop("error_wiper_pump_timeout")
                rte.set("pump_dir_active", 0)
        elif rte.state == ST_DIAG:
            # Test actionneur pompe (RID 0x0203 / 0x0204) :
            # FSR_005 : runtime > 5s → B2008 + arrêt test + ST_ERROR
            # Seuil = PUMP_MAX_RUNTIME + 1 tick T-DIAG (100ms) pour absorber le jitter :
            # thread_diagnostic vérifie elapsed >= _test_duration toutes les 100ms,
            # donc il peut arrêter la pompe jusqu'à 100ms après l'expiration.
            # B2008 seulement si elapsed > 5.100s (pompe vraiment hors limite).
            if elapsed > bcm_rte.PUMP_MAX_RUNTIME + bcm_rte.ACTUATOR_TEST_PERIOD:
                print(f"[POMPE GUARD] {bcm_rte.PUMP_MAX_RUNTIME}s depasse en DIAG "
                      f"(routine=0x{rte._test_routine:04X}) -> B2008 + ST_ERROR")
                self._pump_stop("diag_max_runtime_b2008")
                self._dtc.set_active("B2008", rte.make_snapshot())
                # Clôturer le test avant de transiter vers ERROR
                rte.set_multi(
                    _test_active       = False,
                    pump_dir_active    = 0,
                    front_motor_on     = False,
                    front_motor_speed  = 0,
                    front_blade_moving = False,
                    rear_motor_on      = False,
                    rear_motor_running = False,
                    pump_error         = True,
                    pump_runtime_error = True,   # B2008 : dépassement runtime
                )
                self._enter_state(ST_ERROR)
        else:
            # Tout autre état (ne devrait pas arriver) : arrêt + B2008
            if elapsed > bcm_rte.PUMP_MAX_RUNTIME:
                print(f"[POMPE GUARD] {bcm_rte.PUMP_MAX_RUNTIME}s depasse -> arret + B2008")
                self._pump_stop("max_runtime_5s")
                self._dtc.set_active("B2008", rte.make_snapshot())

    def _check_overcurrent(self):
        rte     = self._rte
        current = rte.motor_current_a
        now     = time.time()

        if rte.front_motor_on:
            motor_id   = "front"
            dtc_code   = "B2001"
            error_key  = "front_motor_error"
            xcp_current = getattr(rte, "xcp_front_current_a", 0.0)
            effective_current = max(current, xcp_current)
        elif rte.rear_motor_on:
            motor_id   = "rear"
            dtc_code   = "B2002"
            error_key  = "rear_motor_error"
            xcp_current = getattr(rte, "xcp_rear_current_a", 0.0)
            effective_current = max(current, xcp_current)
        else:
            rte.t_overcurrent_start.clear()
            return

        # Utiliser le courant effectif (max du réel et de l'injecté XCP)
        current = effective_current

        # ── Condition déclenchement surcourant ────────────────────────────
        # CAS A (wc_available=False) : condition simple → courant > seuil
        # CAS B (wc_available=True)  : condition TRIPLE simultanée :
        #   1. wc_available = True
        #   2. courant (0x201 byte3 × 0.1A) > OVERCURRENT_THRESH (0.8A)
        #   3. 0x202 ErrorCode=0x03 reçu (wc_ack_overcurrent=True)
        #   Les 3 doivent être vraies ensemble.
        wc_ack_oc = getattr(rte, 'wc_ack_overcurrent', False)
        overcurrent_detected = (current > bcm_rte.OVERCURRENT_THRESH)
        if rte.wc_available:
            overcurrent_detected = overcurrent_detected and wc_ack_oc
            if wc_ack_oc and not (current > bcm_rte.OVERCURRENT_THRESH):
                # ErrorCode=0x03 reçu mais courant sous seuil : log uniquement
                print(f"[CAN 0x202] Overcurrent 0x03 reçu mais courant={current:.2f}A "
                      f"< seuil={bcm_rte.OVERCURRENT_THRESH}A — ignoré")
                rte.set("wc_ack_overcurrent", False)

        if overcurrent_detected:
            if motor_id not in rte.t_overcurrent_start:
                rte.t_overcurrent_start[motor_id] = now
                # Log unique au premier cycle : indique la source (XCP ou hardware)
                xcp_c = getattr(rte, f"xcp_{'front' if motor_id == 'front' else 'rear'}_current_a", 0.0)
                hw_c  = rte.motor_current_a
                if xcp_c > bcm_rte.OVERCURRENT_THRESH and hw_c <= bcm_rte.OVERCURRENT_THRESH:
                    print(f"[XCP-INJECT] {dtc_code} : surcourant XCP {motor_id} {xcp_c:.2f}A > {bcm_rte.OVERCURRENT_THRESH}A (hardware={hw_c:.2f}A) — timer 300ms démarré")
                else:
                    print(f"[SECURITE] Surintensite {motor_id} {current:.2f}A detectee...")
            elif now - rte.t_overcurrent_start[motor_id] > bcm_rte.OVERCURRENT_DELAY:
                del rte.t_overcurrent_start[motor_id]
                print(f"[SECURITE] {dtc_code} Surintensite {motor_id} "
                      f"{current:.2f}A > {bcm_rte.OVERCURRENT_THRESH}A pendant "
                      f"{bcm_rte.OVERCURRENT_DELAY*1000:.0f}ms -> arret moteur {motor_id} (pompe NON affectee)")
                # Snapshot avec valeur EFFECTIVE (max HW/XCP) :
                # si DTC déclenché par injection XCP, le snapshot reflète
                # la cause réelle et non la mesure ADS hardware.
                snap = rte.make_snapshot(motor_curr_ma=int(current * 1000))
                self._dtc.set_active(dtc_code, snap)
                # ── Arreter UNIQUEMENT le moteur concerne, pas la pompe ──
                if motor_id == "front":
                    self._front_motor_stop()
                    rte.set_multi(
                        front_motor_on    = False,
                        front_motor_speed = 0,
                        front_blade_moving= False,
                        front_motor_error = True,
                    )
                else:
                    self._rear_motor_stop()
                    rte.set_multi(
                        rear_motor_on     = False,
                        rear_motor_running= False,
                        rear_motor_error  = True,
                    )
                self._enter_state(ST_ERROR)
                # La pompe continue de fonctionner independamment
                # Reset du flag 0x202 Overcurrent après déclenchement DTC
                rte.set("wc_ack_overcurrent", False)
        else:
            # Courant OK : healing B2001/B2002
            # Si le DTC est ACTIVE et que le courant reste sous le seuil
            # pendant HEAL_DELAY (1s) avec le moteur en marche -> DTC INACTIVE
            rte.t_overcurrent_start.pop(motor_id, None)
            # Reset flag 0x202 Overcurrent quand courant revient sous seuil
            if getattr(rte, 'wc_ack_overcurrent', False):
                rte.set("wc_ack_overcurrent", False)
            dtc_status = self._dtc.get_status(dtc_code)
            if dtc_status == "ACTIVE":
                heal_key = "_t_heal_front" if motor_id == "front" else "_t_heal_rear"
                t_heal = getattr(rte, heal_key, 0.0)
                if t_heal == 0.0:
                    setattr(rte, heal_key, now)
                    print(f"[HEAL] {dtc_code} : courant OK, timer healing demarre (1s)")
                elif now - t_heal >= bcm_rte.HEAL_DELAY:
                    setattr(rte, heal_key, 0.0)
                    self._dtc.set_inactive(dtc_code)
                    print(f"[HEAL] {dtc_code} INACTIVE : courant < seuil pendant {bcm_rte.HEAL_DELAY}s")
            else:
                setattr(rte, "_t_heal_front" if motor_id == "front" else "_t_heal_rear", 0.0)

    def _watchdog_kick(self):
        self._rte._watchdog_kick_time = time.time()

    def _watchdog_check(self):
        rte = self._rte
        # TC_FSR_008 : simuler un blocage watchdog depuis la Platform
        if getattr(rte, "watchdog_test_trigger", False):
            rte.watchdog_test_trigger = False
            print("[WATCHDOG] TC_FSR_008 : déclenchement simulé (watchdog_test_trigger)")
            # Forcer un elapsed artificiel en reculant kick_time de WATCHDOG_MAX_MS * 11
            rte._watchdog_kick_time = time.time() - (bcm_rte.WATCHDOG_MAX_MS * 11 / 1000.0)

        elapsed_ms = (time.time() - rte._watchdog_kick_time) * 1000
        if elapsed_ms > bcm_rte.WATCHDOG_MAX_MS * 10:
            # Reset le timer en premier pour eviter les declenchements en cascade
            self._rte.set("_watchdog_kick_time", time.time())
            print(f"[WATCHDOG] Timeout {elapsed_ms:.0f}ms > {bcm_rte.WATCHDOG_MAX_MS * 10:.0f}ms "
                  f"etat={rte.state} -- action securite TSR_005")
            # TSR_005 : action de securite -- arret de tous les actionneurs
            # On ne transite pas vers ST_ERROR depuis ce thread (T-DIAG)
            # pour eviter les race conditions avec T-WSM.
            # On utilise uniquement les primitives bas niveau (GPIO direct).
            if rte.state not in (ST_OFF, ST_ERROR, ST_DIAG):
                self._stop_all()
                rte.set_multi(
                    front_motor_on     = False,
                    front_motor_speed  = 0,
                    front_blade_moving = False,
                    rear_motor_on      = False,
                    rear_motor_running = False,
                    pump_dir_active    = 0,
                    state              = ST_ERROR,
                )

    def _check_rain_sensor(self):
        """
        Surveillance etat capteur pluie (couche Application).

        Regle metier :
          - Si rain_sensor_installed = False : pas de capteur -> pas de DTC.
          - Si rain_sensor_installed = True  :
              B2007 si rain_sensor_ok=False (SensorStatus != 0 dans trame 0x301
              ou injection XCP rain_sensor_raw > 254).

        Healing B2007 (ISO 14229) :
          La condition doit rester OK pendant HEAL_DELAY (1s) avant INACTIVE.
          Pour l injection XCP, on verifie xcp_rain_raw <= RAIN_VALID_MAX (254)
          pendant 1s — evite une desactivation instantanee si la valeur oscille.
          Pour le CAN hardware (rain_sensor_ok revient True via trame 0x301),
          le meme timer s applique.
        """
        rte = self._rte
        now = time.time()

        if not rte.rain_sensor_installed:
            # Capteur non configure -> desactiver B2007 s il etait actif
            self._dtc.set_inactive("B2007")
            rte._t_heal_b2007 = 0.0
            return

        if not rte.rain_sensor_ok:
            # Condition de faute active -> B2007 ACTIVE + reset timer healing
            self._dtc.set_active("B2007", rte.make_snapshot())
            rte._t_heal_b2007 = 0.0
        else:
            # rain_sensor_ok=True : verifier aussi xcp_rain_raw
            # Si XCP a injecte une valeur > 254 et qu elle n est pas encore
            # revenue sous le seuil, ne pas mettre INACTIVE immediatement.
            xcp_raw = getattr(rte, "xcp_rain_raw", 0)
            rain_ok = (xcp_raw <= bcm_rte.RAIN_VALID_MAX)

            if rain_ok:
                # Condition OK : demarrer ou continuer le timer healing
                if self._dtc.get_status("B2007") == "ACTIVE":
                    if rte._t_heal_b2007 == 0.0:
                        rte._t_heal_b2007 = now
                        print(f"[HEAL] B2007 : rain_sensor OK (raw={xcp_raw}), "
                              f"timer healing demarre (1s)")
                    elif now - rte._t_heal_b2007 >= bcm_rte.HEAL_DELAY:
                        rte._t_heal_b2007 = 0.0
                        self._dtc.set_inactive("B2007")
                        print(f"[HEAL] B2007 INACTIVE : capteur pluie OK pendant "
                              f"{bcm_rte.HEAL_DELAY}s (raw={xcp_raw})")
                else:
                    # B2007 deja INACTIVE : rien a faire
                    rte._t_heal_b2007 = 0.0
            else:
                # xcp_rain_raw encore > 254 meme si rain_sensor_ok=True :
                # maintenir B2007 ACTIVE et reset timer
                if rte._t_heal_b2007 != 0.0:
                    print(f"[HEAL] B2007 : timer reset (xcp_rain_raw={xcp_raw} > "
                          f"{bcm_rte.RAIN_VALID_MAX})")
                rte._t_heal_b2007 = 0.0
                self._dtc.set_active("B2007", rte.make_snapshot())

    def _check_wc_timeout(self):
        """
        FSR_002 / TSR_002 : supervision CAN WC.
        Si wc_available et pas de trame 0x201 depuis CAN_WC_TIMEOUT -> B2005.
        Timeout modifie 100ms -> 2000ms (demande utilisateur).
        Supervision suspendue pendant ST_WASH_REAR et ST_REAR_WIPE :
        ces etats n'impliquent pas le moteur avant, le WC ne repond pas.
        """
        from bcm_rte import CAN_WC_TIMEOUT, ST_WASH_REAR, ST_REAR_WIPE
        rte = self._rte
        if not rte.wc_available:
            # Cas A : pas de WC, pas de supervision CAN
            if rte.wc_timeout_active:
                rte.set("wc_timeout_active", False)
            return
        # Suspension pendant etats arriere (WC non sollicite)
        if rte.state in (ST_WASH_REAR, ST_REAR_WIPE):
            if rte.wc_timeout_active:
                rte.set("wc_timeout_active", False)
            rte.set("t_last_wiper_status", time.time())  # reset le timer
            return
        if rte.t_last_wiper_status == 0.0:
            return  # WC pas encore vu

        elapsed = time.time() - rte.t_last_wiper_status
        if elapsed > CAN_WC_TIMEOUT and not rte.wc_timeout_active:
            print(f"[B2005] CAN Timeout WC: pas de 0x201 depuis "
                  f"{elapsed:.1f}s > {CAN_WC_TIMEOUT}s -> B2005 + OFF")
            rte.set("wc_timeout_active", True)
            self._dtc.set_active("B2005", rte.make_snapshot())
            # Etat securise : OFF (FSR_002).
            # Forcer crs_wiper_op=WOP_OFF pour eviter qu'une commande residuelle
            # (ex: lin_op_locked=True + crs_wiper_op=2 pendant TC_FSR_010)
            # ne provoque une retransition OFF->SPEED1 immediatement apres B2005.
            if rte.crs_wiper_op != WOP_OFF:
                rte.set("crs_wiper_op", WOP_OFF)
            if rte.state not in (ST_OFF, ST_ERROR, ST_DIAG):
                self._enter_state(ST_OFF)
        elif elapsed <= CAN_WC_TIMEOUT and rte.wc_timeout_active:
            rte.set("wc_timeout_active", False)
            self._dtc.set_inactive("B2005")

    # ==================================================
    # SECTION F -- TRAITEMENT UDS
    # ==================================================

    def _process_uds_request(self):
        from dtc_manager import handle_read_dtc, handle_clear_dtc
        rte = self._rte
        uds = rte.uds_payload
        sid = rte.uds_sid

        handlers = {
            SID_DSC:   self._handle_dsc,
            SID_RESET: self._handle_reset,
            SID_CLEAR: self._handle_clear,
            SID_RDTC:  lambda u: handle_read_dtc(self._dtc, u),
            SID_RDID:  self._handle_rdid,
            SID_WDID:  self._handle_wdid,
            SID_SA:    self._handle_sa,
            SID_CC:    self._handle_cc,
            SID_RC:    self._handle_rc,
            SID_TP:    self._handle_tp,
        }
        handler  = handlers.get(sid)
        response = handler(uds) if handler else self._nrc(sid, 0x11)

        rte.set_multi(
            uds_response       = response if response is not None else b"",
            uds_response_ready = True,
            uds_request_pending= False,
        )

    # ==================================================
    # SECTION G -- HANDLERS UDS
    # ==================================================

    def _nrc(self, sid: int, code: int) -> bytes:
        return bytes([0x7F, sid, code])

    def _handle_dsc(self, uds: bytes) -> bytes:
        if len(uds) < 2:
            return self._nrc(SID_DSC, 0x13)
        sub      = uds[1] & 0x7F
        suppress = bool(uds[1] & 0x80)
        if sub not in (DSC_DEFAULT, DSC_EXTENDED):
            return self._nrc(SID_DSC, 0x12)
        rte = self._rte
        rte.set("_session", sub)
        if sub == DSC_DEFAULT:
            rte.set("_sec_level", 0)
        rte.set("_pending_seed", {})
        # ISO 14229 : retour Default session → reinitialisation comm (0x28)
        if sub == DSC_DEFAULT:
            rte.set_multi(_comm_tx_enabled=True, _comm_rx_enabled=True)
            print("[DSC 0x10] Default session → Rx+Tx reactivés (ISO 14229)")
        if suppress:
            return b""
        return bytes([0x50, sub, 0x00, 0x32, 0x07, 0xD0])

    def _handle_clear(self, uds: bytes) -> bytes:
        """
        UDS 0x14 -- ClearDiagnosticInformation
        """
        from dtc_manager import handle_clear_dtc
        rte = self._rte

        # Clear DTC standard
        response = handle_clear_dtc(self._dtc, uds)
        print("[CLEAR DTC] DTC effaces")

        # Reset garde anti-boucle B2006 : autoriser re-detection apres reparation mecanique
        rte.set_multi(
           _rest_contact_b2006_active = False,
           _rest_contact_b2009_active = False,  # ← FIX AJOUTÉ
           _rest_contact_stuck_start  = 0.0,    # ← optionnel mais recommandé
           _rest_contact_last_state   = -1,     # ← idem
           # ── Reset erreurs individuelles apres clear DTC ──
           front_motor_error          = False,
           rear_motor_error           = False,
           pump_error                 = False,
           pump_overcurrent_error     = False,
           pump_runtime_error         = False,
           wiper_fault                = False,   # reset B2006/B2009 apres clear DTC
           # ── Reset fault injection pump (Clear DTC = healing complet) ──
           pump_fault_type            = "NONE",
           pump_fault_retry_count     = 0,
           pump_disabled_permanent    = False,
       )
        # Reset alive counter tracking (attributs prives protocole)
        rte._alive_rx_prev       = -1
        rte._alive_rx_freeze_cnt = 0
        # Reset timers healing ISO 14229
        rte._t_heal_front = 0.0
        rte._t_heal_rear  = 0.0
        rte._t_heal_pump  = 0.0
        rte._t_heal_b2008 = 0.0
        rte._t_heal_b2009 = 0.0

        # Verification courant apres clear
        # Note : depuis la correction isolation erreurs, l'overcurrent moteur
        # n'entraine plus ST_ERROR global. Le moteur concerne est simplement
        # arrete et son flag (front_motor_error / rear_motor_error) est remis
        # a False ci-dessus. Aucune reprise d'etat necessaire ici.
        rte.t_overcurrent_start.clear()

        return response

    def _handle_reset(self, uds: bytes) -> bytes:
        sub = uds[1] if len(uds) > 1 else 0x01
        # Cahier des charges : seul le softReset (0x03) est supporte.
        # hardReset (0x01) et keyOffOnReset (0x02) → NRC 0x12 subFunctionNotSupported.
        if sub != 0x03:
            print(f"[ECUReset] sub=0x{sub:02X} non supporte -> NRC 0x12")
            return self._nrc(SID_RESET, 0x12)
        rte = self._rte
        rte.set_multi(
            _session=1, _sec_level=0, _pending_seed={},
            _rest_contact_b2009_active = False,
            _rest_contact_b2006_active = False,   # reset garde anti-boucle B2006
            _rest_contact_stuck_start  = 0.0,
            _rest_contact_last_state   = -1,
            _rest_contact_prev         = None,    # reset detection front montant
            t_motor_stop               = 0.0,     # FIX: evite B2006 residuel apres ECU Reset
            _test_active               = False,
            pump_error                 = False,
            pump_overcurrent_error     = False,
            pump_runtime_error         = False,
            front_motor_error          = False,
            rear_motor_error           = False,
            wiper_fault                = False,
            # ── Reset fault injection pump (ECU Reset = healing complet) ──
            pump_fault_type            = "NONE",
            pump_fault_retry_count     = 0,
            pump_disabled_permanent    = False,
        )
        # Reset timers healing ISO 14229
        rte._t_heal_front = 0.0
        rte._t_heal_rear  = 0.0
        rte._t_heal_pump  = 0.0
        rte._t_heal_b2008 = 0.0
        rte._t_heal_b2009 = 0.0
        # Reset alive counter tracking (attributs prives protocole)
        rte._alive_rx_prev       = -1
        rte._alive_rx_freeze_cnt = 0
        self._enter_state(ST_OFF)
        return bytes([0x51, sub])

    def _compute_key(self, seed: bytes) -> bytes:
        """Backend.zip style : clé = seed inversé."""
        return seed[::-1]

    def _handle_sa(self, uds: bytes) -> bytes:
        """
        Security Access -- seed fixe + clé inversée (comme backend.zip)
        mais avec sub 0x01/0x02 (security level 1).
          sub 0x01 : requestSeed → seed fixe 4 octets (0x11223344)
          sub 0x02 : sendKey     → clé attendue = seed inversé (0x44332211)
        """
        if len(uds) < 2:
            return self._nrc(SID_SA, 0x13)
        sub = uds[1]
        rte = self._rte
        if rte._session != DSC_EXTENDED:
            return self._nrc(SID_SA, 0x7E)
        if sub == SA_REQ_SEED:   # 0x01 requestSeed
            if rte._sec_level >= 1:
                return bytes([0x67, sub, 0x00, 0x00, 0x00, 0x00])
            # Seed fixe comme backend.zip
            seed = b"\x11\x22\x33\x44"
            rte._pending_seed[1] = seed
            return bytes([0x67, sub]) + seed
        elif sub == SA_SEND_KEY:   # 0x02 sendKey
            if 1 not in rte._pending_seed:
                return self._nrc(SID_SA, 0x24)
            if len(uds) < 6:
                return self._nrc(SID_SA, 0x13)
            received_key = uds[2:6]
            expected_key = self._compute_key(rte._pending_seed.pop(1))
            if received_key != expected_key:
                return self._nrc(SID_SA, 0x35)
            rte.set("_sec_level", 1)
            return bytes([0x67, sub])
        return self._nrc(SID_SA, 0x12)

    def _handle_cc(self, uds: bytes) -> bytes:
        """
        UDS 0x28 -- CommunicationControl
        Accessible uniquement en session Extended (DSC_EXTENDED).
        comm_type byte 2 : 0x01 = normalCommunicationMessages (seule valeur supportee).
        sub-fonctions :
          0x00 enableRxAndTx    → _comm_rx=True  _comm_tx=True
          0x01 enableRxDisableTx→ _comm_rx=True  _comm_tx=False
          0x02 disableRxEnableTx→ _comm_rx=False _comm_tx=True
          0x03 disableRxAndTx  → _comm_rx=False _comm_tx=False
        Retour Default session → reinitialise Rx+Tx automatiquement (ISO 14229).
        """
        if len(uds) < 3:
            return self._nrc(SID_CC, 0x13)
        rte       = self._rte
        sub       = uds[1] & 0x7F          # masquer suppress bit
        suppress  = bool(uds[1] & 0x80)
        comm_type = uds[2]

        # Accessible uniquement en session Extended
        if rte._session != DSC_EXTENDED:
            return self._nrc(SID_CC, 0x7E)

        # comm_type : 0x01 = normalCommunicationMessages (seul supporte)
        if comm_type not in (0x01,):
            return self._nrc(SID_CC, 0x31)

        if sub == 0x00:
            rte.set_multi(_comm_tx_enabled=True,  _comm_rx_enabled=True)
            print("[CC 0x28] enableRxAndTx → Rx=ON  Tx=ON")
        elif sub == 0x01:
            rte.set_multi(_comm_tx_enabled=False, _comm_rx_enabled=True)
            print("[CC 0x28] enableRxDisableTx → Rx=ON  Tx=OFF")
        elif sub == 0x02:
            rte.set_multi(_comm_tx_enabled=True,  _comm_rx_enabled=False)
            print("[CC 0x28] disableRxEnableTx → Rx=OFF Tx=ON")
        elif sub == 0x03:
            rte.set_multi(_comm_tx_enabled=False, _comm_rx_enabled=False)
            print("[CC 0x28] disableRxAndTx → Rx=OFF Tx=OFF")
        else:
            return self._nrc(SID_CC, 0x12)

        if suppress:
            return b""
        return bytes([0x68, sub, comm_type])

    def _handle_rdid(self, uds: bytes) -> bytes:
        if len(uds) < 3:
            return self._nrc(SID_RDID, 0x13)
        did = (uds[1] << 8) | uds[2]
        rte = self._rte

        if did == 0xF100:
            return bytes([0x62, 0xF1, 0x00, ST_ENC.get(rte.state, 0)])
        elif did == 0xF101:
            return bytes([0x62, 0xF1, 0x01, rte.front_motor_speed & 0xFF])
        elif did == 0xF102:
            # Diag Spec Section 5.1 : BladePosition 0-100%
            # CAS B (wc_available) : position réelle fournie par WC via trame 0x201 Byte 2
            # CAS A               : contact repos binaire → estimation 0% (repos) / 50% (mouvement)
            if rte.wc_available:
                blade = rte.wc_blade_position if rte.wc_blade_position >= 0 else 0
                blade = max(0, min(100, blade))
            else:
                blade = 50 if rte.front_blade_moving else 0
            return bytes([0x62, 0xF1, 0x02, blade & 0xFF])
        elif did == 0xF103:
            curr_ma = int(rte.motor_current_a * 1000)
            return bytes([0x62, 0xF1, 0x03, (curr_ma >> 8) & 0xFF, curr_ma & 0xFF])
        elif did == 0xF104:
            return bytes([0x62, 0xF1, 0x04, rte.pump_dir_active & 0xFF])
        elif did == 0xF105:
            return bytes([0x62, 0xF1, 0x05, rte.rain_intensity & 0xFF])
        elif did == 0xF106:
            return bytes([0x62, 0xF1, 0x06, 1 if rte.rear_motor_running else 0])
        elif did == 0xF107:
            err = 0
            if rte.state == ST_ERROR:                                         err |= 0x04
            # Bits source d'erreur :
            #   0x04 : WSM en ST_ERROR
            #   0x10 : surcourant moteur (B2001/B2002)
            #   0x20 : erreur lame/contact repos (B2006/B2009)
            #   0x40 : surcourant pompe (B2003)
            #   0x80 : dépassement runtime pompe (B2008)
            if rte.front_motor_error or rte.rear_motor_error:                 err |= 0x10
            if getattr(rte, "wiper_fault", False):                            err |= 0x20
            if getattr(rte, "pump_overcurrent_error", False):                 err |= 0x40
            if getattr(rte, "pump_runtime_error",     False):                 err |= 0x80
            return bytes([0x62, 0xF1, 0x07, err])

        # ── Coding DIDs (lecture) ─────────────────────────────
        elif did == 0xF200:
            return bytes([0x62, 0xF2, 0x00, 1 if rte.rain_sensor_installed else 0])
        elif did == 0xF201:
            return bytes([0x62, 0xF2, 0x01, 1 if rte.wc_available else 0])
        elif did == 0xF202:
            return bytes([0x62, 0xF2, 0x02, 1 if rte.rear_wiper_available else 0])
        elif did == 0xF203:
            return bytes([0x62, 0xF2, 0x03, getattr(rte, 'channel_front_wash', 0) & 0xFF])
        elif did == 0xF204:
            return bytes([0x62, 0xF2, 0x04, getattr(rte, 'channel_rear_camera', 1) & 0xFF])

        return self._nrc(SID_RDID, 0x31)

    def _handle_wdid(self, uds: bytes) -> bytes:
        rte = self._rte
        if rte._session != DSC_EXTENDED:
            return self._nrc(SID_WDID, 0x22)
        if rte._sec_level < 1:
            return self._nrc(SID_WDID, 0x33)
        if len(uds) < 4:
            return self._nrc(SID_WDID, 0x13)
        did = (uds[1] << 8) | uds[2]
        val = uds[3]

        # Identifiant client DoIP basé sur l'adresse source UDS
        owner = f"DoIP-0x{rte.uds_src_addr:04X}"

        # ── DIDs de contrôle du verrou Single Writer ──────────────────
        if did == 0xFFF0:   # LOCK_ACQUIRE
            ok = rte.acquire_write_lock(owner)
            if not ok:
                info = rte.get_write_lock_info()
                print(f"[WDID] LOCK_ACQUIRE refusé pour {owner} "
                      f"— détenu par '{info['owner']}' "
                      f"(reste {info['ttl_left_s']}s)")
                return self._nrc(SID_WDID, 0x22)
            return bytes([0x6E, uds[1], uds[2]])

        elif did == 0xFFF1:   # LOCK_RELEASE
            rte.release_write_lock(owner)
            return bytes([0x6E, uds[1], uds[2]])

        elif did == 0xFFF2:   # LOCK_RENEW
            if not rte.renew_write_lock(owner):
                return self._nrc(SID_WDID, 0x22)
            return bytes([0x6E, uds[1], uds[2]])

        # ── DIDs standards — vérification verrou avant écriture ───────
        if not rte.acquire_write_lock(owner):
            info = rte.get_write_lock_info()
            print(f"[WDID] DID=0x{did:04X} REFUSÉ pour {owner} "
                  f"— verrou détenu par '{info['owner']}'")
            return self._nrc(SID_WDID, 0x22)

        if did == 0xF200:
            rte.set("rain_sensor_installed", bool(val))
            if not bool(val) and rte.state == ST_AUTO:
                self._enter_state(ST_OFF)
        elif did == 0xF201:
            old_val = rte.wc_available
            new_val = bool(val)
            rte.set("wc_available", new_val)
            if new_val and not old_val:
                print("[CODING] F201 WcAvailable=Installed -> Cas B actif "
                      "(BCM envoie CAN 0x200, WC commande moteur avant)")
                self._front_motor_stop()
                self._rte.set_multi(front_motor_on=False, front_motor_speed=0,
                                    front_blade_moving=False)
            elif not new_val and old_val:
                print("[CODING] F201 WcAvailable=NotInstalled -> Cas A actif "
                      "(BCM commande directement moteur avant)")
                rte.set("wc_timeout_active", False)
        elif did == 0xF202:
            rte.set("rear_wiper_available", bool(val))
            if not bool(val) and rte.state in (ST_WASH_REAR, ST_REAR_WIPE):
                self._pump_stop("coding_f202")
                self._rear_motor_stop()
                rte.set_multi(rear_motor_on=False, _one_shot_armed=True)
                self._enter_state(ST_OFF)
        elif did == 0xF203:
            if val > 1:
                return self._nrc(SID_WDID, 0x31)
            rte.set("channel_front_wash", val)
        elif did == 0xF204:
            if val > 1:
                return self._nrc(SID_WDID, 0x31)
            rte.set("channel_rear_camera", val)
        else:
            return self._nrc(SID_WDID, 0x31)

        return bytes([0x6E, uds[1], uds[2]])

    def _handle_rc(self, uds: bytes) -> bytes:
        """RoutineControl BCM -- tests actionneurs.
        Requiert : session Extended (0x03) + SecurityAccess niveau 1 déverrouillé.
        """
        rte = self._rte
        if rte._session != DSC_EXTENDED:
            return self._nrc(SID_RC, 0x22)   # conditionsNotCorrect — session non Extended
        if rte._sec_level < 1:
            return self._nrc(SID_RC, 0x33)   # securityAccessDenied — sécurité verrouillée
        if len(uds) < 4:
            return self._nrc(SID_RC, 0x13)

        sub      = uds[1]
        rid      = struct.unpack(">H", uds[2:4])[0]
        duration = min(uds[4] if len(uds) >= 5 else 10, 60)

        if sub == 0x01:
            if rid == 0x0201:
                if rte.state in (ST_SPEED1, ST_SPEED2, ST_AUTO, ST_TOUCH, ST_WASH_FRONT):
                    return self._nrc(SID_RC, 0x22)
                if rte.state in (ST_DIAG, ST_ERROR):
                    return self._nrc(SID_RC, 0x22)
                rte.set_multi(
                    _test_active=True, _test_routine=rid,
                    _test_duration=duration, _t_test_start=time.time(),
                    front_motor_on=True, front_motor_speed=1,
                    front_blade_moving=True, rear_motor_on=False,
                    rear_motor_running=False, pump_dir_active=0,
                    # Reset timers B2009 : meme comportement que SPEED1/SPEED2/TOUCH
                    _rest_contact_stuck_start = 0.0,
                    _rest_contact_last_state  = -1,
                    _t_last_blade_cycle       = 0.0,
                )
                rte._rest_contact_prev  = None   # ignorer phase courante, attendre prochain front
                rte._front_blade_cycles = 0
                self._front_motor_run(1)
                self._enter_state(ST_DIAG)
                return bytes([0x71, sub, 0x02, 0x01, duration & 0xFF])

            elif rid == 0x0202:
                if not rte.rear_wiper_available:
                    return self._nrc(SID_RC, 0x22)
                if rte.state in (ST_REAR_WIPE, ST_WASH_REAR, ST_DIAG, ST_ERROR):
                    return self._nrc(SID_RC, 0x22)
                rte.set_multi(
                    _test_active=True, _test_routine=rid,
                    _test_duration=duration, _t_test_start=time.time(),
                    rear_motor_on=True, rear_motor_running=True,
                    front_motor_on=False, front_motor_speed=0,
                    front_blade_moving=False, pump_dir_active=0,
                )
                self._rear_motor_run()
                self._enter_state(ST_DIAG)
                return bytes([0x71, sub, 0x02, 0x02, duration & 0xFF])

            elif rid == 0x0203:
                if rte.pump_active or rte.state in (ST_DIAG, ST_ERROR):
                    return self._nrc(SID_RC, 0x22)
                # NOTE : _test_duration garde la durée demandée (ex: 6s).
                # _check_pump_protection() déclenche B2008 + ST_ERROR à 5s si
                # la pompe tourne encore. Le timer T-DIAG ne peut donc jamais
                # appeler _stop_test() avant la protection si duration > 5s.
                rte.set_multi(
                    _test_active=True, _test_routine=rid,
                    _test_duration=duration,
                    _t_test_start=time.time(), pump_dir_active=1,
                    front_motor_on=False, front_motor_speed=0,
                    front_blade_moving=False, rear_motor_on=False,
                    rear_motor_running=False,
                )
                self._pump_start(1)
                self._enter_state(ST_DIAG)
                return bytes([0x71, sub, 0x02, 0x03, duration & 0xFF])

            elif rid == 0x0204:
                if rte.pump_active or rte.state in (ST_DIAG, ST_ERROR):
                    return self._nrc(SID_RC, 0x22)
                rte.set_multi(
                    _test_active=True, _test_routine=rid,
                    _test_duration=duration,
                    _t_test_start=time.time(), pump_dir_active=2,
                    front_motor_on=False, front_motor_speed=0,
                    front_blade_moving=False, rear_motor_on=False,
                    rear_motor_running=False,
                )
                self._pump_start(2)
                self._enter_state(ST_DIAG)
                return bytes([0x71, sub, 0x02, 0x04, duration & 0xFF])

            elif rid == 0x0205:
                rain_val = uds[4] if len(uds) >= 5 else 0
                if rain_val > 100:
                    return self._nrc(SID_RC, 0x31)
                if not rte.rain_sensor_installed:
                    return self._nrc(SID_RC, 0x22)  # rain sensor must be installed
                if rte.state in (ST_DIAG, ST_ERROR):
                    return self._nrc(SID_RC, 0x22)
                RAIN_SIM_DURATION = 10  # duree fixe 10s, uds[4] = rain_intensity (pas duree)
                rte.set_multi(
                    rain_intensity            = rain_val,
                    crs_wiper_op             = WOP_AUTO,
                    lin_op_locked            = True,
                    _test_active             = True,
                    _test_routine            = rid,
                    _test_duration           = RAIN_SIM_DURATION,
                    _t_test_start            = time.time(),
                    # Init variables AUTO pour que _process_auto fonctionne depuis ST_DIAG
                    _auto_speed_prev         = -1,
                    front_motor_on           = False,
                    front_motor_speed        = 0,
                    front_blade_moving       = False,
                    _rest_contact_stuck_start = 0.0,
                    _rest_contact_last_state  = -1,
                )
                rte._front_blade_cycles = 0
                rte._rest_contact_prev  = self._read_rest_contact()
                # Entrer en ST_DIAG : DoIP prend le controle, tick = _process_auto
                self._enter_state(ST_DIAG)
                print(f"[RAIN SIM] rain_intensity={rain_val} -> mode DIAG/AUTO "
                      f"(duree={RAIN_SIM_DURATION}s, crs_wiper_op=WOP_AUTO)")
                return bytes([0x71, sub, 0x02, 0x05, rain_val & 0xFF])

        elif sub == 0x02:
            # NRC 0x22 si aucune routine active
            if not rte._test_active:
                return self._nrc(SID_RC, 0x22)   # conditionsNotCorrect — aucune routine en cours
            # NRC 0x31 si le RID ne correspond pas a la routine en cours
            if rid != rte._test_routine:
                return self._nrc(SID_RC, 0x31)   # requestOutOfRange — RID ne correspond pas
            self._stop_test()
            return bytes([0x71, sub]) + uds[2:4]

        elif sub == 0x03:
            if rte._test_active:
                elapsed = int(time.time() - rte._t_test_start)
                return bytes([0x71, sub]) + uds[2:4] + bytes([elapsed & 0xFF])
            return bytes([0x71, sub]) + uds[2:4] + bytes([0x00])

        return self._nrc(SID_RC, 0x31)

    def _handle_tp(self, uds: bytes) -> bytes:
        sub = uds[1] if len(uds) > 1 else 0x00
        if sub & 0x80:
            return b""
        return bytes([0x7E, 0x00])

    def _stop_test(self):
        rte = self._rte
        routine = rte._test_routine
        print(f"[TEST] Routine 0x{routine:04X} terminee")

        # Si c'etait une simulation pluie, restaurer les commandes LIN
        if routine == 0x0205:
            rte.set_multi(
                _test_active   = False,
                rain_intensity = 0,
                crs_wiper_op   = WOP_OFF,
                lin_op_locked  = False,
            )
            self._front_motor_stop()
            # Garde : ne pas écraser ST_ERROR avec ST_OFF
            if rte.state != ST_ERROR:
                self._enter_state(ST_OFF)
            print("[RAIN SIM] Terminee -> rain=0, mode OFF, LIN debloque")
            return

        rte.set_multi(
            _test_active       = False,
            front_motor_on     = False,
            front_motor_speed  = 0,
            front_blade_moving = False,
            rear_motor_on      = False,
            rear_motor_running = False,
            pump_dir_active    = 0,
        )
        self._front_motor_stop()
        self._rear_motor_stop()
        self._pump_stop("test_complet")
        # Garde : si une erreur est survenue pendant le test (ST_ERROR),
        # ne pas écraser l'état ERROR avec ST_OFF — le healing doit gérer la sortie.
        if rte.state != ST_ERROR:
            self._enter_state(ST_OFF)

    # ==================================================
    # SECTION H -- THREADS
    # ==================================================

    def thread_wsm_control(self):
        """Thread T-WSM -- Machine d'etat wiper (200ms)."""
        import traceback
        print(f"[THREAD T-WSM] Demarre | periode={CONTROL_LOOP_PERIOD*1000:.0f}ms")
        while self._running:
            try:
                self._update_state_machine()
                self._watchdog_kick()   # kick APRES execution : prouve que T-WSM tourne
            except Exception as e:
                print(f"[T-WSM] ERREUR CRITIQUE: {e}")
                traceback.print_exc()
            time.sleep(CONTROL_LOOP_PERIOD)

    def thread_pump_guard(self):
        """
        Thread T-PUMP -- Surveillance pompe + courant ADS1115 + securites (200ms).
        Lit le courant ADS1115 toutes les ADS_READ_PERIOD (100ms).
        Aussi source de kick watchdog independante de T-WSM.
        """
        print(f"[THREAD T-PUMP] Demarre | periode={PUMP_GUARD_PERIOD*1000:.0f}ms")
        t_last_ads      = 0.0
        _current_prev   = -1.0
        _CURRENT_STEP   = 0.050
        t_last_ads_pump = 0.0
        _pump_curr_prev = -1.0
        # Compteur pour kick watchdog depuis T-PUMP toutes les ~500ms
        # (PUMP_GUARD_PERIOD=10ms, 50 iterations = 500ms)
        _wd_kick_ctr    = 0
        _WD_KICK_EVERY  = 50

        while self._running:
            # ── Kick watchdog depuis T-PUMP toutes les 500ms ──────────────
            # Source de kick INDEPENDANTE de T-WSM : si T-WSM est momentanément
            # occupé mais T-PUMP tourne normalement, le watchdog ne déclenchera pas.
            # T-PUMP ne fait PAS de sendall() bloquant -> toujours disponible.
            _wd_kick_ctr += 1
            if _wd_kick_ctr >= _WD_KICK_EVERY:
                _wd_kick_ctr = 0
                self._watchdog_kick()

            # Lecture périodique rest_contact → met à jour rest_contact_raw dans Redis
            # même quand le moteur est à l'arrêt (nécessaire pour WindshieldWidget Platform)
            self._read_rest_contact()

            now = time.time()
            rte = self._rte

            # Lecture ADS1115 moteur wiper toutes les 100ms
            if now - t_last_ads >= ADS_READ_PERIOD:
                current    = self._read_motor_current()
                t_last_ads = now

                stepped = round(round(current / _CURRENT_STEP) * _CURRENT_STEP, 2)

                if stepped != _current_prev:
                    _current_prev = stepped
                    self._rte.set("motor_current_a", stepped)
                    print(f"[ADS1115] Courant moteur : {stepped:.2f}A ({stepped*1000:.0f}mA)")
                    self._tcp.send(self._rte)
                    self._tcp_pump.send(self._rte)
                    self._rte.set("motor_current_a", current)

            # Lecture courant pompe ACS712 si pompe active
            # Gating identique au courant moteur : lecture toutes les ADS_READ_PERIOD,
            # transmission uniquement si la valeur a change (step 0.050 A)
            if self._rte.pump_active:
                if now - t_last_ads_pump >= ADS_READ_PERIOD:
                    t_last_ads_pump = now
                    p_curr, p_volt = self._read_pump_current()
                    # Si une injection test est active (pump_inject_ts != 0),
                    # _read_pump_current a déjà retourné la valeur injectée et
                    # n'a PAS écrasé rte.pump_current_a → ne pas le faire ici non plus.
                    if self._pump_inject_ts == 0.0:
                        stepped_p = round(round(p_curr / _CURRENT_STEP) * _CURRENT_STEP, 2)
                        if stepped_p != _pump_curr_prev:
                            _pump_curr_prev = stepped_p
                            self._rte.set("pump_current_a", stepped_p)
                            print(f"[ADS1115] Courant pompe  : {stepped_p:.2f}A ({stepped_p*1000:.0f}mA)")
                            self._tcp.send(self._rte)
                            self._tcp_pump.send(self._rte)
                            self._rte.set("pump_current_a", p_curr)
                    else:
                        # Injection active : transmettre la valeur injectée telle quelle
                        stepped_p = round(round(p_curr / _CURRENT_STEP) * _CURRENT_STEP, 2)
                        if stepped_p != _pump_curr_prev:
                            _pump_curr_prev = stepped_p
                            print(f"[ADS1115] Courant pompe (injecté) : {p_curr:.2f}A ({p_curr*1000:.0f}mA)")
                            self._tcp.send(self._rte)
                            self._tcp_pump.send(self._rte)
            else:
                _pump_curr_prev = -1.0   # reset pour prochain demarrage
                if self._rte.pump_current_a != 0.0 or self._rte.pump_voltage_v != 0.0:
                    self._rte.set_multi(
                        pump_current_a = 0.0,
                        pump_voltage_v = 0.0,
                        pump_v_b       = 0.0,
                        pump_v_a       = 0.0,
                    )

            self._check_xcp_pump_cmd()
            self._check_pump_protection()
            self._check_overcurrent()
            # _check_pump_overcurrent AVANT _check_rest_contact_stuck :
            # si B2003 se déclenche, il pose _rest_contact_b2009_active=True
            # dans la même itération, avant que _check_rest_contact_stuck
            # ne vérifie le timer → B2009 ne peut pas se déclencher
            # dans la même itération que B2003.
            self._check_pump_overcurrent()
            self._check_rest_contact_stuck()
            self._check_wc_timeout()
            self._check_rain_sensor()
            self._check_b2011_condition()    # B2011 : 0x16 bit6 ET 0x17 bit0 simultanés
            self._check_wc_ack()             # 0x202 ErrorCode : réactions BCM
            self._check_wc_fault_status()    # 0x201 FaultStatus bits → WiperMode=OFF + B2006
            time.sleep(PUMP_GUARD_PERIOD)

    # ==================================================
    # SECTION G -- SURVEILLANCE WC CAN (B2011/B2101/B2102/B2006 + 0x202 ACK)
    # ==================================================

    def _check_b2011_condition(self):
        """
        B2011 : déclenchement par ET logique simultané.
          Condition 1 : 0x16 byte0 bit6 (StickStatus bit2 = Stuck) = 1 pendant >= 10 s
          Condition 2 : 0x17 byte0 bit0 (CRS_InternalFault_Stick)  = 1

        Les deux doivent être vraies simultanément.
        Dès que l'une disparaît → timer reset → B2011 INACTIVE.
        """
        rte = self._rte
        now = time.time()

        stuck        = getattr(rte, 'crs_stuck', False)
        fault_stick  = getattr(rte, 'crs_fault_stick', False)

        if stuck and fault_stick:
            # Les deux conditions actives → démarrer / maintenir timer
            if rte._t_stuck_start == 0.0:
                rte._t_stuck_start = now
                print("[B2011] Timer démarré (0x16 bit6=1 ET 0x17 bit0=1)")
            elif (now - rte._t_stuck_start) >= 10.0 and not rte.b2011_active:
                rte.set("b2011_active", True)   # publie sur Redis → détecté par test_cases
                snap = rte.make_snapshot()
                self._dtc.set_active("B2011", snap)
                rte.set("crs_wiper_op", WOP_OFF)
                if rte.state not in (ST_OFF, ST_ERROR, ST_DIAG):
                    self._enter_state(ST_ERROR)
                print("[B2011] ACTIVE : levier coincé (0x16 bit6=1) ET CRS fault (0x17 bit0=1) >= 10s")
        else:
            # L'une des conditions a disparu → reset timer
            if rte._t_stuck_start != 0.0:
                print("[B2011] Timer reset (condition disparue)")
                rte._t_stuck_start = 0.0
            if rte.b2011_active:
                rte.set("b2011_active", False)  # dépublie sur Redis
                self._dtc.set_inactive("B2011")
                print("[B2011] INACTIVE")

    def _check_wc_ack(self):
        """
        Traite la trame 0x202 Wiper_Ack reçue (stockée dans RTE par _can_process_0x202).
        Le BCM réagit aux ErrorCode par des actions sur WiperMode (0x200) uniquement.
        B2101/B2102 sont des DTC spécifiques WC — non gérés ici côté BCM.

        Réactions selon ErrorCode (byte1) quand AckStatus=1 (NACK) :
          0x00 : incohérence protocole → log + retry ×1
          0x01 : InvalidCmd (désaccord WiperMode) → resync + ST_ERROR si 3 NACK consécutifs
          0x02 : MotorBlocked → WiperMode=OFF dans 0x200
          0x03 : Overcurrent  → flag wc_ack_overcurrent pour condition triple B2001/B2002
          0x04 : PosSensorFault → flag wc_ack_pos_fault pour condition conjointe B2006
          0x05 : InternalFault → WiperMode=OFF (pas de DTC BCM, B2101 est côté WC)
          0x06 / 0x07 : non traitables → log uniquement
        """
        rte = self._rte
        if not getattr(rte, 'wc_ack_pending', False):
            return
        rte.wc_ack_pending = False   # consommer le flag

        ack_status = rte.wc_last_ack_status
        error_code = rte.wc_last_error_code

        if ack_status == 0:
            # ACK : reset compteur NACK consécutifs
            rte.wc_nack_consecutive = 0
            # Healing wc_nack : compter exactement 3 ACK consécutifs pour sortir de ST_ERROR
            if rte.state == ST_ERROR and getattr(self, "_error_origin", None) == "wc_nack":
                count = rte.wc_ack_heal_count + 1
                rte.wc_ack_heal_count = count
                print(f"[HEAL-WC_NACK] ACK #{count}/3 reçu")
            return

        # ── NACK : traitement selon ErrorCode ───────────────────────────
        rte.wc_nack_consecutive += 1
        # Tout nouveau NACK interrompt un healing wc_nack en cours
        if getattr(self, "_error_origin", None) == "wc_nack" and \
                rte.wc_ack_heal_count > 0:
            print(f"[HEAL-WC_NACK] Nouveau NACK reçu → reset compteur ACK healing")
            rte.wc_ack_heal_count = 0

        if error_code == 0x00:
            # Incohérence protocole : NACK sans code erreur
            print("[CAN 0x202] NACK code=0x00 : incohérence protocole — retry 0x200")
            rte.wc_nack_consecutive = 0

        elif error_code == 0x01:
            # InvalidCmd : désaccord de mode → resynchronisation
            curr_mode_wc = getattr(rte, 'wc_current_mode', -1)
            print(f"[CAN 0x202] NACK InvalidCmd (0x01) — "
                  f"WSM={rte.state} CurrentMode_WC={curr_mode_wc} "
                  f"NACK_consec={rte.wc_nack_consecutive}")
            if rte.wc_nack_consecutive >= 3:
                print("[CAN 0x202] 3 NACK 0x01 consécutifs → ST_ERROR")
                if rte.state not in (ST_OFF, ST_ERROR, ST_DIAG):
                    self._error_origin = "wc_nack"
                    rte.wc_ack_heal_count = 0   # reset compteur ACK healing
                    self._enter_state(ST_ERROR)
                rte.wc_nack_consecutive = 0

        elif error_code == 0x02:
            # MotorBlocked : WC signale blocage → WiperMode=OFF dans 0x200
            print("[CAN 0x202] NACK MotorBlocked (0x02) → WiperMode=OFF dans 0x200")
            rte.set("crs_wiper_op", WOP_OFF)

        elif error_code == 0x03:
            # Overcurrent : signal auxiliaire pour condition triple B2001/B2002
            # (wc_available + courant > 0.8A + ErrorCode=0x03 simultanément)
            print(f"[CAN 0x202] NACK Overcurrent (0x03) — "
                  f"MotorCurrent={rte.motor_current_a:.2f}A (seuil={bcm_rte.OVERCURRENT_THRESH}A)")
            rte.set("wc_ack_overcurrent", True)

        elif error_code == 0x04:
            # PosSensorFault : signal auxiliaire pour condition conjointe B2006
            # (0x201 bit2=1 ET ErrorCode=0x04 simultanément → B2006 dans _check_wc_fault_status)
            print("[CAN 0x202] NACK PosSensorFault (0x04) → flag B2006 posé")
            rte.set("wc_ack_pos_fault", True)

        elif error_code == 0x05:
            # InternalFault WC : BCM envoie OFF dans 0x200 uniquement
            # B2101 est un DTC WC — non enregistré dans dtc_database BCM
            print("[CAN 0x202] NACK InternalFault (0x05) → WiperMode=OFF (B2101 géré côté WC)")
            rte.set("wc_fault_wc_internal", True)
            rte.set("crs_wiper_op", WOP_OFF)

        elif error_code in (0x06, 0x07):
            # Non traitables : log uniquement
            label = "SupplyFault" if error_code == 0x06 else "Busy"
            print(f"[CAN 0x202] NACK {label} (0x{error_code:02X}) : non traitable — log uniquement")
            rte.wc_nack_consecutive = 0

        else:
            print(f"[CAN 0x202] NACK ErrorCode inconnu=0x{error_code:02X} — ignoré")
            rte.wc_nack_consecutive = 0

    def _check_wc_fault_status(self):
        """
        Surveille les bits FaultStatus reçus dans 0x201 (mis à jour par _can_process_0x201).
        B2101 et B2102 sont des DTC spécifiques WC — non enregistrés dans dtc_database BCM.
        Le BCM réagit uniquement par des actions sur WiperMode (0x200).

        bit0 (WC_Internal) → WiperMode=OFF dans 0x200.
          Rétablissement : bit0=0 pendant >= HEAL_DELAY → moteur autorisé à reprendre.
          Cohérence avec 0x202 ErrorCode=0x05.

        bit1 (MotorDriver) → WiperMode=OFF dans 0x200.
          Rétablissement : bit1=0 pendant >= HEAL_DELAY → moteur autorisé à reprendre.
          Cohérence avec 0x202 ErrorCode=0x02.

        bit2 (PosSensor) → B2006 BCM : déclenchement uniquement si SIMULTANÉMENT
          0x201 bit2=1 ET 0x202 ErrorCode=0x04 (wc_ack_pos_fault=True).
          Condition conjointe — chacune seule est insuffisante.
          Rétablissement : bit2=0 ET wc_ack_pos_fault=False >= HEAL_DELAY → B2006 INACTIVE.
        """
        rte = self._rte
        now = time.time()

        # ── WC_Internal fault (0x201 byte4 bit0) : WiperMode=OFF ────────
        # B2101 est un DTC WC — le BCM commande seulement l'arrêt moteur.
        fault_wc = getattr(rte, 'wc_fault_wc_internal', False)
        if fault_wc:
            rte.set("crs_wiper_op", WOP_OFF)
            rte._t_b2101_heal = 0.0
            if not getattr(rte, '_wc_internal_logged', False):
                print("[WC] FaultStatus_WC_Internal=1 (0x201 bit0) → WiperMode=OFF (B2101 côté WC)")
                rte._wc_internal_logged = True
        else:
            if getattr(rte, '_wc_internal_logged', False):
                if rte._t_b2101_heal == 0.0:
                    rte._t_b2101_heal = now
                elif (now - rte._t_b2101_heal) >= bcm_rte.HEAL_DELAY:
                    rte._wc_internal_logged = False
                    rte._t_b2101_heal = 0.0
                    print("[WC] FaultStatus_WC_Internal disparu >= 1s → moteur autorisé")
            else:
                rte._t_b2101_heal = 0.0

        # ── MotorDriver fault (0x201 byte4 bit1) : WiperMode=OFF ────────
        # B2102 est un DTC WC — le BCM commande seulement l'arrêt moteur.
        fault_mtr = getattr(rte, 'wc_fault_motor_driver', False)
        if fault_mtr:
            rte.set("crs_wiper_op", WOP_OFF)
            rte._t_b2102_heal = 0.0
            if not getattr(rte, '_wc_motor_logged', False):
                print("[WC] FaultStatus_MotorDriver=1 (0x201 bit1) → WiperMode=OFF (B2102 côté WC)")
                rte._wc_motor_logged = True
        else:
            if getattr(rte, '_wc_motor_logged', False):
                if rte._t_b2102_heal == 0.0:
                    rte._t_b2102_heal = now
                elif (now - rte._t_b2102_heal) >= bcm_rte.HEAL_DELAY:
                    rte._wc_motor_logged = False
                    rte._t_b2102_heal = 0.0
                    print("[WC] FaultStatus_MotorDriver disparu >= 1s → moteur autorisé")
            else:
                rte._t_b2102_heal = 0.0

        # ── B2006 BCM : PosSensor condition conjointe ────────────────────
        # Condition ET : 0x201 byte4 bit2=1  ET  0x202 ErrorCode=0x04
        fault_pos = getattr(rte, 'wc_fault_pos_sensor', False)
        ack_pos   = getattr(rte, 'wc_ack_pos_fault', False)

        if fault_pos and ack_pos:
            if not getattr(rte, '_rest_contact_b2006_active', False):
                snap = rte.make_snapshot()
                self._dtc.set_active("B2006", snap)
                rte._rest_contact_b2006_active = True
                print("[B2006] ACTIVE : PosSensor fault (0x201 bit2=1 ET 0x202 code=0x04)")
            rte._t_b2103_heal = 0.0
        else:
            if getattr(rte, '_rest_contact_b2006_active', False):
                if rte._t_b2103_heal == 0.0:
                    rte._t_b2103_heal = now
                elif (now - rte._t_b2103_heal) >= bcm_rte.HEAL_DELAY:
                    rte._rest_contact_b2006_active = False
                    rte._t_b2103_heal = 0.0
                    rte.set("wc_ack_pos_fault", False)
                    self._dtc.set_inactive("B2006")
                    print("[B2006] INACTIVE : PosSensor fault disparu >= 1s")
            else:
                rte._t_b2103_heal = 0.0

    def thread_diagnostic(self):
        """Thread T-DIAG -- Traitement UDS + test actionneur + watchdog."""
        print("[THREAD T-DIAG] Demarre | mode Event (zero polling)")
        rte = self._rte

        while self._running:
            signaled = rte._uds_event.wait(timeout=ACTUATOR_TEST_PERIOD)

            if not self._running:
                break

            if signaled and rte.uds_request_pending:
                rte._uds_event.clear()
                self._process_uds_request()

            if rte._test_active:
                elapsed = time.time() - rte._t_test_start
                if elapsed >= rte._test_duration:
                    self._stop_test()

            self._watchdog_check()

    # ==================================================
    # DEMARRAGE / ARRET
    # ==================================================

    def start(self):
        self._running = True
        self._tcp.start()
        self._ws.start()
        self._tcp_pump.start()

    def stop(self):
        self._running = False
        self._tcp.stop()
        self._ws.stop()
        self._tcp_pump.stop()
        self._stop_all()
        _gpio_cleanup()
        print("[ApplicationLayer] Arretee proprement")