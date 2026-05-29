#!/usr/bin/env python3
"""
bcm_control.py  --  RPi BCM : Controle Pompe et Moteur + lecture ADS1115
=========================================================================
Script autonome a lancer DIRECTEMENT sur le RPi BCM.

REVISION : Pompe uniquement pour les modes defaut
  [POMPE_ONLY] SPDT (GPIO12), ISO_MOT (GPIO17), ISO_DEF (GPIO27) supprimes
               du rpisimulator -> aucun changement GPIO ici (bcm ne les pilote pas)
               mais les constantes et commentaires sont mis a jour
  [POMPE_ONLY] ADS1115 A3 -> POT moteur connexion directe permanente
               (ISO_MOT supprime -> plus d'isolation pendant defaut moteur)
  [POMPE_ONLY] PIN_PUMP_BWD reste GPIO18 (IN2 L298N) -- a ne PAS confondre
               avec GPIO18 rpisimulator (relais ISO) : deux RPi differents !

PROTECTION ADS1115 (rappel, geree par sim_control.py) :
  GPIO18 rpisimulator (ISO) doit etre HIGH avant tout signal de defaut.
  Cette protection est entierement logicielle dans sim_control.py.
  bcm_control.py n'a pas acces aux GPIO du rpisimulator.

Fonctions :
  - Controle pompe FWD / BWD / STOP (L298N via rpibcm)
  - Controle moteur avant speed1/speed2/stop (relais 4CH)
  - Controle moteur arriere ON/OFF
  - Lecture ADS1115 : tension noeud B (pompe A0) + tension POT moteur (A3)
  - Calibration mode Signal Variable moteur
  - Monitoring continu
"""

import argparse
import sys
import time

# ------------------------------------------------------------------------------
# CONFIGURATION GPIO (rpibcm uniquement)
# ------------------------------------------------------------------------------
PIN_RELAY_FRONT_ON    = 21   # Relais moteur avant ON/OFF (actif bas)
PIN_RELAY_FRONT_SPEED = 23   # Relais moteur avant vitesse (actif bas)
PIN_RELAY_REAR_ON     = 20   # Relais moteur arriere ON/OFF (actif bas)
PIN_PUMP_FWD          = 24   # L298N IN1 : direction pompe FWD
PIN_PUMP_BWD          = 18   # L298N IN2 : direction pompe BWD
                              # [NOTE] GPIO18 rpibcm = IN2 L298N
                              #        GPIO18 rpisimulator = relais ISO (deux RPi distincts !)

RELAY_ON     = 0   # Relais actif bas -> LOW = ON
RELAY_OFF    = 1
RELAY_SPEED1 = 1   # Speed1 lente (relais OFF)
RELAY_SPEED2 = 0   # Speed2 rapide (relais ON)

# ------------------------------------------------------------------------------
# CONSTANTES ADS1115 / DIVISEUR DE TENSION
# ------------------------------------------------------------------------------
ADS_GAIN            = 1
ADS_CHANNEL_MOTOR   = 3    # A3 : POT moteur (connexion directe permanente, ISO_MOT supprime)
ADS_PUMP_CHANNEL    = 0    # A0 : noeud B pompe (via relais ISO rpisimulator)

# Diviseur de tension pompe : 10kOhm / 2.2kOhm
ADS_PUMP_R_HAUTE    = 10000.0
ADS_PUMP_R_BASSE    =  2200.0
ADS_PUMP_R_CHARGE   =    10.0   # Resistance de charge serie (noeud A)
ADS_PUMP_RATIO_DIV  = ADS_PUMP_R_BASSE / (ADS_PUMP_R_HAUTE + ADS_PUMP_R_BASSE)

# Parametres filtrage ADS (rejet des valeurs aberrantes)
ADS_NB_SAMPLES      = 20
ADS_REJECT_PERCENT  = 0.2

# ------------------------------------------------------------------------------
# CALIBRATION MODE SIGNAL VARIABLE
# V_VAR_MAX : tension maximale mesurable sur A3 en mode variable (POT CTRL a fond)
# mesuree experimentalement a 1.21V
# ------------------------------------------------------------------------------
V_VAR_MAX           = 1.21
_v_moteur_ref       = 3.3    # Mise a jour par calibrer_moteur_variable()
_calibration_faite  = False

# ------------------------------------------------------------------------------
# INIT GPIO (rpibcm)
# ------------------------------------------------------------------------------
GPIO_AVAILABLE = False
GPIO = None

try:
    import RPi.GPIO as GPIO
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    GPIO.setup(PIN_RELAY_FRONT_ON,    GPIO.OUT, initial=RELAY_OFF)
    GPIO.setup(PIN_RELAY_FRONT_SPEED, GPIO.OUT, initial=RELAY_SPEED1)
    GPIO.setup(PIN_RELAY_REAR_ON,     GPIO.OUT, initial=RELAY_OFF)
    GPIO.setup(PIN_PUMP_FWD,          GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(PIN_PUMP_BWD,          GPIO.OUT, initial=GPIO.LOW)
    GPIO_AVAILABLE = True
    print("[GPIO] RPi.GPIO initialise (rpibcm)")
    print(f"  PUMP_FWD=GPIO{PIN_PUMP_FWD}  PUMP_BWD=GPIO{PIN_PUMP_BWD} (IN1/IN2 L298N)")
    print(f"  RL_FRONT_ON=GPIO{PIN_RELAY_FRONT_ON}  RL_FRONT_SPEED=GPIO{PIN_RELAY_FRONT_SPEED}  RL_REAR_ON=GPIO{PIN_RELAY_REAR_ON}")
    print(f"  [NOTE] GPIO18 ici = IN2 L298N (BWD pompe)")
    print(f"  [NOTE] GPIO18 rpisimulator = relais ISO (autre RPi -- gere par sim_control.py)")
except ImportError:
    print("[GPIO] RPi.GPIO non disponible -- mode simulation")
except Exception as e:
    print(f"[GPIO] Erreur init: {e} -- mode simulation")

# ------------------------------------------------------------------------------
# INIT ADS1115
# A0 : noeud B pompe (via relais ISO rpisimulator -- connexion active relais ON)
# A3 : POT moteur (connexion directe permanente -- ISO_MOT supprime)
# ------------------------------------------------------------------------------
ADS_AVAILABLE = False
_ads_motor_ch = None
_ads_pump_ch  = None

try:
    import board
    import busio
    from adafruit_ads1x15 import ads1115 as ADS
    from adafruit_ads1x15.analog_in import AnalogIn
    _i2c          = busio.I2C(board.SCL, board.SDA)
    _ads          = ADS.ADS1115(_i2c, address=0x48)
    _ads.gain     = ADS_GAIN
    _ads_motor_ch = AnalogIn(_ads, ADS_CHANNEL_MOTOR)
    _ads_pump_ch  = AnalogIn(_ads, ADS_PUMP_CHANNEL)
    ADS_AVAILABLE = True
    print(f"[ADS1115] Initialise 0x48")
    print(f"  A{ADS_PUMP_CHANNEL} = noeud B pompe (via ISO rpisimulator)")
    print(f"  A{ADS_CHANNEL_MOTOR} = POT moteur direct permanent (ISO_MOT supprime)")
    print(f"  RATIO_DIV={ADS_PUMP_RATIO_DIV:.4f}  R_CHARGE={ADS_PUMP_R_CHARGE} ohm")
    print(f"  FILTRAGE: {ADS_NB_SAMPLES} samples  reject={int(ADS_REJECT_PERCENT*100)}%")
except ImportError:
    print("[ADS1115] Non disponible -- valeurs simulees a 0.0")
except Exception as e:
    print(f"[ADS1115] Erreur init: {e} -- valeurs simulees a 0.0")


# ------------------------------------------------------------------------------
# LECTURE STABILISEE ADS1115
# ------------------------------------------------------------------------------
def _lire_channel_stable(channel, nb_samples=None, reject_percent=None):
    """
    Lit nb_samples echantillons, trie, rejette les extremes, retourne la moyenne.
    """
    if nb_samples     is None: nb_samples     = ADS_NB_SAMPLES
    if reject_percent is None: reject_percent = ADS_REJECT_PERCENT

    lectures = []
    for _ in range(nb_samples):
        lectures.append(abs(channel.voltage))
        time.sleep(0.005)

    lectures.sort()
    n_reject = int(nb_samples * reject_percent)
    lectures_filtrees = lectures[n_reject : nb_samples - n_reject]

    if not lectures_filtrees:
        return 0.0
    return sum(lectures_filtrees) / len(lectures_filtrees)


# ------------------------------------------------------------------------------
# CALIBRATION MODE SIGNAL VARIABLE MOTEUR
# ------------------------------------------------------------------------------
def calibrer_moteur_variable():
    """
    Lit la tension reelle du POT moteur sur A3 en mode normal.
    Doit etre appele avant d'entrer en mode SIGNAL VARIABLE (cote rpisimulator).
    Le rpisimulator doit etre en mode NORMAL au moment de l'appel.

    [POMPE_ONLY] A3 est maintenant en connexion directe permanente (ISO_MOT supprime).
                 La lecture est donc toujours disponible, meme en mode defaut pompe.
    """
    global _v_moteur_ref, _calibration_faite

    if not ADS_AVAILABLE or _ads_motor_ch is None:
        print("[CALIB] ADS non disponible -- valeur par defaut 3.3V utilisee")
        _v_moteur_ref      = 3.3
        _calibration_faite = True
        return _v_moteur_ref

    print("[CALIB] Lecture tension POT moteur sur A3 en cours...")
    v = _lire_channel_stable(_ads_motor_ch, nb_samples=30, reject_percent=0.2)
    if v < 0.05:
        print(f"[CALIB] Attention tension lue tres basse ({v:.4f}V) -- verifier POT moteur")
    _v_moteur_ref      = round(v, 4)
    _calibration_faite = True
    print(f"[CALIB] V_moteur_ref = {_v_moteur_ref:.4f} V  facteur = {_v_moteur_ref / V_VAR_MAX:.4f}")
    return _v_moteur_ref


# ------------------------------------------------------------------------------
# LECTURE TENSIONS
# ------------------------------------------------------------------------------
def lire_tension_moteur(mode_variable=False) -> float:
    """
    Lit la tension sur A3 (POT moteur, connexion directe permanente).
    mode_variable=True : applique la correction de calibration
                         pour remapper 0->V_VAR_MAX vers 0->V_moteur_ref
    """
    if not ADS_AVAILABLE or _ads_motor_ch is None:
        return 0.0
    try:
        v = _lire_channel_stable(_ads_motor_ch)
        if mode_variable and _calibration_faite and V_VAR_MAX > 0:
            facteur = _v_moteur_ref / V_VAR_MAX
            v = v * facteur
        return round(v, 4)
    except Exception as e:
        print(f"[ADS] Erreur lecture moteur A3: {e}")
        return 0.0


def lire_pompe(retries=3):
    """
    Lit la tension sur A0 (noeud B pompe, via relais ISO rpisimulator).
    Retourne (v_noeud_B, v_noeud_A, courant_A).

    [NOTE] En mode defaut pompe actif : ISO ouvert -> A0 recoit le signal injecte
           par le CD4051 (open load / short vcc / variable load / short to gnd).
           La lecture ici refletre donc le defaut injecte si rpisimulator en mode defaut.
    [NOTE] En BACKWARD : tension noeud B ~ -1.08V.
           [C6] pompe_backward() est bloquee si un mode defaut est actif (a enforcer
           dans la logique applicative appelante -- bcm_control.py ne connait pas
           l'etat du rpisimulator).
    """
    for i in range(retries):
        if not ADS_AVAILABLE or _ads_pump_ch is None:
            return 0.0, 0.0, 0.0
        try:
            v_b     = _lire_channel_stable(_ads_pump_ch)
            v_a     = v_b / ADS_PUMP_RATIO_DIV
            courant = v_a / ADS_PUMP_R_CHARGE
            return round(v_b, 4), round(v_a, 4), round(courant, 3)
        except OSError as e:
            print(f"[ADS PUMP] Erreur I2C tentative {i+1}/{retries}: {e}")
            time.sleep(0.1)

    print(f"[ADS PUMP] Lecture echouee apres {retries} tentatives")
    return 0.0, 0.0, 0.0


# ------------------------------------------------------------------------------
# CONTROLE POMPE
# [C3][C6] pompe_start("bwd") interdit si mode defaut actif (a enforcer en amont)
# ------------------------------------------------------------------------------
_pump_active    = False
_pump_direction = 0


def pompe_start(direction: str = "fwd"):
    global _pump_active, _pump_direction
    dir_int = 1 if direction.lower() in ("fwd", "forward", "avant") else 2

    # Securite : couper les deux avant commutation
    if GPIO_AVAILABLE:
        GPIO.output(PIN_PUMP_FWD, GPIO.LOW)
        GPIO.output(PIN_PUMP_BWD, GPIO.LOW)
    time.sleep(0.1)

    if dir_int == 1:
        if GPIO_AVAILABLE:
            GPIO.output(PIN_PUMP_FWD, GPIO.HIGH)
            GPIO.output(PIN_PUMP_BWD, GPIO.LOW)
        else:
            print(f"    [SIM] GPIO{PIN_PUMP_FWD}=HIGH  GPIO{PIN_PUMP_BWD}=LOW")
        print("[POMPE] FORWARD")
    else:
        if GPIO_AVAILABLE:
            GPIO.output(PIN_PUMP_FWD, GPIO.LOW)
            GPIO.output(PIN_PUMP_BWD, GPIO.HIGH)
        else:
            print(f"    [SIM] GPIO{PIN_PUMP_FWD}=LOW  GPIO{PIN_PUMP_BWD}=HIGH")
        print("[POMPE] BACKWARD")
        print("[WARN] Tension noeud B ~ -1.08V en BACKWARD -- verifier qu'aucun mode defaut n'est actif sur rpisimulator")

    _pump_active    = True
    _pump_direction = dir_int


def pompe_stop():
    global _pump_active, _pump_direction
    if GPIO_AVAILABLE:
        GPIO.output(PIN_PUMP_FWD, GPIO.LOW)
        GPIO.output(PIN_PUMP_BWD, GPIO.LOW)
    else:
        print(f"    [SIM] GPIO{PIN_PUMP_FWD}=LOW  GPIO{PIN_PUMP_BWD}=LOW")
    _pump_active    = False
    _pump_direction = 0
    print("[POMPE] STOP")


# ------------------------------------------------------------------------------
# CONTROLE MOTEUR
# ------------------------------------------------------------------------------
_motor_front_active = False
_motor_rear_active  = False


def moteur_avant_start(speed: str = "speed1"):
    global _motor_front_active
    speed_val = RELAY_SPEED1 if speed.lower() == "speed1" else RELAY_SPEED2
    if GPIO_AVAILABLE:
        GPIO.output(PIN_RELAY_FRONT_SPEED, speed_val)
        GPIO.output(PIN_RELAY_FRONT_ON,    RELAY_ON)
    label = "Speed1 lente" if speed.lower() == "speed1" else "Speed2 rapide"
    print(f"[MOTEUR AVANT] {label}")
    _motor_front_active = True


def moteur_avant_stop():
    global _motor_front_active
    if GPIO_AVAILABLE:
        GPIO.output(PIN_RELAY_FRONT_ON,    RELAY_OFF)
        GPIO.output(PIN_RELAY_FRONT_SPEED, RELAY_SPEED1)
    print("[MOTEUR AVANT] STOP")
    _motor_front_active = False


def moteur_arriere_start():
    global _motor_rear_active
    if GPIO_AVAILABLE:
        GPIO.output(PIN_RELAY_REAR_ON, RELAY_ON)
    print("[MOTEUR ARRIERE] ON")
    _motor_rear_active = True


def moteur_arriere_stop():
    global _motor_rear_active
    if GPIO_AVAILABLE:
        GPIO.output(PIN_RELAY_REAR_ON, RELAY_OFF)
    print("[MOTEUR ARRIERE] STOP")
    _motor_rear_active = False


# ------------------------------------------------------------------------------
# AFFICHAGE MESURES
# ------------------------------------------------------------------------------
def afficher_mesures_once(mode_variable_moteur=False):
    v_b, v_a, ip = lire_pompe()
    v_mot = lire_tension_moteur(mode_variable=mode_variable_moteur)

    ep = "FWD"  if (_pump_active and _pump_direction == 1) else \
         "BWD"  if (_pump_active and _pump_direction == 2) else \
         "STOP"
    em = "ON" if _motor_front_active else "OFF"
    mv = " [VAR CORR]" if mode_variable_moteur else ""

    print("")
    print("+" + "=" * 50 + "+")
    print("|  MESURES -- Pompe (defaut) + Moteur (direct)    |")
    print("+" + "=" * 50 + "+")
    print(f"|  [POMPE]  etat {ep:<35}|")
    print(f"|    Tension noeud B (A0) : {v_b:+.4f} V              |")
    print(f"|    Tension noeud A      : {v_a:+.4f} V              |")
    print(f"|    Courant pompe        : {ip:.4f} A              |")
    print(f"|  [MOTEUR] etat {em:<35}|")
    print(f"|    Tension POT A3 (direct) : {v_mot:.4f} V{mv:<11}|")
    print("+" + "=" * 50 + "+")
    print(f"  [INFO] A0 = signal defaut si ISO rpisimulator ouvert")
    print(f"  [INFO] A3 = POT moteur direct permanent (ISO_MOT supprime)")
    print("")


def monitor_continu(interval: float = 0.5, mode_variable_moteur=False):
    print("\n[MONITOR] Lecture en continu -- Ctrl+C pour arreter\n")
    try:
        while True:
            afficher_mesures_once(mode_variable_moteur=mode_variable_moteur)
            time.sleep(interval)
    except KeyboardInterrupt:
        print("\n[MONITOR] Arrete.")


# ------------------------------------------------------------------------------
# MENU INTERACTIF
# ------------------------------------------------------------------------------
def _etat():
    p = ("FWD" if _pump_direction == 1 else "BWD") if _pump_active else "OFF"
    m = "ON" if _motor_front_active else "OFF"
    c = f"  V_ref={_v_moteur_ref:.4f}V" if _calibration_faite else ""
    return f"Pompe={p}  Moteur avant={m}{c}"


def menu_interactif():
    print("\n" + "=" * 54)
    print("  RPi BCM -- Controle Pompe + Moteur")
    print("  Modes defaut : geres par sim_control.py (rpisimulator)")
    print("=" * 54)
    print(f"  GPIO disponible    : {'OUI' if GPIO_AVAILABLE else 'NON simulation'}")
    print(f"  ADS1115 disponible : {'OUI' if ADS_AVAILABLE  else 'NON valeurs 0.0'}")
    print(f"  A0 = pompe (via ISO)  |  A3 = moteur (direct permanent)\n")

    while True:
        print(f"\n  Etat : {_etat()}")
        print()
        print("  [1]  Pompe FWD")
        print("  [2]  Pompe BWD  (verifier aucun defaut actif sur rpisimulator)")
        print("  [3]  Pompe STOP")
        print("  [4]  Moteur avant Speed1 lente")
        print("  [5]  Moteur avant Speed2 rapide")
        print("  [6]  Moteur avant STOP")
        print("  [7]  Moteur arriere ON")
        print("  [8]  Moteur arriere STOP")
        print("  [9]  Afficher mesures une fois")
        print("  [10] Monitoring continu (Ctrl+C pour revenir)")
        print("  [11] Calibrer mode Signal Variable moteur")
        print("  [12] Monitoring avec correction Variable moteur")
        print("  [0]  Quitter\n")

        try:
            choix = input("  Choix : ").strip()
        except KeyboardInterrupt:
            break

        if   choix == "0":  break
        elif choix == "1":  pompe_start("fwd")
        elif choix == "2":  pompe_start("bwd")
        elif choix == "3":  pompe_stop()
        elif choix == "4":  moteur_avant_start("speed1")
        elif choix == "5":  moteur_avant_start("speed2")
        elif choix == "6":  moteur_avant_stop()
        elif choix == "7":  moteur_arriere_start()
        elif choix == "8":  moteur_arriere_stop()
        elif choix == "9":  afficher_mesures_once()
        elif choix == "10": monitor_continu(interval=0.5)
        elif choix == "11": calibrer_moteur_variable()
        elif choix == "12": monitor_continu(interval=0.5, mode_variable_moteur=True)
        else: print("  Choix invalide")

    print("\n[BCM] Arret securise...")
    pompe_stop()
    moteur_avant_stop()
    moteur_arriere_stop()
    if GPIO_AVAILABLE:
        GPIO.cleanup()
    print("[BCM] Termine.")


# ------------------------------------------------------------------------------
# ENTRY POINT
# ------------------------------------------------------------------------------
def main():
    p = argparse.ArgumentParser(
        description="RPi BCM -- Controle pompe et moteur + lecture ADS1115",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            "Exemples :\n"
            "  python3 bcm_control.py\n"
            "  python3 bcm_control.py --pump fwd\n"
            "  python3 bcm_control.py --pump stop\n"
            "  python3 bcm_control.py --motor speed1\n"
            "  python3 bcm_control.py --monitor\n"
            "  python3 bcm_control.py --monitor --interval 0.2\n"
            "  python3 bcm_control.py --calibrate\n"
            "  python3 bcm_control.py --calibrate --monitor\n"
        ),
    )
    p.add_argument("--pump",       choices=["fwd", "bwd", "stop"], help="Commande pompe")
    p.add_argument("--motor",      choices=["speed1", "speed2", "stop"], help="Commande moteur avant")
    p.add_argument("--motor-rear", choices=["on", "off"], help="Commande moteur arriere")
    p.add_argument("--monitor",    action="store_true", help="Lecture en continu")
    p.add_argument("--interval",   type=float, default=0.5,
                   help="Intervalle refresh monitor en secondes (defaut 0.5)")
    p.add_argument("--calibrate",  action="store_true",
                   help="Calibrer mode Signal Variable moteur avant monitoring")
    args = p.parse_args()

    action_done = False
    mode_var    = False

    if args.calibrate:
        calibrer_moteur_variable()
        mode_var    = True
        action_done = True

    if args.pump:
        if args.pump == "stop": pompe_stop()
        else: pompe_start(args.pump)
        action_done = True

    if args.motor:
        if args.motor == "stop": moteur_avant_stop()
        else: moteur_avant_start(args.motor)
        action_done = True

    if args.motor_rear:
        if args.motor_rear == "on": moteur_arriere_start()
        else: moteur_arriere_stop()
        action_done = True

    if args.monitor:
        monitor_continu(args.interval, mode_variable_moteur=mode_var)
        action_done = True

    if not action_done:
        menu_interactif()
    else:
        if GPIO_AVAILABLE:
            GPIO.cleanup()


if __name__ == "__main__":
    main()
