#!/usr/bin/env python3

import glob
import socket
import struct
import threading
import time

import serial

try:
    import can
    CAN_AVAILABLE = True
except ImportError:
    CAN_AVAILABLE = False

from bcm_rte import (
    RTE,
    LIN_SYNC, LIN_BREAK,
    LIN_PORT_CANDIDATES,
    CAN_RECV_TIMEOUT, CAN_IDLE_SLEEP,
    WOP_OFF, WOP_AUTO, WOP_REAR_WASH, WOP_REAR_WIPE, WOP_NAMES,
    ST_OFF,
)
# Constantes LIN/CAN dynamiques — mises à jour par load_ldf_config() et load_dbc_config()
import bcm_rte as _bcm_rte

# =====================================================
# CONSTANTES DoIP (ISO 13400)
# =====================================================
DOIP_PORT             = 13400
DOIP_PROTOCOL_VERSION = 0x02
DOIP_INVERSE_VERSION  = 0xFD
DOIP_VEHICLE_ID_REQ  = 0x0001
DOIP_VEHICLE_ID_RES  = 0x0004
DOIP_ROUTING_ACT_REQ = 0x0005
DOIP_ROUTING_ACT_RES = 0x0006
DOIP_ALIVE_CHECK_REQ = 0x0007
DOIP_ALIVE_CHECK_RES = 0x0008
DOIP_DIAGNOSTIC_MSG  = 0x8001

_PTYPE_NAMES = {
    DOIP_VEHICLE_ID_REQ  : "VehicleIdRequest",
    0x0002               : "VehicleIdRequest(EID)",
    0x0003               : "VehicleIdRequest(VIN)",
    DOIP_VEHICLE_ID_RES  : "VehicleIdResponse",
    DOIP_ROUTING_ACT_REQ : "RoutingActivationRequest",
    DOIP_ROUTING_ACT_RES : "RoutingActivationResponse",
    DOIP_ALIVE_CHECK_REQ : "AliveCheckRequest",
    DOIP_ALIVE_CHECK_RES : "AliveCheckResponse",
    DOIP_DIAGNOSTIC_MSG  : "DiagnosticMessage",
}

# Adresses ECU (Diagnostic Specification Section 1)
BCM_ADDR    = 0x0700
TESTER_ADDR = 0x07DF

VIN = b"BCM_WIPEWASH12345"   # 17 octets

# =====================================================
# UDS SERVICE IDs -- exportes pour bcm_application.py
# =====================================================
SID_DSC   = 0x10   # DiagnosticSessionControl
SID_RESET = 0x11   # ECUReset
SID_CLEAR = 0x14   # ClearDiagnosticInformation
SID_RDTC  = 0x19   # ReadDTCInformation
SID_RDID  = 0x22   # ReadDataByIdentifier
SID_WDID  = 0x2E   # WriteDataByIdentifier
SID_SA    = 0x27   # SecurityAccess
SID_CC    = 0x28   # CommunicationControl
SID_RC    = 0x31   # RoutineControl
SID_TP    = 0x3E   # TesterPresent

DSC_DEFAULT  = 0x01
DSC_EXTENDED = 0x03

_SID_NAMES = {
    SID_DSC   : "DiagnosticSessionControl",
    SID_RESET : "ECUReset",
    SID_CLEAR : "ClearDTC",
    SID_RDTC  : "ReadDTCInformation",
    SID_RDID  : "ReadDataByIdentifier",
    SID_WDID  : "WriteDataByIdentifier",
    SID_SA    : "SecurityAccess",
    SID_CC    : "CommunicationControl",
    SID_RC    : "RoutineControl",
    SID_TP    : "TesterPresent",
}

_DSC_NAMES = {DSC_DEFAULT: "Default(0x01)", DSC_EXTENDED: "Extended(0x03)"}

_NRC_NAMES = {
    0x10: "generalReject",
    0x11: "serviceNotSupported",
    0x12: "subFunctionNotSupported",
    0x13: "incorrectMessageLength",
    0x22: "conditionsNotCorrect",
    0x24: "requestSequenceError",
    0x31: "requestOutOfRange",
    0x33: "securityAccessDenied",
    0x35: "invalidKey",
    0x7E: "subFunctionNotSupportedInActiveSession",
    0x7F: "serviceNotSupportedInActiveSession",
}

# Timeout attente reponse T-DIAG (secondes)
DOIP_RESPONSE_TIMEOUT = 1.000


# =====================================================
# CALCUL PID LIN (ISO 17987)
# =====================================================
def calculate_pid(frame_id: int) -> int:
    """
    Calcule le PID (Protected IDentifier) d'une trame LIN.
    Structure PID : bits 5:0 = frame_id, bit 6 = P0, bit 7 = P1.
    """
    if frame_id > 0x3F:
        raise ValueError(f"Frame ID doit etre 6 bits (0-63), recu: {frame_id:#04x}")
    p0 = (frame_id ^ (frame_id >> 1) ^ (frame_id >> 2) ^ (frame_id >> 4)) & 0x01
    p1 = ~((frame_id >> 1) ^ (frame_id >> 3) ^ (frame_id >> 4) ^ (frame_id >> 5)) & 0x01
    return (frame_id & 0x3F) | (p0 << 6) | (p1 << 7)


def lin_checksum(pid: int, data: bytes) -> int:
    """
    Calcule le checksum LIN Enhanced (ISO 17987).
    Somme PID + donnees avec carry-around, puis complement a 1.
    """
    total = pid
    for b in data:
        total += b
        if total > 0xFF:
            total -= 0xFF
    return (~total) & 0xFF


# =====================================================
# HELPERS TRACE DoIP / UDS
# =====================================================
def _fmt_hex(data: bytes, max_bytes: int = 16) -> str:
    if not data:
        return "(vide)"
    spaced = " ".join(f"{b:02X}" for b in data[:max_bytes])
    if len(data) > max_bytes:
        return f"{spaced} ... ({len(data)} octets)"
    return spaced


def _decode_uds_request(uds: bytes) -> str:
    if not uds:
        return "(vide)"
    sid  = uds[0]
    name = _SID_NAMES.get(sid, f"SID=0x{sid:02X}")

    if sid == SID_DSC and len(uds) >= 2:
        sub = uds[1] & 0x7F
        sup = " [suppressResp]" if uds[1] & 0x80 else ""
        return f"{name}  sub={_DSC_NAMES.get(sub, f'0x{sub:02X}')}{sup}"

    elif sid == SID_RESET and len(uds) >= 2:
        types = {0x01: "hardReset", 0x02: "keyOffOnReset", 0x03: "softReset"}
        return f"{name}  type={types.get(uds[1], f'0x{uds[1]:02X}')}"

    elif sid == SID_SA and len(uds) >= 2:
        subs = {0x01: "RequestSeed", 0x02: "SendKey"}
        sub_name = subs.get(uds[1], f"0x{uds[1]:02X}")
        extra = ""
        if uds[1] == 0x02 and len(uds) >= 4:
            key = (uds[2] << 8) | uds[3]
            extra = f"  key=0x{key:04X}"
        return f"{name}  sub={sub_name}{extra}"

    elif sid == SID_RDID and len(uds) >= 3:
        did = (uds[1] << 8) | uds[2]
        return f"{name}  DID=0x{did:04X}"

    elif sid == SID_WDID and len(uds) >= 4:
        did = (uds[1] << 8) | uds[2]
        val = uds[3]
        return f"{name}  DID=0x{did:04X}  val=0x{val:02X}({val})"

    elif sid == SID_RDTC and len(uds) >= 2:
        subs = {0x02: "reportDTCByStatusMask",
                0x04: "reportDTCSnapshotRecord",
                0x06: "reportDTCExtDataRecord"}
        return f"{name}  sub={subs.get(uds[1], f'0x{uds[1]:02X}')}"

    elif sid == SID_CLEAR and len(uds) >= 4:
        grp = (uds[1] << 16) | (uds[2] << 8) | uds[3]
        label = "allDTCs" if grp == 0xFFFFFF else f"group=0x{grp:06X}"
        return f"{name}  {label}"

    elif sid == SID_RC and len(uds) >= 4:
        sub_names = {0x01: "startRoutine",
                     0x02: "stopRoutine",
                     0x03: "requestResults"}
        rid      = (uds[2] << 8) | uds[3]
        if len(uds) >= 5:
            if rid == 0x0205:
                extra = f"  rain_intensity={uds[4]}"
            else:
                extra = f"  duration={uds[4]}s"
        else:
            extra = "  duration=?"
        return (f"{name}  sub={sub_names.get(uds[1], f'0x{uds[1]:02X}')}"
                f"  RID=0x{rid:04X}{extra}")

    elif sid == SID_CC and len(uds) >= 3:
        subs = {0x00: "enableRxTx", 0x01: "enableRxDisableTx",
                0x02: "disableRxEnableTx", 0x03: "disableRxTx"}
        return (f"{name}  sub={subs.get(uds[1], f'0x{uds[1]:02X}')}"
                f"  commType=0x{uds[2]:02X}")

    elif sid == SID_TP and len(uds) >= 2:
        sup = " [suppressResp]" if uds[1] & 0x80 else ""
        return f"{name}{sup}"

    return f"{name}  data={_fmt_hex(uds[1:], 8)}"


def _decode_uds_response(resp: bytes) -> str:
    if not resp:
        return "(vide)"
    b0 = resp[0]

    if b0 == 0x7F and len(resp) >= 3:
        req_sid  = resp[1]
        nrc_code = resp[2]
        req_name = _SID_NAMES.get(req_sid, f"0x{req_sid:02X}")
        nrc_name = _NRC_NAMES.get(nrc_code, f"0x{nrc_code:02X}")
        return f"NRC  service={req_name}  nrc={nrc_name}"

    if b0 == 0x50 and len(resp) >= 2:
        sub = resp[1]
        return f"DSC+  session={_DSC_NAMES.get(sub, f'0x{sub:02X}')}"

    if b0 == 0x51 and len(resp) >= 2:
        types = {0x01: "hardReset", 0x02: "keyOffOnReset", 0x03: "softReset"}
        return f"ECUReset+  type={types.get(resp[1], f'0x{resp[1]:02X}')}"

    if b0 == 0x67 and len(resp) >= 2:
        sub = resp[1]
        if sub == 0x01 and len(resp) >= 4:
            seed = (resp[2] << 8) | resp[3]
            if seed == 0:
                return "SA+  RequestSeed  [deja deverrouille -> seed=0x0000]"
            return f"SA+  RequestSeed  seed=0x{seed:04X}"
        if sub == 0x02:
            return "SA+  SendKey  [ACCES ACCORDE - secLevel=1]"
        return f"SA+  sub=0x{sub:02X}"

    if b0 == 0x62 and len(resp) >= 3:
        did = (resp[1] << 8) | resp[2]
        val = resp[3:]
        return f"RDID+  DID=0x{did:04X}  data={_fmt_hex(val)}"

    if b0 == 0x6E and len(resp) >= 3:
        did = (resp[1] << 8) | resp[2]
        return f"WDID+  DID=0x{did:04X}  [ecrit OK]"

    if b0 == 0x59:
        return f"RDTC+  {_fmt_hex(resp[1:], 12)}"

    if b0 == 0x54:
        return "ClearDTC+  [DTCs effaces]"

    if b0 == 0x68 and len(resp) >= 2:
        subs = {0x00: "enableRxTx", 0x01: "enableRxDisableTx",
                0x02: "disableRxEnableTx", 0x03: "disableRxTx"}
        return f"CC+  sub={subs.get(resp[1], f'0x{resp[1]:02X}')}"

    if b0 == 0x71 and len(resp) >= 4:
        sub_names = {0x01: "started", 0x02: "stopped", 0x03: "results"}
        rid = (resp[2] << 8) | resp[3]
        if len(resp) >= 5:
            if rid == 0x0205:
                extra = f"  rain_intensity={resp[4]}"
            else:
                extra = f"  duration={resp[4]}s"
        else:
            extra = ""
        return (f"RC+  {sub_names.get(resp[1], f'0x{resp[1]:02X}')}"
                f"  RID=0x{rid:04X}{extra}")

    if b0 == 0x7E:
        return "TesterPresent+  [session maintenue]"

    return f"0x{b0:02X}+  {_fmt_hex(resp[1:], 8)}"


# =====================================================
# COUCHE PROTOCOLE -- LIN + CAN + DoIP
# =====================================================
class ProtocolLayer:
    """
    Gere LIN, CAN et DoIP.

    RESPONSABILITE UNIQUE :
      Recevoir les trames des bus physiques.
      Decoder le contenu.
      Ecrire les donnees dans le RTE.
      Ne prend AUCUNE decision logique.
      Ne lit jamais rte.state pour agir.

    PRINCIPE UNIFORME :
      LIN  → rte.crs_wiper_op        → T-WSM  lit et agit
      CAN  → rte.rain_intensity       → T-WSM  lit et agit
      DoIP → rte.uds_payload         → T-DIAG lit et agit
             attend rte.uds_response  → renvoie au PC

    Threads :
      thread_lin_scheduler() → T-LIN  cycle 20ms (conforme spec)
      thread_can_receiver()  → T-CAN  bloquant
      run()                  → T-DOIP boucle TCP bloquante
      _udp_handler()         → DoIP_UDP thread daemon
    """

    def __init__(self, rte: RTE, dtc_manager):
        self._rte      = rte
        self._dtc      = dtc_manager
        self._running  = False
        self._lin_stop = threading.Event()  # signal arret propre T-LIN
        self.lin_port  = None
        self.can_bus   = None
        self._srv_sock = None
        self._udp_sock = None   # socket UDP DoIP discovery — ferme dans stop()

        # Creer et binder le socket TCP ici dans __init__
        # Evite "Address already in use" au redemarrage :
        # le socket est cree une seule fois, reste ouvert pendant toute la vie du processus.
        # run() fait seulement listen()+accept()  /  stop() fait shutdown() sans close()
        self._srv_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._srv_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            self._srv_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
        except AttributeError:
            pass
        self._srv_sock.bind(("0.0.0.0", DOIP_PORT))
        print(f"[DoIP] Socket TCP bind port {DOIP_PORT} OK")

        # Etat connexion DoIP (architecture backend.zip : single-client)
        self._client_connected = False
        self._client_address = None
        self._routing_active = False

        # Debounce CAN 0x300 / 0x301
        self._CAN_DEBOUNCE   = 0.150

        self._ign_stable     = None
        self._ign_pending    = None
        self._ign_t          = 0.0

        self._rev_stable     = None
        self._rev_pending    = None
        self._rev_t          = 0.0

        self._sensor_stable  = None
        self._sensor_pending = None
        self._sensor_t       = 0.0

        assert calculate_pid(_bcm_rte.LIN_ID_0x16) == _bcm_rte.LIN_PID_0x16, "PID frame 0x16 invalide"
        assert calculate_pid(_bcm_rte.LIN_ID_0x17) == _bcm_rte.LIN_PID_0x17, "PID frame 0x17 invalide"

    # ==================================================
    # SECTION A -- INIT HARDWARE
    # ==================================================

    def init_can(self):
        """Initialise l'interface CAN physique (SocketCAN)."""
        if not CAN_AVAILABLE:
            print("[CAN] python-can non disponible")
            return
        try:
            self.can_bus = can.interface.Bus(channel="can0", bustype="socketcan")
            print("[CAN] Interface can0 OK")
        except Exception as e:
            print(f"[CAN] Init echec: {e}")

    def init_lin(self):
        """Initialise le port serie UART pour le bus LIN."""
        LIN_BAUD = _bcm_rte.LIN_BAUD
        candidates = list(LIN_PORT_CANDIDATES)
        for p in sorted(glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*")):
            if p not in candidates:
                candidates.append(p)
        for port in candidates:
            try:
                self.lin_port = serial.Serial(
                    port=port, baudrate=LIN_BAUD,
                    timeout=0.020, write_timeout=0.200,
                    dsrdtr=False, rtscts=False,
                )
                import time as _t; _t.sleep(0.1)
                self.lin_port.reset_input_buffer()
                self.lin_port.reset_output_buffer()
                print(f"[LIN] Port {port} OK ({LIN_BAUD} baud) — config depuis LDF")
                return
            except Exception:
                continue
        print("[LIN] Aucun port serie -- mode simulation")

    # ==================================================
    # SECTION B -- THREAD LIN SCHEDULER (T-LIN)
    # ==================================================

    def thread_lin_scheduler(self):
        """
        T-LIN : ordonnanceur LIN master (BCM).
        Envoie les headers pour chaque frame du schedule LDF, lit les reponses slave.
        Tous les cycles, PIDs et noms de frames sont lus dynamiquement depuis le LDF.
        """
        # ── Récupérer la config complète depuis le LDF ─────────────────
        cfg      = _bcm_rte.LIN_LDF_CONFIG
        schedule = cfg["schedule"] if cfg else []

        # Construire le dictionnaire de timers : frame_name → last_t
        timers   = {fname: 0.0 for fname, _ in schedule}

        # Récupérer directement les noms/PIDs/DLCs depuis le LDF
        frames   = cfg["frames"] if cfg else {}

        # Noms symboliques des deux frames critiques (compatibilité)
        FRAME_16 = "LeftStickWiperRequester"
        FRAME_17 = "CRS_Status"

        LIN_CYCLE_0x16 = _bcm_rte.LIN_CYCLE_0x16
        LIN_CYCLE_0x17 = _bcm_rte.LIN_CYCLE_0x17

        print(f"[THREAD T-LIN] Demarre | LDF schedule : "
              + " | ".join(f"{n}={d*1000:.0f}ms" for n, d in schedule))

        while self._running and not self._lin_stop.is_set():
            now = time.time()

            # ── Parcourir chaque entrée du schedule LDF ─────────────────
            for frame_name, cycle_s in schedule:
                if now - timers[frame_name] >= cycle_s:
                    timers[frame_name] = now

                    if frame_name == FRAME_16:
                        self._lin_poll_frame(frame_name)
                    elif frame_name == FRAME_17:
                        # Interframe supplementaire si FRAME_16 vient d'etre envoye
                        if (now - timers.get(FRAME_16, 0)) < 0.100:
                            time.sleep(_bcm_rte.LIN_INTERFRAME)
                        timers[frame_name] = time.time()
                        self._lin_poll_frame(frame_name)
                    else:
                        # Frame LDF supplémentaire inconnue de la logique applicative
                        self._lin_poll_frame(frame_name)

                    time.sleep(_bcm_rte.LIN_INTERFRAME)
                    now = time.time()   # Rafraîchir après sleep

            self._check_lin_timeout()
            time.sleep(0.005)   # 5ms resolution scheduler

    # ── Primitives LIN ────────────────────────────────

    def _lin_flush_all(self):
        """
        Vide le buffer RX UART completement.
        Appele avant chaque header pour garantir qu'il n'y a pas de
        residus de trames precedentes qui pourraient polluer la lecture.
        """
        if not self.lin_port:
            return
        self.lin_port.reset_input_buffer()
        time.sleep(0.003)
        # Double flush : parfois le driver USB-Serial a encore des octets
        # dans son FIFO interne apres le premier reset
        self.lin_port.reset_input_buffer()

    def _lin_send_break(self):
        """
        Envoyer le BREAK LIN via baudrate/4 (dominant >= 13 bits).
        Strategie identique au code Arduino v7 :
          - Passer a baud/4
          - Envoyer un 0x00
          - Attendre la duree exacte du break + marge USB
          - Repasser a LIN_BAUD
        """
        if not self.lin_port:
            return
        self.lin_port.baudrate = _bcm_rte.LIN_BAUD // 4
        self.lin_port.write(bytes([LIN_BREAK]))
        self.lin_port.flush()
        # 13 bits @ LIN_BAUD/4 + 3ms marge USB-Serial adapter
        time.sleep(13.0 / (_bcm_rte.LIN_BAUD // 4) + 0.003)
        self.lin_port.baudrate = _bcm_rte.LIN_BAUD
        # Stabilisation baudrate (USB-CDC peut avoir un delai interne)
        time.sleep(0.002)

    def _lin_send_header(self, pid: int):
        """
        Envoyer le HEADER LIN complet : BREAK + SYNC(0x55) + PID.
        Le flush initial garantit qu'aucun residus de trame precedente
        ne pollue la fenetre de lecture de la reponse slave.
        """
        if not self.lin_port:
            return
        self._lin_flush_all()
        self._lin_send_break()
        self.lin_port.write(bytes([LIN_SYNC, pid]))
        self.lin_port.flush()
        # Pause minimale : le slave (Arduino/RPi) doit avoir le temps de
        # decoder le PID avant de commencer a repondre.
        # A 19200 baud : 1 octet = ~520us. SYNC+PID = ~1.04ms.
        # On attend 3ms supplementaires = marge USB-Serial.
        time.sleep(0.003)

    def _lin_read_byte(self, timeout_s: float) -> int:
        """
        Lire un octet depuis le bus LIN avec timeout.
        Retourne l'octet (0-255) ou -1 si timeout.
        Polling court (0.3ms) pour ne pas manquer un octet rapide.
        """
        if timeout_s <= 0:
            return -1
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            if self.lin_port and self.lin_port.in_waiting:
                return self.lin_port.read(1)[0]
            time.sleep(0.0003)
        return -1

    def _lin_read_response(self, pid: int) -> bytes:
        """
        Lire la reponse du slave apres envoi du header LIN.

        Architecture hardware : le BCM est MASTER LIN (half-duplex).
        Le transceiver TJA1020 cote BCM fait un loopback UART :
        les octets envoyes (BREAK + SYNC + PID) reapparaissent en RX.
        Ensuite arrivent les 3 octets du slave (data[0], data[1], cs).

        Strategie alignee sur le code Arduino CRS v7 (linReadHeader) :
          1. Drainer les 0x00 du BREAK (un ou plusieurs selon baud/4)
          2. Chercher 0x55 (SYNC) dans le flux (max 12 octets)
          3. Verifier l'echo du PID
          4. Lire les 3 octets de reponse slave
          5. Flush final + pause pour laisser les derniers octets arriver

        Timeout global 130ms :
          BREAK ~2.7ms + SYNC+PID ~1ms + slave reponse ~1.6ms + USB ~5ms
          + marge 120ms pour absorber les pics de latence USB-Serial.

        Retourne b"" si la reponse est invalide ou timeout.
        """
        BYTE_TMO  = 0.015    # 15ms par octet (USB-Serial P99 latency)
        FRAME_TMO = 0.130    # 130ms total frame window

        deadline = time.time() + FRAME_TMO

        # ── Etape 1 : drainer l'echo du BREAK (0x00 consecutifs) ────────
        # Le BREAK envoye a baud/4 est decode comme plusieurs 0x00 a 19200.
        # On consomme tous les 0x00 jusqu'au premier octet non-nul.
        b = -1
        while time.time() < deadline:
            b = self._lin_read_byte(min(BYTE_TMO, deadline - time.time()))
            if b < 0:
                # Aucun octet recu dans le timeout global : pas de loopback
                self._lin_flush_all()
                return b""
            if b != 0x00:
                break   # Premier octet non-nul : peut etre 0x55 ou autre

        if b < 0:
            self._lin_flush_all()
            return b""

        # ── Etape 2 : chercher l'echo du SYNC (0x55) ────────────────────
        # L'octet non-nul peut deja etre 0x55 (cas frequent).
        # Sinon on cherche sur les 12 prochains octets (robustesse).
        sync_found = (b == LIN_SYNC)
        if not sync_found:
            for _ in range(12):
                if time.time() >= deadline:
                    break
                b = self._lin_read_byte(min(BYTE_TMO, deadline - time.time()))
                if b < 0:
                    break
                if b == LIN_SYNC:
                    sync_found = True
                    break
                # Octet inattendu : continuer a chercher (peut etre residus)

        if not sync_found:
            self._lin_flush_all()
            return b""

        # ── Etape 3 : verifier l'echo du PID ────────────────────────────
        b = self._lin_read_byte(min(BYTE_TMO, deadline - time.time()))
        if b < 0:
            self._lin_flush_all()
            return b""
        if b != pid:
            # PID different : residus d'une trame precedente -> flush total
            self._lin_flush_all()
            return b""

        # ── Etape 4 : lire les 3 octets de reponse du slave ─────────────
        # data[0] = byte0 (wiper_op | stick_status), ou fault
        # data[1] = byte1 (alive counter), ou reserved
        # data[2] = checksum LIN enhanced
        resp = bytearray()
        for i in range(3):
            remaining = deadline - time.time()
            if remaining <= 0:
                self._lin_flush_all()
                return b""
            b = self._lin_read_byte(min(BYTE_TMO, remaining))
            if b < 0:
                # Timeout sur un octet de reponse slave
                self._lin_flush_all()
                return b""
            resp.append(b)

        # ── Etape 5 : flush final ────────────────────────────────────────
        # Pause 3ms pour laisser arriver les eventuels octets residuels
        # (echo TX du slave si son transceiver a aussi un loopback),
        # puis flush pour nettoyer avant la prochaine trame.
        time.sleep(0.003)
        if self.lin_port and self.lin_port.in_waiting:
            self.lin_port.reset_input_buffer()

        return bytes(resp)

    def _lin_poll_frame(self, frame_name: str):
        """
        Méthode générique de polling LIN, pilotée par le LDF.

        Remplace _lin_poll_0x16() et _lin_poll_0x17() statiques.
        Toutes les métadonnées (PID, DLC, nom du frame) sont lues
        dynamiquement depuis _bcm_rte.LIN_LDF_CONFIG, lui-même chargé
        depuis le fichier wiperwash.ldf au démarrage.

        Comportement selon frame_name :
          "LeftStickWiperRequester" → logique 0x16 (WiperRequestOperation, StickStatus, AliveCounter, DTC B2004)
          "CRS_Status"              → logique 0x17 (CRS_InternalFault_Stick/Supply/Comms + CRS_Version, non critique)
          autre (frame LDF future)  → poll brut sans traitement applicatif
        """
        if not self.lin_port:
            return

        # ── Récupérer PID et DLC depuis le LDF ──────────────────────────
        cfg    = _bcm_rte.LIN_LDF_CONFIG
        frames = cfg["frames"] if cfg else {}
        meta   = frames.get(frame_name)

        if meta is None:
            # Frame absente du LDF (ne devrait pas arriver si schedule cohérent)
            print(f"[LIN] Frame '{frame_name}' inconnue dans le LDF — ignorée")
            return

        pid = meta["pid"]
        dlc = meta["dlc"]
        fid = meta["id"]

        rte = self._rte

        # ── LeftStickWiperRequester (ex-0x16) ────────────────────────────
        if frame_name == "LeftStickWiperRequester":
            try:
                self._lin_send_header(pid)
                resp = self._lin_read_response(pid)

                if len(resp) < dlc + 1:
                    # Pas de reponse slave (silence) : ne pas mettre a jour t_last
                    # Le timeout sera detecte par _check_lin_timeout()
                    return

                data    = resp[:dlc]
                rx_cs   = resp[dlc]
                calc_cs = lin_checksum(pid, data)
                if rx_cs != calc_cs:
                    print(f"[LIN 0x{fid:02X}] Checksum KO rx=0x{rx_cs:02X} "
                          f"calc=0x{calc_cs:02X} -- trame ignoree")
                    # TC_LIN_CS : mémoriser le défaut de checksum pour le test
                    if not rte.lin_checksum_fault:
                        rte.set("lin_checksum_fault", True)
                    return

                # Décoder les signaux depuis les bits définis dans le LDF (0x16) :
                #   WiperRequestOperation : bits 3:0 de byte0  (start_bit=0, length=4) — Enum
                #   StickStatus           : bits 7:4 de byte0  (start_bit=4, length=4) — Bitfield
                #     StickStatus bit0 (byte0 bit4) = Valid     → trame valide
                #     StickStatus bit1 (byte0 bit5) = Debounce
                #     StickStatus bit2 (byte0 bit6) = Stuck     → contribue à B2011
                #   AliveCounter          : byte1               (start_bit=8, length=8) — Counter 0-255
                new_op          = data[0] & 0x0F
                stick_status    = (data[0] >> 4) & 0x0F
                new_alive       = data[1] if dlc >= 2 else 0
                stick_valid     = bool(stick_status & 0x01)   # bit0 du nibble haut = bit4 de byte0
                stick_debounce  = bool(stick_status & 0x02)   # bit1
                stick_stuck     = bool(stick_status & 0x04)   # bit2 = bit6 de byte0
                old_op          = rte.crs_wiper_op

                # ── Mise à jour bits StickStatus dans RTE ──────────────
                rte.set_multi(
                    crs_stick_valid    = stick_valid,
                    crs_stick_debounce = stick_debounce,
                    crs_stuck          = stick_stuck,
                )

                # Alive counter gele → DTC B2004 (slave bloque)
                if rte.crs_alive_prev != 0xFF and new_alive == rte.crs_alive_prev:
                    if not rte.lin_timeout_active:
                        print(f"[TSR_001] Alive counter gele: 0x{new_alive:02X} → B2004")
                        self._dtc.set_active("B2004", rte.make_snapshot())
                        rte.set("lin_timeout_active", True)
                        rte.set("lin_alive_fault", True)
                    return

                # Mise a jour RTE (sous lock unique pour atomicite)
                with rte._lock:
                    rte.crs_alive_prev  = new_alive
                    if hasattr(rte, "crs_alive_in"):
                        rte.crs_alive_in = new_alive
                    rte.crs_stick_valid = stick_valid
                    rte.t_last_lin0x16  = time.time()

                    # TC_FSR_010 / TC_CAN_003 : si lin_op_locked=True, le LIN ne peut
                    # pas écraser crs_wiper_op. Le test contrôle directement la commande
                    # via Redis pour que le BCM reste en ST_SPEED1 avec wc_available=True.
                    if rte.lin_op_locked:
                        return

                    ign_active = rte.ignition_status != 0
                    if not ign_active:
                        if new_op != WOP_OFF and rte.crs_wiper_op != WOP_OFF:
                            ign_name = {0: "OFF", 1: "ON/ACC", 2: "START"}.get(
                                rte.ignition_status, f"0x{rte.ignition_status:02X}")
                            print(f"[LIN 0x{fid:02X}] Ignition {ign_name} → "
                                  f"{WOP_NAMES.get(new_op,'?')} ignoree (SRD_WW_001)")
                        rte.crs_wiper_op    = WOP_OFF
                        rte._freeze_pending = False
                    elif new_op > 7:
                        # SRS_LIN_001 : WiperOp hors plage [0..7] → rejeté
                        print(f"[LIN 0x{fid:02X}] SRS_LIN_001 : WiperOp=0x{new_op:02X} "
                              f"hors plage [0..7] → commande ignorée, crs_wiper_op inchangé")
                    elif stick_valid:
                        # FIX : si le WSM a posé un verrou _freeze_pending après un cycle
                        # one-shot (TOUCH, FRONT_WASH, REAR_WASH), le LIN ne doit pas
                        # réécrire crs_wiper_op avec la même commande — sinon T-WSM voit
                        # l'opération comme une nouvelle demande alors que le levier n'a
                        # pas bougé (SRD_WW_020 : un seul cycle par impulsion).
                        # Le verrou est levé uniquement quand le LIN envoie WOP_OFF
                        # (levier relâché) ou une commande DIFFÉRENTE.
                        if rte._freeze_pending and new_op == rte._freeze_last_op:
                            # Levier toujours en position TOUCH/WASH : ignorer silencieusement
                            pass
                        else:
                            rte.crs_wiper_op = new_op
                            if new_op == WOP_OFF:
                                rte._freeze_pending = False
                    else:
                        # SRS_LIN_002 : stick_valid=False → WOP_OFF forcé
                        # + démarrer timer B2004 si pas déjà en cours
                        print(f"[LIN 0x{fid:02X}] SRS_LIN_002 : stickStatus invalide "
                              f"(0x{stick_status:02X}) → commande "
                              f"{WOP_NAMES.get(new_op,'?')} rejetée, WOP_OFF forcé")
                        rte.crs_wiper_op    = WOP_OFF
                        rte._freeze_pending = False
                        # Démarrer le timer B2004 au premier cycle bit4=0
                        if rte._t_b2004_invalid_start == 0.0:
                            rte._t_b2004_invalid_start = time.time()
                            print(f"[B2004] bit4=0 détecté — timer démarré (seuil 2.5s)")

                # ── B2004 : bit4=0 maintenu ≥ 2.5s → B2004 ACTIVE ──────────
                B2004_INVALID_DELAY = 2.500   # s — SRS_LIN_002
                if not stick_valid and rte._t_b2004_invalid_start > 0.0:
                    elapsed = time.time() - rte._t_b2004_invalid_start
                    if elapsed >= B2004_INVALID_DELAY and not rte.lin_timeout_active:
                        print(f"[B2004] bit4=0 pendant {elapsed:.2f}s ≥ 2.5s → B2004 ACTIVE")
                        self._dtc.set_active("B2004", rte.make_snapshot())
                        rte.set("lin_timeout_active", True)
                elif stick_valid:
                    # bit4 revenu à 1 → reset timer
                    if rte._t_b2004_invalid_start > 0.0:
                        print(f"[B2004] bit4=1 restauré → timer reset")
                        rte._t_b2004_invalid_start = 0.0

                # Retablissement communication → desactiver B2004
                if rte.lin_timeout_active and stick_valid:
                    print(f"[LIN 0x{fid:02X}] Communication retablie → B2004 inactif")
                    self._dtc.set_inactive("B2004")
                    rte.set("lin_timeout_active", False)
                    rte.set("lin_alive_fault", False)
                    rte._t_b2004_invalid_start = 0.0

                if old_op != rte.crs_wiper_op:
                    print(f"[LIN 0x{fid:02X}] WiperOp: {WOP_NAMES.get(old_op,'?')} → "
                          f"{WOP_NAMES.get(rte.crs_wiper_op,'?')} "
                          f"alive=0x{new_alive:02X} WSM={rte.state}")

            except serial.SerialTimeoutException:
                print(f"[LIN 0x{fid:02X}] Write timeout UART (overhead USB) -- pas de DTC")
            except serial.SerialException as e:
                print(f"[LIN 0x{fid:02X}] Erreur port serie: {e}")
                self._handle_lin_timeout()
            except Exception as e:
                print(f"[LIN 0x{fid:02X}] Exception inattendue: {e}")
                self._handle_lin_timeout()

        # ── CRS_Status (0x17) ────────────────────────────────────────────
        # Catalogue WW-MCAT-005 Rev5 :
        #   byte0 bit0 : CRS_InternalFault_Stick  (0=OK, 1=Fault)
        #   byte0 bit1 : CRS_InternalFault_Supply (0=OK, 1=Fault)
        #   byte0 bit2 : CRS_InternalFault_Comms  (0=OK, 1=Fault)
        #   byte0 bit3-7 : CRS_Reserved           (Set to 0)
        #   byte1       : CRS_Version             (version nominale=0x20, invalide=0xFF ignorée)
        #                 Toute valeur != 0x20 est rejetée (sauf 0xFF qui génère un log dédié).
        elif frame_name == "CRS_Status":
            try:
                self._lin_send_header(pid)
                resp = self._lin_read_response(pid)
                if len(resp) < dlc + 1:
                    return   # Pas de reponse slave pour 0x17 : non critique
                data  = resp[:dlc]
                rx_cs = resp[dlc]
                if lin_checksum(pid, data) != rx_cs:
                    print(f"[LIN 0x{fid:02X}] Checksum KO -- trame ignoree")
                    return

                # ── Filtre CRS_Version (byte1) ────────────────────────────
                # Seule la version nominale 0x20 est acceptée.
                # 0xFF : version invalide → trame ignorée (log dédié, pas de DTC).
                # Toute autre valeur ≠ 0x20 → trame ignorée (log avertissement).
                crs_version = data[1] if dlc >= 2 else 0x00
                if crs_version == 0xFF:
                    print(f"[LIN 0x{fid:02X}] CRS_Version=0xFF invalide — trame ignoree (pas de DTC)")
                    return
                if crs_version != 0x20:
                    print(f"[LIN 0x{fid:02X}] CRS_Version=0x{crs_version:02X} != 0x20 — trame rejetee")
                    return

                # ── Décomposer byte0 en bits de faute individuels ─────────
                fault_byte   = data[0] & 0xFF
                fault_stick  = bool(fault_byte & 0x01)   # bit0 → contribue à B2011
                fault_supply = bool(fault_byte & 0x02)   # bit1
                fault_comms  = bool(fault_byte & 0x04)   # bit2

                old_fault = rte.crs_fault
                new_fault = fault_byte & 0x07   # bits 0-2 seulement
                rte.set_multi(
                    crs_fault              = new_fault,
                    crs_fault_stick        = fault_stick,
                    crs_fault_supply       = fault_supply,
                    crs_fault_comms        = fault_comms,
                    crs_version            = crs_version,
                    t_last_lin0x17         = time.time(),
                )
                if old_fault != new_fault:
                    print(f"[LIN 0x{fid:02X}] CRS_Status: "
                          f"Stick={int(fault_stick)} Supply={int(fault_supply)} "
                          f"Comms={int(fault_comms)} Version=0x{crs_version:02X} "
                          f"(raw=0x{fault_byte:02X})")
            except serial.SerialTimeoutException:
                pass   # Non critique
            except Exception as e:
                print(f"[LIN 0x{fid:02X}] Exception (non critique): {e}")

        # ── Frame LDF future / inconnue de la logique applicative ────────
        else:
            try:
                self._lin_send_header(pid)
                resp = self._lin_read_response(pid)
                if resp:
                    print(f"[LIN 0x{fid:02X}] {frame_name}  PID=0x{pid:02X}"
                          f"  data={' '.join(f'0x{b:02X}' for b in resp[:dlc])}")
            except Exception as e:
                print(f"[LIN 0x{fid:02X}] {frame_name} exception: {e}")

    # ── Compatibilité backwards : thin wrappers pour tout code existant ──
    def _lin_poll_0x16(self):
        """Wrapper de compatibilité → délègue à _lin_poll_frame (LDF)."""
        self._lin_poll_frame("LeftStickWiperRequester")

    def _lin_poll_0x17(self):
        """Wrapper de compatibilité → délègue à _lin_poll_frame (LDF)."""
        self._lin_poll_frame("CRS_Status")

    def _handle_lin_timeout(self):
        """
        Declencher le timeout LIN → B2004 actif + forcer WOP_OFF.
        Idempotent : n'agit que si lin_timeout_active == False.
        """
        rte = self._rte
        if not rte.lin_timeout_active:
            print("[LIN] TIMEOUT detecte → B2004 actif (FSR_001)")
            rte.set_multi(lin_timeout_active=True, crs_wiper_op=WOP_OFF)
            self._dtc.set_active("B2004", rte.make_snapshot())

    def _check_lin_timeout(self):
        """
        Verifier periodiquement si le CRS repond encore.
        Appele par thread_lin_scheduler toutes les 5ms.
        Declenche B2004 si t_last_lin0x16 depasse LIN_TIMEOUT (2s).
        La mise a jour de t_last_lin0x16 n'est faite QUE sur reponse
        valide dans _lin_poll_frame, donc un silence slave declenche bien
        le timeout apres LIN_TIMEOUT secondes.
        """
        rte = self._rte
        if rte.t_last_lin0x16 == 0.0:
            return   # Pas encore de premiere trame recue
        if (time.time() - rte.t_last_lin0x16) > _bcm_rte.LIN_TIMEOUT and not rte.lin_timeout_active:
            self._handle_lin_timeout()

    # ==================================================
    # SECTION C -- THREAD CAN RECEIVER (T-CAN)
    # ==================================================

    def thread_can_receiver(self):
        """
        Thread T-CAN -- Recepteur CAN. Bloquant sur bus.
        ID=0x300 → Vehicle_Status  : ignition, marche arriere, vitesse
        ID=0x301 → RainSensorData  : intensite pluie, etat capteur
        ID=0x201 → Wiper_Status    : statut retour WC (Cas B)
        """
        print("[THREAD T-CAN] Demarre")
        rte = self._rte
        _last_log = 0.0

        while self._running:
            if not self.can_bus:
                now = time.time()
                if now - _last_log >= 5.0:
                    print("[THREAD T-CAN] can_bus non disponible, attente...")
                    _last_log = now
                time.sleep(CAN_IDLE_SLEEP)
                continue
            try:
                msg = self.can_bus.recv(timeout=CAN_RECV_TIMEOUT)
                if msg is None:
                    continue
                # SID_CC 0x28 : comm RX/TX est DoIP uniquement → CAN non affecte
                data = bytes(msg.data)
                if msg.arbitration_id == _bcm_rte.CAN_ID_VEHICLE:
                    self._can_process_0x300(data)
                elif msg.arbitration_id == _bcm_rte.CAN_ID_RAIN_SENSOR:
                    self._can_process_0x301(data)
                elif msg.arbitration_id == _bcm_rte.CAN_ID_WIPER_STATUS:
                    self._can_process_0x201(data)
                elif msg.arbitration_id == _bcm_rte.CAN_ID_WIPER_ACK:
                    self._can_process_0x202(data)
            except Exception as e:
                print(f"[THREAD T-CAN] Exception: {e}")

    def _can_process_0x300(self, data: bytes):
        """
        Trame CAN 0x300 -- Vehicle_Status.
        Décode via unpack_frame() DBC si disponible, sinon hardcodé.
        byte 0 : Ignition_Status | byte 1 : ReverseGear | byte 2-3 : VehicleSpeed (0.1 km/h/bit)
        """
        if len(data) < 4:
            return
        rte = self._rte
        now = time.time()

        # Décodage DBC si disponible
        dbc_cfg = _bcm_rte.CAN_DBC_CONFIG
        if dbc_cfg and _bcm_rte._DBC_LOADER_OK:
            msg = dbc_cfg["messages"].get(_bcm_rte.CAN_ID_VEHICLE)
            if msg:
                try:
                    sigs = _bcm_rte._dbc_unpack(msg, data)
                    new_ign   = int(sigs.get("IgnitionStatus", data[0]))
                    new_rev   = bool(int(sigs.get("ReverseGear", data[1])))
                    speed_kmh = round(sigs.get("VehicleSpeed", 0.0), 1)
                except Exception:
                    new_ign   = data[0]
                    new_rev   = bool(data[1])
                    speed_raw = (data[2] << 8) | data[3]
                    speed_kmh = round(speed_raw / 10.0, 1)
            else:
                new_ign   = data[0]
                new_rev   = bool(data[1])
                speed_raw = (data[2] << 8) | data[3]
                speed_kmh = round(speed_raw / 10.0, 1)
        else:
            new_ign   = data[0]
            new_rev   = bool(data[1])
            speed_raw = (data[2] << 8) | data[3]
            speed_kmh = round(speed_raw / 10.0, 1)

        rte.set_multi(
            reverse_gear    = new_rev,
            vehicle_speed   = speed_kmh,
        )

        # Priorité Redis sur CAN 0x300 pour ignition_status :
        # si Redis a écrit ignition_status dans la dernière seconde,
        # ne pas l'écraser avec la valeur CAN (qui peut être périmée
        # de plusieurs cycles de 200ms côté bcmcan).
        _redis_override = (time.time() - rte._t_ignition_redis) < 1.0
        if not _redis_override:
            rte.set("ignition_status", new_ign)

        if new_ign != self._ign_pending:
            self._ign_pending = new_ign
            self._ign_t       = now
        elif (new_ign != self._ign_stable and
              now - self._ign_t >= self._CAN_DEBOUNCE):
            print(f"[CAN 0x300] Ignition: {self._ign_stable} → {new_ign}")
            prev_ign         = self._ign_stable
            self._ign_stable = new_ign
            # Notifier DTCManager du changement de cycle allumage
            try:
                if prev_ign == 0 and new_ign != 0:
                    self._dtc.notify_ignition_on()   # OFF -> ON
                elif prev_ign is not None and prev_ign != 0 and new_ign == 0:
                    self._dtc.notify_ignition_off()  # ON -> OFF
            except Exception:
                pass

        if self._ign_stable is None:
            self._ign_stable = new_ign

        if new_rev != self._rev_pending:
            self._rev_pending = new_rev
            self._rev_t       = now
        elif (new_rev != self._rev_stable and
              now - self._rev_t >= self._CAN_DEBOUNCE):
            print(f"[CAN 0x300] Marche arriere: "
                  f"{'ENGAGEE' if new_rev else 'DESENGAGEE'}")
            self._rev_stable = new_rev

        if self._rev_stable is None:
            self._rev_stable = new_rev

    def _can_process_0x301(self, data: bytes):
        """
        Trame CAN 0x301 -- RainSensorData.
        Décode via unpack_frame() DBC si disponible, sinon hardcodé.
        byte 0 : RainIntensity | byte 1 : SensorStatus (0=OK)
        """
        if len(data) < 2:
            return
        rte = self._rte

        dbc_cfg = _bcm_rte.CAN_DBC_CONFIG
        if dbc_cfg and _bcm_rte._DBC_LOADER_OK:
            msg = dbc_cfg["messages"].get(_bcm_rte.CAN_ID_RAIN_SENSOR)
            if msg:
                try:
                    sigs      = _bcm_rte._dbc_unpack(msg, data)
                    intensity = int(sigs.get("RainIntensity", data[0]))
                    sensor_ok = bool(int(sigs.get("SensorOK", 1)))
                except Exception:
                    intensity = data[0]
                    sensor_ok = (data[1] == 0)
            else:
                intensity = data[0]
                sensor_ok = (data[1] == 0)
        else:
            intensity = data[0]
            sensor_ok = (data[1] == 0)

        # Ne pas ecraser rain_intensity pendant une simulation pluie (RID 0x0205)
        if rte._test_active and rte._test_routine == 0x0205:
            return
        rte.set_multi(rain_intensity=intensity, rain_sensor_ok=sensor_ok)

    # --------------------------------------------------
    # CAN 0x201 -- Wiper_Status (WC → BCM, Cas B)
    # --------------------------------------------------

    def _can_process_0x201(self, data: bytes):
        """
        Trame CAN 0x201 -- Wiper_Status (WC → BCM, Cas B).
        DBC utilisé pour CurrentMode, CurrentSpeed, BladePosition, FaultStatus, AliveCounter.
        MotorCurrent reste hardcodé (big-endian 16-bit non supporté par le parser DBC LE).
        CRC = XOR(byte0..byte6).

        FIX TC_FSR_010 : la vérification CRC est effectuée EN PREMIER, avant le guard
        wc_available. L'ancien ordre (guard → CRC) faisait ignorer silencieusement toutes
        les trames CRC KO si wc_available=False arrivait avec un délai Redis (pub/sub latence
        lors du setup 600ms). Résultat : wc_crc_fault jamais levé → timeout test.
        Le CRC KO est une faute de sécurité critique qui doit être détectée quelle que soit
        la valeur de wc_available — c'est une information sur le bus, pas sur l'état logique.
        """
        if len(data) < 8:
            return
        rte = self._rte

        # ── CRC Wiper_Status (WW-MCAT-005 Rev5) — vérification AVANT guard wc_available ──
        # (voir commentaire complet dans le nouveau bloc de décodage ci-dessous)

        # ── CRC Wiper_Status (WW-MCAT-005 Rev5) ─────────────────────────────
        # Nouveau layout 8 bytes :
        #   byte0 : CurrentMode    byte1 : CurrentSpeed   byte2 : BladePosition
        #   byte3 : MotorCurrent (8-bit, 0.1A/bit)
        #   byte4 : FaultStatus bitfield (bits 0-5)
        #   byte5 : AliveCounter_RX    byte6 : CRC_Low (XOR byte0..byte5)
        #   byte7 : Reserved
        # CRC_Low = XOR(byte0..byte5)  — byte7 exclu (Reserved=0)
        crc_calc = 0
        for b in data[:6]:
            crc_calc ^= b
        crc_calc &= 0xFF
        if crc_calc != (data[6] & 0xFF):
            print(f"[CAN 0x201] CRC KO recu=0x{data[6]:02X} calc=0x{crc_calc:02X} -- trame ignoree")
            if not rte.wc_crc_fault:
                rte.set("wc_crc_fault", True)
            return

        # CRC valide : appliquer le guard wc_available (Cas A / Cas B)
        if not rte.wc_available:
            return   # Cas A : trame valide mais WC non installé → ignorer

        # MotorCurrent : 8 bits (byte3), résolution 0.1A/bit, range 0-25.5A
        motor_curr = data[3] * 0.1

        # CRC valide : NE PAS remettre wc_crc_fault à False automatiquement.
        rte = self._rte

        # CurrentMode : byte0 — mode actuel du WC
        curr_mode = data[0] & 0xFF
        rte.wc_current_mode = curr_mode   # stocké pour log 0x202 et diagnostic

        # FaultStatus : byte4 décomposé en 6 bits individuels
        fault_byte = data[4] & 0xFF
        fault_wc_internal   = bool(fault_byte & 0x01)   # bit0
        fault_motor_driver  = bool(fault_byte & 0x02)   # bit1
        fault_pos_sensor    = bool(fault_byte & 0x04)   # bit2
        fault_supply        = bool(fault_byte & 0x08)   # bit3
        fault_can_timeout   = bool(fault_byte & 0x10)   # bit4
        fault_motor_blocked = bool(fault_byte & 0x20)   # bit5

        # Décodage DBC pour les signaux 8-bit si disponible
        dbc_cfg = _bcm_rte.CAN_DBC_CONFIG
        if dbc_cfg and _bcm_rte._DBC_LOADER_OK:
            msg = dbc_cfg["messages"].get(_bcm_rte.CAN_ID_WIPER_STATUS)
            if msg:
                try:
                    sigs       = _bcm_rte._dbc_unpack(msg, data)
                    curr_speed = int(sigs.get("CurrentSpeed",    data[1]))
                    blade_pos  = int(sigs.get("BladePosition",   data[2]))
                    alive_rx   = int(sigs.get("AliveCounter_RX", data[5]))
                    # FaultStatus bits depuis DBC (individuels)
                    fault_wc_internal   = bool(int(sigs.get("FaultStatus_WC_Internal",  fault_wc_internal)))
                    fault_motor_driver  = bool(int(sigs.get("FaultStatus_MotorDriver",   fault_motor_driver)))
                    fault_pos_sensor    = bool(int(sigs.get("FaultStatus_PosSensor",     fault_pos_sensor)))
                    fault_supply        = bool(int(sigs.get("FaultStatus_Supply",        fault_supply)))
                    fault_can_timeout   = bool(int(sigs.get("FaultStatus_CAN_Timeout",   fault_can_timeout)))
                    fault_motor_blocked = bool(int(sigs.get("FaultStatus_MotorBlocked",  fault_motor_blocked)))
                except Exception:
                    curr_speed = data[1] & 0xFF
                    blade_pos  = data[2] & 0xFF
                    alive_rx   = data[5] & 0xFF
            else:
                curr_speed = data[1] & 0xFF
                blade_pos  = data[2] & 0xFF
                alive_rx   = data[5] & 0xFF
        else:
            curr_speed = data[1] & 0xFF
            blade_pos  = data[2] & 0xFF
            alive_rx   = data[5] & 0xFF

        # Résumé OR pour rétrocompatibilité RTE (fault_st)
        fault_st = fault_byte & 0x3F

        is_moving  = (curr_speed > 0)
        was_moving = rte.front_blade_moving
        if was_moving and not is_moving:
             rte.set("t_motor_stop", time.time())
             print(f"[CAN 0x201] WC confirme arrêt moteur (speed={curr_speed}) -> t_motor_stop mis à jour")

        # ── Vérification AliveCounter_RX 0x201 ──────────────────────────────
        # Le counter doit s'incrémenter à chaque trame. S'il reste identique
        # pendant CAN_ALIVE_FREEZE_THRESHOLD trames consécutives, la trame est
        # considérée invalide (anti-replay) → inclus dans B2005 (CAN Timeout WC)
        # car le BCM ne reçoit plus de trame 0x201 valide.
        # Remise à zéro automatique si le counter recommence à changer.
        CAN_ALIVE_FREEZE_THRESHOLD = 3
        if not hasattr(rte, '_alive_rx_prev'):
            rte._alive_rx_prev       = -1
            rte._alive_rx_freeze_cnt = 0

        if rte._alive_rx_prev < 0:
            # Premier appel : initialisation sans jugement
            rte._alive_rx_prev       = alive_rx
            rte._alive_rx_freeze_cnt = 1
            alive_valid = True
        elif rte._alive_rx_prev == alive_rx:
            rte._alive_rx_freeze_cnt += 1
            if rte._alive_rx_freeze_cnt >= CAN_ALIVE_FREEZE_THRESHOLD:
                print(f"[CAN 0x201] AliveCounter_RX fige=0x{alive_rx:02X} "
                      f"x{rte._alive_rx_freeze_cnt} -> trame invalide (B2005)")
                # Trame invalide : ne pas mettre à jour t_last_wiper_status
                # → le timeout B2005 se déclenchera naturellement
                alive_valid = False
            else:
                alive_valid = True
        else:
            # Counter a changé : remise à zéro
            rte._alive_rx_prev       = alive_rx
            rte._alive_rx_freeze_cnt = 1
            alive_valid = True

        if not alive_valid:
            return  # trame rejetée, t_last_wiper_status non mis à jour → B2005

        rte.set_multi(
            front_motor_speed          = curr_speed,
            front_blade_moving         = is_moving,
            wc_blade_position          = blade_pos,
            motor_current_a            = round(motor_curr, 3),
            t_last_wiper_status        = time.time(),
            wc_alive_rx                = alive_rx,
            wc_fault_wc_internal       = fault_wc_internal,
            wc_fault_motor_driver      = fault_motor_driver,
            wc_fault_pos_sensor        = fault_pos_sensor,
            wc_fault_supply            = fault_supply,
            wc_fault_can_timeout       = fault_can_timeout,
            wc_fault_motor_blocked     = fault_motor_blocked,
        )
        if fault_st != 0:
            print(f"[CAN 0x201] WC FaultStatus=0x{fault_st:02X} "
                  f"(WCInt={int(fault_wc_internal)} MtrDrv={int(fault_motor_driver)} "
                  f"PosSns={int(fault_pos_sensor)} Sup={int(fault_supply)} "
                  f"CANTo={int(fault_can_timeout)} MtrBlk={int(fault_motor_blocked)})")

    def _can_process_0x202(self, data: bytes):
        """
        Trame CAN 0x202 -- Wiper_Ack (WC → BCM, event-based, DLC=4).
        Catalogue WW-MCAT-005 Rev5 :
          byte0 bit0    : AckStatus  (0=ACK, 1=NACK)
          byte0 bits7:1 : Ack_Reserved
          byte1         : ErrorCode  (0x00..0x07)
          byte2         : AliveCounter_AK
          byte3         : CRC_Ack = XOR(byte0, byte1, byte2)

        Stocke AckStatus et ErrorCode dans le RTE pour traitement
        dans bcm_application._check_wc_ack().
        CRC vérifié avant tout traitement.
        """
        if len(data) < 4:
            return
        rte = self._rte

        # ── Vérification CRC ────────────────────────────────────────────
        crc_calc = (data[0] ^ data[1] ^ data[2]) & 0xFF
        if crc_calc != (data[3] & 0xFF):
            print(f"[CAN 0x202] CRC KO recu=0x{data[3]:02X} calc=0x{crc_calc:02X} -- trame ignoree")
            return

        ack_status = data[0] & 0x01   # bit0 seulement
        error_code = data[1] & 0xFF
        alive_ak   = data[2] & 0xFF

        rte.set_multi(
            wc_last_ack_status = ack_status,
            wc_last_error_code = error_code,
            wc_ack_pending     = True,     # signal à T-WSM qu'une nouvelle Ack est disponible
        )
        status_str = "ACK" if ack_status == 0 else "NACK"
        print(f"[CAN 0x202] Wiper_Ack: {status_str} ErrorCode=0x{error_code:02X} "
              f"Alive=0x{alive_ak:02X}")

    def _build_wiper_command(self, wiper_mode: int, speed: int, wash: int) -> bytes:
        """
        Construit trame CAN 0x200 Wiper_Command (WW-MCAT-005 Rev5).
        Layout 8 bytes :
          byte0 [3:0]  WiperMode        (Enum)
          byte0 [7:4]  WiperSpeedLevel  (Enum)
          byte1 [1:0]  WashRequest      (Enum)
          byte1 [7:2]  Reserved_1       (0x00)
          byte2        AliveCounter_TX  (Counter 0-255)
          byte3        Checksum         (CRC-8 sur bytes 0-2 et 4-7)
          byte4-7      Reserved_2..5    (0x00)
        CRC-8 : XOR de bytes 0,1,2,4,5,6,7 (Reserved inclus = 0).
        """
        rte = self._rte
        # FIX TC_CAN_003 : geler le counter si alive_tx_frozen est actif
        if not getattr(rte, 'alive_tx_frozen', False):
            rte.wc_can_alive_tx = (rte.wc_can_alive_tx + 1) % 256
        # sinon : wc_can_alive_tx reste identique → counter figé dans chaque trame 0x200

        b0  = (wiper_mode & 0x0F) | ((speed & 0x0F) << 4)
        b1  = wash & 0x03          # bits [1:0] = WashRequest ; bits [7:2] = Reserved_1 = 0
        b2  = rte.wc_can_alive_tx & 0xFF
        # Checksum (byte3) = CRC-8 XOR sur bytes 0,1,2,4,5,6,7
        # bytes 4-7 sont tous 0x00 → leur contribution XOR est nulle
        crc = (b0 ^ b1 ^ b2) & 0xFF   # XOR(b0,b1,b2) ^ 0 ^ 0 ^ 0 ^ 0

        # Utiliser pack_frame DBC si disponible
        dbc_cfg = _bcm_rte.CAN_DBC_CONFIG
        if dbc_cfg:
            msg = dbc_cfg["messages"].get(_bcm_rte.CAN_ID_WIPER_COMMAND)
            if msg and _bcm_rte._DBC_LOADER_OK:
                try:
                    data = _bcm_rte._dbc_pack(msg, {
                        "WiperMode":       float(wiper_mode & 0x0F),
                        "WiperSpeedLevel": float(speed & 0x0F),
                        "WashRequest":     float(wash & 0x03),
                        "Reserved_1":      0.0,
                        "AliveCounter_TX": float(b2),
                        "Checksum":        float(crc),
                        "Reserved_2":      0.0,
                        "Reserved_3":      0.0,
                        "Reserved_4":      0.0,
                        "Reserved_5":      0.0,
                    })
                    return data
                except Exception as e:
                    print(f"[CAN 0x200] DBC pack_frame erreur: {e} — fallback hardcodé")

        # Fallback hardcodé (layout identique au DBC)
        # byte0=WiperMode|SpeedLevel, byte1=WashRequest, byte2=AliveCounter,
        # byte3=Checksum, byte4-7=Reserved
        return bytes([b0, b1, b2, crc, 0x00, 0x00, 0x00, 0x00])

    def _can_send_wiper_command(self, data: bytes):
        """Envoie trame CAN 0x200 vers WC."""
        if not self.can_bus:
            return
        try:
            msg = can.Message(
                arbitration_id=_bcm_rte.CAN_ID_WIPER_COMMAND,
                data=data, is_extended_id=False
            )
            self.can_bus.send(msg)
        except Exception as e:
            print(f"[CAN TX 0x200] Erreur: {e}")

    def thread_can_wc_command(self):
        """
        Thread T-CAN-WC -- BCM → WC Wiper_Command 0x200 (Cas B, 20ms).
        Actif seulement quand rte.wc_available = True.
        Traduit l'etat WSM BCM en commande CAN vers WC.
        """
        from bcm_rte import (
            WOP_OFF, WOP_TOUCH, WOP_SPEED1, WOP_SPEED2, WOP_AUTO, WOP_FRONT_WASH,
            ST_OFF, ST_TOUCH, ST_SPEED1, ST_SPEED2, ST_AUTO,
            ST_WASH_FRONT, ST_WASH_REAR, ST_REAR_WIPE, ST_ERROR, ST_DIAG,
        )
        print(f"[THREAD T-CAN-WC] Demarre | periode={_bcm_rte.CAN_WC_CMD_PERIOD*1000:.0f}ms")
        rte = self._rte

        while self._running:
            if not rte.wc_available:
                time.sleep(_bcm_rte.CAN_WC_CMD_PERIOD)
                continue

            # SID_CC 0x28 : comm TX est DoIP uniquement → CAN non affecte

            state = rte.state

            if state in (ST_OFF, ST_ERROR):
                wiper_mode, speed = WOP_OFF, 0
            elif state == ST_TOUCH:
                wiper_mode, speed = WOP_TOUCH, 1
            elif state == ST_SPEED1:
                wiper_mode, speed = WOP_SPEED1, 1
            elif state == ST_SPEED2:
                wiper_mode, speed = WOP_SPEED2, 2
            elif state == ST_AUTO:
                wiper_mode, speed = WOP_AUTO, rte.front_motor_speed
            elif state == ST_WASH_FRONT:
                wiper_mode, speed = WOP_FRONT_WASH, 1
            elif state == ST_WASH_REAR:
                wiper_mode, speed = WOP_OFF, 0
            elif state == ST_DIAG:
                # En mode DIAG, le BCM continue d'envoyer 0x200 vers WC
                # avec la commande du test actif (front_motor_on/speed du RTE)
                # pour que le WC puisse executer le test moteur avant.
                # Si aucun test actif (ou test pompe/pluie) -> WOP_OFF
                if rte._test_active and rte.front_motor_on:
                    wiper_mode = WOP_SPEED1 if rte.front_motor_speed == 1 else WOP_SPEED2
                    speed      = rte.front_motor_speed
                else:
                    wiper_mode, speed = WOP_OFF, 0
            else:
                wiper_mode, speed = WOP_OFF, 0

            wash = 0
            if state == ST_WASH_FRONT and rte.pump_active and rte.pump_direction == 1:
                wash = 1
            elif state == ST_WASH_REAR and rte.pump_active and rte.pump_direction == 2:
                wash = 2

            frame = self._build_wiper_command(wiper_mode, speed, wash)
            self._can_send_wiper_command(frame)

            # Afficher seulement si changement
            sig = (wiper_mode, speed, wash)
            if sig != getattr(self, '_last_wc_cmd', None):
                self._last_wc_cmd = sig
                from bcm_rte import WOP_NAMES as _WOP_NAMES
                print(f"[CAN TX 0x200] Wiper_Command: "
                      f"mode={_WOP_NAMES.get(wiper_mode,'?')} "
                      f"speed={speed} wash={wash}")

            time.sleep(_bcm_rte.CAN_WC_CMD_PERIOD)

    def run(self):
        """
        Boucle TCP principale -- architecture backend.zip :
        single-client, rx_buffer avec assemblage multi-frames,
        gestion routing_active, refus 2eme client.
        """
        # Socket deja cree et binde dans __init__
        self._srv_sock.listen(5)
        print(f"[DoIP] Serveur TCP en ecoute sur port {DOIP_PORT}")

        while self._running:
            try:
                conn, addr = self._srv_sock.accept()
                print(f"\n[DoIP TCP] New TCP connection from {addr}")

                # Refuser si un autre client est deja connecte
                if self._client_connected and self._client_address != addr:
                    print(f"[DoIP TCP] Connection refused: another client "
                          f"already connected from {self._client_address}")
                    conn.close()
                    continue

                self._client_connected = True
                self._client_address = addr
                self._routing_active = False

                # Reinitialiser session UDS pour cette connexion
                rte = self._rte
                rte.set_multi(
                    _session      = DSC_DEFAULT,
                    _sec_level    = 0,
                    _pending_seed = {},
                )

                try:
                    rx_buffer = b""

                    while True:
                        chunk = conn.recv(4096)
                        if not chunk:
                            print(f"[DoIP TCP] Connection closed by client {addr}")
                            break

                        rx_buffer += chunk

                        while True:
                            if len(rx_buffer) < 8:
                                break

                            try:
                                proto_ver, inv_ver, payload_type = struct.unpack(
                                    ">BBH", rx_buffer[:4]
                                )
                                payload_len = struct.unpack(">I", rx_buffer[4:8])[0]
                            except struct.error:
                                print(f"[DoIP TCP] Malformed DoIP header from {addr}")
                                rx_buffer = b""
                                break

                            total_len = 8 + payload_len
                            if len(rx_buffer) < total_len:
                                break

                            frame = rx_buffer[:total_len]
                            rx_buffer = rx_buffer[total_len:]

                            response = self._handle_doip_frame(frame, conn, addr)
                            if response:
                                conn.send(response)

                except ConnectionResetError:
                    print(f"[DoIP TCP] Connection lost with {addr}")
                except Exception as e:
                    print(f"[DoIP TCP] Communication error with {addr}: {e}")
                finally:
                    self._client_connected = False
                    self._client_address = None
                    self._routing_active = False
                    conn.close()
                    print(f"[DoIP TCP] TCP connection closed with {addr}")

            except OSError:
                break
            except Exception as e:
                if self._running:
                    print(f"[DoIP] Erreur accept TCP: {e}")

    # ------------------------------------------------------------------
    # DoIP Frame Handler -- architecture backend.zip
    # ------------------------------------------------------------------

    def _handle_doip_frame(self, data: bytes, conn, addr) -> bytes:
        """
        Traite une trame DoIP complete.
        Architecture backend.zip : validation version, routing activation
        avec response_code 0x00/0x02 (3 octets), verification routing_active
        avant diagnostic, verification rx_enabled/tx_enabled.
        Retourne la reponse complete (header DoIP + payload) ou None.
        """
        if len(data) < 8:
            return None

        try:
            proto_ver, inv_ver, payload_type = struct.unpack(">BBH", data[:4])
            payload_len = struct.unpack(">I", data[4:8])[0]
        except struct.error:
            return None

        if proto_ver != DOIP_PROTOCOL_VERSION or inv_ver != DOIP_INVERSE_VERSION:
            return None

        if len(data) < 8 + payload_len:
            return None

        payload = data[8:8 + payload_len]

        # ------------------------------------------------------------------
        # Routing Activation (format backend.zip : reponse 3 octets)
        # ------------------------------------------------------------------
        if payload_type == DOIP_ROUTING_ACT_REQ:
            print(f"[DoIP] Routing activation request from {addr}")

            if self._client_connected and self._client_address != addr:
                response_code = 0x02
            else:
                self._client_connected = True
                self._client_address = addr
                self._routing_active = True
                response_code = 0x00

            # ISO 13400-2 : [tester(2)][ecu(2)][response_code(1)][reserved(4)]
            response_payload = struct.pack(">H H B 4s",
                                           TESTER_ADDR,
                                           BCM_ADDR,
                                           0x10 if response_code == 0x00 else response_code,
                                           b'\x00\x00\x00\x00')
            header = self._doip_header(
                DOIP_ROUTING_ACT_RES, len(response_payload)
            )
            if response_code == 0x00:
                print(f"[DoIP] Routing activated successfully!")
            else:
                print(f"[DoIP] Routing activation failed, code: 0x{response_code:02X}")
            return header + response_payload

        # ------------------------------------------------------------------
        # Diagnostic Payload
        # ------------------------------------------------------------------
        elif payload_type == DOIP_DIAGNOSTIC_MSG:
            if not self._routing_active:
                return None

            if len(payload) < 4:
                return None

            source_addr = struct.unpack(">H", payload[:2])[0]
            target_addr = struct.unpack(">H", payload[2:4])[0]
            uds_payload = payload[4:]

            if target_addr != BCM_ADDR:
                return None

            # Verifier rx_enabled (CommunicationControl 0x28)
            # ISO 14229 : SID 0x28 (CommunicationControl) est toujours recu,
            # meme si Rx est desactive — sinon impossible de re-activer Rx.
            # Le 0x28 est un service DoIP uniquement, sans lien avec le CAN.
            if not getattr(self._rte, '_comm_rx_enabled', True):
                if len(uds_payload) >= 1 and uds_payload[0] == SID_CC:
                    pass   # 0x28 toujours traite
                else:
                    return None

            # Log requete
            is_tp_suppress = (len(uds_payload) >= 2
                              and uds_payload[0] == SID_TP
                              and bool(uds_payload[1] & 0x80))
            if not is_tp_suppress:
                print(f"[DoIP UDS] REQ  "
                      f"0x{source_addr:04X}->0x{target_addr:04X}  "
                      f"{_decode_uds_request(uds_payload)}  "
                      f"raw=[{_fmt_hex(uds_payload, 8)}]")

            # Capturer tx_enabled AVANT le dispatch :
            # _handle_cc(sub=0x01) met _comm_tx_enabled=False pendant le dispatch,
            # ce qui causerait l'auto-suppression de sa propre reponse positive.
            # ISO 14229 : la reponse doit etre envoyee avant que la restriction prenne effet.
            tx_was_enabled = getattr(self._rte, '_comm_tx_enabled', True)

            # Dispatch UDS via RTE (logique 22.zip conservee)
            uds_response = self._dispatch_uds_via_rte(uds_payload, source_addr)

            if uds_response is None:
                return None

            # Verifier tx_enabled avec la valeur capturee AVANT le dispatch
            if not tx_was_enabled:
                return None

            # Log reponse
            if not is_tp_suppress:
                rsp_str = _decode_uds_response(uds_response)
                tag = "NRC " if uds_response[0] == 0x7F else "RSP "
                print(f"[DoIP UDS] {tag} "
                      f"0x{BCM_ADDR:04X}->0x{source_addr:04X}  "
                      f"{rsp_str}  "
                      f"raw=[{_fmt_hex(uds_response, 8)}]")

            resp_payload = (
                struct.pack(">HH", BCM_ADDR, source_addr) + uds_response
            )
            return self._doip_header(
                DOIP_DIAGNOSTIC_MSG, len(resp_payload)
            ) + resp_payload

        return None

    # ------------------------------------------------------------------
    # UDS Dispatch via RTE (mecanique inter-thread 22.zip conservee)
    # ------------------------------------------------------------------

    def _dispatch_uds_via_rte(self, uds: bytes, source_addr: int):
        """
        Envoie la requete UDS au thread T-DIAG via le RTE.
        Retourne la reponse UDS (bytes) ou None.
        Conserve la mecanique RTE+Event+Mutex de 22.zip.
        """
        if not uds:
            return bytes([0x7F, 0x00, 0x13])

        sid = uds[0]
        rte = self._rte

        # TesterPresent avec suppress → pas de reponse
        is_tp_suppress = (sid == SID_TP and len(uds) >= 2
                          and bool(uds[1] & 0x80))

        # ── SERIALISATION -- un seul client a la fois ──────────────
        with rte._uds_mutex:

            # ── ECRITURE RTE ──────────────────────────────────────
            rte.set_multi(
                uds_sid             = sid,
                uds_payload         = uds,
                uds_src_addr        = source_addr,
                uds_dst_addr        = BCM_ADDR,
                uds_response        = b"",
                uds_response_ready  = False,
                uds_request_pending = True,   # signal a T-DIAG
            )
            # Reveiller T-DIAG immediatement via Event (zero polling CPU)
            rte._uds_event.set()

            # -- ATTENDRE REPONSE DE T-DIAG ---
            deadline = time.time() + DOIP_RESPONSE_TIMEOUT
            while time.time() < deadline:
                if rte.uds_response_ready:
                    break
                time.sleep(0.002)   # 2ms granularite fine

            if not rte.uds_response_ready:
                print(f"[DoIP UDS] TIMEOUT reponse T-DIAG SID=0x{sid:02X}")
                rte.set("uds_request_pending", False)
                return bytes([0x7F, sid, 0x78])

            resp = rte.uds_response if rte.uds_response else b""
            rte.set_multi(uds_response_ready=False, uds_request_pending=False)

        if is_tp_suppress:
            return None

        return resp if resp else None

    # ── Primitives DoIP bas niveau ────────────────────

    def _doip_header(self, ptype: int, plen: int) -> bytes:
        """Construire le header DoIP (8 bytes) -- format backend.zip."""
        return struct.pack(
            ">BBH I",
            DOIP_PROTOCOL_VERSION,
            DOIP_INVERSE_VERSION,
            ptype,
            plen
        )

    def _doip_parse(self, data: bytes):
        """Parser un header DoIP avec validation version (comme backend.zip).
        Retourne (ptype, payload) ou None."""
        if len(data) < 8:
            return None
        proto_ver, inv_ver, ptype, plen = struct.unpack(">BBH I", data[:8])
        if proto_ver != DOIP_PROTOCOL_VERSION or inv_ver != DOIP_INVERSE_VERSION:
            print(f"[DoIP] Version incorrecte: ver=0x{proto_ver:02X} inv=0x{inv_ver:02X}")
            return None
        if len(data) < 8 + plen:
            return None
        return ptype, data[8:8 + plen]

    def _doip_send(self, sock, ptype: int, payload: bytes):
        """Envoyer une trame DoIP complete sur le socket TCP."""
        frame = self._doip_header(ptype, len(payload)) + payload
        sock.send(frame)

    # ── UDP Discovery ─────────────────────────────────

    def _start_udp_discovery(self):
        """Lancer le thread UDP DoIP discovery (non bloquant)."""
        threading.Thread(
            target=self._doip_udp_handler,
            daemon=True,
            name="DoIP_UDP"
        ).start()

    def _doip_udp_handler(self):
        """
        Thread UDP -- Vehicle Identification Discovery.
        Repond aux requetes VehicleIdRequest du PC avec VIN + adresse BCM.
        Format de reponse aligne sur backend.zip (VIN + addr + EID + GID + IP).
        """
        udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        udp.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            udp.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
        except AttributeError:
            pass
        udp.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        udp.bind(("0.0.0.0", DOIP_PORT))
        udp.settimeout(1.0)
        self._udp_sock = udp   # reference pour fermeture dans stop()
        print(f"[DoIP] UDP discovery server listening on port {DOIP_PORT}")

        while self._running:
            try:
                data, addr = udp.recvfrom(4096)
                if len(data) < 8:
                    continue
                proto_ver, inv_ver, ptype = struct.unpack(">BBH", data[:4])
                payload_len = struct.unpack(">I", data[4:8])[0]

                if (proto_ver == DOIP_PROTOCOL_VERSION
                        and inv_ver == DOIP_INVERSE_VERSION
                        and ptype == DOIP_VEHICLE_ID_REQ):

                    print(f"[DoIP UDP] Discovery request received from {addr}")

                    vin = VIN[:17].ljust(17, b"\x00")
                    logical_address = BCM_ADDR.to_bytes(2, "big")
                    eid = b"\x00\x00\x00\x00\x00\x01"
                    gid = b"\x00\x00\x00\x00\x00\x02"
                    ip_bytes = socket.inet_aton(self._get_local_ip())

                    resp_payload = vin + logical_address + eid + gid + ip_bytes
                    header = self._doip_header(
                        DOIP_VEHICLE_ID_RES, len(resp_payload)
                    )
                    udp.sendto(header + resp_payload, addr)
                    print(f"[DoIP UDP] Discovery response sent to {addr}")

            except socket.timeout:
                continue
            except Exception as e:
                if self._running:
                    print(f"[DoIP] UDP discovery error: {e}")
        udp.close()

    # ── Network Helper ────────────────────────────────

    def _get_local_ip(self) -> str:
        """Obtenir l'adresse IP locale."""
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("8.8.8.8", 80))
            ip = s.getsockname()[0]
            s.close()
            return ip
        except Exception:
            return "127.0.0.1"

    # ==================================================
    # SECTION E -- DEMARRAGE / ARRET
    # ==================================================

    def start(self):
        """
        Activer le flag running + lancer le thread UDP DoIP discovery.
        Appele avant le lancement des threads depuis bcm_main.py.
        """
        self._running = True
        self._start_udp_discovery()

    def stop(self):
        """Arreter tous les protocoles proprement."""
        self._running = False

        # Fermer le socket UDP immediatement — libere le port 13400
        # Sans ca : thread UDP daemon reste en vie et tient le port
        # → OSError: [Errno 98] Address already in use au redemarrage
        if self._udp_sock:
            try:
                self._udp_sock.close()
            except Exception:
                pass
            self._udp_sock = None

        # Signaler T-LIN de s'arreter et attendre qu'il finisse
        # son iteration courante avant de fermer le port serie.
        # Sans cette attente : race condition → "port not open"
        self._lin_stop.set()
        time.sleep(_bcm_rte.LIN_CYCLE_0x16 + 0.100)  # attendre 1 cycle LIN max (lu depuis LDF)

        if self._srv_sock:
            try:
                # shutdown() debloque accept() dans run() immediatement
                # Ne pas faire close() ici — le socket reste utilisable
                # pour un eventuel redemarrage dans le meme processus
                self._srv_sock.shutdown(socket.SHUT_RDWR)
            except Exception:
                pass
        if self.can_bus:
            try:
                self.can_bus.shutdown()
            except Exception:
                pass
        if self.lin_port:
            try:
                self.lin_port.close()
            except Exception:
                pass
        print("[ProtocolLayer] Arrete proprement")