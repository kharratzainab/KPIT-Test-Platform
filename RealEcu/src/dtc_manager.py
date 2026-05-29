#!/usr/bin/env python3
"""
DTC Manager - WipeWash System
Diagnostic Specification - Section 9 & 10

DTC Format: 3-byte ISO DTC (SAE J2012 compliant)
DTC List (Section 9.2):
  B2001: Front Motor Blocked
  B2002: Rear Motor Blocked
  B2003: Pump Overcurrent
  B2004: LIN Timeout CRS
  B2005: CAN Timeout WC
  B2006: Blade Position Implausible
  B2007: Rain Sensor Signal Fault
  B2008: Pump Runtime Exceeded
  B2009: Rest Contact Failure

Status Byte (Section 10):
  Bit 0: Test Failed
  Bit 1: Test Failed This Operation Cycle
  Bit 2: Pending
  Bit 3: Confirmed
  Bit 4: Test Not Completed
  Bit 5: Test Failed Since Clear
  Bit 6: Warning Indicator
  Bit 7: Test Not Completed Since Clear

Snapshot Data (Section 11):
  Each DTC stores:
    - Ignition status
    - Wiper mode
    - Motor current
    - Blade position
    - Rain intensity
    - Vehicle speed

Supported UDS sub-functions (Section 8):
  0x02: Report DTC by status mask
  0x04: Snapshot record
  0x06: Extended data
"""

import json
import os
import time as _time_mod
from datetime import datetime


def _now_ts() -> float:
    """Timestamp Unix courant (pour calcul durees ISO 14229-1)."""
    return _time_mod.time()

# =====================================================
# STATUS BYTE (Diagnostic Specification Section 10)
# =====================================================
STATUS_CLEAN              = 0x00   # no fault
STATUS_ACTIVE             = 0x2F   # bits 0,1,2,3,5 set = test failed + confirmed + pending
STATUS_INACTIVE           = 0x2E   # bits 1,2,3,5 set = confirmed but test passed now
STATUS_AVAILABILITY_MASK  = 0xFF

# Bit positions
BIT_TEST_FAILED        = 0x01
BIT_FAILED_THIS_CYCLE  = 0x02
BIT_PENDING            = 0x04
BIT_CONFIRMED          = 0x08
BIT_NOT_COMPLETED      = 0x10
BIT_FAILED_SINCE_CLEAR = 0x20
BIT_WARNING_INDICATOR  = 0x40
BIT_NOT_SINCE_CLEAR    = 0x80

# Snapshot DID identifiers (Section 11)
SNAP_DID_IGNITION    = 0xF190   # Ignition status
SNAP_DID_WIPER_MODE  = 0xF191   # Wiper mode at fault time
SNAP_DID_MOTOR_CURR  = 0xF192   # Motor current (mA)
SNAP_DID_BLADE_POS   = 0xF193   # Blade position (0-100%)
SNAP_DID_RAIN_INTENS = 0xF194   # Rain intensity (0-100)
SNAP_DID_VEHICLE_SPD = 0xF195   # Vehicle speed (km/h)

MAX_SNAPSHOT_RECORDS = 5

# Extended Data Record numbers (ISO 14229-1 Section 7.3.4)
EXT_REC_OCCURRENCE_COUNT  = 0x01   # Nombre total d'occurrences (2B)
EXT_REC_FAILED_CYCLES     = 0x03   # Cycles où le DTC était actif (1B)
EXT_REC_TIME_FIRST_OCC    = 0x04   # Secondes depuis première occurrence (4B)
EXT_REC_TIME_LAST_OCC     = 0x05   # Secondes depuis dernière occurrence (4B)

DTC_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "dtc_database.json")


# =====================================================
# DTC MANAGER
# =====================================================
class DTCManager:
    def __init__(self, filepath=DTC_FILE):
        self.filepath = filepath
        self._load()
        print(f"[DTC] Manager loaded - {len(self.dtcs)} DTCs in database")

    def _load(self):
        with open(self.filepath, "r") as f:
            self.db = json.load(f)
        self.dtcs = self.db["dtcs"]

    def _save(self):
        with open(self.filepath, "w") as f:
            json.dump(self.db, f, indent=4)

    def _now(self) -> str:
        return datetime.now().astimezone().strftime("%Y-%m-%d %H:%M:%S %Z")

    # -------------------------------------------------
    # Set DTC ACTIVE (fault detected)
    # -------------------------------------------------
    def set_active(self, code: str, snapshot: dict = None):
        """
        Set DTC status to ACTIVE (0x2F).
        snapshot keys (Section 11):
          ignition    : int (0=OFF, 1=ON)
          wiper_mode  : str (OFF/TOUCH/SPEED1...)
          motor_curr  : int (mA)
          blade_pos   : int (0-100%)
          rain        : int (0-100)
          vehicle_spd : int (km/h)
        """
        if code not in self.dtcs:
            print(f"[DTC] Unknown DTC: {code}")
            return
        dtc = self.dtcs[code]
        now = self._now()

        # Ne pas incr?menter l'occurrence si le DTC est d?j? ACTIVE
        # (?vite les incr?ments r?p?t?s lors d'une surveillance cyclique).
        # Le cycle normal est : INACTIVE ? ACTIVE (faute) ? INACTIVE (cleanup)
        # ? ACTIVE (prochain run avec affichage complet).
        already_active = (dtc["status"] == STATUS_ACTIVE)

        if already_active:
            return

        is_new = dtc["first_occurrence"] is None

        dtc["status"]           = STATUS_ACTIVE
        dtc["occurrence_count"] += 1
        dtc["last_occurrence"]  = now
        dtc["last_occurrence_ts"] = _now_ts()
        if is_new:
            dtc["first_occurrence"]    = now
            dtc["first_occurrence_ts"] = _now_ts()
        # failed_cycles : incrémenté UNE SEULE FOIS par cycle d'allumage
        if not dtc.get("_seen_this_cycle", False):
            dtc["failed_cycles"]    = dtc.get("failed_cycles", 0) + 1
            dtc["_seen_this_cycle"] = True

        if snapshot:
            dtc["snapshot"] = snapshot
            existing = dtc.get("snapshot_records", [])
            last_num = existing[-1]["record_number"] if existing else 0
            next_num = (last_num % 255) + 1

            record = {
                "record_number": next_num,
                "timestamp": now,
                "data": {
                    "F190_ignition":    snapshot.get("ignition", 1),
                    "F191_wiper_mode":  snapshot.get("wiper_mode", "UNKNOWN"),
                    "F192_motor_curr":  snapshot.get("motor_curr", 0),
                    "F193_blade_pos":   snapshot.get("blade_pos", 0),
                    "F194_rain":        snapshot.get("rain", 0),
                    "F195_vehicle_spd": snapshot.get("vehicle_spd", 0),
                }
            }
            existing.append(record)
            if len(existing) > MAX_SNAPSHOT_RECORDS:
                existing = existing[-MAX_SNAPSHOT_RECORDS:]
            dtc["snapshot_records"] = existing

        self._save()

        b = dtc["bytes"]
        hex_code = f"{b[0]:02X}{b[1]:02X}{b[2]:02X}"
        print(f"")
        print(f"  {'!'*52}")
        print(f"  ! DTC ACTIVE  {code} [{hex_code}]")
        print(f"  ! {dtc['description']}")
        print(f"  ! Status: 0x{STATUS_ACTIVE:02X}  Occurrence: #{dtc['occurrence_count']}")
        if snapshot:
            print(f"  ! Snapshot: WiperMode={snapshot.get('wiper_mode','?')}  "
                  f"MotorCurr={snapshot.get('motor_curr',0)}mA  "
                  f"Rain={snapshot.get('rain',0)}  "
                  f"VehicleSpeed={snapshot.get('vehicle_spd',0)}km/h")
        print(f"  {'!'*52}")
        print(f"")

    # -------------------------------------------------
    # Set DTC INACTIVE (fault gone, stays in memory)
    # -------------------------------------------------
    def set_inactive(self, code: str):
        if code not in self.dtcs:
            return
        dtc = self.dtcs[code]
        if dtc["status"] == STATUS_ACTIVE:
            dtc["status"] = STATUS_INACTIVE
            self._save()
            b = dtc["bytes"]
            hex_code = f"{b[0]:02X}{b[1]:02X}{b[2]:02X}"
            print(f"  [DTC INACTIVE] {code} [{hex_code}] - {dtc['description']}")

    # -------------------------------------------------
    # Clear DTCs (UDS 0x14)
    # -------------------------------------------------
    def get_status(self, code: str) -> str:
        """Retourne statut DTC : ACTIVE / INACTIVE / CLEAN / UNKNOWN."""
        if code not in self.dtcs:
            return "UNKNOWN"
        s = self.dtcs[code]["status"]
        if s == STATUS_ACTIVE:   return "ACTIVE"
        if s == STATUS_INACTIVE: return "INACTIVE"
        return "CLEAN"

    # -------------------------------------------------
    # Clear DTCs (UDS 0x14)
    # -------------------------------------------------
    def clear_all(self):
        for code in self.dtcs:
            self.dtcs[code]["status"]              = STATUS_CLEAN
            self.dtcs[code]["occurrence_count"]    = 0
            self.dtcs[code]["first_occurrence"]    = None
            self.dtcs[code]["first_occurrence_ts"] = None
            self.dtcs[code]["last_occurrence"]     = None
            self.dtcs[code]["last_occurrence_ts"]  = None
            self.dtcs[code]["failed_cycles"]       = 0
            self.dtcs[code]["_seen_this_cycle"]    = False
            self.dtcs[code]["snapshot"]            = {}
            self.dtcs[code]["snapshot_records"]    = []
        self._save()
        now = self._now()
        print(f"")
        print(f"  {'='*52}")
        print(f"  = DTC CLEARED - {len(self.dtcs)} DTCs reset to CLEAN (0x00)")
        print(f"  = Cleared at: {now}")
        print(f"  {'='*52}")
        print(f"")

    # -------------------------------------------------
    # UDS 0x19 sub-function 0x02 - Report DTC by status mask
    # -------------------------------------------------
    def get_dtcs_by_mask(self, mask: int) -> list:
        result = []
        for code, dtc in self.dtcs.items():
            status = dtc["status"]
            if mask == 0xFF:
                # 0xFF = tous les DTC non CLEAN
                if status != STATUS_CLEAN:
                    result.append((bytes(dtc["bytes"]), status))
            else:
                # Comparaison exacte : mask=0x2F -> ACTIVE seulement
                #                      mask=0x2E -> INACTIVE seulement
                if status == mask:
                    result.append((bytes(dtc["bytes"]), status))
        return result

    def get_all_supported(self) -> list:
        return [(bytes(dtc["bytes"]), dtc["status"]) for dtc in self.dtcs.values()]

    # -------------------------------------------------
    # UDS 0x19 0x02 response
    # -------------------------------------------------
    def build_response_02(self, mask: int) -> bytes:
        dtc_list = self.get_dtcs_by_mask(mask)
        resp = bytes([0x59, 0x02, STATUS_AVAILABILITY_MASK])
        for dtc_bytes, status in dtc_list:
            resp += dtc_bytes + bytes([status])
        return resp

    # -------------------------------------------------
    # UDS 0x19 sub-function 0x04 - Snapshot record
    # (Diagnostic Spec Section 8 & 11)
    # -------------------------------------------------
    def build_response_04(self, dtc_bytes_target: bytes, record_number: int) -> bytes:
        """
        0x19 0x04 response:
        [59][04][DTC 3B][StatusByte]
        then for each record:
          [RecordNumber 1B]
          [DID_F190 2B][len][ignition 1B]
          [DID_F191 2B][len][wiper_mode 10B]
          [DID_F192 2B][len][motor_curr 2B]
          [DID_F193 2B][len][blade_pos 1B]
          [DID_F194 2B][len][rain 1B]
          [DID_F195 2B][len][vehicle_spd 2B]
        """
        target = None
        for code, dtc in self.dtcs.items():
            if bytes(dtc["bytes"]) == dtc_bytes_target:
                target = dtc
                break
        if target is None:
            return bytes([0x7F, 0x19, 0x31])

        all_records = target.get("snapshot_records", [])

        if record_number == 0xFF:
            # 0xFF = send all records
            records = all_records
        else:
            # Try exact match first
            records = [r for r in all_records if r["record_number"] == record_number]
            # If not found and records exist, return the LAST (most recent) record
            if not records and all_records:
                records = [all_records[-1]]

        resp = bytes([0x59, 0x04]) + dtc_bytes_target + bytes([target["status"]])

        if not records:
            resp += bytes([0xFF])   # no snapshot
            return resp

        for rec in records:
            resp += bytes([rec["record_number"] & 0xFF])
            d = rec["data"]

            # F190 - Ignition (1 byte: 0=OFF, 1=ON)
            resp += bytes([0xF1, 0x90, 0x01, d["F190_ignition"] & 0xFF])

            # F191 - WiperMode (ASCII 10 bytes)
            mode_bytes = d["F191_wiper_mode"].encode("ascii")[:10].ljust(10, b"\x00")
            resp += bytes([0xF1, 0x91, 0x0A]) + mode_bytes

            # F192 - MotorCurrent mA (2 bytes)
            curr = int(round(d["F192_motor_curr"]))   # FIX: cast float ? int (mA)
            resp += bytes([0xF1, 0x92, 0x02, (curr >> 8) & 0xFF, curr & 0xFF])

            # F193 - BladePosition 0-100% (1 byte)
            resp += bytes([0xF1, 0x93, 0x01, d["F193_blade_pos"] & 0xFF])

            # F194 - RainIntensity 0-100 (1 byte)
            resp += bytes([0xF1, 0x94, 0x01, d["F194_rain"] & 0xFF])

            # F195 - VehicleSpeed km/h (2 bytes)
            spd = int(round(d["F195_vehicle_spd"]))   # FIX: cast float ? int (km/h)
            resp += bytes([0xF1, 0x95, 0x02, (spd >> 8) & 0xFF, spd & 0xFF])

        return resp

    # -------------------------------------------------
    # UDS 0x19 sub-function 0x06 - Extended data (ISO 14229-1)
    # Records : 0x01 occurrence_count | 0x03 failed_cycles
    #           0x03 failed_cycles    | 0x04 time_first_occ
    #           0x05 time_last_occ
    # -------------------------------------------------
    def build_response_06(self, dtc_bytes_target: bytes) -> bytes:
        import struct as _struct
        target = None
        for code, dtc in self.dtcs.items():
            if bytes(dtc["bytes"]) == dtc_bytes_target:
                target = dtc
                break
        if target is None:
            return bytes([0x7F, 0x19, 0x31])

        resp = bytes([0x59, 0x06]) + dtc_bytes_target + bytes([target["status"]])

        now = _now_ts()

        # Record 0x01 : occurrence counter (2 octets)
        occ = target.get("occurrence_count", 0)
        resp += bytes([EXT_REC_OCCURRENCE_COUNT, 0x02,
                       (occ >> 8) & 0xFF, occ & 0xFF])

        # Record 0x03 : failed cycles counter (1 octet)
        fc = min(target.get("failed_cycles", 0), 0xFF)
        resp += bytes([EXT_REC_FAILED_CYCLES, 0x01, fc])

        # Record 0x04 : time since first occurrence (4 octets, secondes)
        ts_first = target.get("first_occurrence_ts")
        dt_first = int(now - ts_first) if ts_first else 0xFFFFFFFF
        dt_first = min(dt_first, 0xFFFFFFFF)
        resp += bytes([EXT_REC_TIME_FIRST_OCC, 0x04]) + _struct.pack(">I", dt_first)

        # Record 0x05 : time since last occurrence (4 octets, secondes)
        ts_last = target.get("last_occurrence_ts")
        dt_last = int(now - ts_last) if ts_last else 0xFFFFFFFF
        dt_last = min(dt_last, 0xFFFFFFFF)
        resp += bytes([EXT_REC_TIME_LAST_OCC, 0x04]) + _struct.pack(">I", dt_last)

        return resp

    # -------------------------------------------------
    # Debug
    # -------------------------------------------------
    # -------------------------------------------------
    # Gestion cycle d'allumage (ISO 14229-1 failed_cycles)
    # -------------------------------------------------
    def notify_ignition_on(self):
        """
        Appeler lors de la transition ignition OFF->ON.
        - Si un DTC est encore ACTIVE au debut du nouveau cycle,
          incrementer failed_cycles maintenant (persistant du cycle
          precedent vers le nouveau).
        - Sinon remettre _seen_this_cycle=False.
        """
        for dtc in self.dtcs.values():
            if dtc.get("status") == STATUS_ACTIVE:
                dtc["failed_cycles"]    = dtc.get("failed_cycles", 0) + 1
                dtc["_seen_this_cycle"] = True
            else:
                dtc["_seen_this_cycle"] = False
        print("[DTC] Nouveau cycle allumage - failed_cycles mis a jour")

    def notify_ignition_off(self):
        """
        Appeler lors de la transition ignition ON->OFF.
        Sauvegarde l'etat final du cycle.
        """
        self._save()
        print("[DTC] Fin cycle allumage - base DTC sauvegardee")

    def print_all(self):
        print(f"\n{'='*56}")
        print(f"  DTC DATABASE ({len(self.dtcs)} entries)")
        print(f"{'='*56}")
        for code, dtc in self.dtcs.items():
            s = dtc["status"]
            if s == STATUS_ACTIVE:
                label = "ACTIVE   (0x2F)"
            elif s == STATUS_INACTIVE:
                label = "INACTIVE (0x2E)"
            else:
                label = "CLEAN    (0x00)"
            print(f"  {code} | {label} | #{dtc['occurrence_count']}")
            print(f"       {dtc['description']}")
        print(f"{'='*56}\n")


# =====================================================
# UDS 0x19 HANDLER
# =====================================================
def handle_read_dtc(dtc_mgr: DTCManager, uds: bytes) -> bytes:
    if len(uds) < 2:
        return bytes([0x7F, 0x19, 0x13])
    subfunc = uds[1]

    # 0x02 - Report DTC by status mask
    if subfunc == 0x02:
        mask = uds[2] if len(uds) >= 3 else 0xFF
        dtc_list = dtc_mgr.get_dtcs_by_mask(mask)
        print(f"  [UDS 0x19 0x02] mask=0x{mask:02X} - {len(dtc_list)} DTC(s) found")
        for dtc_bytes, status in dtc_list:
            label = "ACTIVE(0x2F)" if status == STATUS_ACTIVE else "INACTIVE(0x2E)"
            for code, d in dtc_mgr.dtcs.items():
                if bytes(d["bytes"]) == dtc_bytes:
                    print(f"    {code} {dtc_bytes.hex().upper()} {label}")
        return dtc_mgr.build_response_02(mask)

    # 0x04 - Snapshot record (per Diagnostic Spec)
    elif subfunc == 0x04:
        if len(uds) < 5:
            return bytes([0x7F, 0x19, 0x13])
        dtc_b  = bytes(uds[2:5])
        rec_num = uds[5] if len(uds) >= 6 else 0xFF
        print(f"  [UDS 0x19 0x04] DTC={dtc_b.hex().upper()} record=0x{rec_num:02X}")
        return dtc_mgr.build_response_04(dtc_b, rec_num)

    # 0x06 - Extended data
    elif subfunc == 0x06:
        if len(uds) < 5:
            return bytes([0x7F, 0x19, 0x13])
        dtc_b = bytes(uds[2:5])
        print(f"  [UDS 0x19 0x06] DTC={dtc_b.hex().upper()}")
        return dtc_mgr.build_response_06(dtc_b)

    else:
        return bytes([0x7F, 0x19, 0x12])   # subFunctionNotSupported


# =====================================================
# UDS 0x14 HANDLER
# =====================================================
def handle_clear_dtc(dtc_mgr: DTCManager, uds: bytes) -> bytes:
    if len(uds) < 4:
        return bytes([0x7F, 0x14, 0x13])
    group = (uds[1] << 16) | (uds[2] << 8) | uds[3]
    print(f"  [UDS 0x14] Clear DTC group=0x{group:06X}")
    if group == 0xFFFFFF or group == 0x000000:
        dtc_mgr.clear_all()
        return bytes([0x54])
    # Clear specific DTC
    for code, dtc in dtc_mgr.dtcs.items():
        b = dtc["bytes"]
        if (b[0] << 16 | b[1] << 8 | b[2]) == group:
            dtc["status"] = 0x00
            dtc["occurrence_count"] = 0
            dtc["snapshot_records"] = []
            dtc_mgr._save()
            print(f"  [DTC] Cleared {code}")
            return bytes([0x54])
    return bytes([0x7F, 0x14, 0x31])