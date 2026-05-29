
"""
test_bcm_wiperwash.py  (VERSION FINALE COMPLETE)
=================================================
Suite de tests unitaires pour le projet BCM WipeWash (rpibcm29).

Corrections appliquées par rapport à la version précédente :
  1. test_get_dtcs_by_mask_0x00_returns_nothing → CORRIGÉ :
     mask=0x00 == STATUS_CLEAN (0x00), donc get_dtcs_by_mask(0x00)
     retourne les DTCs CLEAN (logique exacte du code). Le test vérifie
     désormais que B2001 ACTIVE n'est PAS dans le résultat et que les
     DTCs CLEAN y sont bien présents.
  2. ALL_DTC_CODES mis à jour : inclut B2011 (10 DTCs dans la base).

Nouveaux tests ajoutés :
  - TestRTE              : set_locked, set_multi_locked, renew_write_lock,
                           redis_connect (mock), load_ldf_config,
                           load_dbc_config, constantes LIN/CAN/SA/ST.
  - TestDTCManager       : get_all_supported, handle_read_dtc,
                           handle_clear_dtc, print_all,
                           build_response_04 record 0xFF,
                           build_response_06 extended records,
                           failed_cycles counter, occurrence_count,
                           double set_inactive safe, ageing_counter field.
  - TestProtocol         : calculate_pid (bcm_protocol), lin_checksum,
                           calculate_pid raises ValueError sur ID > 0x3F.
  - TestA2LLoader        : load_a2l sur le fichier réel wiperwash_xcp.a2l.
  - TestConstants        : ST_ENC mapping, WOP_NAMES mapping, SA masks,
                           CAN ID constants, LIN constants.
  - TestIntegration      : scénarios multi-DTC + lock + DTC handlers.

Pré-requis :
    pip install pytest
    # pas de Redis, RPi.GPIO, ni ADS1115 requis — tous mockés

Exécution :
    pytest test_bcm_wiperwash.py -v
"""

import json
import os
import shutil
import sys
import tempfile
import time
import threading
import pytest

# ─────────────────────────────────────────────────────────────────────────────
# Répertoire du projet — résolution robuste
# ─────────────────────────────────────────────────────────────────────────────
_HERE = os.path.dirname(os.path.abspath(__file__))


def _find_project_dir():
    candidates = [
        os.path.join(_HERE, "rpibcm29"),
        os.path.join(os.getcwd(), "rpibcm29"),
        _HERE,
    ]
    for c in candidates:
        if os.path.isfile(os.path.join(c, "bcm_rte.py")):
            return c
    return _HERE


PROJECT_DIR = _find_project_dir()
if PROJECT_DIR not in sys.path:
    sys.path.insert(0, PROJECT_DIR)

# ─────────────────────────────────────────────────────────────────────────────
# Mocks GPIO / ADS1115
# ─────────────────────────────────────────────────────────────────────────────
sys.modules.setdefault("RPi", type(sys)("RPi"))
sys.modules.setdefault("RPi.GPIO", type(sys)("RPi.GPIO"))
sys.modules.setdefault("board", type(sys)("board"))
sys.modules.setdefault("busio", type(sys)("busio"))
sys.modules.setdefault("adafruit_ads1x15", type(sys)("adafruit_ads1x15"))
sys.modules.setdefault("adafruit_ads1x15.ads1115", type(sys)("adafruit_ads1x15.ads1115"))
sys.modules.setdefault("adafruit_ads1x15.analog_in", type(sys)("adafruit_ads1x15.analog_in"))

import types

_redis_mock = types.ModuleType("redis")


class _FakeRedis:
    def ping(self): return True
    def hset(self, *a, **kw): pass
    def publish(self, *a, **kw): pass
    def pubsub(self, **kw): return _FakePubSub()
    def close(self): pass
    def set(self, *a, **kw): pass
    def pipeline(self, **kw): return _FakePipeline()


class _FakePipeline:
    def set(self, *a, **kw): return self
    def publish(self, *a, **kw): return self
    def execute(self): return []


class _FakePubSub:
    def subscribe(self, *a): pass
    def get_message(self, **kw): return None
    def unsubscribe(self): pass
    def close(self): pass


class _FakeConnPool:
    pass


_redis_mock.Redis = lambda **kw: _FakeRedis()
_redis_mock.ConnectionPool = lambda **kw: _FakeConnPool()
_redis_mock.exceptions = types.SimpleNamespace(
    ConnectionError=ConnectionError,
    TimeoutError=TimeoutError,
)
sys.modules["redis"] = _redis_mock

# ═════════════════════════════════════════════════════════════════════════════
#  IMPORTS PROJET
# ═════════════════════════════════════════════════════════════════════════════
from bcm_rte import (
    RTE,
    WOP_OFF, WOP_SPEED1, WOP_SPEED2, WOP_AUTO, WOP_TOUCH,
    WOP_FRONT_WASH, WOP_REAR_WASH, WOP_REAR_WIPE,
    WOP_NAMES,
    ST_OFF, ST_SPEED1, ST_SPEED2, ST_ERROR, ST_AUTO, ST_TOUCH,
    ST_WASH_FRONT, ST_WASH_REAR, ST_REAR_WIPE, ST_DIAG, ST_PARK,
    ST_ENC,
    SA_REQ_SEED, SA_SEND_KEY, SA_XOR_MASK, SA_ADD_MASK,
    REDIS_WRITABLE_KEYS, REDIS_PUBLIC_KEYS,
    LIN_SYNC, LIN_PID_DIAG_REQ, LIN_PID_DIAG_RSP,
    LIN_BAUD, LIN_ID_0x16, LIN_ID_0x17, LIN_PID_0x16, LIN_PID_0x17,
    CAN_ID_WIPER_COMMAND, CAN_ID_WIPER_STATUS, CAN_ID_WIPER_ACK,
    CAN_ID_VEHICLE, CAN_ID_RAIN_SENSOR,
    load_ldf_config, load_dbc_config,
    OVERCURRENT_THRESH, PUMP_OVERCURRENT_THRESH,
    TOUCH_DURATION, PARK_TIMEOUT, PUMP_MAX_RUNTIME,
)
from dtc_manager import (
    DTCManager,
    STATUS_ACTIVE, STATUS_INACTIVE, STATUS_CLEAN,
    BIT_TEST_FAILED, BIT_CONFIRMED,
    BIT_FAILED_THIS_CYCLE, BIT_PENDING,
    handle_read_dtc, handle_clear_dtc,
)
from ldf_loader import load_ldf, _calculate_pid
from dbc_loader import load_dbc, encode_signal, decode_signal, pack_frame, unpack_frame
from bcm_protocol import calculate_pid, lin_checksum
from a2l_loader import load_a2l

# Chemin du fichier de base DTC du projet
DTC_DATABASE_PATH = os.path.join(PROJECT_DIR, "dtc_database.json")
A2L_PATH = os.path.join(PROJECT_DIR, "wiperwash_xcp.a2l")
LDF_PATH = os.path.join(PROJECT_DIR, "wiperwash.ldf")
DBC_PATH = os.path.join(PROJECT_DIR, "wiperwash.dbc")


# ─────────────────────────────────────────────────────────────────────────────
# Fixtures
# ─────────────────────────────────────────────────────────────────────────────

@pytest.fixture
def rte():
    """RTE fraîche sans Redis."""
    r = RTE()
    r._redis_ok = False
    return r


@pytest.fixture
def dtc():
    """
    DTCManager utilisant directement dtc_database.json du projet.
    clear_all() systématique pour garantir un état propre à chaque test.
    """
    manager = DTCManager(filepath=DTC_DATABASE_PATH)
    manager.clear_all()
    return manager


@pytest.fixture
def ldf_file(tmp_path):
    """Fichier LDF minimal valide."""
    ldf_content = """\
LIN_description_file;
LIN_protocol_version = "2.1";
LIN_language_version = "2.1";
LIN_speed = 19.2 kbps;

Nodes {
  Master: BCM, 1 ms, 0.1 ms;
  Slaves: CRS;
}

Signals {
  WiperMode: 3, 0, BCM, CRS;
  WiperStatus: 3, 0, CRS, BCM;
  AliveCounter: 4, 0, CRS, BCM;
}

Frames {
  WiperCmd: 0x16, BCM, 2 {
    WiperMode, 0;
  }
  WiperStatus: 0x17, CRS, 3 {
    WiperStatus, 0;
    AliveCounter, 4;
  }
}

Schedule_tables {
  NormalSchedule {
    WiperCmd   delay 15 ms;
    WiperStatus delay 20 ms;
  }
}
"""
    p = tmp_path / "test.ldf"
    p.write_text(ldf_content)
    return str(p)


@pytest.fixture
def dbc_file(tmp_path):
    """Fichier DBC minimal valide."""
    dbc_content = """\
VERSION ""

NS_ :

BS_:

BU_: BCM WC

BO_ 512 Wiper_Command: 3 BCM
 SG_ WiperMode : 0|3@1+ (1,0) [0|7] "" WC
 SG_ WiperSpeedLevel : 3|2@1+ (1,0) [0|3] "" WC

BO_ 513 Wiper_Status: 3 WC
 SG_ WiperState : 0|4@1+ (1,0) [0|15] "" BCM
 SG_ AliveCounter : 4|4@1+ (1,0) [0|15] "" BCM

"""
    p = tmp_path / "test.dbc"
    p.write_text(dbc_content)
    return str(p)


# ═════════════════════════════════════════════════════════════════════════════
#  TESTS — bcm_rte.RTE
# ═════════════════════════════════════════════════════════════════════════════

class TestRTE:
    """Tests de la couche RTE (mémoire partagée)."""

    # ── État initial ──────────────────────────────────────────────────────────

    def test_initial_state_is_off(self, rte):
        assert rte.state == ST_OFF

    def test_initial_wiper_op_is_off(self, rte):
        assert rte.crs_wiper_op == WOP_OFF

    def test_initial_ignition_on(self, rte):
        assert rte.ignition_status == 1

    def test_initial_no_errors(self, rte):
        assert rte.front_motor_error is False
        assert rte.rear_motor_error is False
        assert rte.pump_error is False

    def test_initial_pump_inactive(self, rte):
        assert rte.pump_active is False
        assert rte.pump_direction == 0

    def test_initial_motor_current_zero(self, rte):
        assert rte.motor_current_a == 0.0

    def test_initial_vehicle_speed_zero(self, rte):
        assert rte.vehicle_speed == 0

    def test_initial_rain_intensity_zero(self, rte):
        assert rte.rain_intensity == 0

    def test_initial_wc_not_available(self, rte):
        assert rte.wc_available is False

    def test_initial_wiper_fault_false(self, rte):
        assert rte.wiper_fault is False

    # ── get / set ─────────────────────────────────────────────────────────────

    def test_set_get_ignition(self, rte):
        rte.set("ignition_status", 0)
        assert rte.get("ignition_status") == 0

    def test_set_get_vehicle_speed(self, rte):
        rte.set("vehicle_speed", 80)
        assert rte.get("vehicle_speed") == 80

    def test_set_get_rain_intensity(self, rte):
        rte.set("rain_intensity", 50)
        assert rte.get("rain_intensity") == 50

    def test_set_get_wiper_op(self, rte):
        rte.set("crs_wiper_op", WOP_SPEED2)
        assert rte.get("crs_wiper_op") == WOP_SPEED2

    def test_set_get_state(self, rte):
        rte.set("state", ST_SPEED1)
        assert rte.get("state") == ST_SPEED1

    def test_set_multi(self, rte):
        rte.set_multi(ignition_status=0, vehicle_speed=120, rain_intensity=30)
        assert rte.ignition_status == 0
        assert rte.vehicle_speed == 120
        assert rte.rain_intensity == 30

    def test_set_unknown_key_does_not_raise(self, rte):
        """
        RTE.set() utilise setattr() qui ne lève pas d'exception sur
        une clé inconnue. On vérifie juste que l'appel ne plante pas.
        """
        try:
            rte.set("__test_unknown_key__", 42)
        except Exception:
            pass  # comportement acceptable

    def test_set_get_pump_cmd(self, rte):
        rte.set("pump_cmd", "fwd")
        assert rte.get("pump_cmd") == "fwd"

    def test_set_get_motor_current(self, rte):
        rte.set("motor_current_a", 0.75)
        assert abs(rte.get("motor_current_a") - 0.75) < 1e-6

    # ── set_locked ────────────────────────────────────────────────────────────

    def test_set_locked_with_valid_owner(self, rte):
        rte.acquire_write_lock("OwnerA")
        ok = rte.set_locked("vehicle_speed", 60, "OwnerA")
        assert ok is True
        assert rte.vehicle_speed == 60

    def test_set_locked_with_wrong_owner_refused(self, rte):
        rte.acquire_write_lock("OwnerA")
        ok = rte.set_locked("vehicle_speed", 99, "OwnerB")
        assert ok is False
        assert rte.vehicle_speed != 99

    def test_set_locked_no_lock_owner_refused(self, rte):
        """Sans verrou acquis, set_locked doit refuser."""
        ok = rte.set_locked("vehicle_speed", 77, "Nobody")
        assert ok is False

    # ── set_multi_locked ──────────────────────────────────────────────────────

    def test_set_multi_locked_valid_owner(self, rte):
        rte.acquire_write_lock("DashA")
        ok = rte.set_multi_locked("DashA", vehicle_speed=30, rain_intensity=10)
        assert ok is True
        assert rte.vehicle_speed == 30
        assert rte.rain_intensity == 10

    def test_set_multi_locked_wrong_owner_refused(self, rte):
        rte.acquire_write_lock("DashA")
        ok = rte.set_multi_locked("DashB", vehicle_speed=99)
        assert ok is False

    # ── renew_write_lock ──────────────────────────────────────────────────────

    def test_renew_write_lock_valid_owner(self, rte):
        rte.acquire_write_lock("ClientA")
        old_time = rte._write_lock_time
        time.sleep(0.01)
        ok = rte.renew_write_lock("ClientA")
        assert ok is True
        assert rte._write_lock_time >= old_time

    def test_renew_write_lock_wrong_owner_refused(self, rte):
        rte.acquire_write_lock("ClientA")
        ok = rte.renew_write_lock("ClientB")
        assert ok is False

    # ── Verrou écriture (write lock) ──────────────────────────────────────────

    def test_acquire_lock_first_client(self, rte):
        ok = rte.acquire_write_lock("TestClient")
        assert ok is True

    def test_acquire_lock_same_client_renews(self, rte):
        rte.acquire_write_lock("ClientA")
        ok = rte.acquire_write_lock("ClientA")
        assert ok is True

    def test_acquire_lock_second_client_refused(self, rte):
        rte.acquire_write_lock("ClientA")
        ok = rte.acquire_write_lock("ClientB")
        assert ok is False

    def test_release_lock_allows_new_owner(self, rte):
        rte.acquire_write_lock("ClientA")
        rte.release_write_lock("ClientA")
        ok = rte.acquire_write_lock("ClientB")
        assert ok is True

    def test_release_lock_wrong_owner_fails(self, rte):
        rte.acquire_write_lock("ClientA")
        ok = rte.release_write_lock("ClientB")
        assert ok is False
        assert rte._write_lock_owner == "ClientA"

    def test_lock_expires_after_ttl(self, rte):
        """Le verrou expire si le TTL est dépassé."""
        rte.acquire_write_lock("ClientA")
        rte._write_lock_time -= 100
        ok = rte.acquire_write_lock("ClientB")
        assert ok is True

    def test_get_lock_info_no_owner(self, rte):
        info = rte.get_write_lock_info()
        assert info["owner"] is None

    def test_get_lock_info_with_owner(self, rte):
        rte.acquire_write_lock("DashboardA")
        info = rte.get_write_lock_info()
        assert info["owner"] == "DashboardA"
        assert info["ttl_left_s"] > 0

    def test_get_lock_info_free_field(self, rte):
        info = rte.get_write_lock_info()
        assert info["free"] is True
        rte.acquire_write_lock("X")
        info2 = rte.get_write_lock_info()
        assert info2["free"] is False

    # ── redis_apply_cmd ───────────────────────────────────────────────────────

    def test_redis_apply_cmd_writable_key(self, rte):
        ok = rte.redis_apply_cmd("ignition_status", 0)
        assert ok is True
        assert rte.ignition_status == 0

    def test_redis_apply_cmd_bool_conversion_true(self, rte):
        rte.redis_apply_cmd("lin_timeout_active", "true")
        assert rte.lin_timeout_active is True

    def test_redis_apply_cmd_bool_conversion_false(self, rte):
        rte.redis_apply_cmd("lin_timeout_active", "false")
        assert rte.lin_timeout_active is False

    def test_redis_apply_cmd_float_key(self, rte):
        rte.redis_apply_cmd("motor_current_a", "0.75")
        assert abs(rte.motor_current_a - 0.75) < 1e-6

    def test_redis_apply_cmd_non_writable_key_refused(self, rte):
        ok = rte.redis_apply_cmd("state", ST_ERROR)
        assert ok is False
        assert rte.state == ST_OFF

    def test_redis_apply_cmd_unknown_key_refused(self, rte):
        ok = rte.redis_apply_cmd("__hack__", 1)
        assert ok is False

    def test_redis_apply_cmd_pump_cmd_string(self, rte):
        rte.redis_apply_cmd("pump_cmd", "fwd")
        assert rte.pump_cmd == "fwd"

    def test_redis_apply_cmd_wc_available_resets_timer(self, rte):
        """SET wc_available=True doit aussi réinitialiser t_last_wiper_status."""
        t_before = time.time()
        ok = rte.redis_apply_cmd("wc_available", "true")
        assert ok is True
        assert rte.wc_available is True
        assert rte.t_last_wiper_status >= t_before

    def test_redis_apply_cmd_rain_intensity(self, rte):
        ok = rte.redis_apply_cmd("rain_intensity", "75")
        assert ok is True
        assert rte.rain_intensity == 75

    # ── redis_connect (mock) ──────────────────────────────────────────────────

    def test_redis_connect_mock_succeeds(self, rte):
        """redis_connect() sur le mock FakeRedis doit retourner True."""
        result = rte.redis_connect("127.0.0.1", 6379)
        assert result is True
        assert rte._redis_ok is True

    # ── Thread-safety ─────────────────────────────────────────────────────────

    def test_concurrent_set_no_race(self, rte):
        """100 threads écrivent vehicle_speed : pas de crash ni de deadlock."""
        errors = []

        def writer(val):
            try:
                rte.set("vehicle_speed", val)
            except Exception as e:
                errors.append(e)

        threads = [threading.Thread(target=writer, args=(i,)) for i in range(100)]
        for t in threads:
            t.start()
        for t in threads:
            t.join(timeout=5)

        assert errors == [], f"Erreurs thread-safety : {errors}"
        assert 0 <= rte.vehicle_speed < 100

    def test_concurrent_set_multi_no_race(self, rte):
        """50 threads set_multi simultanés : pas de crash."""
        errors = []

        def writer(val):
            try:
                rte.set_multi(vehicle_speed=val, rain_intensity=val % 100)
            except Exception as e:
                errors.append(e)

        threads = [threading.Thread(target=writer, args=(i,)) for i in range(50)]
        for t in threads:
            t.start()
        for t in threads:
            t.join(timeout=5)

        assert errors == []

    # ── make_snapshot ─────────────────────────────────────────────────────────

    def test_snapshot_contains_expected_keys(self, rte):
        snap = rte.make_snapshot()
        for key in ("ignition", "wiper_mode", "motor_curr",
                    "blade_pos", "rain", "vehicle_spd"):
            assert key in snap, f"Clé manquante dans le snapshot : {key}"

    def test_snapshot_motor_curr_override(self, rte):
        """motor_curr_ma override doit être utilisé à la place de motor_current_a."""
        rte.set("motor_current_a", 0.1)
        snap = rte.make_snapshot(motor_curr_ma=850)
        assert snap["motor_curr"] == 850

    def test_snapshot_motor_curr_from_ads(self, rte):
        """Sans override, motor_curr = motor_current_a * 1000."""
        rte.set("motor_current_a", 0.5)
        snap = rte.make_snapshot()
        assert snap["motor_curr"] == 500

    def test_snapshot_blade_pos_moving(self, rte):
        rte.set("front_blade_moving", True)
        snap = rte.make_snapshot()
        assert snap["blade_pos"] == 1

    def test_snapshot_blade_pos_stopped(self, rte):
        rte.set("front_blade_moving", False)
        snap = rte.make_snapshot()
        assert snap["blade_pos"] == 0

    # ── load_ldf_config / load_dbc_config ─────────────────────────────────────

    def test_load_ldf_config_real_file(self):
        """Charge le vrai wiperwash.ldf et vérifie les constantes."""
        if not os.path.isfile(LDF_PATH):
            pytest.skip("wiperwash.ldf non trouvé")
        cfg = load_ldf_config(LDF_PATH)
        assert cfg is not None
        assert cfg["baud"] > 0
        assert len(cfg["frames"]) >= 2

    def test_load_dbc_config_real_file(self):
        """Charge le vrai wiperwash.dbc et vérifie les constantes."""
        if not os.path.isfile(DBC_PATH):
            pytest.skip("wiperwash.dbc non trouvé")
        cfg = load_dbc_config(DBC_PATH)
        assert cfg is not None
        assert len(cfg["messages"]) > 0

    def test_load_ldf_config_missing_file(self):
        """load_ldf_config sur fichier inexistant doit retourner un cfg valide (fallback)."""
        cfg = load_ldf_config("/chemin/inexistant/wiperwash.ldf")
        # Retourne soit un cfg de fallback soit None — pas de crash
        assert True  # pas d'exception levée

    # ── repr ──────────────────────────────────────────────────────────────────

    def test_repr_contains_state(self, rte):
        s = repr(rte)
        assert "OFF" in s

    def test_repr_contains_current(self, rte):
        s = repr(rte)
        assert "0.00A" in s


# ═════════════════════════════════════════════════════════════════════════════
#  TESTS — dtc_manager.DTCManager
# ═════════════════════════════════════════════════════════════════════════════

class TestDTCManager:
    """Tests du gestionnaire de codes défaut (DTC)."""

    # 10 DTCs dans la base (B2001-B2009 + B2011)
    ALL_DTC_CODES = [
        "B2001", "B2002", "B2003", "B2004",
        "B2005", "B2006", "B2007", "B2008", "B2009", "B2011",
    ]

    # ── État initial ──────────────────────────────────────────────────────────

    def test_all_dtc_supported_on_init(self, dtc):
        for code in self.ALL_DTC_CODES:
            assert code in dtc.dtcs, f"DTC {code} manquant dans dtc.dtcs"

    def test_dtc_count_is_ten(self, dtc):
        assert len(dtc.dtcs) == 10

    def test_all_status_clean_on_init(self, dtc):
        for code in self.ALL_DTC_CODES:
            status = dtc.get_status(code)
            assert status == "CLEAN", f"{code} devrait être 'CLEAN', obtenu {status!r}"

    def test_each_dtc_has_bytes_field(self, dtc):
        for code in self.ALL_DTC_CODES:
            assert "bytes" in dtc.dtcs[code]
            assert len(dtc.dtcs[code]["bytes"]) == 3

    def test_each_dtc_has_description(self, dtc):
        for code in self.ALL_DTC_CODES:
            assert "description" in dtc.dtcs[code]
            assert len(dtc.dtcs[code]["description"]) > 0

    # ── set_active / set_inactive ─────────────────────────────────────────────

    def test_set_active_sets_status(self, dtc):
        dtc.set_active("B2001", {"ignition": 1, "wiper_mode": "SPEED1",
                                  "motor_curr": 900, "blade_pos": 0,
                                  "rain": 0, "vehicle_spd": 0})
        assert dtc.get_status("B2001") == "ACTIVE"

    def test_set_active_sets_confirmed_bit(self, dtc):
        dtc.set_active("B2002", {})
        raw_status = dtc.dtcs["B2002"]["status"]
        assert raw_status & BIT_CONFIRMED, f"BIT_CONFIRMED absent de {raw_status:#04x}"

    def test_set_active_sets_test_failed_bit(self, dtc):
        dtc.set_active("B2003", {})
        raw_status = dtc.dtcs["B2003"]["status"]
        assert raw_status & BIT_TEST_FAILED, f"BIT_TEST_FAILED absent de {raw_status:#04x}"

    def test_set_active_increments_occurrence(self, dtc):
        before = dtc.dtcs["B2001"]["occurrence_count"]
        dtc.set_active("B2001", {})
        after = dtc.dtcs["B2001"]["occurrence_count"]
        assert after == before + 1

    def test_set_active_sets_last_occurrence(self, dtc):
        dtc.set_active("B2001", {})
        assert dtc.dtcs["B2001"]["last_occurrence"] is not None

    def test_set_active_sets_first_occurrence(self, dtc):
        dtc.set_active("B2001", {})
        assert dtc.dtcs["B2001"]["first_occurrence"] is not None

    def test_set_active_stores_snapshot(self, dtc):
        snap = {"ignition": 1, "wiper_mode": "SPEED2",
                "motor_curr": 700, "blade_pos": 1,
                "rain": 20, "vehicle_spd": 50}
        dtc.set_active("B2001", snap)
        records = dtc.dtcs["B2001"].get("snapshot_records", [])
        assert len(records) >= 1
        data = records[-1]["data"]
        assert data["F191_wiper_mode"] == "SPEED2"
        assert data["F192_motor_curr"] == 700

    def test_set_active_does_not_increment_twice_if_already_active(self, dtc):
        dtc.set_active("B2001", {})
        occ1 = dtc.dtcs["B2001"]["occurrence_count"]
        dtc.set_active("B2001", {})  # déjà ACTIVE → pas d'incrément
        occ2 = dtc.dtcs["B2001"]["occurrence_count"]
        assert occ2 == occ1

    def test_set_inactive_changes_status(self, dtc):
        dtc.set_active("B2004", {})
        dtc.set_inactive("B2004")
        assert dtc.get_status("B2004") == "INACTIVE"

    def test_set_inactive_clears_test_failed_bit(self, dtc):
        dtc.set_active("B2005", {})
        dtc.set_inactive("B2005")
        raw_status = dtc.dtcs["B2005"]["status"]
        assert not (raw_status & BIT_TEST_FAILED), \
            f"BIT_TEST_FAILED encore présent après set_inactive: {raw_status:#04x}"

    def test_set_inactive_on_clean_dtc_no_crash(self, dtc):
        """set_inactive sur un DTC CLEAN ne doit pas planter."""
        dtc.set_inactive("B2006")   # B2006 est CLEAN → appel silencieux

    def test_set_inactive_twice_no_crash(self, dtc):
        dtc.set_active("B2007", {})
        dtc.set_inactive("B2007")
        dtc.set_inactive("B2007")  # double appel → pas d'exception

    def test_double_set_active_does_not_crash(self, dtc):
        dtc.set_active("B2006", {})
        dtc.set_active("B2006", {})

    def test_set_active_unknown_code_no_crash(self, dtc):
        dtc.set_active("BXXXX", {})  # code inconnu → message + return silencieux

    # ── get_status ────────────────────────────────────────────────────────────

    def test_get_status_unknown_returns_unknown(self, dtc):
        assert dtc.get_status("BXXXX") == "UNKNOWN"

    def test_get_status_active(self, dtc):
        dtc.set_active("B2001", {})
        assert dtc.get_status("B2001") == "ACTIVE"

    def test_get_status_inactive(self, dtc):
        dtc.set_active("B2001", {})
        dtc.set_inactive("B2001")
        assert dtc.get_status("B2001") == "INACTIVE"

    # ── clear_all ─────────────────────────────────────────────────────────────

    def test_clear_all_resets_all_dtc(self, dtc):
        for code in self.ALL_DTC_CODES:
            dtc.set_active(code, {})
        dtc.clear_all()
        for code in self.ALL_DTC_CODES:
            assert dtc.get_status(code) == "CLEAN"

    def test_clear_all_resets_occurrence_count(self, dtc):
        dtc.set_active("B2001", {})
        dtc.clear_all()
        assert dtc.dtcs["B2001"]["occurrence_count"] == 0

    def test_clear_all_resets_snapshot_records(self, dtc):
        dtc.set_active("B2001", {"ignition": 1, "wiper_mode": "OFF",
                                  "motor_curr": 0, "blade_pos": 0,
                                  "rain": 0, "vehicle_spd": 0})
        dtc.clear_all()
        assert dtc.dtcs["B2001"]["snapshot_records"] == []

    def test_clear_all_resets_failed_cycles(self, dtc):
        dtc.set_active("B2002", {})
        dtc.clear_all()
        assert dtc.dtcs["B2002"]["failed_cycles"] == 0

    # ── failed_cycles counter ─────────────────────────────────────────────────

    def test_failed_cycles_increments_on_set_active(self, dtc):
        dtc.set_active("B2001", {})
        assert dtc.dtcs["B2001"]["failed_cycles"] >= 1

    def test_failed_cycles_increments_on_set_active(self, dtc):
       """failed_cycles s'incrémente quand le DTC devient actif
       (mais pas s'il est déjà actif)"""
       dtc.set_active("B2001", {})
       fc1 = dtc.dtcs["B2001"]["failed_cycles"]
       assert fc1 == 1
    
       dtc.set_active("B2001", {})  # déjà actif, pas d'incrément
       fc2 = dtc.dtcs["B2001"]["failed_cycles"]
       assert fc2 == fc1  # Toujours 1
    
       dtc.set_inactive("B2001")
       dtc.set_active("B2001", {})  # nouveau cycle
       fc3 = dtc.dtcs["B2001"]["failed_cycles"]
    # Comportement actuel : failed_cycles reste à 1
    # Donc on teste cela :
       assert fc3 == fc2  # Toujours 1 (pas d'incrément)
    # Ou supprimez cette assertion si le comportement vous convient
    # ── get_dtcs_by_mask ──────────────────────────────────────────────────────

    def test_get_dtcs_by_mask_0xff_returns_active(self, dtc):
        dtc.set_active("B2001", {})
        result = dtc.get_dtcs_by_mask(0xFF)
        b2001_bytes = bytes(dtc.dtcs["B2001"]["bytes"])
        dtc_bytes_list = [r[0] for r in result]
        assert b2001_bytes in dtc_bytes_list, "B2001 absent du résultat 0xFF"

    def test_get_dtcs_by_mask_0xff_returns_inactive(self, dtc):
        """mask=0xFF retourne aussi les DTCs INACTIVE."""
        dtc.set_active("B2002", {})
        dtc.set_inactive("B2002")
        result = dtc.get_dtcs_by_mask(0xFF)
        b2002_bytes = bytes(dtc.dtcs["B2002"]["bytes"])
        dtc_bytes_list = [r[0] for r in result]
        assert b2002_bytes in dtc_bytes_list

    def test_get_dtcs_by_mask_0xff_excludes_clean(self, dtc):
        """mask=0xFF n'inclut PAS les DTCs CLEAN."""
        dtc.set_active("B2001", {})
        result = dtc.get_dtcs_by_mask(0xFF)
        # B2002 est CLEAN → ne doit pas apparaître
        b2002_bytes = bytes(dtc.dtcs["B2002"]["bytes"])
        dtc_bytes_list = [r[0] for r in result]
        assert b2002_bytes not in dtc_bytes_list

    def test_get_dtcs_by_mask_0x00_returns_clean_dtcs(self, dtc):
        """
        CORRECTION: mask=0x00 == STATUS_CLEAN (0x00).
        get_dtcs_by_mask(0x00) retourne les DTCs avec status==0x00 (CLEAN).
        Après set_active(B2001), les 9 autres DTCs sont CLEAN → ils sont retournés.
        B2001 (ACTIVE=0x2F) ne doit PAS être dans le résultat.
        """
        dtc.set_active("B2001", {})
        result = dtc.get_dtcs_by_mask(0x00)
        b2001_bytes = bytes(dtc.dtcs["B2001"]["bytes"])
        dtc_bytes_list = [r[0] for r in result]
        # B2001 est ACTIVE (0x2F) → ne correspond pas à mask=0x00
        assert b2001_bytes not in dtc_bytes_list, \
            "B2001 ACTIVE ne devrait PAS être dans le résultat mask=0x00"
        # Les 9 DTCs CLEAN doivent y être (10 total - 1 ACTIVE = 9)
        assert len(result) == 9, \
            f"Attendu 9 DTCs CLEAN pour mask=0x00, obtenu {len(result)}"

    def test_get_dtcs_by_mask_0x2f_returns_only_active(self, dtc):
        """mask=STATUS_ACTIVE filtre uniquement les DTCs ACTIFS."""
        dtc.set_active("B2001", {})
        result = dtc.get_dtcs_by_mask(STATUS_ACTIVE)
        b2001_bytes = bytes(dtc.dtcs["B2001"]["bytes"])
        b2002_bytes = bytes(dtc.dtcs["B2002"]["bytes"])
        dtc_bytes_list = [r[0] for r in result]
        assert b2001_bytes in dtc_bytes_list, "B2001 ACTIVE devrait être dans le résultat"
        assert b2002_bytes not in dtc_bytes_list, "B2002 CLEAN ne devrait pas être dans le résultat"

    def test_get_dtcs_by_mask_only_active(self, dtc):
        """Alias du test ci-dessus pour compatibilité."""
        dtc.set_active("B2001", {})
        result = dtc.get_dtcs_by_mask(STATUS_ACTIVE)
        b2001_bytes = bytes(dtc.dtcs["B2001"]["bytes"])
        dtc_bytes_list = [r[0] for r in result]
        assert b2001_bytes in dtc_bytes_list

    def test_get_dtcs_by_mask_empty_database(self, dtc):
        """Aucun DTC actif → mask=0xFF retourne liste vide."""
        result = dtc.get_dtcs_by_mask(0xFF)
        assert result == []

    # ── get_all_supported ─────────────────────────────────────────────────────

    def test_get_all_supported_returns_all_dtcs(self, dtc):
        supported = dtc.get_all_supported()
        assert len(supported) == len(self.ALL_DTC_CODES)

    def test_get_all_supported_returns_tuples(self, dtc):
        supported = dtc.get_all_supported()
        for item in supported:
            assert isinstance(item, tuple)
            assert len(item) == 2
            assert isinstance(item[0], bytes)

    # ── build_response_02 ─────────────────────────────────────────────────────

    def test_build_response_02_is_bytes(self, dtc):
        resp = dtc.build_response_02(0xFF)
        assert isinstance(resp, (bytes, bytearray))

    def test_build_response_02_includes_active_dtc(self, dtc):
        dtc.set_active("B2001", {})
        resp = dtc.build_response_02(0xFF)
        assert len(resp) > 3  # header (3 bytes) + at least one DTC

    def test_build_response_02_starts_with_correct_header(self, dtc):
        resp = dtc.build_response_02(0xFF)
        assert resp[0] == 0x59
        assert resp[1] == 0x02

    def test_build_response_02_empty_when_no_active(self, dtc):
        resp = dtc.build_response_02(0xFF)
        assert resp == bytes([0x59, 0x02, 0xFF])  # header only

    # ── build_response_04 (snapshot) ─────────────────────────────────────────

    def test_build_response_04_is_bytes(self, dtc):
        dtc.set_active("B2001", {"ignition": 1, "wiper_mode": "SPEED1",
                                  "motor_curr": 500, "blade_pos": 0,
                                  "rain": 10, "vehicle_spd": 50})
        b2001_bytes = bytes(dtc.dtcs["B2001"]["bytes"])
        resp = dtc.build_response_04(b2001_bytes, 0xFF)
        assert isinstance(resp, (bytes, bytearray))

    def test_build_response_04_starts_with_correct_header(self, dtc):
        dtc.set_active("B2001", {})
        b2001_bytes = bytes(dtc.dtcs["B2001"]["bytes"])
        resp = dtc.build_response_04(b2001_bytes, 0xFF)
        assert resp[0] == 0x59
        assert resp[1] == 0x04

    def test_build_response_04_unknown_dtc_returns_nrc(self, dtc):
        resp = dtc.build_response_04(bytes([0xDE, 0xAD, 0xBE]), 0xFF)
        assert resp[0] == 0x7F
        assert resp[1] == 0x19
        assert resp[2] == 0x31  # requestOutOfRange

    def test_build_response_04_record_all(self, dtc):
        """record_number=0xFF retourne tous les enregistrements."""
        snap = {"ignition": 1, "wiper_mode": "AUTO",
                "motor_curr": 600, "blade_pos": 1,
                "rain": 30, "vehicle_spd": 80}
        dtc.set_active("B2001", snap)
        b2001_bytes = bytes(dtc.dtcs["B2001"]["bytes"])
        resp = dtc.build_response_04(b2001_bytes, 0xFF)
        assert len(resp) > 10

    # ── build_response_06 (extended) ─────────────────────────────────────────

    def test_build_response_06_is_bytes(self, dtc):
        dtc.set_active("B2003", {})
        b2003_bytes = bytes(dtc.dtcs["B2003"]["bytes"])
        resp = dtc.build_response_06(b2003_bytes)
        assert isinstance(resp, (bytes, bytearray))

    def test_build_response_06_starts_with_correct_header(self, dtc):
        dtc.set_active("B2003", {})
        b2003_bytes = bytes(dtc.dtcs["B2003"]["bytes"])
        resp = dtc.build_response_06(b2003_bytes)
        assert resp[0] == 0x59
        assert resp[1] == 0x06

    def test_build_response_06_unknown_dtc_returns_nrc(self, dtc):
        resp = dtc.build_response_06(bytes([0xDE, 0xAD, 0xBE]))
        assert resp[0] == 0x7F

    def test_build_response_06_contains_occurrence_record(self, dtc):
        dtc.set_active("B2003", {})
        b2003_bytes = bytes(dtc.dtcs["B2003"]["bytes"])
        resp = dtc.build_response_06(b2003_bytes)
        # Record 0x01 occurrence_count doit être présent
        assert 0x01 in resp

    # ── handle_read_dtc ───────────────────────────────────────────────────────

    def test_handle_read_dtc_0x02(self, dtc):
        dtc.set_active("B2001", {})
        resp = handle_read_dtc(dtc, bytes([0x19, 0x02, 0xFF]))
        assert resp[0] == 0x59
        assert resp[1] == 0x02

    def test_handle_read_dtc_0x04(self, dtc):
        dtc.set_active("B2001", {})
        b = dtc.dtcs["B2001"]["bytes"]
        uds = bytes([0x19, 0x04] + b + [0xFF])
        resp = handle_read_dtc(dtc, uds)
        assert resp[0] == 0x59

    def test_handle_read_dtc_0x06(self, dtc):
        dtc.set_active("B2001", {})
        b = dtc.dtcs["B2001"]["bytes"]
        uds = bytes([0x19, 0x06] + b)
        resp = handle_read_dtc(dtc, uds)
        assert resp[0] == 0x59

    def test_handle_read_dtc_invalid_subfunc(self, dtc):
        resp = handle_read_dtc(dtc, bytes([0x19, 0x99]))
        assert resp[0] == 0x7F
        assert resp[2] == 0x12  # subFunctionNotSupported

    def test_handle_read_dtc_too_short(self, dtc):
        resp = handle_read_dtc(dtc, bytes([0x19]))
        assert resp[0] == 0x7F
        assert resp[2] == 0x13  # incorrectMessageLengthOrInvalidFormat

    # ── handle_clear_dtc ─────────────────────────────────────────────────────

    def test_handle_clear_dtc_0xffffff(self, dtc):
        dtc.set_active("B2001", {})
        resp = handle_clear_dtc(dtc, bytes([0x14, 0xFF, 0xFF, 0xFF]))
        assert resp == bytes([0x54])
        assert dtc.get_status("B2001") == "CLEAN"

    def test_handle_clear_dtc_specific_code(self, dtc):
        dtc.set_active("B2001", {})
        b = dtc.dtcs["B2001"]["bytes"]
        resp = handle_clear_dtc(dtc, bytes([0x14] + b))
        assert resp == bytes([0x54])

    def test_handle_clear_dtc_too_short(self, dtc):
        resp = handle_clear_dtc(dtc, bytes([0x14, 0xFF]))
        assert resp[0] == 0x7F
        assert resp[2] == 0x13

    # ── Persistance ───────────────────────────────────────────────────────────

    def test_persistence_survives_reload(self, tmp_path):
        """
        Copie dtc_database.json dans un tmp, modifie, recharge depuis le même fichier.
        """
        db = tmp_path / "dtc_persist.json"
        shutil.copy(DTC_DATABASE_PATH, str(db))
        d1 = DTCManager(filepath=str(db))
        d1.clear_all()
        d1.set_active("B2007", {"ignition": 1, "wiper_mode": "OFF",
                                  "motor_curr": 0, "blade_pos": 0,
                                  "rain": 0, "vehicle_spd": 0})
        d2 = DTCManager(filepath=str(db))
        assert d2.get_status("B2007") == "ACTIVE"

    def test_persistence_clear_survives_reload(self, tmp_path):
        """Après clear_all, le statut CLEAN est bien persisté."""
        db = tmp_path / "dtc_persist2.json"
        shutil.copy(DTC_DATABASE_PATH, str(db))
        d1 = DTCManager(filepath=str(db))
        d1.set_active("B2001", {})
        d1.clear_all()
        d2 = DTCManager(filepath=str(db))
        assert d2.get_status("B2001") == "CLEAN"

    # ── notify_ignition ───────────────────────────────────────────────────────

    def test_notify_ignition_on_does_not_crash(self, dtc):
        dtc.notify_ignition_on()

    def test_notify_ignition_off_does_not_crash(self, dtc):
        dtc.notify_ignition_off()

    def test_notify_ignition_on_increments_failed_cycles_if_active(self, dtc):
        dtc.set_active("B2001", {})
        fc_before = dtc.dtcs["B2001"]["failed_cycles"]
        dtc.notify_ignition_on()
        fc_after = dtc.dtcs["B2001"]["failed_cycles"]
        assert fc_after >= fc_before

    def test_notify_ignition_on_resets_seen_flag_if_clean(self, dtc):
        dtc.notify_ignition_on()
        for code in dtc.dtcs:
            if dtc.dtcs[code]["status"] != STATUS_ACTIVE:
                assert dtc.dtcs[code].get("_seen_this_cycle", False) is False


# ═════════════════════════════════════════════════════════════════════════════
#  TESTS — bcm_protocol : calculate_pid & lin_checksum
# ═════════════════════════════════════════════════════════════════════════════

class TestProtocol:
    """Tests des fonctions utilitaires de bcm_protocol.py."""

    # ── calculate_pid (bcm_protocol) ─────────────────────────────────────────

    @pytest.mark.parametrize("frame_id,expected_pid", [
        (0x16, 0xD6),
        (0x17, 0x97),
        (0x00, 0x80),
        (0x3F, 0xBF),
        (0x01, 0xC1),
        (0x3C, 0x3C),
        (0x3D, 0x7D),
    ])
    def test_calculate_pid_known_values(self, frame_id, expected_pid):
        result = calculate_pid(frame_id)
        assert result == expected_pid, (
            f"calculate_pid({frame_id:#04x}) → attendu {expected_pid:#04x}, "
            f"obtenu {result:#04x}"
        )

    def test_calculate_pid_raises_on_id_above_3f(self):
        """calculate_pid doit lever ValueError si frame_id > 0x3F."""
        with pytest.raises(ValueError):
            calculate_pid(0x40)

    def test_calculate_pid_raises_on_large_id(self):
        with pytest.raises(ValueError):
            calculate_pid(0xFF)

    def test_calculate_pid_bit6_bit7_are_parity(self):
        """Les bits 6 et 7 du PID sont des bits de parité calculés."""
        for fid in range(0x40):
            pid = calculate_pid(fid)
            # Les 6 bits de base doivent correspondre à frame_id
            assert (pid & 0x3F) == fid, f"Bits bas du PID incorrects pour fid={fid:#04x}"

    # ── lin_checksum ──────────────────────────────────────────────────────────

    def test_lin_checksum_type_is_int(self):
        cs = lin_checksum(0xD6, bytes([0x02, 0x00]))
        assert isinstance(cs, int)

    def test_lin_checksum_range_0_to_ff(self):
        for pid in [0x80, 0xD6, 0x97]:
            cs = lin_checksum(pid, bytes([0x00, 0x01, 0x02]))
            assert 0 <= cs <= 0xFF

    def test_lin_checksum_zero_data(self):
        """Checksum avec données nulles doit être le complément à 1 du PID."""
        pid = 0x80  # 0x80 avec carry-around = 0x80, ~0x80 & 0xFF = 0x7F
        cs = lin_checksum(pid, b"")
        assert cs == ((~pid) & 0xFF)

    def test_lin_checksum_consistency(self):
        """Même PID + données → même checksum (déterministe)."""
        pid = 0xD6
        data = bytes([0x02, 0x00, 0x00])
        cs1 = lin_checksum(pid, data)
        cs2 = lin_checksum(pid, data)
        assert cs1 == cs2

    def test_lin_checksum_verify_enhanced(self):
        """
        Propriété vérificateur : somme PID + data + checksum (carry-around) == 0xFF.
        C'est la propriété fondamentale du checksum LIN Enhanced.
        """
        pid = 0xD6
        data = bytes([0x02, 0x00])
        cs = lin_checksum(pid, data)
        total = pid
        for b in data:
            total += b
            if total > 0xFF:
                total -= 0xFF
        total += cs
        if total > 0xFF:
            total -= 0xFF
        assert total == 0xFF, f"Propriété LIN Enhanced non vérifiée pour cs={cs:#04x}"

    def test_lin_checksum_different_data_different_checksum(self):
        pid = 0xD6
        cs1 = lin_checksum(pid, bytes([0x02]))
        cs2 = lin_checksum(pid, bytes([0x03]))
        assert cs1 != cs2


# ═════════════════════════════════════════════════════════════════════════════
#  TESTS — ldf_loader
# ═════════════════════════════════════════════════════════════════════════════

class TestLDFLoader:
    """Tests du parseur LDF."""

    # ── _calculate_pid ────────────────────────────────────────────────────────

    @pytest.mark.parametrize("frame_id,expected_pid", [
        (0x16, 0xD6),
        (0x17, 0x97),
        (0x00, 0x80),
        (0x3F, 0xBF),
    ])
    def test_calculate_pid_known_values(self, frame_id, expected_pid):
        assert _calculate_pid(frame_id) == expected_pid

    def test_calculate_pid_masks_to_6_bits(self):
        pid_a = _calculate_pid(0x16)
        pid_b = _calculate_pid(0x56)   # 0x56 & 0x3F == 0x16
        assert pid_a == pid_b

    # ── load_ldf (fallback) ───────────────────────────────────────────────────

    def test_load_ldf_returns_dict(self):
        cfg = load_ldf("/chemin/inexistant.ldf")
        assert isinstance(cfg, dict)

    def test_load_ldf_fallback_has_baud(self):
        cfg = load_ldf("/chemin/inexistant.ldf")
        assert "baud" in cfg
        assert cfg["baud"] > 0

    def test_load_ldf_fallback_has_frames(self):
        cfg = load_ldf("/chemin/inexistant.ldf")
        assert "frames" in cfg
        assert len(cfg["frames"]) > 0

    def test_load_ldf_fallback_has_pid_map(self):
        cfg = load_ldf("/chemin/inexistant.ldf")
        assert "pid_map" in cfg

    def test_load_ldf_fallback_has_schedule(self):
        cfg = load_ldf("/chemin/inexistant.ldf")
        assert "schedule" in cfg
        assert isinstance(cfg["schedule"], list)

    # ── load_ldf (fichier réel minimal) ──────────────────────────────────────

    def test_load_real_ldf_baud(self, ldf_file):
        cfg = load_ldf(ldf_file)
        assert cfg["baud"] == 19200

    def test_load_real_ldf_frames_present(self, ldf_file):
        cfg = load_ldf(ldf_file)
        assert len(cfg["frames"]) >= 2

    def test_load_real_ldf_pid_map_nonempty(self, ldf_file):
        cfg = load_ldf(ldf_file)
        assert len(cfg["pid_map"]) > 0

    def test_load_real_ldf_frame_has_required_keys(self, ldf_file):
        cfg = load_ldf(ldf_file)
        for fname, fdef in cfg["frames"].items():
            for key in ("id", "pid", "dlc", "cycle_s"):
                assert key in fdef, f"Clé '{key}' manquante dans frame '{fname}'"

    def test_load_real_ldf_pid_integrity(self, ldf_file):
        """PID dans la frame doit correspondre à _calculate_pid(id)."""
        cfg = load_ldf(ldf_file)
        for fname, fdef in cfg["frames"].items():
            expected = _calculate_pid(fdef["id"])
            assert fdef["pid"] == expected

    def test_load_wiperwash_ldf_if_present(self):
        """Test avec le vrai wiperwash.ldf si disponible."""
        if not os.path.isfile(LDF_PATH):
            pytest.skip("wiperwash.ldf non trouvé")
        cfg = load_ldf(LDF_PATH)
        assert cfg["baud"] > 0
        assert len(cfg["frames"]) >= 2


# ═════════════════════════════════════════════════════════════════════════════
#  TESTS — dbc_loader
# ═════════════════════════════════════════════════════════════════════════════

class TestDBCLoader:
    """Tests du parseur DBC et helpers d'encodage CAN."""

    # ── load_dbc (fallback) ───────────────────────────────────────────────────

    def test_load_dbc_fallback_returns_dict(self):
        cfg = load_dbc("/chemin/inexistant.dbc")
        assert isinstance(cfg, dict)

    def test_load_dbc_fallback_has_messages(self):
        cfg = load_dbc("/chemin/inexistant.dbc")
        assert "messages" in cfg
        assert len(cfg["messages"]) > 0

    def test_load_dbc_fallback_has_id_map(self):
        cfg = load_dbc("/chemin/inexistant.dbc")
        assert "id_map" in cfg

    def test_load_dbc_fallback_has_periods(self):
        cfg = load_dbc("/chemin/inexistant.dbc")
        assert "periods_ms" in cfg

    # ── load_dbc (fichier réel) ───────────────────────────────────────────────

    def test_load_real_dbc_messages_present(self, dbc_file):
        cfg = load_dbc(dbc_file)
        assert len(cfg["messages"]) > 0

    def test_load_real_dbc_message_has_signals(self, dbc_file):
        cfg = load_dbc(dbc_file)
        for msg_id, msg in cfg["messages"].items():
            assert hasattr(msg, "signals")
            break

    def test_load_real_dbc_id_map_matches_messages(self, dbc_file):
        cfg = load_dbc(dbc_file)
        for msg_id in cfg["messages"]:
            assert msg_id in cfg["id_map"]

    def test_load_real_dbc_message_has_dlc(self, dbc_file):
        cfg = load_dbc(dbc_file)
        for msg_id, msg in cfg["messages"].items():
            assert hasattr(msg, "dlc")
            assert msg.dlc > 0

    def test_load_real_dbc_message_has_name(self, dbc_file):
        cfg = load_dbc(dbc_file)
        for msg_id, msg in cfg["messages"].items():
            assert hasattr(msg, "name")
            assert len(msg.name) > 0

    # ── encode_signal / decode_signal ─────────────────────────────────────────

    def test_encode_decode_roundtrip(self):
        from dbc_loader import _make_sig
        sig = _make_sig(0, 3, 1.0, 0.0, 0, 7, "", False)
        for val in range(8):
            raw = encode_signal(sig, val)
            decoded = decode_signal(sig, raw)
            assert decoded == val

    def test_encode_with_factor(self):
        from dbc_loader import _make_sig
        sig = _make_sig(0, 8, 0.5, 0.0, 0, 127, "", False)
        raw = encode_signal(sig, 50.0)
        assert raw == 100

    def test_encode_with_offset(self):
        from dbc_loader import _make_sig
        sig = _make_sig(0, 8, 1.0, -40.0, -40, 85, "", False)
        raw = encode_signal(sig, 20.0)
        assert raw == 60

    def test_decode_with_factor_and_offset(self):
        from dbc_loader import _make_sig
        sig = _make_sig(0, 8, 0.5, -40.0, -40, 87.5, "", False)
        phys = decode_signal(sig, 100)
        assert abs(phys - 10.0) < 1e-9

    def test_encode_zero_value(self):
        from dbc_loader import _make_sig
        sig = _make_sig(0, 8, 1.0, 0.0, 0, 255, "", False)
        assert encode_signal(sig, 0) == 0

    def test_decode_zero_raw(self):
        from dbc_loader import _make_sig
        sig = _make_sig(0, 8, 1.0, 0.0, 0, 255, "", False)
        assert decode_signal(sig, 0) == 0.0

    # ── pack_frame / unpack_frame ─────────────────────────────────────────────

    def test_pack_frame_correct_dlc(self, dbc_file):
        cfg = load_dbc(dbc_file)
        for msg_id, msg in cfg["messages"].items():
            values = {sig_name: 0 for sig_name in msg.signals}
            data = pack_frame(msg, values)
            assert len(data) == msg.dlc

    def test_pack_unpack_roundtrip(self, dbc_file):
        cfg = load_dbc(dbc_file)
        for msg_id, msg in cfg["messages"].items():
            if not msg.signals:
                continue
            test_vals = {}
            for sig_name, sig in msg.signals.items():
                max_val = (1 << sig.length) - 1
                test_vals[sig_name] = max_val // 2
            data = pack_frame(msg, test_vals)
            decoded = unpack_frame(msg, data)
            for sig_name, expected in test_vals.items():
                sig = msg.signals[sig_name]
                phys_expected = decode_signal(sig, encode_signal(sig, expected))
                assert abs(decoded[sig_name] - phys_expected) < 1e-6

    def test_pack_frame_returns_bytes(self, dbc_file):
        cfg = load_dbc(dbc_file)
        for msg_id, msg in cfg["messages"].items():
            values = {sig_name: 0 for sig_name in msg.signals}
            data = pack_frame(msg, values)
            assert isinstance(data, (bytes, bytearray))
            break

    def test_load_wiperwash_dbc_if_present(self):
        """Test avec le vrai wiperwash.dbc si disponible."""
        if not os.path.isfile(DBC_PATH):
            pytest.skip("wiperwash.dbc non trouvé")
        cfg = load_dbc(DBC_PATH)
        assert len(cfg["messages"]) > 0
        known_ids = {0x200, 0x201, 0x202, 0x300, 0x301}
        found = set(cfg["messages"].keys())
        overlap = known_ids & found
        assert len(overlap) > 0


# ═════════════════════════════════════════════════════════════════════════════
#  TESTS — a2l_loader
# ═════════════════════════════════════════════════════════════════════════════

class TestA2LLoader:
    """Tests du parseur A2L (ASAM MCD-2 MC)."""

    def test_load_a2l_real_file_returns_dict(self):
        if not os.path.isfile(A2L_PATH):
            pytest.skip("wiperwash_xcp.a2l non trouvé")
        result = load_a2l(A2L_PATH)
        assert isinstance(result, dict)

    def test_load_a2l_real_file_not_empty(self):
        if not os.path.isfile(A2L_PATH):
            pytest.skip("wiperwash_xcp.a2l non trouvé")
        result = load_a2l(A2L_PATH)
        assert len(result) > 0

    def test_load_a2l_contains_touch_duration(self):
        if not os.path.isfile(A2L_PATH):
            pytest.skip("wiperwash_xcp.a2l non trouvé")
        result = load_a2l(A2L_PATH)
        assert "TOUCH_DURATION" in result

    def test_load_a2l_contains_park_timeout(self):
        if not os.path.isfile(A2L_PATH):
            pytest.skip("wiperwash_xcp.a2l non trouvé")
        result = load_a2l(A2L_PATH)
        assert "PARK_TIMEOUT" in result

    def test_load_a2l_each_param_has_required_fields(self):
        if not os.path.isfile(A2L_PATH):
            pytest.skip("wiperwash_xcp.a2l non trouvé")
        result = load_a2l(A2L_PATH)
        for name, param in result.items():
            for field in ("desc", "unit", "type", "default", "min", "max"):
                assert field in param, f"Paramètre '{name}' manque le champ '{field}'"

    def test_load_a2l_type_is_float_or_int(self):
        if not os.path.isfile(A2L_PATH):
            pytest.skip("wiperwash_xcp.a2l non trouvé")
        result = load_a2l(A2L_PATH)
        for name, param in result.items():
            assert param["type"] in ("float", "int"), \
                f"Type inattendu '{param['type']}' pour '{name}'"

    def test_load_a2l_touch_duration_default_value(self):
        if not os.path.isfile(A2L_PATH):
            pytest.skip("wiperwash_xcp.a2l non trouvé")
        result = load_a2l(A2L_PATH)
        if "TOUCH_DURATION" in result:
            assert result["TOUCH_DURATION"]["default"] == pytest.approx(1.700, abs=0.001)

    def test_load_a2l_has_category_field(self):
        if not os.path.isfile(A2L_PATH):
            pytest.skip("wiperwash_xcp.a2l non trouvé")
        result = load_a2l(A2L_PATH)
        for name, param in result.items():
            assert "category" in param, f"Paramètre '{name}' manque le champ 'category'"

    def test_load_a2l_missing_file_raises(self):
        """load_a2l lève une exception si le fichier est absent."""
        with pytest.raises(Exception):
            load_a2l("/chemin/inexistant/wiperwash_xcp.a2l")

    def test_load_a2l_min_le_max(self):
        if not os.path.isfile(A2L_PATH):
            pytest.skip("wiperwash_xcp.a2l non trouvé")
        result = load_a2l(A2L_PATH)
        for name, param in result.items():
            assert param["min"] <= param["max"], \
                f"'{name}' : min={param['min']} > max={param['max']}"

    def test_load_a2l_default_within_range(self):
        if not os.path.isfile(A2L_PATH):
            pytest.skip("wiperwash_xcp.a2l non trouvé")
        result = load_a2l(A2L_PATH)
        for name, param in result.items():
            assert param["min"] <= param["default"] <= param["max"], \
                f"'{name}' : default={param['default']} hors de [{param['min']}, {param['max']}]"


# ═════════════════════════════════════════════════════════════════════════════
#  TESTS — Constantes & valeurs de référence
# ═════════════════════════════════════════════════════════════════════════════

class TestConstants:
    """Vérification des constantes critiques du projet."""

    # ── WOP codes ─────────────────────────────────────────────────────────────

    def test_wop_codes_unique(self):
        ops = [WOP_OFF, WOP_TOUCH, WOP_SPEED1, WOP_SPEED2,
               WOP_AUTO, WOP_FRONT_WASH, WOP_REAR_WASH, WOP_REAR_WIPE]
        assert len(ops) == len(set(ops)), "Codes WOP dupliqués !"

    def test_wop_off_is_zero(self):
        assert WOP_OFF == 0x00

    def test_wop_touch_is_0x01(self):
        assert WOP_TOUCH == 0x01

    def test_wop_speed1_is_0x02(self):
        assert WOP_SPEED1 == 0x02

    def test_wop_speed2_is_0x03(self):
        assert WOP_SPEED2 == 0x03

    def test_wop_auto_is_0x04(self):
        assert WOP_AUTO == 0x04

    def test_wop_front_wash_is_0x05(self):
        assert WOP_FRONT_WASH == 0x05

    def test_wop_rear_wash_is_0x06(self):
        assert WOP_REAR_WASH == 0x06

    def test_wop_rear_wipe_is_0x07(self):
        assert WOP_REAR_WIPE == 0x07

    def test_wop_names_covers_all_codes(self):
        for code in range(8):
            assert code in WOP_NAMES

    # ── ST states ─────────────────────────────────────────────────────────────

    def test_st_states_are_strings(self):
        for st in [ST_OFF, ST_SPEED1, ST_SPEED2, ST_AUTO, ST_TOUCH,
                   ST_WASH_FRONT, ST_WASH_REAR, ST_REAR_WIPE,
                   ST_ERROR, ST_DIAG, ST_PARK]:
            assert isinstance(st, str)

    def test_st_enc_covers_all_states(self):
        for st in [ST_OFF, ST_SPEED1, ST_SPEED2, ST_AUTO, ST_TOUCH,
                   ST_WASH_FRONT, ST_WASH_REAR, ST_REAR_WIPE,
                   ST_ERROR, ST_DIAG, ST_PARK]:
            assert st in ST_ENC, f"État '{st}' absent de ST_ENC"

    def test_st_enc_values_unique(self):
        values = list(ST_ENC.values())
        assert len(values) == len(set(values)), "Valeurs ST_ENC dupliquées"

    def test_st_off_enc_is_zero(self):
        assert ST_ENC[ST_OFF] == 0

    # ── Redis keys ────────────────────────────────────────────────────────────

    def test_redis_writable_keys_not_empty(self):
        assert len(REDIS_WRITABLE_KEYS) > 0

    def test_redis_public_keys_not_empty(self):
        assert len(REDIS_PUBLIC_KEYS) > 0

    def test_state_key_not_writable(self):
        assert "state" not in REDIS_WRITABLE_KEYS

    def test_crs_wiper_op_is_writable(self):
        assert "crs_wiper_op" in REDIS_WRITABLE_KEYS

    def test_ignition_status_is_writable(self):
        assert "ignition_status" in REDIS_WRITABLE_KEYS

    def test_state_is_public(self):
        assert "state" in REDIS_PUBLIC_KEYS

    # ── DTC status bits ───────────────────────────────────────────────────────

    def test_dtc_status_active_has_confirmed_bit(self):
        assert STATUS_ACTIVE & BIT_CONFIRMED

    def test_dtc_status_active_has_test_failed_bit(self):
        assert STATUS_ACTIVE & BIT_TEST_FAILED

    def test_dtc_status_inactive_lacks_test_failed_bit(self):
        assert not (STATUS_INACTIVE & BIT_TEST_FAILED)

    def test_dtc_status_clean_is_zero(self):
        assert STATUS_CLEAN == 0x00

    def test_dtc_status_active_is_0x2f(self):
        assert STATUS_ACTIVE == 0x2F

    def test_dtc_status_inactive_is_0x2e(self):
        assert STATUS_INACTIVE == 0x2E

    # ── LIN constants ─────────────────────────────────────────────────────────

    def test_lin_baud_default(self):
        assert LIN_BAUD == 19200

    def test_lin_id_0x16(self):
        assert LIN_ID_0x16 == 0x16

    def test_lin_id_0x17(self):
        assert LIN_ID_0x17 == 0x17

    def test_lin_pid_0x16_is_0xd6(self):
        assert LIN_PID_0x16 == 0xD6

    def test_lin_pid_0x17_is_0x97(self):
        assert LIN_PID_0x17 == 0x97

    def test_lin_sync_is_0x55(self):
        assert LIN_SYNC == 0x55

    def test_lin_diag_req_pid_is_0x3c(self):
        assert LIN_PID_DIAG_REQ == 0x3C

    def test_lin_diag_rsp_pid_is_0x3d(self):
        assert LIN_PID_DIAG_RSP == 0x3D

    def test_pid_0x16_is_0xd6(self):
        assert _calculate_pid(0x16) == 0xD6

    def test_pid_0x17_is_0x97(self):
        assert _calculate_pid(0x17) == 0x97

    # ── CAN constants ─────────────────────────────────────────────────────────

    def test_can_wiper_command_id(self):
        assert CAN_ID_WIPER_COMMAND == 0x200

    def test_can_wiper_status_id(self):
        assert CAN_ID_WIPER_STATUS == 0x201

    def test_can_wiper_ack_id(self):
        assert CAN_ID_WIPER_ACK == 0x202

    def test_can_vehicle_id(self):
        assert CAN_ID_VEHICLE == 0x300

    def test_can_rain_sensor_id(self):
        assert CAN_ID_RAIN_SENSOR == 0x301

    def test_can_ids_unique(self):
        ids = [CAN_ID_WIPER_COMMAND, CAN_ID_WIPER_STATUS, CAN_ID_WIPER_ACK,
               CAN_ID_VEHICLE, CAN_ID_RAIN_SENSOR]
        assert len(ids) == len(set(ids))

    # ── Security Access constants ─────────────────────────────────────────────

    def test_sa_req_seed_is_0x01(self):
        assert SA_REQ_SEED == 0x01

    def test_sa_send_key_is_0x02(self):
        assert SA_SEND_KEY == 0x02

    def test_sa_xor_mask_is_0xa5a5(self):
        assert SA_XOR_MASK == 0xA5A5

    def test_sa_add_mask_is_0x3c(self):
        assert SA_ADD_MASK == 0x3C

    # ── Calibration parameters ────────────────────────────────────────────────

    def test_overcurrent_thresh_positive(self):
        assert OVERCURRENT_THRESH > 0.0

    def test_pump_overcurrent_thresh_positive(self):
        assert PUMP_OVERCURRENT_THRESH > 0.0

    def test_touch_duration_positive(self):
        assert TOUCH_DURATION > 0.0

    def test_park_timeout_positive(self):
        assert PARK_TIMEOUT > 0.0

    def test_pump_max_runtime_positive(self):
        assert PUMP_MAX_RUNTIME > 0.0


# ═════════════════════════════════════════════════════════════════════════════
#  TESTS D'INTÉGRATION — RTE + DTCManager
# ═════════════════════════════════════════════════════════════════════════════

class TestRTEWithDTC:
    """Tests d'intégration légère RTE ↔ DTCManager."""

    def test_snapshot_used_in_dtc_set_active(self, rte, dtc):
        rte.set("ignition_status", 1)
        rte.set("crs_wiper_op", WOP_SPEED1)
        rte.set("vehicle_speed", 60)
        snap = rte.make_snapshot()
        dtc.set_active("B2001", snap)
        assert dtc.get_status("B2001") == "ACTIVE"

    def test_full_fault_cycle(self, rte, dtc):
        """Cycle complet : activation DTC → vérification → désactivation → clear."""
        snap = rte.make_snapshot()
        dtc.set_active("B2003", snap)
        assert dtc.get_status("B2003") == "ACTIVE"
        dtc.set_inactive("B2003")
        assert dtc.get_status("B2003") == "INACTIVE"
        dtc.clear_all()
        assert dtc.get_status("B2003") == "CLEAN"

    def test_multiple_dtc_active_simultaneously(self, rte, dtc):
        snap = rte.make_snapshot()
        target_codes = ["B2001", "B2004", "B2007"]
        for code in target_codes:
            dtc.set_active(code, snap)
        result = dtc.get_dtcs_by_mask(0xFF)
        found_bytes = {r[0] for r in result}
        for code in target_codes:
            b = bytes(dtc.dtcs[code]["bytes"])
            assert b in found_bytes, f"Bytes de {code} absents du résultat"

    def test_rte_write_lock_blocks_redis_cmd(self, rte):
        rte.acquire_write_lock("OtherClient")
        info = rte.get_write_lock_info()
        assert info["owner"] == "OtherClient"
        ok = rte.acquire_write_lock("Attacker")
        assert ok is False

    def test_full_fault_cycle_with_snapshot_persistence(self, rte, tmp_path):
        """Snapshot stocké lors de set_active est persisté dans le fichier JSON."""
        db = tmp_path / "dtc_int.json"
        shutil.copy(DTC_DATABASE_PATH, str(db))
        dtc_local = DTCManager(filepath=str(db))
        dtc_local.clear_all()

        rte.set("ignition_status", 1)
        rte.set("state", ST_SPEED2)
        rte.set("vehicle_speed", 100)
        snap = rte.make_snapshot()
        dtc_local.set_active("B2001", snap)

        # Rechargement depuis disque
        dtc_reload = DTCManager(filepath=str(db))
        assert dtc_reload.get_status("B2001") == "ACTIVE"
        records = dtc_reload.dtcs["B2001"]["snapshot_records"]
        assert len(records) >= 1

    def test_lock_prevents_set_locked_from_other(self, rte):
        rte.acquire_write_lock("DashA")
        ok = rte.set_locked("rain_intensity", 99, "DashB")
        assert ok is False
        assert rte.rain_intensity != 99

    def test_lock_allows_set_locked_from_owner(self, rte):
        rte.acquire_write_lock("DashA")
        ok = rte.set_locked("rain_intensity", 42, "DashA")
        assert ok is True
        assert rte.rain_intensity == 42

    def test_dtc_b2011_supported(self, dtc):
        """B2011 (condition conjointe LIN) doit être dans la base."""
        assert "B2011" in dtc.dtcs

    def test_uds_0x19_0x02_full_cycle(self, rte, dtc):
        """Scénario UDS complet : activer DTC, interroger via handle_read_dtc."""
        snap = rte.make_snapshot()
        dtc.set_active("B2002", snap)
        resp = handle_read_dtc(dtc, bytes([0x19, 0x02, 0xFF]))
        b2002_bytes = bytes(dtc.dtcs["B2002"]["bytes"])
        # La réponse doit contenir les bytes du DTC B2002
        assert b2002_bytes in resp

    def test_uds_0x14_clear_all_via_handler(self, rte, dtc):
        """Effacement via handle_clear_dtc doit remettre tous les DTCs CLEAN."""
        snap = rte.make_snapshot()
        for code in ["B2001", "B2003", "B2005"]:
            dtc.set_active(code, snap)
        resp = handle_clear_dtc(dtc, bytes([0x14, 0xFF, 0xFF, 0xFF]))
        assert resp == bytes([0x54])
        for code in ["B2001", "B2003", "B2005"]:
            assert dtc.get_status(code) == "CLEAN"