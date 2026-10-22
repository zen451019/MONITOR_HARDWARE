#!/usr/bin/env python3
"""
Sparkplug B test consumer / basic compliance checker.

Uso basico:
    python sparkplug_decode.py --host 127.0.0.1

Rebirth conforme (solo alias, QoS 0, sin seq):
    python sparkplug_decode.py --host 127.0.0.1 \
        --send-rebirth --group EMASA --node nemo_medidor1

Rebirth por nombre (solo para pruebas; un host conforme usa alias):
    python sparkplug_decode.py --host 127.0.0.1 \
        --send-rebirth --rebirth-by-name --group EMASA --node nemo_medidor1

Requiere:
    pip install -r requirements.txt
"""

import argparse
import os
import sys
import time
from datetime import datetime, timezone

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "python"))
import sparkplug_b_pb2  # noqa: E402

try:
    import paho.mqtt.client as mqtt
except ImportError as exc:  # pragma: no cover
    raise SystemExit("Falta paho-mqtt. Ejecuta: pip install -r requirements.txt") from exc


DT = sparkplug_b_pb2.DataType
REBIRTH_ALIAS = 6

ALIASES = {}   # key -> {alias: name}
TYPES = {}     # key -> {alias: datatype}
SEQ = {}       # node_key -> ultimo seq
BDSEQ = {}     # node_key -> bdSeq del ultimo NBIRTH


def warn(msg):
    print(f"[WARN] {msg}")


def key_from_topic(topic):
    parts = topic.split("/")
    if len(parts) < 4 or parts[0] != "spBv1.0":
        return None, None, None, None
    group, msg_type, node = parts[1], parts[2], parts[3]
    device = parts[4] if len(parts) >= 5 else None
    key = f"{group}/{node}" + (f"/{device}" if device else "")
    node_key = f"{group}/{node}"
    return msg_type, key, node_key, device


def dt_name(dt):
    try:
        return DT.Name(dt)
    except Exception:
        return f"DataType({dt})"


def iso(ts_ms):
    if not ts_ms:
        return "-"
    try:
        return datetime.fromtimestamp(ts_ms / 1000.0, tz=timezone.utc).isoformat()
    except Exception:
        return str(ts_ms)


def metric_datatype(key, m):
    if m.HasField("datatype"):
        return m.datatype
    if m.HasField("alias"):
        return TYPES.get(key, {}).get(m.alias, 0)
    return 0


def metric_name(key, m):
    if m.HasField("name"):
        return m.name
    if m.HasField("alias"):
        return ALIASES.get(key, {}).get(m.alias, f"alias:{m.alias}")
    return "?"


def metric_value(key, m):
    if m.HasField("is_null") and m.is_null:
        return "null"

    which = m.WhichOneof("value")
    if which is None:
        return "-"

    dt = metric_datatype(key, m)

    if which == "int_value":
        v = m.int_value
        if dt == DT.Int8 and v >= 0x80:
            v -= 0x100
        elif dt == DT.Int16 and v >= 0x8000:
            v -= 0x10000
        elif dt == DT.Int32 and v >= 0x80000000:
            v -= 0x100000000
        return v

    if which == "long_value":
        v = m.long_value
        if dt == DT.Int64 and v >= 0x8000000000000000:
            v -= 0x10000000000000000
        return v

    return getattr(m, which)


def check_seq(node_key, msg_type, payload):
    if msg_type == "NBIRTH":
        if not payload.HasField("seq"):
            warn(f"{node_key} NBIRTH sin seq")
            SEQ[node_key] = 0
        else:
            if payload.seq != 0:
                warn(f"{node_key} NBIRTH seq={payload.seq}; debe ser 0")
            SEQ[node_key] = payload.seq
        return

    if msg_type == "NDEATH":
        if payload.HasField("seq"):
            warn(f"{node_key} NDEATH no debe incluir seq")
        return

    if msg_type in ("DBIRTH", "NDATA", "DDATA", "DDEATH"):
        if not payload.HasField("seq"):
            warn(f"{node_key} {msg_type} sin seq")
            return
        expected = (SEQ.get(node_key, -1) + 1) & 0xFF
        if payload.seq != expected:
            warn(f"{node_key} {msg_type} seq={payload.seq}; esperado={expected}")
        SEQ[node_key] = payload.seq


def check_timestamp(key, m, msg_type):
    if msg_type in ("NBIRTH", "DBIRTH", "NDATA", "DDATA"):
        if not m.HasField("timestamp"):
            warn(f"{key}: metrica '{metric_name(key, m)}' sin timestamp")


def validate_birth(key, payload, msg_type):
    for m in payload.metrics:
        if not m.HasField("name"):
            warn(f"{key} {msg_type}: metrica sin name")
        if not m.HasField("alias"):
            warn(f"{key} {msg_type}: metrica '{metric_name(key, m)}' sin alias")
        if not m.HasField("datatype"):
            warn(f"{key} {msg_type}: metrica '{metric_name(key, m)}' sin datatype")
        check_timestamp(key, m, msg_type)

    if msg_type == "NBIRTH":
        rebirth = next((m for m in payload.metrics
                        if m.HasField("name") and m.name == "Node Control/Rebirth"), None)
        if rebirth is None:
            warn(f"{key} NBIRTH sin Node Control/Rebirth")
        elif metric_value(key, rebirth) is not False:
            warn(f"{key} NBIRTH Node Control/Rebirth debe ser false")


def validate_data(key, payload, msg_type):
    for m in payload.metrics:
        if m.HasField("name"):
            warn(f"{key} {msg_type}: DATA no debe incluir name ('{m.name}')")
        if not m.HasField("alias"):
            warn(f"{key} {msg_type}: metrica sin alias")
        if m.HasField("datatype"):
            warn(f"{key} {msg_type}: DATA no deberia incluir datatype")
        check_timestamp(key, m, msg_type)


def store_birth(key, node_key, payload, msg_type, qos, retain):
    ALIASES[key] = {}
    TYPES[key] = {}

    print(f"\n{key}  {msg_type}  seq={payload.seq if payload.HasField('seq') else '-'}  "
          f"ts={iso(payload.timestamp if payload.HasField('timestamp') else 0)}  "
          f"qos={qos} retain={retain}")

    validate_birth(key, payload, msg_type)

    for m in payload.metrics:
        name = metric_name(key, m)
        if m.HasField("alias"):
            ALIASES[key][m.alias] = name
            if m.HasField("datatype"):
                TYPES[key][m.alias] = m.datatype
        print(f"  + {name:24s} alias={m.alias if m.HasField('alias') else '-':>4} "
              f"type={dt_name(metric_datatype(key, m)):10s} "
              f"value={metric_value(key, m)}")

    if msg_type == "NBIRTH":
        bd = next((m for m in payload.metrics
                   if (m.HasField("name") and m.name == "bdSeq") or
                      (m.HasField("alias") and m.alias == 5)), None)
        if bd is None:
            warn(f"{node_key} NBIRTH sin bdSeq")
        else:
            BDSEQ[node_key] = metric_value(key, bd)


def print_data(key, payload, msg_type, qos, retain):
    print(f"\n{key}  {msg_type}  seq={payload.seq if payload.HasField('seq') else '-'}  "
          f"ts={iso(payload.timestamp if payload.HasField('timestamp') else 0)}  "
          f"qos={qos} retain={retain}")

    validate_data(key, payload, msg_type)

    for m in payload.metrics:
        name = metric_name(key, m)
        if m.HasField("alias") and m.alias not in ALIASES.get(key, {}):
            warn(f"{key}: alias {m.alias} sin BIRTH previo")
        print(f"  - {name:24s} alias={m.alias if m.HasField('alias') else '-':>4} "
              f"type={dt_name(metric_datatype(key, m)):10s} "
              f"value={metric_value(key, m)}")


def on_connect(client, userdata, flags, rc):
    print(f"Conectado al broker (rc={rc}). Suscribiendo spBv1.0/# ...")
    client.subscribe("spBv1.0/#", qos=1)


def on_message(client, userdata, msg):
    msg_type, key, node_key, device = key_from_topic(msg.topic)
    if key is None:
        return

    payload = sparkplug_b_pb2.Payload()
    try:
        payload.ParseFromString(msg.payload)
    except Exception as exc:
        print(f"\n{msg.topic}  payload no decodificable ({len(msg.payload)} bytes): {exc}")
        return

    check_seq(node_key, msg_type, payload)

    if msg_type in ("NBIRTH", "DBIRTH"):
        store_birth(key, node_key, payload, msg_type, msg.qos, msg.retain)
    elif msg_type in ("NDATA", "DDATA"):
        print_data(key, payload, msg_type, msg.qos, msg.retain)
    elif msg_type == "NDEATH":
        bd = next((m for m in payload.metrics
                   if (m.HasField("name") and m.name == "bdSeq") or
                      (m.HasField("alias") and m.alias == 5)), None)
        bd_value = metric_value(node_key, bd) if bd is not None else "-"
        if bd is None:
            warn(f"{node_key} NDEATH sin bdSeq")
        if bd_value != "-" and node_key in BDSEQ and bd_value != BDSEQ[node_key]:
            warn(f"{node_key} NDEATH bdSeq={bd_value}; ultimo NBIRTH={BDSEQ[node_key]}")
        print(f"\n{key}  NDEATH  bdSeq={bd_value}  "
              f"ts={iso(payload.timestamp if payload.HasField('timestamp') else 0)}  "
              f"qos={msg.qos} retain={msg.retain}")
    elif msg_type == "DDEATH":
        if len(payload.metrics) != 0:
            warn(f"{key} DDEATH no debe incluir metricas")
        print(f"\n{key}  DDEATH  "
              f"ts={iso(payload.timestamp if payload.HasField('timestamp') else 0)}  "
              f"qos={msg.qos} retain={msg.retain}")
    else:
        print(f"\n{msg.topic}  tipo={msg_type}  qos={msg.qos} retain={msg.retain}  "
              f"{len(msg.payload)} bytes")


def send_rebirth(args):
    client = make_client(args)
    client.connect(args.host, args.port, keepalive=15)
    client.loop_start()
    time.sleep(1.0)

    payload = sparkplug_b_pb2.Payload()
    payload.timestamp = int(time.time() * 1000)

    m = payload.metrics.add()
    if args.rebirth_by_name:
        m.name = "Node Control/Rebirth"
    else:
        m.alias = REBIRTH_ALIAS
    m.boolean_value = True

    topic = f"spBv1.0/{args.group}/NCMD/{args.node}"
    data = payload.SerializeToString()
    info = client.publish(topic, data, qos=0, retain=False)
    info.wait_for_publish()
    print(f"Rebirth enviado a {topic} ({len(data)} bytes, qos=0, seq=no)")
    time.sleep(1.0)
    client.loop_stop()
    client.disconnect()


def make_client(args):
    try:
        client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION1)
    except (AttributeError, TypeError):
        client = mqtt.Client()

    if args.username:
        client.username_pw_set(args.username, args.password)
    client.on_connect = on_connect
    client.on_message = on_message
    return client


def main():
    parser = argparse.ArgumentParser(description="Consumidor / verificador Sparkplug B")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=1883)
    parser.add_argument("--username", default="")
    parser.add_argument("--password", default="")
    parser.add_argument("--send-rebirth", action="store_true",
                        help="Publica NCMD Node Control/Rebirth y termina")
    parser.add_argument("--rebirth-by-name", action="store_true",
                        help="Envia Rebirth por name (no conforme; solo pruebas)")
    parser.add_argument("--group", default="EMASA")
    parser.add_argument("--node", default="nemo_medidor1")
    args = parser.parse_args()

    if args.send_rebirth:
        send_rebirth(args)
        return

    client = make_client(args)
    client.connect(args.host, args.port, keepalive=15)
    print(f"Esperando mensajes en spBv1.0/# desde {args.host}:{args.port} ...")
    client.loop_forever()


if __name__ == "__main__":
    main()
