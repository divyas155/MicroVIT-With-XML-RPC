"""
RobotOps_Streamlit_UI – AI message monitoring and troubleshooting for multi-robot system.
Run: streamlit run app.py (from RobotOps_Streamlit_UI directory)
"""
from __future__ import annotations

import json
import random
import time
from datetime import datetime
from typing import List, Optional

import pandas as pd
import plotly.graph_objects as go
import streamlit as st

from models import Message, SourceType
from simulator import run_simulator_step
from alerts import run_alert_rules, get_robot_health

try:
    from mqtt_bridge import MQTTBridge
except ImportError:
    MQTTBridge = None

# Page config – dark, wide
st.set_page_config(
    page_title="RobotOps – AI Message Monitor",
    page_icon="🤖",
    layout="wide",
    initial_sidebar_state="expanded",
)

# Attractive dark command-center theme
st.markdown("""
<style>
    /* Base */
    .stApp { background: linear-gradient(180deg, #0c0e14 0%, #13161f 50%, #0c0e14 100%); min-height: 100vh; }
    .main .block-container { padding: 2rem 2.5rem; max-width: 1500px; }
    h1, h2, h3 { color: #f0f4ff; font-weight: 600; letter-spacing: -0.02em; }
    .stCaption { color: #94a3b8; }
    
    /* Hero header */
    .hero {
        background: linear-gradient(135deg, rgba(59, 130, 246, 0.12) 0%, rgba(139, 92, 246, 0.08) 100%);
        border: 1px solid rgba(99, 102, 241, 0.25);
        border-radius: 16px;
        padding: 1.5rem 2rem;
        margin-bottom: 2rem;
        text-align: center;
    }
    .hero h1 { margin: 0; font-size: 1.85rem; background: linear-gradient(90deg, #93c5fd, #c4b5fd); -webkit-background-clip: text; -webkit-text-fill-color: transparent; background-clip: text; }
    .hero p { margin: 0.35rem 0 0 0; color: #94a3b8; font-size: 0.95rem; }
    
    /* Status tiles – glass cards */
    .tile {
        background: linear-gradient(160deg, rgba(30, 33, 45, 0.95) 0%, rgba(22, 25, 35, 0.98) 100%);
        border: 1px solid rgba(71, 85, 105, 0.4);
        border-radius: 16px;
        padding: 1.5rem;
        margin: 0.5rem 0;
        color: #cbd5e1;
        box-shadow: 0 4px 24px rgba(0,0,0,0.25);
        transition: transform 0.2s, box-shadow 0.2s;
    }
    .tile:hover { transform: translateY(-2px); box-shadow: 0 8px 32px rgba(59, 130, 246, 0.08); }
    .tile h4 { margin: 0 0 0.75rem 0; color: #f1f5f9; font-size: 1.15rem; font-weight: 600; display: flex; align-items: center; gap: 0.5rem; }
    .tile .stat-row { font-size: 0.9rem; margin: 0.4rem 0; }
    .status-ok { color: #4ade80; font-weight: 600; }
    .status-ok::before { content: "● "; }
    .status-warn { color: #fbbf24; font-weight: 600; }
    .status-warn::before { content: "● "; }
    .status-down { color: #f87171; font-weight: 600; }
    .status-down::before { content: "● "; }
    .severity-info { color: #60a5fa; }
    .severity-warn { color: #fbbf24; }
    .severity-critical { color: #f87171; }
    
    /* Severity pills */
    .pill { display: inline-block; padding: 0.2rem 0.6rem; border-radius: 9999px; font-size: 0.75rem; font-weight: 600; }
    .pill-info { background: rgba(59, 130, 246, 0.2); color: #60a5fa; }
    .pill-warn { background: rgba(251, 191, 36, 0.2); color: #fbbf24; }
    .pill-critical { background: rgba(248, 113, 113, 0.2); color: #f87171; }
    
    /* Section cards */
    .card {
        background: rgba(30, 33, 45, 0.6);
        border: 1px solid rgba(71, 85, 105, 0.35);
        border-radius: 12px;
        padding: 1.25rem;
        margin: 1rem 0;
    }
    .card h3 { margin: 0 0 0.75rem 0; font-size: 1.05rem; color: #e2e8f0; }
    
    /* Alerts */
    .alert-box {
        background: linear-gradient(135deg, rgba(251, 191, 36, 0.08) 0%, rgba(248, 113, 113, 0.06) 100%);
        border: 1px solid rgba(251, 191, 36, 0.3);
        border-radius: 12px;
        padding: 1rem 1.25rem;
        margin: 0.75rem 0;
    }
    .alert-box strong { color: #fbbf24; }
    
    /* Buttons area */
    div[data-testid="column"] button { border-radius: 10px; font-weight: 500; transition: all 0.2s; }
    div[data-testid="column"] button:hover { transform: translateY(-1px); }
    
    /* Sidebar */
    [data-testid="stSidebar"] { background: linear-gradient(180deg, #0f1117 0%, #13161f 100%); }
    [data-testid="stSidebar"] .stRadio label { color: #cbd5e1 !important; }
    [data-testid="stSidebar"] .stRadio label div { font-weight: 500; }
    
    /* DataFrame */
    div[data-testid="stDataFrame"] { background: rgba(30, 33, 45, 0.7); border-radius: 12px; border: 1px solid rgba(71, 85, 105, 0.3); overflow: hidden; }
    .stSelectbox label, .stMultiSelect label { color: #e2e8f0 !important; }
    .stExpander { border: 1px solid rgba(71, 85, 105, 0.35); border-radius: 10px; }
</style>
""", unsafe_allow_html=True)


def init_session_state():
    if "messages" not in st.session_state:
        st.session_state.messages = []
    if "alerts" not in st.session_state:
        st.session_state.alerts = []
    if "simulate_robot1_down" not in st.session_state:
        st.session_state.simulate_robot1_down = False
    if "simulate_lidar_stall" not in st.session_state:
        st.session_state.simulate_lidar_stall = False
    if "simulate_critical_burst" not in st.session_state:
        st.session_state.simulate_critical_burst = False
    if "simulator_running" not in st.session_state:
        st.session_state.simulator_running = False
    if "last_sim_time" not in st.session_state:
        st.session_state.last_sim_time = 0.0
    if "use_live_mqtt" not in st.session_state:
        st.session_state.use_live_mqtt = False
    if "mqtt_broker_host" not in st.session_state:
        st.session_state.mqtt_broker_host = "10.13.68.48"
    if "mqtt_broker_port" not in st.session_state:
        st.session_state.mqtt_broker_port = 1883
    if "mqtt_bridge" not in st.session_state:
        st.session_state.mqtt_bridge = None


def severity_color(s: str) -> str:
    if s == "critical":
        return "severity-critical"
    if s == "warn":
        return "severity-warn"
    return "severity-info"


# ---------- Dashboard Page ----------
def render_dashboard():
    init_session_state()
    now = datetime.now()
    health = get_robot_health(st.session_state.messages, now)
    st.session_state.alerts = run_alert_rules(st.session_state.messages, now)

    # Hero header
    st.markdown("""
    <div class="hero">
        <h1>🤖 RobotOps Command Center</h1>
        <p>Unified view: Robot1 (Orin + Nano) · Controller · Helper Robot</p>
    </div>
    """, unsafe_allow_html=True)

    # Tiles: Robot1, Controller, Helper Robot (with icons)
    icons = {"Robot1": "🦾", "Controller": "🎛️", "Helper Robot": "🤝"}
    col1, col2, col3 = st.columns(3)
    for col, (name, key) in zip(
        [col1, col2, col3],
        [("Robot1", "robot1"), ("Controller", "controller"), ("Helper Robot", "helper")],
    ):
        with col:
            h = health.get(key, {})
            status = h.get("status", "DOWN")
            status_class = "status-ok" if status == "OK" else ("status-warn" if status == "WARN" else "status-down")
            last_seen = h.get("last_seen")
            ts_str = last_seen.strftime("%H:%M:%S") if last_seen else "—"
            msg_preview = (h.get("last_message") or "—")[:55] + ("…" if len(h.get("last_message") or "") > 55 else "")
            sev = h.get("severity_badge", "info")
            pill_class = "pill-info" if sev == "info" else ("pill-warn" if sev == "warn" else "pill-critical")
            st.markdown(f"""
            <div class="tile">
                <h4>{icons.get(name, "•")} {name}</h4>
                <p class="stat-row"><span class="{status_class}">Status: {status}</span></p>
                <p class="stat-row">Last message: <span style="color:#94a3b8;">{msg_preview}</span></p>
                <p class="stat-row">Last seen: <strong>{ts_str}</strong></p>
                <p class="stat-row"><span class="pill {pill_class}">{sev.upper()}</span></p>
            </div>
            """, unsafe_allow_html=True)

    st.markdown("")
    b1, b2, b3 = st.columns(3)
    with b1:
        if st.button("📡 View Live Feed", use_container_width=True, type="primary"):
            st.session_state.page = "live_feed"
            st.rerun()
    with b2:
        if st.button("⚠️ Show Critical Only", use_container_width=True):
            st.session_state.filter_critical_only = True
            st.session_state.page = "live_feed"
            st.rerun()
    with b3:
        if st.button("🔴 Simulate Failure", use_container_width=True):
            st.session_state.simulate_robot1_down = True
            st.session_state.simulator_running = True
            st.rerun()

    # Alerts in styled boxes
    if st.session_state.alerts:
        st.markdown("### ⚠️ Active Alerts")
        for a in st.session_state.alerts:
            st.markdown(f"""
            <div class="alert-box">
                <strong>{a.rule}</strong> ({a.source}) — {a.meaning}
            </div>
            """, unsafe_allow_html=True)

    # Chart: message count by source (last 50)
    recent = st.session_state.messages[-50:]
    if recent:
        st.markdown("### 📊 Message activity (last 50)")
        counts = pd.Series([m.source for m in recent]).value_counts()
        colors = ["#60a5fa", "#4ade80", "#fbbf24"]
        fig = go.Figure(data=[go.Bar(x=counts.index, y=counts.values, marker_color=colors[: len(counts)], text=counts.values, textposition="auto")])
        fig.update_layout(
            template="plotly_dark",
            paper_bgcolor="rgba(0,0,0,0)",
            plot_bgcolor="rgba(30,33,45,0.6)",
            margin=dict(t=30, b=30, l=40, r=20),
            height=240,
            font=dict(color="#e2e8f0", size=12),
            xaxis=dict(showgrid=False),
            yaxis=dict(showgrid=True, gridcolor="rgba(71,85,105,0.3)"),
            showlegend=False,
        )
        st.plotly_chart(fig, use_container_width=True)


# ---------- Live AI Message Feed ----------
def render_live_feed():
    init_session_state()
    st.markdown("""
    <div class="hero">
        <h1>📡 Live AI Message Feed</h1>
        <p>Auto-refresh every 2 seconds · Filter by source, severity, or search</p>
    </div>
    """, unsafe_allow_html=True)

    # Filters
    col_f1, col_f2, col_f3 = st.columns(3)
    with col_f1:
        source_filter = st.multiselect(
            "Source",
            options=["robot1", "controller", "helper"],
            default=["robot1", "controller", "helper"],
            key="feed_source_filter",
        )
    with col_f2:
        severity_filter = st.multiselect(
            "Severity",
            options=["info", "warn", "critical"],
            default=["info", "warn", "critical"],
            key="feed_severity_filter",
        )
    with col_f3:
        search = st.text_input("Search message text", key="feed_search", placeholder="e.g. obstacle")

    msgs = st.session_state.messages
    if source_filter:
        msgs = [m for m in msgs if m.source in source_filter]
    if severity_filter:
        msgs = [m for m in msgs if m.severity in severity_filter]
    if search:
        msgs = [m for m in msgs if search.lower() in m.text.lower()]
    if getattr(st.session_state, "filter_critical_only", False):
        msgs = [m for m in msgs if m.severity == "critical"]
        st.session_state.filter_critical_only = False

    if not msgs:
        st.info("No messages yet. Start the **Simulator** (sidebar) or connect to live MQTT.")
        return

    st.markdown("### 📋 Stream")
    rev_msgs = list(reversed(msgs[-200:]))
    df = pd.DataFrame([
        {
            "Time": m.ts.strftime("%H:%M:%S"),
            "Source": m.source,
            "Severity": m.severity,
            "Message": m.text[:100] + ("..." if len(m.text) > 100 else ""),
        }
        for m in rev_msgs
    ])
    st.table(df)

    st.markdown("### 🔍 Message detail")
    idx = st.number_input("Row index (0 = latest)", min_value=0, max_value=max(0, len(rev_msgs) - 1), value=0, key="feed_idx")
    if rev_msgs:
        m = rev_msgs[min(idx, len(rev_msgs) - 1)]
        label = f"{m.source} · {m.severity} · {m.text[:50]}…" if len(m.text) > 50 else f"{m.source} · {m.severity} · {m.text}"
        with st.expander(label, expanded=True):
            c1, c2 = st.columns(2)
            with c1:
                st.markdown("**Vision summary**"); st.write(m.vision or "—")
                st.markdown("**Action suggestion**"); st.write(m.action or "—")
            with c2:
                st.markdown("**LiDAR data**"); st.write(m.lidar or "—")
            st.markdown("**Raw JSON**"); st.json(json.loads(m.model_dump_json()))


# ---------- Robot1 Detail (unified) ----------
def render_robot1_detail():
    init_session_state()
    st.session_state.alerts = run_alert_rules(st.session_state.messages, datetime.now())
    st.markdown("""
    <div class="hero">
        <h1>🤖 Robot1 – Unified View</h1>
        <p>AI Perception (Orin) + Navigation & LiDAR (Nano) as one entity</p>
    </div>
    """, unsafe_allow_html=True)

    robot1_msgs = [m for m in st.session_state.messages if m.source == "robot1"]
    latest = robot1_msgs[-1] if robot1_msgs else None

    c1, c2 = st.columns(2)
    with c1:
        st.markdown("""
        <div class="card">
            <h3>🧠 AI Perception (from Orin)</h3>
        </div>
        """, unsafe_allow_html=True)
        if latest:
            st.write(latest.text)
            st.write("**Vision:**", latest.vision or "—")
        else:
            st.info("No Robot1 messages yet. Run the Simulator or connect live.")
    with c2:
        st.markdown("""
        <div class="card">
            <h3>📡 Navigation & LiDAR (from Nano)</h3>
        </div>
        """, unsafe_allow_html=True)
        if latest and latest.lidar:
            st.json(latest.lidar)
        else:
            st.info("No LiDAR data in last message.")

    st.markdown("""
    <div class="card">
        <h3>✅ Suggested Actions</h3>
    </div>
    """, unsafe_allow_html=True)
    if latest and latest.action:
        st.success(latest.action)
    else:
        st.caption("—")

    st.markdown("### ⚠️ Recent Alerts (Robot1)")
    robot1_alerts = [a for a in st.session_state.alerts if a.source == "robot1"]
    for a in robot1_alerts:
        st.markdown(f'<div class="alert-box"><strong>{a.rule}</strong>: {a.meaning}</div>', unsafe_allow_html=True)
    if not robot1_alerts:
        st.caption("No Robot1 alerts.")


# ---------- Troubleshoot Page ----------
def render_troubleshoot():
    init_session_state()
    now = datetime.now()
    st.session_state.alerts = run_alert_rules(st.session_state.messages, now)
    st.markdown("""
    <div class="hero">
        <h1>🔧 Troubleshoot</h1>
        <p>Alert rules: SOURCE_DOWN · LIDAR_STALL · CRITICAL_MSG</p>
    </div>
    """, unsafe_allow_html=True)

    if not st.session_state.alerts:
        st.info("No active alerts. Run the Simulator with failure toggles to see alerts here.")
        return

    for a in st.session_state.alerts:
        st.markdown(f"""
        <div class="card" style="border-left: 4px solid #fbbf24;">
            <h3>⚠️ {a.rule} – {a.source}</h3>
        </div>
        """, unsafe_allow_html=True)
        st.write("**Meaning:**", a.meaning)
        st.write("**Probable causes:**")
        for c in a.probable_causes:
            st.write(f"- {c}")
        st.write("**Step-by-step checks:**")
        for c in a.checks:
            st.write(f"- {c}")
        st.write("**Recommended commands:**")
        for cmd in a.commands:
            st.code(cmd, language="bash")
        st.markdown("---")


# ---------- Simulator Page ----------
def render_simulator():
    init_session_state()
    st.markdown("""
    <div class="hero">
        <h1>🎮 Simulator</h1>
        <p>Generate realistic AI messages every 1–3 sec</p>
    </div>
    """, unsafe_allow_html=True)

    st.markdown("""
    <div class="card">
        <h3>⚙️ Failure toggles</h3>
    </div>
    """, unsafe_allow_html=True)
    st.session_state.simulate_robot1_down = st.checkbox("Simulate Robot1 Down", value=st.session_state.simulate_robot1_down, key="sim_down")
    st.session_state.simulate_lidar_stall = st.checkbox("Simulate LiDAR Stall", value=st.session_state.simulate_lidar_stall, key="sim_lidar")
    st.session_state.simulate_critical_burst = st.checkbox("Simulate Critical Burst", value=st.session_state.simulate_critical_burst, key="sim_crit")

    st.markdown("")
    col1, col2, _ = st.columns([1, 1, 2])
    with col1:
        if st.button("▶ Start simulator", type="primary", use_container_width=True):
            st.session_state.simulator_running = True
            st.rerun()
    with col2:
        if st.button("⏹ Stop simulator", use_container_width=True):
            st.session_state.simulator_running = False
            st.rerun()

    if st.session_state.simulator_running:
        now_ts = time.time()
        if now_ts - st.session_state.last_sim_time >= random.uniform(1, 3):
            msg = run_simulator_step(
                st.session_state.simulate_robot1_down,
                st.session_state.simulate_lidar_stall,
                st.session_state.simulate_critical_burst,
            )
            if msg:
                st.session_state.messages.append(msg)
            st.session_state.last_sim_time = now_ts
        st.success("Simulator running. Messages are appended to the feed.")
        time.sleep(1)
        st.rerun()


# ---------- Sidebar & main ----------
def main():
    init_session_state()

    # Live MQTT: drain pending messages from bridge into session
    host = st.session_state.get("mqtt_broker_host", "10.13.68.48")
    port = int(st.session_state.get("mqtt_broker_port", 1883))
    bridge = st.session_state.mqtt_bridge
    if bridge is not None and (bridge.host != host or bridge.port != port):
        bridge.stop()
        st.session_state.mqtt_bridge = None
        bridge = None
    if st.session_state.use_live_mqtt and MQTTBridge is not None:
        if bridge is None:
            bridge = MQTTBridge(host, port)
            if bridge.start():
                st.session_state.mqtt_bridge = bridge
            else:
                st.session_state.use_live_mqtt = False
                st.session_state.mqtt_bridge = None
        if st.session_state.mqtt_bridge is not None:
            for d in st.session_state.mqtt_bridge.get_pending_messages():
                try:
                    st.session_state.messages.append(Message(**d))
                except Exception:
                    pass
    else:
        if st.session_state.mqtt_bridge is not None:
            st.session_state.mqtt_bridge.stop()
            st.session_state.mqtt_bridge = None

    st.sidebar.markdown("""
    <div style="padding: 0.5rem 0 1rem 0; border-bottom: 1px solid rgba(71,85,105,0.4); margin-bottom: 1rem;">
        <p style="margin:0; font-size: 1.35rem; font-weight: 700; color: #f1f5f9;">🤖 RobotOps</p>
        <p style="margin:0.25rem 0 0 0; font-size: 0.8rem; color: #94a3b8;">Command Center</p>
    </div>
    """, unsafe_allow_html=True)
    page = st.sidebar.radio(
        "Navigate",
        ["Dashboard", "Live Feed", "Robot1 Detail", "Troubleshoot", "Simulator"],
        key="nav",
        label_visibility="collapsed",
    )
    st.sidebar.markdown("---")
    st.sidebar.markdown("**Live system**")
    use_mqtt = st.sidebar.checkbox("Use Live MQTT", value=st.session_state.use_live_mqtt, key="use_mqtt_cb")
    st.session_state.use_live_mqtt = use_mqtt
    if use_mqtt:
        st.session_state.mqtt_broker_host = st.sidebar.text_input("MQTT broker host", value=st.session_state.mqtt_broker_host, key="mqtt_host")
        st.session_state.mqtt_broker_port = int(st.sidebar.number_input("MQTT broker port", value=st.session_state.mqtt_broker_port, min_value=1, max_value=65535, key="mqtt_port"))
        if st.session_state.mqtt_bridge and st.session_state.mqtt_bridge.is_connected:
            st.sidebar.success("Connected")
        else:
            st.sidebar.caption("Broker = Controller Pi (e.g. 10.13.68.48)")
    st.sidebar.markdown("---")
    n = len(st.session_state.messages)
    st.sidebar.metric("Messages", n, delta=None)
    st.sidebar.caption("Live MQTT or Simulator to see messages.")

    if page == "Dashboard":
        render_dashboard()
    elif page == "Live Feed":
        render_live_feed()
    elif page == "Robot1 Detail":
        render_robot1_detail()
    elif page == "Troubleshoot":
        render_troubleshoot()
    elif page == "Simulator":
        render_simulator()

    # Auto-refresh Live Feed every 2 sec; run simulator step if running
    if page == "Live Feed":
        if st.session_state.simulator_running:
            now_ts = time.time()
            if now_ts - st.session_state.last_sim_time >= 1.5:
                msg = run_simulator_step(
                    st.session_state.simulate_robot1_down,
                    st.session_state.simulate_lidar_stall,
                    st.session_state.simulate_critical_burst,
                )
                if msg:
                    st.session_state.messages.append(msg)
                st.session_state.last_sim_time = now_ts
        time.sleep(2)
        st.rerun()


if __name__ == "__main__":
    main()
