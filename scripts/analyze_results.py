#!/usr/bin/env python3
import os
import pandas as pd
import matplotlib
matplotlib.use("Agg")           # backend senza display
import matplotlib.pyplot as plt

BASE = os.path.expanduser("~/tesi_ur3")
RES  = os.path.join(BASE, "results")
PLOT = os.path.join(BASE, "docs", "plots")
os.makedirs(PLOT, exist_ok=True)

def read_csv(path):
    try:
        return pd.read_csv(path, sep=None, engine="python")  # auto-separa
    except Exception:
        return pd.read_csv(path)  # fallback su virgola

def savefig(name):
    out = os.path.join(PLOT, name)
    plt.tight_layout()
    plt.savefig(out, dpi=160)
    plt.close()
    print(f"[OK] Salvato: {out}")

# ---- E1: Time-to-First-Robot ----
e1_path = os.path.join(RES, "E1_tempo_avvio.csv")
if os.path.exists(e1_path):
    e1 = read_csv(e1_path).dropna(subset=["tempo_s"])
    if not e1.empty:
        grp = e1.groupby("scenario")["tempo_s"].agg(["mean","std","count"])
        print("\n[E1] Tempo di avvio (s):\n", grp.round(3))
        ax = grp["mean"].plot(kind="bar", yerr=grp["std"], capsize=4,
                              title="E1 - Tempo di avvio (media ± std)")
        ax.set_ylabel("secondi")
        savefig("E1_tempo_avvio.png")
    else:
        print("\n[E1] Nessun dato utile in", e1_path)
else:
    print("\n[E1] File non trovato:", e1_path)

# ---- E2: FPS / CPU ----
e2_path = os.path.join(RES, "E2_fps_cpu.csv")
if os.path.exists(e2_path):
    e2 = read_csv(e2_path)
    if not e2.empty:
        for metric in ("fps_media","cpu_percent"):
            if metric in e2.columns:
                pivot = e2.pivot_table(values=metric, index="script",
                                       columns="modalita", aggfunc="mean")
                ax = pivot.plot(kind="bar", title=f"E2 - {metric}")
                ax.set_ylabel(metric)
                savefig(f"E2_{metric}.png")
        print("\n[E2] Anteprima:\n", e2.head())
    else:
        print("\n[E2] Nessun dato utile in", e2_path)
else:
    print("\n[E2] File non trovato:", e2_path)

# ---- E3: Frequenza /joint_states ----
e3_path = os.path.join(RES, "E3_joint_states.csv")
if os.path.exists(e3_path):
    e3 = read_csv(e3_path)
    if not e3.empty:
        if {"pub_rate_hz","tf_hz"}.issubset(e3.columns):
            e3.sort_values("pub_rate_hz", inplace=True)
            ax = e3.plot(x="pub_rate_hz", y="tf_hz", marker="o",
                         legend=False, title="E3 - /tf Hz vs PUB_RATE")
            ax.set_xlabel("PUB_RATE (Hz)"); ax.set_ylabel("/tf (Hz)")
            savefig("E3_tf_vs_pubrate.png")
        if {"pub_rate_hz","cpu_percent"}.issubset(e3.columns):
            ax = e3.plot(x="pub_rate_hz", y="cpu_percent", marker="o",
                         legend=False, title="E3 - CPU vs PUB_RATE")
            ax.set_xlabel("PUB_RATE (Hz)"); ax.set_ylabel("CPU (%)")
            savefig("E3_cpu_vs_pubrate.png")
        print("\n[E3] Anteprima:\n", e3.head())
    else:
        print("\n[E3] Nessun dato utile in", e3_path)
else:
    print("\n[E3] File non trovato:", e3_path)

# ---- E4: MoveIt planner ----
e4_path = os.path.join(RES, "E4_moveit_planner.csv")
if os.path.exists(e4_path):
    e4 = read_csv(e4_path)
    if not e4.empty:
        if "success" in e4.columns:
            e4["success"] = e4["success"].astype(float)
        grp = e4.groupby(["planner","ostacolo"]).agg(
            planning_time_s=("planning_time_s","mean"),
            success_rate=("success","mean"),
            n=("planner","count"),
        )
        print("\n[E4] Planner (media su prove):\n", grp.round(3))

        pt = grp.reset_index().pivot(index="planner", columns="ostacolo",
                                     values="planning_time_s")
        ax = pt.plot(kind="bar", title="E4 - Tempo di planning (s)")
        ax.set_ylabel("secondi")
        savefig("E4_planning_time.png")

        sr = grp.reset_index().pivot(index="planner", columns="ostacolo",
                                     values="success_rate")
        ax = sr.plot(kind="bar", title="E4 - Success rate")
        ax.set_ylabel("successo (0..1)")
        savefig("E4_success_rate.png")
    else:
        print("\n[E4] Nessun dato utile in", e4_path)
else:
    print("\n[E4] File non trovato:", e4_path)

print("\nFatto. Grafici in:", PLOT)
