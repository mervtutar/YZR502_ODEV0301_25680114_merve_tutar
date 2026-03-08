import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp


OUTDIR = "pid_odev_ciktilari"
os.makedirs(OUTDIR, exist_ok=True)

# Sistem modeli: m * dv/dt + b * v = F
m = 10.0          # kg
b = 5.0           # Ns/m
Tsim = 10.0       # s
dt = 0.01         # s
t = np.arange(0.0, Tsim + dt, dt)

v0 = 0.0          # m/s
vref_const = 1.0  # m/s

# Bu kazanc degerleri odev icin secilen ornek degerler.
controllers = {
    "P":   {"Kp": 20.0, "Ki": 0.0,  "Kd": 0.0},
    "PI":  {"Kp": 20.0, "Ki": 20.0, "Kd": 0.0},
    "PD":  {"Kp": 20.0, "Ki": 0.0,  "Kd": 5.0},
    "PID": {"Kp": 30.0, "Ki": 15.0, "Kd": 3.0},
}


def calc_metrics(time, y, ref, settling_band=0.02):
    """Birim basamak benzeri cevap icin temel performans metriklerini hesaplar."""
    metrics = {}

    # Kalici durum hatasi
    metrics["KDH (m/s)"] = abs(ref - y[-1])

    # Asma
    peak = np.max(y)
    metrics["Asma (%)"] = max(0.0, (peak - ref) / abs(ref) * 100.0) if ref != 0 else 0.0

    # Yukselme suresi (%10-%90)
    y10 = 0.1 * ref
    y90 = 0.9 * ref
    idx10 = np.where(y >= y10)[0]
    idx90 = np.where(y >= y90)[0]
    if len(idx10) == 0 or len(idx90) == 0:
        metrics["Yukselme Suresi (s)"] = np.nan
    else:
        metrics["Yukselme Suresi (s)"] = time[idx90[0]] - time[idx10[0]]

    # Yerlesme suresi (%2 bandi)
    band = settling_band * abs(ref)
    ts = np.nan
    for k in range(len(y)):
        if np.all(np.abs(y[k:] - ref) <= band):
            ts = time[k]
            break
    metrics["Yerlesme Suresi (s)"] = ts

    return metrics


def simulate_closed_loop(Kp, Ki, Kd, time, ref_signal, disturbance):
    """
    Kapali cevrim PID simulasyonu.
    x[0] = v (hiz), x[1] = ei (integral hata).
    Turev terimi ayrik farkla yaklasik hesaplanir.
    """
    N = len(time)
    v = np.zeros(N)
    ei = np.zeros(N)
    u = np.zeros(N)
    e_hist = np.zeros(N)

    v[0] = v0
    ei[0] = 0.0
    e_prev = ref_signal[0] - v[0]

    for k in range(N - 1):
        tk = time[k]
        tk1 = time[k + 1]

        e = ref_signal[k] - v[k]
        edot = (e - e_prev) / dt if k > 0 else 0.0
        uk = Kp * e + Ki * ei[k] + Kd * edot

        # Bu adimda kontrolu ve bozucuyu sabit alip tek adim integrasyon yapiyoruz.
        def ode(_, x):
            vv, eii = x
            dvdt = (uk + disturbance[k] - b * vv) / m
            deidt = ref_signal[k] - vv
            return [dvdt, deidt]

        sol = solve_ivp(
            ode,
            (tk, tk1),
            [v[k], ei[k]],
            t_eval=[tk1],
            method="RK45"
        )

        v[k + 1] = sol.y[0, -1]
        ei[k + 1] = sol.y[1, -1]
        u[k] = uk
        e_hist[k] = e
        e_prev = e

    u[-1] = u[-2]
    e_hist[-1] = ref_signal[-1] - v[-1]

    return v, u, e_hist


# 1) Referans hiz takibi, 2) Farkli kazanclarin etkisi
ref_nominal = np.ones_like(t) * vref_const
dist_nominal = np.zeros_like(t)

results = {}

for name, gains in controllers.items():
    v, u, e = simulate_closed_loop(
        gains["Kp"], gains["Ki"], gains["Kd"],
        t, ref_nominal, dist_nominal
    )
    results[name] = {
        "v": v,
        "u": u,
        "e": e,
        "metrics": calc_metrics(t, v, vref_const),
        "Kp": gains["Kp"],
        "Ki": gains["Ki"],
        "Kd": gains["Kd"],
    }

# Sekil 1: P, PI, PD, PID hiz cevaplari
plt.figure(figsize=(10, 6))
for name in ["P", "PI", "PD", "PID"]:
    plt.plot(t, results[name]["v"], label=name, linewidth=2)
plt.plot(t, ref_nominal, "--", label="Referans", linewidth=2)
plt.xlabel("Zaman (s)")
plt.ylabel("Hiz v(t) (m/s)")
plt.title("P, PI, PD ve PID Denetleyicileri icin Referans Hiz Takibi")
plt.grid(True, alpha=0.3)
plt.legend()
plt.tight_layout()
plt.savefig(os.path.join(OUTDIR, "sekil_1_referans_takibi.png"), dpi=300)
plt.close()

# Sekil 2: Kontrol girdileri
plt.figure(figsize=(10, 6))
for name in ["P", "PI", "PD", "PID"]:
    plt.plot(t, results[name]["u"], label=name, linewidth=2)
plt.xlabel("Zaman (s)")
plt.ylabel("Kontrol Girdisi F(t) (N)")
plt.title("P, PI, PD ve PID Denetleyicileri icin Kontrol Girdileri")
plt.grid(True, alpha=0.3)
plt.legend()
plt.tight_layout()
plt.savefig(os.path.join(OUTDIR, "sekil_2_kontrol_girdileri.png"), dpi=300)
plt.close()

# Tablo 1: Performans metrikleri
rows = []
for name in ["P", "PI", "PD", "PID"]:
    M = results[name]["metrics"]
    rows.append({
        "Denetleyici": name,
        "Kp": results[name]["Kp"],
        "Ki": results[name]["Ki"],
        "Kd": results[name]["Kd"],
        "Yukselme Suresi (s)": M["Yukselme Suresi (s)"],
        "Yerlesme Suresi (s)": M["Yerlesme Suresi (s)"],
        "Asma (%)": M["Asma (%)"],
        "Kalici Durum Hatasi (m/s)": M["KDH (m/s)"],
    })

df_perf = pd.DataFrame(rows)
df_perf.to_csv(
    os.path.join(OUTDIR, "tablo_1_performans_metrikleri.csv"),
    index=False,
    encoding="utf-8-sig"
)

# 3) Bozucu etkiye karsi dayaniklilik (PID)
# t = 5 s aninda, 0.2 s boyunca 10 N bozucu uygulaniyor.
pid = controllers["PID"]

ref_dist = np.ones_like(t) * 1.0
dist = np.zeros_like(t)
dist[(t >= 5.0) & (t < 5.2)] = 10.0

v_dist, u_dist, _ = simulate_closed_loop(
    pid["Kp"], pid["Ki"], pid["Kd"],
    t, ref_dist, dist
)

idx_dist = np.where(t >= 5.0)[0][0]
max_dev = np.max(np.abs(v_dist[idx_dist:] - ref_dist[idx_dist:]))

recovery_time = np.nan
band = 0.02 * abs(ref_dist[-1])
for k in range(idx_dist, len(t)):
    if np.all(np.abs(v_dist[k:] - ref_dist[k:]) <= band):
        recovery_time = t[k] - 5.0
        break

fig, ax1 = plt.subplots(figsize=(10, 6))
ax1.plot(t, v_dist, label="Hiz", linewidth=2)
ax1.plot(t, ref_dist, "--", label="Referans", linewidth=2)
ax1.set_xlabel("Zaman (s)")
ax1.set_ylabel("Hiz v(t) (m/s)")
ax1.grid(True, alpha=0.3)

ax2 = ax1.twinx()
ax2.plot(t, dist, ":", label="Bozucu Kuvvet", linewidth=2)
ax2.set_ylabel("Bozucu Kuvvet (N)")

h1, l1 = ax1.get_legend_handles_labels()
h2, l2 = ax2.get_legend_handles_labels()
ax1.legend(h1 + h2, l1 + l2, loc="best")

plt.title("PID Denetleyicide Bozucu Etkiye Karsi Davranis")
plt.tight_layout()
plt.savefig(os.path.join(OUTDIR, "sekil_3_bozucu_etki.png"), dpi=300)
plt.close()

# 4) Referans degisimine tepki (PID)
# t = 6 s aninda referans 1.0 -> 0.5 m/s oluyor.
ref_change = np.ones_like(t) * 1.0
ref_change[t >= 6.0] = 0.5
dist_change = np.zeros_like(t)

v_change, u_change, _ = simulate_closed_loop(
    pid["Kp"], pid["Ki"], pid["Kd"],
    t, ref_change, dist_change
)

idx_step = np.where(t >= 6.0)[0][0]
t2 = t[idx_step:] - 6.0
y2 = v_change[idx_step:]
new_ref = 0.5

# Adimdan sonra yeni referans etrafinda yerlesme suresi (%2)
settle2 = np.nan
band2 = 0.02 * abs(new_ref)
for k in range(len(y2)):
    if np.all(np.abs(y2[k:] - new_ref) <= band2):
        settle2 = t2[k]
        break

# Referans altina inme (undershoot)
min_after = np.min(y2)
undershoot = max(0.0, (new_ref - min_after) / abs(new_ref) * 100.0)

plt.figure(figsize=(10, 6))
plt.plot(t, v_change, label="Hiz", linewidth=2)
plt.plot(t, ref_change, "--", label="Referans", linewidth=2)
plt.xlabel("Zaman (s)")
plt.ylabel("Hiz v(t) (m/s)")
plt.title("PID Denetleyicide Referans Degisimine Tepki")
plt.grid(True, alpha=0.3)
plt.legend()
plt.tight_layout()
plt.savefig(os.path.join(OUTDIR, "sekil_4_referans_degisim.png"), dpi=300)
plt.close()

# Ek tablo: ek senaryolardan ozet metrikler
df_extra = pd.DataFrame([
    {
        "Senaryo": "Bozucu Etki",
        "Metrik 1": f"Maksimum sapma = {max_dev:.4f} m/s",
        "Metrik 2": f"Toparlanma suresi = {recovery_time:.4f} s",
        "Metrik 3": f"Son hiz = {v_dist[-1]:.4f} m/s",
    },
    {
        "Senaryo": "Referans Degisimi",
        "Metrik 1": f"Yeni yerlesme suresi = {settle2:.4f} s",
        "Metrik 2": f"Alttan tasma = {undershoot:.4f} %",
        "Metrik 3": f"Son hiz = {v_change[-1]:.4f} m/s",
    }
])

df_extra.to_csv(
    os.path.join(OUTDIR, "tablo_2_ek_senaryolar.csv"),
    index=False,
    encoding="utf-8-sig"
)

# Sonuclarin kisa metin ozetini de disari yaziyoruz.
with open(os.path.join(OUTDIR, "sonuc_ozeti.txt"), "w", encoding="utf-8") as f:
    f.write("TABLO 1 - PERFORMANS METRIKLERI\n")
    f.write(df_perf.to_string(index=False))
    f.write("\n\nTABLO 2 - EK SENARYOLAR\n")
    f.write(df_extra.to_string(index=False))

print("Tum ciktilar kaydedildi:", OUTDIR)
print(df_perf.to_string(index=False))
print()
print(df_extra.to_string(index=False))
