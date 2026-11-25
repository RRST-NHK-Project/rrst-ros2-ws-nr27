# 有限旋回独ステの制御モデル：MPPI ＋ FF ＋ PID ＋ DOB

このフォルダのコードが実装している式をまとめた文書。実行は `python3 scenarios.py`（比較表、約1分半）と `python3 plot_results.py`（`results.png`）。必要なのは numpy と matplotlib だけ。記号の定義、プラント（シミュレータ上の「本物」）、制御器の各層の順で書く。最後に、直進・旋回移動・坂の比較結果と、実装中に分かったことを載せる。

## 0. 全体構成

```
参照軌道 r(t) = (x, y, ψ, vx, vy, ω)_ref
     │
  [MPPI 25 Hz]   機体速度指令 u = (vx, vy, ω)          mppi.py
     │
  [速度整形 1 kHz] 加速度制限 → ν_ref, そこから a_ref     stack.py
     │
  [逆運動学＋角度解決]  各輪の (ステア目標角 δ*_i, 駆動速度 s_i)   kinematics.py
     │                      │
  [ステア軸 ×4]           [駆動軸 ×4]
  参照軌道+FF+PID+DOB      FF+PI+DOB (+坂の重力FF)        steer_axis.py / drive_axis.py
     │                      │
  ステアトルク τ_s,i      駆動トルク τ_w,i  ──→ プラント（plant.py）
```

各層は周波数で役割が分かれている。上の層ほど遅く「どこへ行くか」を決め、下の層ほど速く「言われた通りに動かす」。

| 層 | 周期・帯域 | 担当 |
|---|---|---|
| MPPI | 25 Hz（ホライズン 1 s） | 位置・姿勢の追従。可動域の端や反転を避ける計画 |
| ステア軸の位置ループ | 帯域 約 30 rad/s | 角度を参照軌道に追従させる |
| 駆動軸の速度ループ | 帯域 約 40 rad/s | 車輪速度を参照に追従させる |
| DOB | 遮断 150〜200 rad/s | モデル誤差・外乱を打ち消す |

## 1. 座標系と記号

- 世界座標：$x$ 前方（坂を登る向き）、$y$ 左、反時計回りが正。
- 機体座標：機体の幾何中心が原点。$\psi$ は機体の向き。
- $R(\psi) = \begin{pmatrix}\cos\psi & -\sin\psi\\ \sin\psi & \cos\psi\end{pmatrix}$ は機体座標を世界座標に回す行列。
- モジュール $i$（$i=0..3$）の取り付け位置 $\mathbf r_i=(x_i,y_i)$（機体座標）。一辺 0.5 m の正方形の角。
- ステア角 $\delta_i$ は機体座標で測る。可動域は $|\delta_i|\le L = 170°$。
- 車輪の前方向（世界座標）は $\mathbf e_i = (\cos(\psi+\delta_i),\ \sin(\psi+\delta_i))$、左方向は $\mathbf n_i$。
- $\Omega_i$：車輪の回転速度（車輪軸換算）、$r_w$：車輪半径。
- 添字 $n$ は公称値（コントローラが「そうだと思っている」値）。添字なしは真値。

## 2. プラント（真のモデル）— `plant.py`

### 2.1 機体（平面3自由度）

$$
m\,\dot{\mathbf v} = \sum_i \mathbf F_i + \mathbf F_g,\qquad
I_z\,\dot\omega = \sum_i (R\mathbf r_i)\times \mathbf F_i
$$

$\mathbf v$ は世界座標の速度、$\mathbf F_i$ は輪 $i$ のタイヤ力、$\times$ は2次元の外積 $a_x b_y - a_y b_x$。

### 2.2 タイヤ

接地点の速度を、車輪の前後方向と横方向に分ける。

$$
\mathbf v_{c,i} = \mathbf v + \omega\,\hat{\mathbf z}\times(R\mathbf r_i),\qquad
u_i = \mathbf v_{c,i}\cdot\mathbf e_i,\qquad w_i = \mathbf v_{c,i}\cdot\mathbf n_i
$$

前後力は「車輪の周速と接地点速度の差（すべり）」に比例し、横力は横すべり速度に逆らう。

$$
F_{x,i} = C_x\,(r_w\Omega_i - u_i),\qquad F_{y,i} = -C_y\,w_i
$$

合力が摩擦円 $\mu N_i$ を超えたら同じ比率で縮める（$N_i = mg\cos\theta/4$）。

### 2.3 駆動輪

$$
J_w\,\dot\Omega_i = \tau_{w,i} - r_w F_{x,i} - b_w\Omega_i
$$

### 2.4 ステア

$$
J_s\,\ddot\delta_i = \tau_{s,i} - b_s\dot\delta_i - \tau_c\tanh(\dot\delta_i/\varepsilon) - t_p F_{y,i} + \tau_{\rm stop}
$$

$t_p F_{y,i}$ は横力がステアを戻そうとするモーメント（$t_p$：トレール）。$\tau_{\rm stop}$ は可動域端のストッパ（硬いばね）。

### 2.5 坂

$x$ が $[1.2, 2.8]$ m の範囲だけ勾配 $\theta = 10°$。重力の斜面成分 $\mathbf F_g = (-mg\sin\theta,\ 0)$ が機体を押し戻す。

### 2.6 真値と公称値のずれ

DOB が効くかを確かめるため、わざとずらしてある。

| 量 | 真値 | 公称値 |
|---|---|---|
| 質量 $m$ | 25 kg | 20 kg |
| 車輪慣性 $J_w$ | 0.004 | 0.003 kg m² |
| ステア慣性 $J_s$ | 0.003 | 0.002 kg m² |
| ステア摩擦 $\tau_c$ | 0.05 Nm | 0（知らない） |
| 横力の戻しモーメント | あり | なし（知らない） |

数値積分は半陰的オイラー法で、刻みは 0.2 ms。速度を先に更新し、その新しい速度で位置を進める。

## 3. 上位：MPPI — `mppi.py`

### 3.1 予測モデル

刻み $\Delta t = 0.04$ s（呼び出し周期と同じ）、ホライズン $T = 25$ ステップ（1 秒先まで）。状態は位置 $\mathbf p$、姿勢 $\psi$、機体速度 $\boldsymbol\nu = (v_x, v_y, \omega)$、ステア角 $\delta_{1..4}$。入力は機体速度指令 $\mathbf u$。

1. **ステア**：指令 $\mathbf u$ から各輪の向き $\phi_i$ を出し、角度解決（§4）で目標角 $\delta^*_i$ にする。$\delta_i$ は角速度の上限 $\dot\delta_{\max}$ で $\delta^*_i$ に近づく。
2. **速度**：向きが合っていない輪は進めない。

$$
c = \frac14\sum_i \mathbb 1[s_i>s_{\min}]\,\max\!\big(0,\cos(\delta^*_i-\delta_i)\big),\qquad
\boldsymbol\nu \leftarrow \boldsymbol\nu + \mathrm{sat}_{a_{\max}}\!\Big(\frac{c\,\mathbf u-\boldsymbol\nu}{\tau_v}\Big)\Delta t
$$

   $\mathrm{sat}$ は「向きを保ったまま」大きさだけ制限する（§8.1）。
3. **姿勢**：$\mathbf p \leftarrow \mathbf p + R(\psi)\,\boldsymbol\nu_{xy}\Delta t$、$\psi \leftarrow \psi + \omega\Delta t$。

### 3.2 コスト

各ステップ $k$ で次を足し合わせる（最終ステップだけ追従項を 5 倍にする）。

$$
\begin{aligned}
q_{\rm pos}\|\mathbf p-\mathbf p_{\rm ref}\|^2 &+ q_\psi\,\mathrm{wrap}(\psi-\psi_{\rm ref})^2 + q_v\|\mathbf u-\mathbf u_{\rm ref}\|^2 + r_{\Delta u}\|\mathbf u_k-\mathbf u_{k-1}\|^2\\
&+ w_{\rm travel}\sum_i|\Delta\delta_i| + w_{\rm lim}\sum_i\max\!\big(0,\ |\delta_i|-(L-m)\big)^2 + w_{\rm align}\sum_i\big(1-\cos(\delta^*_i-\delta_i)\big)
\end{aligned}
$$

2行目が有限旋回のための項で、ステアの移動量、可動域の端への近さ、向きのずれを罰する。

### 3.3 更新

1. 時間方向に相関したノイズ $\boldsymbol\varepsilon_{k,t} = a\,\boldsymbol\varepsilon_{k,t-1} + \sqrt{1-a^2}\,\mathbf n_{k,t}$（$a=0.8$）を $K=300$ 本作る。
2. 各サンプルの入力列 $U + \boldsymbol\varepsilon_k$ を予測モデルで転がし、コスト $S_k$ を計算する。
3. 重み付き平均で入力列を更新する。

$$
w_k = \frac{\exp\!\big(-(S_k-\min S)/\lambda\big)}{\sum_j \exp\!\big(-(S_j-\min S)/\lambda\big)},\qquad
U \leftarrow U + \sum_k w_k\boldsymbol\varepsilon_k
$$

4. 先頭の $\mathbf u_0$ を出力し、$U$ を1ステップずらして次回の初期値にする（ウォームスタート）。

有効サンプル数 $\mathrm{ESS}=1/\sum w_k^2$ が 1 に近いと、1本の乱数サンプルをそのまま採用することになり、指令が暴れる。$\lambda$ はこれを見て決める（§8.4）。

## 4. 逆運動学と角度解決 — `kinematics.py`

各輪が出すべき速度（機体座標）：

$$
\mathbf v_i = \begin{pmatrix} v_x - \omega y_i\\ v_y + \omega x_i\end{pmatrix},\qquad
\phi_i = \operatorname{atan2}(\mathbf v_i),\qquad s_i = \|\mathbf v_i\|
$$

**角度解決（有限旋回の本体）**：候補 $\phi_i + k\pi$（$k=-3..3$）のうち、可動域 $[-L, L]$ に入るものから現在角 $\delta_i$ に最も近いものを選ぶ。$k$ が奇数なら 180° ずらした向きなので、駆動を反転する（$\sigma_i=-1$）。

$$
\delta^*_i = \arg\min_{\substack{c=\phi_i+k\pi\\ |c|\le L}} \big(|c-\delta_i| + p_{\rm flip}\,[k\ \text{奇数}]\big)
$$

**駆動速度**：角度が合うまで駆動を絞る。

$$
v_{{\rm drive},i} = \sigma_i\, s_i\,\max\!\big(0,\cos(\delta^*_i-\delta_i)\big)
$$

**低速の不感帯**：$s_i$ がしきい値より小さいとき、向きは更新せず保持し、駆動は 0 にする。しきい値にはヒステリシスを付ける（動き出し 0.10 m/s、止まる 0.05 m/s）。

## 5. ステア軸：参照軌道＋FF＋PID＋DOB — `steer_axis.py`

### 5.1 参照軌道（`ref_profile.py`）

目標角 $\delta^*$ は飛び飛びに変わるので、そのままでは速度も加速度も定義できない。そこで、速度と加速度に上限を付けた参照 $(\theta_r, \dot\theta_r, \ddot\theta_r)$ を作る。

$$
v_{\rm des} = \mathrm{sign}(e)\min\!\big(v_{\max},\ k|e|,\ \sqrt{2a_b|e|}\big),\qquad
\ddot\theta_r = \mathrm{clip}\Big(\frac{v_{\rm des}-\dot\theta_r}{\tau_a},\ \pm a_{\max}\Big)
$$

$e = \delta^*-\theta_r$。$\sqrt{2a_b|e|}$ は「残りの距離で止まれる速度」なので、目標に近づくと自動で減速する。

### 5.2 制御則

$$
\tau_s = \underbrace{J_{s,n}\ddot\theta_r + b_{s,n}\dot\theta_r}_{\rm FF}
+ \underbrace{K_p e + K_i\!\int\! e + K_d\dot e}_{\rm PID}
- \hat d
$$

$e=\theta_r-\delta$、$\dot e = \dot\theta_r-\dot\delta$。微分項は偏差を数値微分せず、速度の差をそのまま使う。

ゲインは公称慣性から決める（$\omega_n = 30$ rad/s）：$K_p = J_{s,n}\omega_n^2$、$K_d = 2J_{s,n}\omega_n$。

### 5.3 外乱オブザーバ（`dob.py`）

公称モデル $J_n\dot\omega + B_n\omega = \tau + d$ が合わない分を、すべて外乱 $d$ とみなす。

$$
\hat d = Q(s)\big(J_n s\,\omega + B_n\omega - \tau\big),\qquad Q(s)=\frac{g}{s+g}
$$

$s\omega$（微分）はノイズを増やすので、恒等式 $Q(s)J_n s = J_n g\,(1-Q(s))$ で微分を消してから離散化する。

$$
\omega_f \mathrel{+}= \alpha(\omega-\omega_f),\quad \tau_f \mathrel{+}= \alpha(\tau-\tau_f),\quad \alpha = 1-e^{-g\Delta t}
$$

$$
\hat d = J_n g\,(\omega-\omega_f) + B_n\omega_f - \tau_f
$$

DOB に渡すトルク $\tau$ は、前の周期に実際に出した値（飽和した後の値）にする。飽和する前の値を渡すと、飽和で削られた分まで外乱と見なしてしまい、推定が巻き上がる。

## 6. 駆動軸：FF＋PI＋DOB — `drive_axis.py`

### 6.1 公称モデル

車輪の慣性に、機体質量の 1/4 を車輪軸から見た慣性として足す。

$$
J_{\rm eq}\dot\Omega + B_n\Omega = \tau_w - \tau_{\rm known} + d,\qquad
J_{\rm eq} = J_{w,n} + \frac{m_n}{4}r_w^2
$$

### 6.2 既知の負荷（坂の重力）

坂を登る向きの成分だけを各輪が受け持つ。$\theta_{\rm meas}$ は IMU で測ったピッチ。

$$
\tau_{{\rm known},i} = r_w\,\frac{m_n g\sin\theta_{\rm meas}}{4}\,\cos(\psi+\delta_i)
$$

### 6.3 制御則

$$
\tau_w = \underbrace{J_{\rm eq}\dot\Omega_r + B_n\Omega_r + \tau_{\rm known}}_{\rm FF} + K_p e + K_i\!\int\! e - \hat d
$$

$\Omega_r$ は「駆動速度 ÷ $r_w$」を角加速度の上限付きで追わせた参照で、$\dot\Omega_r$ はその差分。ゲインは $K_p = J_{\rm eq}\omega_c$、$K_i = J_{\rm eq}\omega_c^2/5$（$\omega_c=40$ rad/s）。

### 6.4 FF と DOB を整合させる

DOB には $\tau_{\rm applied} - \tau_{\rm known}$ を渡す。坂の重力を FF で入れておきながら、DOB に $\tau_{\rm applied}$ をそのまま渡すと、DOB は同じ重力をもう一度外乱として推定する。その結果、重力を二重に補償してしまう。

**「FF で入れた既知の負荷は、公称モデルの一部として扱う」**のが整合の取り方。同じ理由で、機体の加速に要る力は $J_{\rm eq}$ に含めたので、加速度 FF として別に足してはいけない。

## 7. 結果 — `scenarios.py`（シード 3 本の平均）

| 場面 | 構成 | 位置RMS [mm] | 車輪速度RMS [rad/s] | ステア追従RMS [mrad] | ステア総移動 [rad] |
|---|---|---:|---:|---:|---:|
| 直進 | PID | 14.3 | 0.70 | 35.1 | 11.9 |
| | PID+FF | 13.1 | 0.28 | 27.2 | 10.5 |
| | PID+FF+DOB | 10.8 | 0.13 | 3.1 | 11.8 |
| 旋回移動 | PID | 31.0 | 1.17 | 58.3 | 29.4 |
| | PID+FF | 30.1 | 0.95 | 50.1 | 29.8 |
| | PID+FF+DOB | 28.6 | 0.43 | 4.1 | 29.2 |
| | 同上、MPPI ステア非考慮 | 28.2 | 0.53 | 4.9 | 36.2 |
| 坂 | PID | 13.5 | 0.72 | 34.8 | 12.4 |
| | PID+FF | 12.1 | 0.32 | 28.0 | 12.5 |
| | PID+FF+DOB | 10.0 | 0.14 | 3.3 | 14.3 |
| | PID+FF+DOB、IMU なし | 10.1 | 0.14 | 3.3 | 14.6 |

坂区間での車輪速度誤差のピーク：PID 1.33 → +FF 0.56 → +DOB 0.25 rad/s。

**読み方**

- **内側のループは劇的に良くなる。**
  - ステアの追従誤差は、DOB で約 1/10 になる。ステアは摩擦と横力の戻しモーメントを公称モデルで知らないので、FF だけではほとんど改善しない。
  - 車輪速度の誤差は、FF で半分、DOB でさらに半分になる。
- **位置誤差の改善は小さい（14 → 11 mm）。**
  - MPPI が外側で位置のずれを補正し続けるので、内側の誤差の多くは吸収される。
  - 残る約 10 mm は、MPPI 自体の分解能（25 Hz、乱数サンプルの平均）で決まっている。
  - つまり内側を固めた効果は、位置精度より「MPPI が予測したとおりに機体が動く」ことに現れる。
- **坂は DOB があれば IMU がなくても同じ結果になる。**
  - 坂の重力はゆっくり変わる外乱なので、DOB の得意分野。
  - IMU の重力 FF が効くのは DOB を使わない構成。PID+FF で坂区間の車輪速度ピークは IMU なし 0.77 → IMU あり 0.56 rad/s（約 3 割減）。積分項が重力分を後から拾うので、差はピークに出て RMS にはほとんど出ない。
- **旋回移動では、MPPI がステアを予測に入れると、ステア総移動が約 20% 減る**（36 → 29 rad）。
  - 位置誤差はほぼ同じなので、同じ追従を少ない振り回しで実現している。
  - `results.png` の右図では、非考慮版が序盤に可動域の端（−160°）まで振れてから戻している。

## 8. 実装して分かったこと

### 8.1 加速度制限は「向きを保って」かける

$v_x, v_y$ を軸ごとに clip すると、加速の途中で $v_y$ だけ先に目標に届き、速度ベクトルの向きが歪む。独ステでは向きがそのままステア角なので、ステアが無駄に振れる。差分ベクトル全体を同じ倍率で縮める必要がある。MPPI の予測モデルも同じ方式にそろえた。

### 8.2 低速の不感帯とヒステリシス

速度ゼロのベクトルには向きがない。MPPI が出す 0.1 m/s 程度の補正指令でも、各輪の向きは大きく変わる。不感帯は必須で、予測モデル側にも「不感帯以下の輪は進まない」と教えておく。そうしないと、MPPI は効かない小さな補正を出し続ける。

### 8.3 到着判定

目標が止まっていて誤差が許容内（2 cm、2°）なら、MPPI の出力を 0 にする。これを入れないと、停止後もステアが振り回され続けた。

### 8.4 MPPI の温度 λ と刻み

- **λ**：小さい（0.5）と ESS ≈ 1 になり、指令が 1 本の乱数そのものになった。位置誤差は 70 mm を超えた。λ = 20（ESS ≈ 30）で落ち着いた。
- **刻み**：予測の刻み $\Delta t$ と呼び出し周期がずれていると、ウォームスタートで $U$ をずらす量が実時間と合わず、計画が早送りになる。両方 0.04 s にそろえた。

## 9. このモデルの限界

- **測定が理想。** エンコーダの量子化、速度のノイズ、自己位置推定の誤差を入れていない。DOB は $J_n g\,\Delta\omega$ の形でノイズを増幅するので、実機では遮断周波数 $g$ を下げる必要がある可能性が高い。
- **荷重移動を無視している。** 各輪の荷重を一律 $mg\cos\theta/4$ にしている。加減速や横斜面で輪ごとの荷重が変わる効果はない。
- **ステア回転と車輪の連成がない。** ステアを回すと駆動輪が連れ回る効果（ベルト駆動で出る）は入れていない。
- **MPPI の乱数の影響が残る。** 低速時のステアの振れ（直進でも総移動 10 rad 程度）は、乱数サンプルの平均がどの向きにもなりうることに由来する。直進の始めと止まり際に集中している。
- **段差は扱っていない。**
