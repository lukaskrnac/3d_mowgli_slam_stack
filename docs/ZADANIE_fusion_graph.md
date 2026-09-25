# Zadanie: napojiť GLIM + VLP16 + lidar_localization_ros2 na MowgliNext fusion_graph

> Pôvodné zadanie (kontrakt medzi týmto repom a MowgliNext `fusion_graph`).
> Stav implementácie je v [README.md](../README.md#integrácia-s-mowglinext-fusion_graph).

## Kontext (prečítaj celé, predtým než začneš meniť kód)

Toto je **druhá strana** integrácie, ktorá je z väčšej časti hotová na strane MowgliNext forku
(samostatný git repo/fork, nie tento). Tam pribudol v balíku `fusion_graph` nový vstup
absolútnej pozície, ktorý číta presne dva topicy z tohto repa:

- `/pcl_pose` (`geometry_msgs/PoseWithCovarianceStamped`) — pozícia z NDT/GICP scan-matchingu
  proti pred-nahratej GLIM mape (`.ply`).
- `/alignment_status` (`diagnostic_msgs/DiagnosticArray`) — zdravie/kvalita toho matchingu.

`fusion_graph_node` (GTSAM iSAM2 factor-graph localizer, súčasť MowgliNext) je jediný node,
ktorý smie publikovať TF `map → odom` a `odom → base_footprint` (REP-105). Nefunguje to ako
klasický `robot_localization` EKF — je to jeden centrálny graf, do ktorého sa GPS aj LiDAR
pozícia len **queue-ujú ako merania (unary factors)**, nič iné nesmie TF publikovať.

Existuje aj manuálny prepínač GPS ↔ LiDAR (`primary_localization_source: "gps"|"lidar"`,
ROS2 dynamic parameter) — ten je celý na strane `fusion_graph`, tohto repa sa netýka. Tvoja
úloha je len **dodať čisté, správne formátované topicy**, nič viac.

## Úloha 1 — VYPNÚŤ TF publishing v lidar_localization_ros2 (dôležité, môže inak rozbiť Nav2)

`lidar_localization_ros2` (a NDT/GICP localizery vo všeobecnosti) typicky sám od seba
publikuje `map → odom` alebo rovno `map → base_link` TF. **To musí byť vypnuté.**

- Nájdi v configu/launch súbore parameter na broadcast TF (často niečo ako
  `broadcast_tf`, `publish_tf`, `use_odom_frame`/podobne — pomenovanie závisí od verzie/forku,
  over si to priamo v zdrojáku node-u, nie len v README).
- Nastav ho na `false`/vypnuté. Node má naďalej normálne fungovať a počítať pozíciu — má len
  **prestať ju publikovať ako TF**, keďže tú istú informáciu teraz posiela ako obyčajnú správu
  na `/pcl_pose`.
- Ak by zostali dva uzly publikovať `map→odom` súčasne, TF strom sa rozbije (konflikt/warning
  o duplicitnom broadcasterovi), Nav2 dostane nekonzistentné transformácie a správanie bude
  nepredvídateľné (nie hneď pád, skôr "divné" správanie navigácie, ťažko sa to potom hľadá).

## Úloha 2 — formát `/pcl_pose`

`geometry_msgs/PoseWithCovarianceStamped`, presne:

- `header.frame_id` **musí byť `"map"`** (fusion_graph's `map_frame` parameter, presná
  hodnota, case-sensitive) — nie vlastný frame lidar_localization_ros2-ky, nie `"odom"`.
- `header.stamp` = čas nasnímania scanu (nie wall-clock v čase publikovania). Prijímacia
  strana odmieta vzorky staršie ako 0.5 s.
- `pose.pose.position.x/y` = XY v tomto `map` frame-e — **ENU meter, dok = origin (0,0)**.
  Znamená to, že GLIM `.ply` mapa musí byť vopred (offline, jednorazovo) zarovnaná do tohto
  súradnicového systému — na to slúži pomocný skript `align_glim_map.py` (Umeyama 2D
  similarity transform) — over si, že mapa, ktorú `lidar_localization_ros2` používa na
  lokalizáciu, je už **výstup** tohto zarovnania, nie surová GLIM mapa.
- `pose.covariance` — 6×6 riadkovo usporiadaná matica (x,y,z,roll,pitch,yaw). Prijímacia
  strana číta:
  - XY 2×2 blok z indexov `[0]`, `[1]`, `[6]`, `[7]`
  - yaw variance z indexu `[35]`
  - **Toto musí byť reálny odhad neistoty z NDT/GICP fitness/matching skóre, nie nuly ani
    identity.** Prijímacia strana **zahodí** vzorku, ak je kovariancia
    nekonečná/nulová/nie pozitívne semidefinitná, a tiež zahodí, ak je odhadovaná sigma
    nad ~0.75 m (konfigurovateľné na druhej strane).
- Orientácia (quaternion) sa tiež používa (yaw sa z nej extrahuje cez atan2) — vyplň ju
  reálnou orientáciou z matchingu, nie identitou.

## Úloha 3 — formát `/alignment_status`

`diagnostic_msgs/DiagnosticArray`, s jedným `DiagnosticStatus` v poli `status[]`, ktorého
`name` **obsahuje** podreťazec `"lidar_localization_ros2/alignment"`.

V `status.values[]` (zoznam `KeyValue`) očakáva presne tieto kľúče:

| key | hodnota | poznámka |
|---|---|---|
| `failure_category` | string, `"healthy"` keď OK | akýkoľvek iný string = nezdravé |
| `consecutive_rejected_updates` | string s integerom (napr. `"0"`) | koľko posledných aktualizácií po sebe localizer sám vyhodnotil ako zlé/odmietnuté |
| `reinitialization_requested` | `"true"`/`"1"` alebo čokoľvek iné (=false) | keď localizer potrebuje re-init |

Publikuje sa nezávisle od `/pcl_pose`, ale musí byť **čerstvé** — prijímacia strana ho
považuje za platné len do 2 s starosti; ak `/alignment_status` prestane chodiť, `/pcl_pose`
sa začne tíško ignorovať (fail-closed).

## Úloha 4 — 2D `/scan` pre Nav2 (VLP16 → LaserScan)

> **Zrušené v tomto repe** — `/scan` bridge je už hotový na strane MowgliNext.

Oddelená vetva od `/pcl_pose` (obstacle avoidance pre Nav2, nie lokalizácia).
`pointcloud_to_laserscan` nad VLP16 3D mrakom s height-band konverziou (nie single-ring):

- `target_frame: "base_link"`
- `min_height: 0.03`, `max_height: 0.35` (senzor ~40 cm vysoko)
- `range_min: 0.30`
- výstup na `/scan` (default `input_topic` pre MowgliNext `scan_deskew_node`)
- Fyzické obmedzenie: VLP-16 má ±15° vertikálne FOV → pri 40 cm montáži mŕtvy kužeľ
  ~1.5 m pred robotom. Softvérovo neopraviteľné (treba napr. nárazníkovú lištu).

## Sieť/deployment

Tento stack beží v **samostatnom kontajneri** od hlavného MowgliNext image. Kontajner musí mať:

- `network_mode: host`
- rovnaký `ROS_DOMAIN_ID` ako hlavný `mowgli` kontajner
- `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`
- ten istý `cyclonedds.xml` mount (a teda aj `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`)

Ak niečo chýba, výsledok nie je chyba, ale ticho: `fusion_graph` nikdy nič nedostane.

## Priority

1. Vypnúť TF broadcasting v `lidar_localization_ros2`.
2. `/pcl_pose` v správnom frame-e/formáte s reálnou kovarianciou.
3. `/alignment_status` health reporting.
4. ~~VLP16 → `/scan` bridge~~ (hotové na strane MowgliNext).
