# Mowgli SLAM stack

Docker-composed stack pre lokalizáciu kosačky voči `.ply` mape z GLIM-u,
pomocou `VLP16` skenov a [rsasaki0109/lidar_localization_ros2](https://github.com/rsasaki0109/lidar_localization_ros2).

## Štruktúra repozitára

```
.
├── docker-compose.yaml          # hlavný compose súbor, buildí a spúšťa všetky služby
├── common/
│   └── cyclonedds.xml           # zdieľaná DDS konfigurácia (mountuje sa do všetkých kontajnerov)
├── velodyne/
│   ├── Dockerfile               # image pre velodyne_driver (TODO: doplň podľa svojho pôvodného buildu)
│   └── config/
│       └── VLP16-velodyne_driver_node-params.yaml
├── glim/
│   ├── Dockerfile               # image pre GLIM (TODO: doplň podľa svojho pôvodného buildu)
│   ├── config/                  # config_relocalization.json a pod.
│   ├── ext_modules/             # libglim_relocalization.so a pod. (negitované, veľké binárky)
│   └── dump/                    # výstupné mapy z GLIM po ukončení behu
├── lidar_localization/
│   ├── Dockerfile               # lidar_localization_ros2 (pinnutý commit + patche)
│   ├── patches/                 # patche na upstream (0001: parameter publish_tf)
│   ├── config/
│   │   └── lidar_localization.yaml  # /pcl_pose + /alignment_status pre fusion_graph, TF vypnutý
│   └── tools/
│       └── check_fusion_contract.py # overenie topicov voči kontraktu fusion_graph
├── docs/
│   └── ZADANIE_fusion_graph.md  # kontrakt s MowgliNext fusion_graph (pôvodné zadanie)
├── maps/                        # sem patrí .ply mapa z GLIM-u (negitované, veľké súbory)
└── .github/workflows/
    └── docker-build.yml         # CI - overí, že sa všetky 3 images zbuildia
```

## Predpoklady

- Doplniť skutočný obsah `velodyne/Dockerfile` a `glim/Dockerfile` (momentálne sú tam
  len rozumné placeholdery) podľa toho, ako si pôvodne staval `vlp16_dds` a `glim_dds`.
- Do `maps/garden_map/` nakopírovať `.ply` mapu vyprodukovanú GLIM-om.
- Skontrolovať/upraviť `lidar_localization/config/lidar_localization.yaml`
  (najmä `map_path`, `ndt_resolution`).

## Build

```bash
git clone <tento repozitár>
cd mowgli-slam-stack
docker compose build
```

Alebo len jednu službu:

```bash
docker compose build lidar_localization
```

## Beh

Bežná prevádzka (živý VLP16 sken + lokalizácia voči mape):

```bash
docker compose up velodyne_driver lidar_localization
```

Test s nahratým `.pcap` súborom namiesto živého senzora
(daj `example.pcap` a `run_pcap.sh` do `velodyne/`):

```bash
docker compose --profile test up velodyne_pcap_test lidar_localization
```

Spustenie GLIM-u (napr. ak chceš súbežnú odometriu/relokalizáciu):

```bash
docker compose --profile glim up glim
```

## Poznámky

- Všetky služby bežia s `network_mode: host` a rovnakým `ROS_DOMAIN_ID`
  (nastaviteľné cez `.env`, pozri `ROS_DOMAIN_ID=` premennú v `docker-compose.yaml`),
  aby DDS discovery cez multicast fungoval bez ďalšej konfigurácie na jednom stroji.
- `lidar_localization` berie vstupný cloud z `/velodyne_points` (launch argument
  `cloud_topic:=` v `command:`). Ak tvoj velodyne launch publikuje pod iným názvom,
  uprav ho tam.
- CI (`.github/workflows/docker-build.yml`) len overuje, že sa images zbuildia
  (bez pushu). Pre publikovanie do registry (napr. GHCR) odkomentuj login/push
  kroky vo workflowe.

## Integrácia s MowgliNext fusion_graph

Kontrakt: [docs/ZADANIE_fusion_graph.md](docs/ZADANIE_fusion_graph.md). Tento stack len
**dodáva merania**, TF strom (`map → odom → base_footprint`) vlastní výhradne `fusion_graph`.

| Výstup | Kto | Stav |
|---|---|---|
| žiadny TF | `lidar_localization` | upstream vypínač nemá (s `enable_map_odom_tf: false` posiela `map → base_link`, s `true` posiela `map → odom`), preto patch `0001` pridáva `publish_tf`, v configu `false` (overené: 0 správ na `/tf`; s `true` ide `map -> base_link`) |
| `/pcl_pose` | `lidar_localization` | frame `map`, stamp = čas scanu, len akceptované matche, kovariancia z fitness (`error_floor`: XY σ 0.15–0.35 m, yaw σ 2–4° pri akceptovanom matchi) |
| `/alignment_status` | `lidar_localization` | upstream už publikuje `lidar_localization_ros2/alignment` s `failure_category` (`healthy`/`missing_map`/`missing_initial_pose`/`weak_overlap`/`bad_match`/`stale_prediction`/`overload`), `consecutive_rejected_updates`, `reinitialization_requested` pri každom scane |

Pred nasadením:

1. **Mapa:** do `/home/mowgli/mowglinext/docker/slam/maps/` daj výstup `align_glim_map.py`
   (ENU, dok = 0,0) ako `garden_map_aligned.ply`, nie surovú mapu z `glim_dump`.
2. **Yaw doku:** `initial_pose_q*` v `config/lidar_localization.yaml` (default 0° = východ).
3. **TF `base_link → velodyne`:** lokalizácia ho potrebuje. Ak ho MowgliNext URDF
   neobsahuje, nastav `LIDAR_TF_PUBLISH=true` (+ `LIDAR_TF_X/Y/Z`) v `.env`.
4. **Sieť:** všetky služby majú `network_mode: host`, `ROS_DOMAIN_ID=0`, `rmw_cyclonedds_cpp`
   a mountujú ten istý `cyclonedds.xml` ako hlavný `mowgli` kontajner.

Spustenie a overenie:

```bash
docker compose up -d velodyne_driver lidar_localization
docker exec -it lidar_localization python3 /opt/tools/check_fusion_contract.py --duration 10 --tf
```

Pri úspechu `check_fusion_contract.py` vypíše na konci `OK`. V zozname `/tf` hrán majú byť
len hrany od MowgliNext (`map -> odom`, `odom -> base_footprint`), nič z tohto stacku.
