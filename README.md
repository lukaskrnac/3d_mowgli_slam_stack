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
│   ├── Dockerfile               # image pre lidar_localization_ros2 (hotový, buildí sa zo zdroja)
│   └── config/
│       └── lidar_localization.yaml
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
- `lidar_localization` očakáva vstupný cloud na topicu `/cloud` - v `command:`
  je preto remap `-r /cloud:=/velodyne_points`. Ak tvoj velodyne launch
  publikuje pod iným názvom, uprav remap.
- CI (`.github/workflows/docker-build.yml`) len overuje, že sa images zbuildia
  (bez pushu). Pre publikovanie do registry (napr. GHCR) odkomentuj login/push
  kroky vo workflowe.
