# Mowgli SLAM stack

Docker stack pre Velodyne VLP16, GLIM mapovanie a lokalizáciu voči GLIM mape.
Vetva `main` spája GLIM supervisor, izoláciu TF lokalizátora, nástroje pre
fusion_graph a Velodyne filter `min_range: 0.2` m.

## Images a spustenie

GitHub Actions na push do `main` zostaví a publikuje images `velodyne`, `glim`
a `lidar_localization` do `ghcr.io/lukaskrnac/3d_mowgli_slam_stack/`.
Tagy sú `latest` a `sha-<7 znakov commitu>`; PR buildy images nepublikujú.
Po úspešnom builde nastav spoločný `SLAM_IMAGE_TAG` v `.env`, ak chceš pripnutú verziu.

```bash
cp .env.example .env
docker compose pull
docker compose up -d velodyne_driver glim lidar_localization
```

Lokálny build: `docker compose build`. Cesty v `volumes` sú nastavené pre
`/home/mowgli/mowglinext/docker/slam`; pred spustením musia existovať príslušné
konfigurácie, mapa, launch súbor a `cyclonedds.xml`. Compose nemení ich obsah.
Všetky služby používajú host sieť, CycloneDDS a `ROS_DOMAIN_ID=0`.

PCAP test je voliteľný profil. Najprv zastav živý driver:

```bash
docker compose stop velodyne_driver
docker compose --profile test up velodyne_pcap_test lidar_localization
```

## GLIM supervisor

GLIM sa spúšťa na požiadanie z GUI MowgliNext alebo cez
`/glim_supervisor/start_mapping`. Supervisor riadi mapovanie, uloženie,
offline viewer, spojenie máp a export aktívnej mapy.

- Relácie: host `glim_sessions` → `/glim/sessions`.
- Aktívna mapa: host `glim_dump/garden_map.ply`; GLIM ju vidí pod
  `/glim/active_map/garden_map.ply`, lokalizátor pod `/maps/garden_map/garden_map.ply`.
- Pri exporte sa zachovajú tri zálohy. Po výmene mapy reštartuj lokalizátor.
- `stop_grace_period: 3m` dáva GLIM-u čas uložiť mapovanie.

Podrobnosti: [GLIM supervisor](glim/supervisor/README.md).

## Integrácia s MowgliNext fusion_graph

Aktuálny compose zachováva nastavenie z vetvy `lidar-loc-no-tf`:

- `/pcl_pose` je vo frame **`lidar_map`**. Prevod do `map` rieši kalibrácia
  `/calibrate_lidar_map_node/start` na strane MowgliNext. `/initialpose` posielaj
  vo frame `lidar_map`; mapa nemusí byť offline zarovnaná do ENU.
- Extrinziku `base_link → lidar_link` publikuje MowgliNext URDF. Velodyne má
  `frame_id: lidar_link`; lokalizátor má `publish_lidar_tf:=false`.
- Vlastný launch `lidar_localization/launch/mowgli_lidar_localization.launch.py`
  presmeruje `/tf` na `/lidar_localization/tf`. Tento remap ovplyvňuje aj TF listener;
  používa sa Mode A (`enable_map_odom_tf:=false`), nie Mode B.
- Dynamické TF na spoločnom `/tf` vlastní MowgliNext. Lokalizátor dodáva merania
  `/pcl_pose` a diagnostiku `/alignment_status`.

Build lokalizátora navyše obsahuje patch s parametrom `publish_tf` a používa
pinnuté upstream commity. `publish_tf: false` vypne samotný broadcast; predvolene
je `true`, takže existujúce host konfigurácie si zachovajú správanie s remapom.
Compose naďalej načítava **host priečinok `slam/config`** cez pôvodný mount do
`share/lidar_localization_ros2/param`, nie vzorový súbor z repozitára.

`lidar_localization/config/lidar_localization.yaml` je voliteľná vzorová konfigurácia
(v image `/config/lidar_localization.yaml`). Obsahuje vypnutie TF, publikovanie
akceptovaných matchov a kovarianciu podľa fitness. Ak ju chceš použiť, pridaj launch
argument `localization_param_dir:=/config/lidar_localization.yaml`; najprv prispôsob
jej parametre svojej mape a inicializácii.

Overenie v ROS prostredí kontajnera:

```bash
ros2 param get /velodyne_transform_node min_range
ros2 run tf2_ros tf2_echo base_link lidar_link
ros2 topic echo /pcl_pose --field header.frame_id --once
python3 /opt/tools/check_fusion_contract.py --duration 10 --pose-frame lidar_map --tf
```

[Historické zadanie fusion_graph](docs/ZADANIE_fusion_graph.md) opisuje starší
variant s offline zarovnanou mapou; jeho požiadavka na frame `map` nie je aktuálnym
nastavením compose.

## Kontroly bez ROS

```bash
python3 -m unittest discover -s glim/supervisor -p 'test_*.py' -v
docker compose config --quiet
```
