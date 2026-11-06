# glim_supervisor

Kontajner `glim_slam` beží stále, ale GLIM v ňom beží len vtedy, keď ho
spustíš. Supervisor (`glim_supervisor.py`) je malý ROS node, ktorý na
požiadanie spustí mapovanie, offline viewer alebo export mapy. Vždy beží
najviac jeden z nich.

Ovládanie je v MowgliNext GUI: **Diagnostika → Kalibrácia → Mapovanie (GLIM)**.

## Kam sa čo ukladá (na hoste, `/home/mowgli/mowglinext/docker/slam/`)

| Priečinok | V kontajneri | Obsah |
|---|---|---|
| `glim_sessions/mapping_<dátum_čas>/` | `/glim/sessions/…` | dump jedného mapovania. Každé mapovanie má nový priečinok, nič sa neprepisuje. |
| `glim_sessions/<tvoj_názov>/` | `/glim/sessions/…` | spojená mapa, ktorú si v offline vieweri uložil cez *Save Map* |
| `glim_dump/garden_map.ply` | `/glim/active_map/garden_map.ply` | aktívna mapa lokalizátora (u neho `/maps/garden_map/garden_map.ply`) |
| `glim_dump/garden_map.backup-<dátum_čas>.ply` | | predchádzajúce aktívne mapy (posledné 3) |
| `glim_dump/garden_map.ply.source.json` | | z ktorej relácie a kedy bola aktívna mapa exportovaná |

Aktívnu mapu prepíše **len** export z GUI („Použiť ako mapu“). Mapovanie ju
nikdy nemení.

## Postup

1. **Začať mapovanie**: GLIM zapisuje do novej relácie. Viewer uvidíš cez AnyDesk.
2. **Ukončiť a uložiť**: supervisor pošle GLIM-u SIGINT a počká, kým dokončí
   optimalizáciu a zapíše dump (stav *saving*).
3. **Spojenie viacerých relácií** (voliteľné): *Otvoriť vo vieweri* pri prvej
   relácii. Supervisor pri tom vytvorí prázdny priečinok
   `/glim/sessions/merged_<dátum_čas>` a nastaví dialógy viewera (cez jeho
   „recent files“ v `/tmp/tmp_recent_files.ini`), takže:
   - *File → Open Additional Map* sa otvorí v `/glim/sessions`,
   - *File → Save → Save Map* sa otvorí rovno na pripravenom priečinku, stačí potvrdiť.

   Po zatvorení viewera sa uložená mapa objaví v zozname ako nová relácia
   (typ *merged*). Ak si do priečinka nič neuložil, prázdny sa zmaže.
4. **Použiť ako mapu**: export relácie do `garden_map.ply` (bez okien, bez
   ďalšej optimalizácie). Stará mapa ide do zálohy.
5. Lokalizátor načíta novú mapu až po reštarte (`docker restart lidar_localization`)
   a LiDAR kalibráciu mapy treba spraviť znova.

## Presun z pôvodného usporiadania

Predtým bol `glim_dump` zároveň `/tmp/dump` GLIM-u. Ak v ňom máš starý dump,
skopíruj ho medzi relácie, nech sa dá otvoriť vo vieweri. Nič sa nemaže:

```bash
cd /home/mowgli/mowglinext/docker/slam
mkdir -p glim_sessions && cp -a glim_dump glim_sessions/povodna_mapa
rm -f glim_sessions/povodna_mapa/*.ply   # .ply do relácie nepatrí
```

Image treba prebuildovať (supervisor a zenity sú v ňom):
`docker build -t glim_dds:latest glim/ && docker compose up -d glim`.

## Bez GUI

```bash
ros2 service call /glim_supervisor/start_mapping std_srvs/srv/Trigger
ros2 service call /glim_supervisor/stop_mapping  std_srvs/srv/Trigger
ros2 param set /glim_supervisor target_session mapping_2026-09-27_164005
ros2 service call /glim_supervisor/open_viewer   std_srvs/srv/Trigger   # alebo export_map
ros2 topic echo /glim_supervisor/status          # JSON so stavom a zoznamom relácií
```

Testy (bez ROS): `python3 -m unittest glim/supervisor/test_glim_supervisor.py`
