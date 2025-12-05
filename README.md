# Mobil gépek mechatronikája projektfeladat

> A Budapesti Műszaki és Gazdaságtudományi Egyetem, Közlekedés- és Járműirányítási Tanszék, Mobil Gépek Mechatronikája tantárgy féléves projektfeladata.

> Készítette: 
> Hódi Balassa, Simon Zalán és Vincze Máté

> Konzulens: Csuzdi Domonkos

> 2025/26 őszi félév

## Általános információk

**Feladat rövid leírása:** Foglaltsági térkép készítése lidar és odometria információk alapján.

A kódsorok egy foglaltsági térkép előállítását valósítják meg ROS2-es környezetben. Az térkép megalkotásához szükséges algoritmusok objektum orientált Python kódolással vannak létrehozva.

A feladat során egy *turtlebot* segítségével kell bejárni egy adott térképet *gazebo* környezetben. Majd ennek a robotnak a lidar és odometria adatait kiolvasva meg kell alkotni egy foglaltsági térképet, ami megfelelően tárolja a feltértképezett terület információit.

A feladatban a lokalizáció adott, nincs szükség lokalizációs algoritmusok létrehozására, ugyanis a robot $x-y-\theta$ értékei közvetlenül az odometria üzenetből kiolvashatóak.

A foglaltsági térkép frissítése a *Bayes-elven* alapul, az egyes cellák valószínűségi értékeinek meghatározása pedig *log-odds elvre* épül. Fontos azonban megjegyezni, hogy a térkép dimenziói nem változnak dinamikusan, hanem a program kezdetekor vannak definiálva. Így ha a robot kimegy a térkép határain, akkor az ott levő adatok nincsenek feldolgozva.

A lidar adatok feldolgozása, azaz a *ray tracing* (sugárkövető) algoritmus egy saját elv szerint lett implementálva. Az algoritmus úgy diszkretizálja a folytonos sugarat, hogy megvizsgálja az éleket, amelyeken áthalad, és az adott élek szomszédos celláit dolgozza fel.

Ezekkel az algoritmusokkal egy megfelelő foglaltsági térkép készíthető. Erre egy példa a szimuláció során lementett kép is, ami ezen algoritmusok alkalmazásával a szimuláció közben folyamatosan frissítve lett létrehozva:

![A foglaltsági térképről egy mentett kép](/occupancy_grid_mapper/data/occupancy_grid_map.png)


## Mappa struktúra

Az adattár mappáinak a tartalma az alábbiakban olvasható:
```
occupancy_grid_mapper/  # ROS2 package
├── data/               # szimuláció során mentett adatok
├── launch/             # package-hez szükséges launch fájl mappája
├── local/              # saját Python modul (OccupancyGridMap objektum) mappája
├── resource/           # ROS2 package-hez szükséges mappa
├── test/               # ROS2 package-hez szükséges mappa
├── package.xml         # ROS2 package-hez szükséges fájl
├── setup.cfg           # ROS2 package-hez szükséges fájl
└── setup.py            # ROS2 package-hez szükséges fájl

Python/                 # az algoritmusok tesztelése során létrehozott kódok mappája
├── data/               # előzetesen mentett LiDAR adatok
├── improved/           # dinamikus térképezés algortimusai (copilot)
├── original/           # statikus térképezés algoritmusai
├── shared/             # saját modulok
├── README.md           # mappához tartozó README.md (copilot)
├── test.fixed.png      # mentett térkép (copilot)
└── test_imports.py     # fájlok iportálásának tesztelése (copilot)
```

A feladat megoldása az `occupancy_grid_mapper/` mappában található. A `Python/`mappa mindössze az algoritmusok létrehozásához kellett. A *(copilot)* megjegyzéssel ellátott mappák és fájlok a gtihub copilot segítségével lettek létrehozva.

## ROS2 package futtatása
Az adattár mentését egy ROS2-es "workspace" `src/` mappájában érdemes megtenni, ugyanis így lesz a package könnyen futtatható.

A mentés után a ROS2 "workspace" mappáját "build"-elni kell, majd újraindítani a .bashrc-t. Ezt a "workspace" mappájában az alábbi terminál parancsokkal lehet megtenni:
```bash
colcon build --packages-select occupancy_grid_mapper
source ~/.bashrc
```

Ezt követően a package futtatható a launch fájljával:
```bash
ros2 launch occupancy_grid_mapper occupancy_grid_mapper.launch.xml
```

Ezzel megnyílik a gazebo-ban a szimuláció. A turtlebot irányításához egy másik package szükséges, ennek elindítása:
```bash
ros2 run turtlebot3_teleop teleop_keyboard
```
Ilyenkor a "w-a-s-d-x" billentyűkkel lehet a robot sebességeit vezérelni.

A létehozott foglaltsági tértképet el lehet menteni a futás során. Ehhez az alábbi paraméterek beállítása szükséges a futás során:
```bash
ros2 param set file_path "<a_mentés_helyének_útvonala>"
ros2 param set want_to_save "true"
```
Ilyenkor fontos, hogy a kód a térkép egyes celláinak az értékeit menti el egy 2D-s tömbben a meghatározott fájlba. Érdemes .csv vagy .txt kiterjesztésű fájlba menteni az adatokat.

Ha a szimuláció során már nem kívánjuk menteni a tértképet azt a következő módon tehetjük meg:
```bash
ros2 param set want_to_save "false"
```

A mentett térképről .png kép is alkotható, ehhez azonban először szükséges a `occupancy_grid_mapper/local/OccupancyGridMap.py` fájl végnek módosítása:
```python
# debug the OccupancyGridMap class
if __name__ == "__main__":
    # load data from a file and visualize the map
    map = OccupancyGridMap(8.0, 8.0, 0.05)          # <-- ide ugyanazok az adatok kerüljenek, amik a mentett fájl térképére jellemzőek
    map.load_map(
        "<a_mentett_térkép_helyének_útvonala>"      # <-- ide a paraméterben magadott elérési út kell
    )
    map.visualize_map(
        saveMap=False,                              # <-- ha menteni szeretnénk legyen "True"
        filePath="<a_menteni_kívánt_kép_útvonala>", # <-- a kép mentésének a helyének megadása
    )

```
Majd ezután a kód lefuttatható az `occupancy_grid_mapper/local` mappából:
```bash
python3 OccupancyGridMap.py
```
Ilyenkor ha mentjük a képet, akkor nem jelenik meg, ha viszont csak vizualizálni szeretnénk, akkor megjelenik a kép egy külön ablakban.