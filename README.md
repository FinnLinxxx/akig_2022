# akig_2022
TUW akig course W2022

# First steps

Als erste Aufgabe sollen mit ROS ´topics´ ausgelesen und interpretiert werden, die für die Steuerung eines Roboters als relevant betrachtet wurden.
Erste Ergebnisse hierzu werden beim nächsten Termin am **19.11.2021 13h im EDV-Raum** präsentiert.

Hierfür clonen sie zunächst den Inhalt dieses Repositorys in ihre Linux-Arbeitsumgebung, es wird davon ausgegeangen, dass sie den Ordner in ihr Homeverzeichnis laden
```bash
$ cd ~
$ git clone https://github.com/FinnLinxxx/akig_2022.git
(den download können sie mit ls prüfen)
$ ls
```

In einem rosbag (dieses liegt in diesem Repository im Ordner [/rosbag](https://github.com/FinnLinxxx/akig_2022/tree/main/rosbag)) wurden Daten bei einer Testfahrt des Husky-Roboters aufgenommen. Diese können nun, nachdem der `roscore` gestartet wurde, wieder abgespielt werden:
```bash
$ cd ~/akig_2022/rosbag
$ rosbag play husky_with_imu.bag --loop
```

Schauen wir in die Auswahl der Topics 
```bash
$ rostopic list
```
sollen diese davon betrachtet werden:

* /odometry/filtered
* /joint_states
* /amcl_pose
* /novatel_data/rawimudata_SIunits
* /particlecloud


Hierfür erinnern wir uns an die Inhalte des [ROS Tutorials](http://wiki.ros.org/ROS/Tutorials) und der Übungen der ersten 3 Termine. So können wir etwa den Inhalt des topics `/joint_states` mit folgendem Befehl in der Konsole auslesen `$ rostopic echo /joint_states`. Für ein grundlegendes Verständnis wird davon ausgegangen, dass sie alle Inhalte des Kapitels **1.1 Beginner Level** bearbeitet und ausreichend nachvollzogen haben. Sie sind in der Lage nachzuvollziehen, als welcher Datentyp ein Topic vorliegt `$ rostopic info /joint_states` und können hierzu weitere [Infos](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/JointState.html) einholen.

Besonders augenmerk liegt auf dem listener, der sowohl im Tutorial behandelt wird, als auch in einer Grundversion zum auslesen von /joint_states in diesem Github-Repository beiliegt (Ordner: [scripts/listener.py](https://github.com/FinnLinxxx/akig_2022/blob/main/scripts/listener.py)). Dem Tutorial folgend, wird klar, wie man python-skripte in ROS ausführbar macht `$ rosrun beginner_tutorials listener.py`, hierfür musste die CMakeLists.txt angepasst werden, `catkin_make` ausgeführt und die der entsprechende workspace gesourced werden. 

Stattdessen kann zunächst die Ausführung der Datei auch direkt mit python getestet werden 

```bash
$ cd ~/akig_2022/scripts/
$ python listener.py
```

**Erstellen sie ein oder mehrere python Programme mit denen es möglich ist, die Daten der oben genannten Topics so zu verarbeiten, dass über deren Verhalten ein weiterführender Eindruck vermittelt werden kann. Dabei steht die Visualisierung und die Beschreibung der Daten zB. in Plots im Vordergrund. Nutzen sie hierfür auch die Möglichkeit aus dem Code heraus Textdateien zu erzeugen. Die einzelne Umsetzung ist ihnen überlassen, richten sie sich nach der Verständlichkeit ihrer Aussage, die zum Beispiel mit einschließt wie viele Kurven der Husky zur Laufzeit gefahren war.**


# Jupyter-Notebook

```bash
$ pip install --upgrade nbconvert
$ pip install --upgrade "ipython[all]"
$ pip install --upgrade jupyterhub
```




