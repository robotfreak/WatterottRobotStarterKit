#Sharp Sensor Modifikation

Der Infrarot Sensor (Sharp GP2Y0A21YK0F) ist eine preiswerte Möglichkeit um Entfernungen zu Objekten zu messen, die sich vor dem Roboter befinden. Leider ist das Ausgangssignal dieser Sensoren nicht sehr stabil und gibt daher keine genauen Werte zurück. Das Problem ist die Stromversorgung des Sensors. Das hatte ich ja schon mal vor einiger Zeit <a href="http://www.robotfreak.de/blog/robotik/sharp-ir-sensor-ausgabe-verbessern/479">geblogged</a>. Auch hier helfen Kondensatoren,die Störungen zu eliminieren. Dazu sind allerdings 2 Kondensatoren notwendig. Ein kleiner Keramik Kondensator von 100nF wie bei den Motoren für die kleinen Peaks und ein großer dicker Elektrolyt Kondensator von 100µF für die groben Schwankungen des Ausgangssignals. Am besten sind dafür SMD Bauteile geeignet. Durch die geringe Bauteilgröße passen diese Kondensatoren direkt an die Stromversorgungspins des Sensors. Die Kondensatoren müssen so nahe wie möglich am Sensor angebracht werden.

![Sharp-Sensor](images/circ/Sharp-Sensor-Mod.jpg "Sharp-Sensor")

![Sharp-Sensor-Modifikation](images/circ/Sharp-Sensor-Mod2.jpg "Sharp-Sensor-Modifikation")


##Benötigte Bauteile:

* 1 x <a href="http://www.reichelt.de/Vielschicht-SMD-G0805/X7R-G0805-100N/3//index.html?ACTION=3&GROUPID=3165&ARTICLE=31879&OFFSET=16&WKID=0&">100nF SMD 0805 Vielschichtkondensator </a>
* 1 x <a href="http://www.reichelt.de/Vielschicht-SMD-G1206-High-Cap/X5R-G1206-100-6/3//index.html?ACTION=3&GROUPID=4340&ARTICLE=89744&OFFSET=16&WKID=0&">100µF SMD 1206 Vielschichtkondensator </a>


