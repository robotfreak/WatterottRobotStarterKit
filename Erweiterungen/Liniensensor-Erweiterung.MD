# Liniensensor Erweiterung

Mit dieser  Erweiterung kann der Roboter einen schwarzen Linie auf weißem Unterrund folgen

## Was wird benötigt?

* Pololu QTR-3A Liniensensor (analog)
* 5 polige Buchsenleiste 
* 5 polige Stiftleiste 
* 2 polige Stiftleiste 
* 2 Schrauben M2x15, 4 Muttern M2
* Lötkolben, Seitenschneider, Dremel
* schwarzes Isolier Klebeband 10mm breit
* Plexiglas Halterung (im Roboter Kit enthalten)

## Durchführung

* Es müssen die beiden Lötbrücken SNS1 und SNS2 am Motor Shield entfernt werden.
* Die Verbindung Sensor / Arduino Shield erfolgt mit den Drähten, Buchsen- und Stiftleisten
* Der Sensor wird mit den M2 Schrauben und Muttern und der Plexiglas Halterung am Roboter Starter Kt befestigt 

## Verdrahtung

![Verdrahtung](../Fritzing/Line-Sensor-Erweiterung.png "Verdrahtung")

## Arduino Sketch

Es wird die Polou QTR Bibliothek benötigt
